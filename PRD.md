# 🏭 Project Mustang — Product Requirements Document (PRD)

> **Decentralized Autonomous Mobile Robot (AMR) Swarm for Smart Warehouse Order Fulfillment**
>
> Last updated: 2026-09-19 | Status: **Active Development (Alpha)**

---

## Table of Contents

1.  [Project Overview](#1-project-overview)
2.  [Problem Statement (SIH Context)](#2-problem-statement-sih-context)
3.  [System Process Flow (4 Phases)](#3-system-process-flow-4-phases)
4.  [System Inputs & Outputs](#4-system-inputs--outputs)
5.  [Tech Stack & Dependencies](#5-tech-stack--dependencies)
6.  [Repository Structure (File-by-File)](#6-repository-structure-file-by-file)
7.  [System Architecture](#7-system-architecture)
8.  [Warehouse Layout & Physical Design](#8-warehouse-layout--physical-design)
9.  [The Robot (AMR)](#9-the-robot-amr)
10. [Navigation System — Topological Graph A*](#10-navigation-system--topological-graph-a)
11. [Multi-Robot Coordination (Decentralized)](#11-multi-robot-coordination-decentralized)
12. [Task Lifecycle & State Machine](#12-task-lifecycle--state-machine)
13. [Edge AI — YOLO Vision](#13-edge-ai--yolo-vision)
14. [Dashboard & Telemetry](#14-dashboard--telemetry)
15. [How to Build & Run](#15-how-to-build--run)
16. [Current Status — What Works ✅](#16-current-status--what-works-)
17. [Current Status — What's Broken / Stuck 🔴](#17-current-status--whats-broken--stuck-)
18. [What Needs to Be Built Next 🔨](#18-what-needs-to-be-built-next-)
19. [Key Design Decisions & Rationale](#19-key-design-decisions--rationale)
20. [KPIs & Success Metrics](#20-kpis--success-metrics)
21. [Glossary](#21-glossary)

---

## 1. Project Overview

**Mustang** is a fully decentralized warehouse robot swarm simulation. Instead of a central server telling robots what to do, each AMR (Autonomous Mobile Robot) is an independent agent that:

- **Self-assigns tasks** from a shared task pool (no dispatcher)
- **Plans its own routes** using A* on a topological rail graph
- **Negotiates traffic conflicts** peer-to-peer with other AMRs
- **Hands off blocked tasks** via Contract Net Protocol (CNP) auctions
- **Detects humans and obstacles** using onboard YOLOv8 computer vision
- **Predicts and prevents congestion** using XGBoost on shared trajectory data
- **Self-audits swarm integrity** using Isolation Forest anomaly detection

The entire fleet runs **without any central controller, master node, or cloud server**. All communication happens via **Zenoh peer-to-peer** pub/sub middleware.

---

## 2. Problem Statement (SIH Context)

This project was built for the **Smart India Hackathon (SIH)** — a national-level hackathon in India. The problem statement (SIH26123) requires designing an autonomous warehouse fulfillment system using multi-robot coordination.

### Core Challenge

Design a system where **3+ AMRs** can autonomously:
1. Pick up cargo from any shelf bay in a warehouse
2. Deliver cargo to a designated drop-off point
3. Avoid collisions, deadlocks, and traffic jams — all **without a central controller**
4. Handle real-time obstacles (humans, fallen boxes, dead robots)

---

## 3. System Process Flow (4 Phases)

> This is the complete lifecycle of every task in the system.

### Phase 1: Task Injection & The Decentralized Auction (CNP)

1. **The Injection**: A Bash script (or any external system) acts as the warehouse management system, injecting a new delivery task (pickup and drop-off coordinates) directly into the shared Zenoh P2P topic `/swarm/tasks/pool`.
2. **The Battery Gate**: Every AMR currently in an `IDLE` state hears the broadcast. Before they are allowed to bid, they check their battery. If they are below ~25%, they disqualify themselves from the auction and route to a charger.
3. **The Math & The Bid**: Eligible AMRs calculate their driving distance to the pickup point and broadcast a numerical bid score back to the channel within a strict **500ms window**.
4. **Symmetry Breaking**: The AMRs compare bids locally. The robot with the best score automatically claims the task and transitions to `ASSIGNED`. If there is an exact tie, they apply a **SHA-256 hash tiebreaker** of `(task_id + amr_id)` to deterministically force a winner and prevent auction livelock — no central coordinator needed.

### Phase 2: Cargo Profiling & Intent Broadcast

1. **Visual Profiling (YOLOv8)**: Upon arriving at the cargo, the winning robot visually classifies the payload type. It dynamically adjusts its linear interpolation (speed) and P-controller (turning dynamics) for safe handling, and updates its required safety buffer on the Zenoh network.
2. **The Mapping**: The AMR uses its **Dynamic A*** algorithm on the topological rail graph to calculate the absolute shortest path to the drop-off point. Movement is strictly Manhattan (no diagonals).
3. **The Swarm Update**: Before moving, it broadcasts its planned `(x, y, time)` trajectory to the `/swarm/trajectories` topic. The dashboard passively renders this movement without issuing any commands.

### Phase 3: The Drive & Active Edge-AI Guardrails

1. **Predictive Routing (XGBoost)**: The swarm constantly analyzes the shared trajectory data. If the XGBoost model predicts that two or more AMRs will collide or congest a specific spot within the next **10 seconds**, it triggers proactive bidding and reroutes the robots **before** the bottleneck physically forms.
2. **Dynamic Yielding**: If AMRs meet at a physical choke point, they cross-reference the **Priority Aging formula** (`wait_time + battery`) to deterministically decide right-of-way without a central server.
3. **Obstacle Handling (YOLOv8)**: The onboard camera actively scans the physical path:
   - **Human Worker**: Triggers an immediate motor cut-off for human safety.
   - **Box / Dead AMR**: The robot recalculates a local detour and instantly broadcasts a "no way" alert across the Zenoh network. The entire swarm updates their local costmaps to permanently avoid that impassable aisle.
4. **The Trust Network (Isolation Forest + YOLO)**:
   - The Isolation Forest continuously audits Zenoh telemetry. If a glitching robot broadcasts impossible mathematical spikes (like teleporting), it is instantly flagged.
   - For plausible but false coordinates, YOLO physically verifies the location. If the robot isn't visibly there, its data is muted from the swarm's pathfinding math.

### Phase 4: Extreme Fail-Safes & Completion

1. **Degraded Network**: If a robot enters a severe Wi-Fi dead-zone, it falls back to purely local LiDAR/vision navigation and caps its speed at ~40% to proceed safely until it reconnects.
2. **Ghost Node Timeout**: If a robot goes entirely silent, the swarm uses a **2-tier timeout**. It waits ~2 seconds for network jitter; if the silence reaches ~6 seconds, the swarm marks the robot as dead and automatically re-auctions its uncompleted task.
3. **Delivery & Reset**: The cargo is dropped off, the AMR transitions back to `IDLE`, clears its trajectory, and immediately listens to the task pool for the next job.

---

## 4. System Inputs & Outputs

### Inputs (What Feeds the System)

| Input | Source | Format | Topic/Channel |
|-------|--------|--------|---------------|
| **Task Trigger** | Bash script / WMS API | JSON: `{task_id, pickup: {x,y}, dropoff: {x,y}}` | `/swarm/tasks/pool` |
| **Visual Data** | Onboard RGB Camera (per AMR) | Live camera feed → YOLOv8n-nano inference | `/amr_N/camera/image_raw` |
| **Spatial Data** | LiDAR sensor (per AMR) | 360° laser scan ranges | `/amr_N/scan` |
| **Odometry** | Wheel encoders + IMU | Position `(x,y)`, orientation `(yaw)` | `/amr_N/odom` |
| **Swarm Telemetry** | Other AMRs via Zenoh P2P | `(x, y, time)`, battery levels, hazard alerts | `/swarm/trajectories`, `/swarm/status` |

### Outputs (What the System Produces)

| Output | Description | Consumer |
|--------|-------------|----------|
| **Physical Delivery** | Cargo physically moved from pickup to drop-off, collision-free | Warehouse operations |
| **Live Visualization** | Real-time `(x, y)` positions, battery %, task states | Web Dashboard (port 5050) |
| **Evolved Swarm Knowledge** | "No way" broadcasts permanently update all AMR costmaps | The swarm itself |
| **KPI Metrics** | `order_to_delivery_latency`, `active_travel_time`, `idle_time`, `total_distance` | Dashboard + logs |
| **Safety Events** | `SAFETY_STOP` events logged and broadcast on person detection | Dashboard alerts |

---

## 5. Tech Stack & Dependencies

| Layer | Technology | Version | Purpose |
|-------|-----------|---------|---------|
| **Simulation** | Gazebo Harmonic (gz-sim) | 8.x | 3D physics simulation of warehouse + robots |
| **Robot Framework** | ROS 2 Jazzy Jalisco | Jazzy | Node communication, launch system, URDF |
| **Middleware** | Zenoh (rmw_zenoh_cpp) | 1.x | Peer-to-peer pub/sub (replaces DDS) |
| **Path Planning** | Custom A* on Topological Graph | — | Manhattan-only rail navigation |
| **Computer Vision** | YOLOv8n-nano (Ultralytics) | v8 | Person/obstacle detection via ONNX |
| **ML — Prediction** | XGBoost | — | Congestion prediction from trajectory data |
| **ML — Anomaly** | Isolation Forest (scikit-learn) | — | Telemetry anomaly detection |
| **Dashboard** | Flask + SSE + HTML/CSS/JS | — | Real-time operator UI |
| **Language** | Python 3.12 | 3.12 | All nodes |
| **Build System** | colcon | — | ROS 2 workspace build |
| **OS** | Ubuntu 24.04 LTS | Noble | Development & deployment |
| **VCS** | Git | — | Version control |

### Python Dependencies (per node)

```
rclpy, sensor_msgs, std_msgs, nav_msgs, geometry_msgs  # ROS 2
ultralytics, onnxruntime                                 # YOLO vision
xgboost, scikit-learn, numpy                            # ML prediction & anomaly
flask                                                    # Dashboard
```

### System Dependencies

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-gz-sim ros-jazzy-ros-gz-bridge
# Optional: rmw_zenoh_cpp for P2P mode (falls back to FastDDS otherwise)
```

---

## 6. Repository Structure (File-by-File)

> **This is the most important section for contributors.** Every file in the repo is documented here.

```
mustang/                          # ROS 2 package root
├── package.xml                   # ROS 2 package manifest (dependencies)
├── setup.py                      # Python package setup (entry points)
├── setup.cfg                     # colcon/setuptools config
├── LICENSE                       # MIT License
├── PRABU.docx                    # Solution process flow document (reference)
├── PRD.md                        # This file
├── .gitignore                    # Git ignore rules
│
├── src/                          # All source code
│   ├── task_manager_node.py      # ⭐ CORE: The brain of each AMR (1154 lines)
│   ├── cnp_auction_node.py       # Contract Net Protocol auction handler
│   ├── priority_negotiator_node.py # Priority-based yielding negotiation
│   ├── yolo_vision_node.py       # YOLOv8 onboard vision system
│   ├── multi_spawn.launch.py     # ROS 2 launch file for 3-AMR fleet + Gazebo
│   ├── run_swarm.sh              # One-command launcher script
│   ├── amr.urdf                  # Robot physical model (differential drive)
│   ├── sih_warehouse.sdf         # Gazebo world file (warehouse + racks + rails)
│   ├── zenoh_config.json5        # Zenoh P2P middleware configuration
│   ├── generate_visual_markers.py # Generates green dot/edge markers for Gazebo
│   ├── visual_rails.sdf          # Generated SDF fragment for rail visualization
│   ├── test_traffic_jam.sh       # Test script: injects 3 conflicting tasks
│   ├── benchmark_test.py         # Performance benchmarking script
│   │
│   └── dashboard/                # Real-time web dashboard
│       ├── app.py                # Flask server with SSE streaming
│       └── templates/
│           └── index.html        # Dashboard UI (HTML/CSS/JS, offline-capable)
│
├── build/                        # colcon build output (auto-generated)
├── install/                      # colcon install output (auto-generated)
└── log/                          # colcon log output (auto-generated)
```

### Detailed File Descriptions

---

#### `src/task_manager_node.py` — ⭐ The Core Brain (1154 lines)

**This is the most important file in the entire project.** Each AMR runs its own instance of this node. It contains:

| Component | Description |
|-----------|-------------|
| **`Segment` class** | Represents one axis-aligned rail move (pure X or pure Y). Enforces Manhattan-only locomotion. |
| **`collapse_to_segments()`** | Converts A* waypoints into corner-to-corner segments, eliminating diagonal drift. |
| **`AStarPlanner` class** | Topological graph A* pathfinder. Hardcoded warehouse graph with ~60 nodes and ~100 edges. Supports dual rails per aisle. |
| **`TaskManager` class (ROS Node)** | The FSM brain. Subscribes to `/swarm/tasks/pool`, `/odom`, `/cmd_vel`, `/scan`. Publishes `/cmd_vel` to drive the robot. |
| **State Machine** | `IDLE → CLAIMED → EN_ROUTE_PICKUP → DELIVERING → COMPLETED`. Also handles `BLOCKED_AUCTION` for handoff. |
| **SHA-256 Tiebreaker** | Deterministic conflict resolution when multiple AMRs claim the same task. |
| **Manhattan Segment FSM** | `ROTATE → DRIVE → NEXT` sub-states for each segment. Robot turns in place, then drives straight. |
| **P-Controller** | Cross-track error correction to keep robot precisely on rail. |
| **LiDAR Safety** | Emergency stop if any object is within 0.6m (configurable). |
| **KPI Logger** | Tracks `active_time`, `idle_time`, `total_distance`, `order_to_delivery_latency`. |

**Key topics this node uses:**
```
SUB: /swarm/tasks/pool         → Listens for new tasks
SUB: /amr_N/odom               → Gets current position
SUB: /amr_N/scan               → LiDAR safety braking
SUB: /amr_N/cmd_vel            → Velocity monitoring (for KPI)
SUB: /amr/obstruction_status   → YOLO obstacle alerts
PUB: /swarm/tasks/pool         → Publishes CLAIM messages
PUB: /amr_N/cmd_vel            → Motor velocity commands
PUB: /swarm/<id>/mission_metrics → KPI data
PUB: /task_manager_status      → State machine debug
```

---

#### `src/cnp_auction_node.py` — Contract Net Protocol (CNP) Auction

Handles task re-allocation when an AMR gets blocked. Uses the [Contract Net Protocol](https://en.wikipedia.org/wiki/Contract_Net_Protocol):

1. Blocked AMR broadcasts a `CALL_FOR_PROPOSALS` with the stuck task
2. Free AMRs calculate their bid (distance + battery + priority)
3. Best bidder wins and takes over the task
4. Original AMR transitions to `IDLE`

**Status**: ✅ Implemented, integrated with task_manager_node.py

---

#### `src/priority_negotiator_node.py` — Priority-Based Yielding

Handles real-time traffic negotiation at intersections:

1. When two AMRs approach the same graph node, they broadcast their priority score
2. Priority = `base_priority + wait_time_bonus + battery_penalty`
3. Lower-priority AMR yields (stops and waits)
4. **Priority Aging**: Wait time increases priority to prevent starvation

**Status**: ✅ Implemented, publishes to `/swarm/priority` topics

---

#### `src/yolo_vision_node.py` — Onboard YOLOv8 Vision

Each AMR runs a YOLOv8n-nano model for real-time object detection:

| Detection | Action |
|-----------|--------|
| **Person** | Immediate `SAFETY_STOP` — all motors killed |
| **Box/Obstacle** | Broadcast "no way" alert, trigger local detour |
| **Cargo Classification** | Adjusts speed profile based on payload type |

- Uses ONNX runtime for inference (5 FPS throttled to save CPU)
- Currently uses COCO proxy classes (`backpack` → pallet, `suitcase` → box)
- Publishes to `/amr/obstruction_status`

**Status**: ✅ Working with COCO proxies. Needs fine-tuning on warehouse dataset.

---

#### `src/multi_spawn.launch.py` — Fleet Launch Configuration

ROS 2 launch file that:
1. Starts Gazebo Harmonic with `sih_warehouse.sdf`
2. Spawns 3 AMRs at predefined positions on the rail graph
3. Sets up `ros_gz_bridge` for each AMR (odom, cmd_vel, scan, camera)
4. Configures namespaces (`/amr_1`, `/amr_2`, `/amr_3`)

**Current spawn positions** (on rail nodes in staging area):
```python
amr_1: x=4.25, y=1.0    # Aisle 1, Rail B, staging row
amr_2: x=8.75, y=1.0    # Aisle 2, Rail B, staging row
amr_3: x=13.25, y=1.0   # Aisle 3, Rail B, staging row
```

---

#### `src/run_swarm.sh` — One-Command Launcher

Simple bash script that:
1. Sources ROS 2 Jazzy
2. Configures Zenoh P2P middleware (or falls back to FastDDS)
3. Launches `multi_spawn.launch.py`

```bash
# To run:
cd /home/siva/mustang
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./src/run_swarm.sh
```

> **Note**: Task manager nodes must currently be started separately (see Section 15 — Known Issues).

---

#### `src/amr.urdf` — Robot Physical Model

Differential-drive robot with:
- **Chassis**: 0.4m × 0.3m × 0.15m box
- **Two drive wheels**: radius 0.075m, separated 0.3m
- **Front caster wheel**: frictionless (mu=0.0) to prevent camera pitch
- **LiDAR sensor**: 360° scan, 10m range, mounted on top
- **RGB Camera**: forward-facing, 1.396 rad FOV, 640×480

---

#### `src/sih_warehouse.sdf` — Gazebo World File

The complete 3D warehouse environment:

- **Dimensions**: 20m × 16m floor
- **8 rack shelves** (4 per side) organized in 3 aisles
- **3 aisles**, each 2.5m wide with dual parallel rails
- **Staging area** at bottom (y=0.0 to y=2.0)
- **Cross-corridors** at top (y=14.0) and bottom (y=0.0)
- **Visual markers**: Green dots at graph nodes, cyan lines along edges, bay dividers
- **Obstacle zones**: Red "trap zone" blocks for testing obstacle avoidance

---

#### `src/zenoh_config.json5` — Zenoh P2P Configuration

```json5
{
  mode: "peer",           // No broker/router needed
  connect: { endpoints: [] },  // Pure peer discovery
  scouting: {
    multicast: { enabled: true },  // UDP multicast for auto-discovery
    gossip: { enabled: true }      // Gossip protocol for larger networks
  }
}
```

---

#### `src/dashboard/app.py` — Flask Dashboard Server

- Subscribes to ROS 2 topics for all 3 AMRs
- Streams real-time data via **Server-Sent Events (SSE)**
- Serves the dashboard UI on **http://localhost:5050**
- Tracks: position, battery, state, current task, speed
- **Offline-capable**: Works without internet (no CDN dependencies)

---

#### `src/dashboard/templates/index.html` — Dashboard UI

Single-page HTML/CSS/JS dashboard with:
- **3 AMR status cards** showing real-time state, position, battery
- **Task feed** showing active and completed tasks
- **KPI panels** with delivery time, fleet utilization, collision count
- **Dark theme** with glassmorphism design
- **Auto-reconnect SSE** for reliable streaming

---

#### `src/test_traffic_jam.sh` — Traffic Jam Test Script

Injects 3 tasks designed to create maximum congestion:
```bash
# All 3 AMRs are sent through the same aisle at the same time
# Tests: CNP auction, priority negotiation, dynamic rerouting
```

---

#### `src/benchmark_test.py` — Performance Benchmarking

Measures:
- A* path computation time
- Task claiming latency
- End-to-end delivery time
- Fleet throughput (tasks/minute)

---

#### `src/generate_visual_markers.py` — Visual Marker Generator

Python script that generates SDF `<model>` elements for:
- **Green spheres** at each graph node (debug visualization)
- **Cyan cylinders** along each graph edge (rail track visualization)
- **Bay divider markers** at semantic bay locations
- Output is spliced into `sih_warehouse.sdf`

---

## 7. System Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                        WAREHOUSE FLOOR                              │
│                                                                     │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐                      │
│  │  AMR #1  │    │  AMR #2  │    │  AMR #3  │  ... (expandable)    │
│  │          │    │          │    │          │                       │
│  │ ┌──────┐ │    │ ┌──────┐ │    │ ┌──────┐ │                      │
│  │ │TaskMgr│ │    │ │TaskMgr│ │    │ │TaskMgr│ │  ← Each runs its  │
│  │ │CNP   │ │    │ │CNP   │ │    │ │CNP   │ │    own full stack   │
│  │ │PriNeg│ │    │ │PriNeg│ │    │ │PriNeg│ │                      │
│  │ │YOLO  │ │    │ │YOLO  │ │    │ │YOLO  │ │                      │
│  │ │A*    │ │    │ │A*    │ │    │ │A*    │ │                      │
│  │ └──────┘ │    │ └──────┘ │    │ └──────┘ │                      │
│  └─────┬────┘    └─────┬────┘    └─────┬────┘                      │
│        │               │               │                            │
│        └───────────────┼───────────────┘                            │
│                        │                                            │
│              ┌─────────▼─────────┐                                  │
│              │   Zenoh P2P Bus   │  ← No central broker             │
│              │  (UDP Multicast)  │                                   │
│              └─────────┬─────────┘                                  │
│                        │                                            │
│              ┌─────────▼─────────┐                                  │
│              │   Gazebo Physics  │  ← Simulates warehouse           │
│              │   (gz-sim 8.x)   │    + robot physics                │
│              └───────────────────┘                                  │
│                                                                     │
└──────────────────────────────────────────────────────────────────────┘
                         │
               ┌─────────▼─────────┐
               │  Flask Dashboard  │  ← Passive observer only
               │  (Port 5050, SSE) │    Never issues commands
               └───────────────────┘
```

### Key Principle: **No Central Server**

- The Dashboard is a **passive observer** — it reads telemetry but never sends commands
- All AMR decisions are made **locally** on each robot
- Communication is **peer-to-peer** via Zenoh pub/sub
- If any single AMR fails, all others continue operating independently

---

## 8. Warehouse Layout & Physical Design

### Floor Plan (20m × 16m)

```
      0m    2.5m  5.0m  7.0m  9.5m  11.5m  14.0m  16.5m  20m
 16m  ┌──────┬─────┬──────┬─────┬──────┬──────┬──────┬──────┐
      │      │R1A  │      │R2A  │      │R3A   │      │      │
      │      │     │ A1   │     │ A2   │      │ A3   │      │
      │      │R1B  │      │R2B  │      │R3B   │      │      │
      │ WALL │     │2.5m  │     │2.5m  │      │2.5m  │ WALL │
      │      │R1A  │wide  │R2A  │wide  │R3A   │wide  │      │
      │      │     │      │     │      │      │      │      │
      │      │R1B  │      │R2B  │      │R3B   │      │      │
  2m  ├──────┴─────┴──────┴─────┴──────┴──────┴──────┴──────┤
      │           STAGING AREA / CROSS-CORRIDOR              │
  0m  └──────────────────────────────────────────────────────┘

      R = Rack shelf (0.5m deep)
      A = Aisle (2.5m wide, dual parallel rails)
```

### Dual Parallel Rails Per Aisle

Each 2.5m wide aisle contains **two parallel rail tracks** spaced 1.0m apart:

```
       Rack Shelf (edge)
       ─────────────────
       |← 35cm →|
       |         Rail A (x = 3.25m)    ← Track 1
       |         |← 1.0m →|
       |         |         Rail B (x = 4.25m)    ← Track 2
       |         |         |← 35cm →|
       ─────────────────────────────
       Rack Shelf (edge)
```

- **35cm clearance** between robot and shelf on each side
- Robots can **overtake** by switching rails at junction nodes
- **Rail switching** happens at cross-corridor intersections (y=0.0 and y=14.0)

### Semantic Bay System

Each rack has **10 bays** (numbered 0–9) along its length:
- Bays are the pickup/dropoff locations for tasks
- Named as `r1a_bay_3`, `r2b_bay_7`, etc.
- Each bay has a dedicated graph node for precise navigation

---

## 9. The Robot (AMR)

### Physical Specs

| Property | Value |
|----------|-------|
| Chassis | 0.4m × 0.3m × 0.15m box |
| Weight | ~5 kg (simulated) |
| Drive type | Differential drive (2 wheels) |
| Wheel radius | 0.075m |
| Wheel separation | 0.3m |
| Max linear speed | 0.3 m/s |
| Max angular speed | 1.0 rad/s |
| Front caster | Frictionless (prevents camera pitch) |

### Sensors

| Sensor | Specs | Topic |
|--------|-------|-------|
| **LiDAR** | 360°, 10m range, on chassis top | `/amr_N/scan` |
| **RGB Camera** | Forward-facing, 640×480, 80° FOV | `/amr_N/camera/image_raw` |
| **Wheel Encoders** | Built into differential drive plugin | `/amr_N/odom` |

### URDF File

Located at `src/amr.urdf`. Uses Gazebo Harmonic plugins:
- `gz::sim::systems::DiffDrive` — wheel velocity control
- `gz::sim::systems::Sensors` — LiDAR + Camera
- `gz::sim::systems::Imu` — Inertial measurement

---

## 10. Navigation System — Topological Graph A*

### Why Not SLAM?

Warehouse layouts are **known and fixed**. Using SLAM would be:
- Computationally expensive for no benefit
- Unnecessary — we already know where every shelf is
- Prone to localization drift in repetitive environments

### The Topological Graph

Instead of a 2D occupancy grid, the warehouse is modeled as a **weighted graph**:

```
Nodes = Named intersections and bay positions
Edges = Rail segments connecting nodes (Manhattan-only)
```

The graph is **hardcoded** in `task_manager_node.py` (`AStarPlanner` class) with:
- ~60 nodes (intersections, bay positions, staging spots)
- ~100 edges (rail segments, cross-corridor connections)
- Edge weights = Euclidean distance
- **+999.0 penalty** on edges with detected head-on conflicts

### A* Algorithm

Standard A* with:
- **Heuristic**: Manhattan distance (admissible for grid graphs)
- **Dynamic edge costs**: Edges are penalized (+999.0) if another AMR is heading towards the robot on the same edge
- **Replanning**: Path is recalculated every time a new obstacle or conflict is detected

### Path Execution — Manhattan Segment FSM

Once A* produces a path (list of waypoints), it's converted to **Segments**:

```
Path: [(3.25, 1.0), (3.25, 5.0), (8.75, 5.0), (8.75, 10.0)]
                         ↓
Segments:
  1. DRIVE Y: (3.25, 1.0) → (3.25, 5.0)   [straight up aisle]
  2. DRIVE X: (3.25, 5.0) → (8.75, 5.0)   [cross-corridor]
  3. DRIVE Y: (8.75, 5.0) → (8.75, 10.0)  [straight up aisle]
```

For each segment, the robot:
1. **ROTATE** in place to face the segment heading (±0.15 rad threshold)
2. **DRIVE** straight with P-controller cross-track correction
3. **ARRIVE** when within 0.15m of target waypoint

This eliminates "banana curve" drift that differential drive robots exhibit on diagonal paths.

---

## 11. Multi-Robot Coordination (Decentralized)

### Layer 1: Task Claiming (Auction)

When a new task appears on `/swarm/tasks/pool`:
1. All `IDLE` AMRs calculate distance to pickup
2. Each broadcasts a `CLAIM` message with their bid
3. **Tiebreaker**: `SHA-256(task_id + amr_id)` — deterministic, no coordinator
4. Winner transitions to `CLAIMED`, losers stay `IDLE`

### Layer 2: Traffic Negotiation (Priority)

When two AMRs approach the same intersection:
1. Each broadcasts priority score: `base + wait_bonus + battery_factor`
2. Lower-priority AMR stops and yields
3. **Priority aging** prevents starvation (waiting increases priority)

### Layer 3: Stuck Task Handoff (CNP)

When an AMR is blocked for too long:
1. It broadcasts `CALL_FOR_PROPOSALS` with the stuck task
2. Free AMRs bid based on their distance and availability
3. Best bidder takes over the task
4. Original AMR returns to `IDLE`

### Layer 4: Dynamic Rerouting (A* + Edge Penalties)

When a head-on conflict is detected:
1. The conflicting edge gets a +999.0 cost penalty
2. Both AMRs replan their paths
3. The system naturally finds non-conflicting routes

---

## 12. Task Lifecycle & State Machine

```
                    ┌─────────────┐
                    │    IDLE     │◄──────────────────────────────┐
                    └──────┬──────┘                               │
                           │ Task appears on /swarm/tasks/pool    │
                           ▼                                      │
                    ┌─────────────┐                               │
                    │   CLAIMED   │  SHA-256 tiebreaker wins      │
                    └──────┬──────┘                               │
                           │ A* path computed to pickup           │
                           ▼                                      │
                    ┌──────────────────┐                          │
                    │ EN_ROUTE_PICKUP  │  Driving to shelf bay    │
                    └──────┬───────────┘                          │
                           │ Arrived at pickup (< 0.15m)          │
                           ▼                                      │
                    ┌─────────────┐                               │
                    │  DELIVERING │  Driving to drop-off          │
                    └──────┬──────┘                               │
                           │ Arrived at drop-off (< 0.15m)        │
                           ▼                                      │
                    ┌─────────────┐                               │
                    │  COMPLETED  │──────────────────────────────►┘
                    └─────────────┘  KPIs logged, return to IDLE

                    ┌──────────────────┐
                    │ BLOCKED_AUCTION  │  ← If stuck > timeout
                    └──────┬───────────┘
                           │ CNP handoff to another AMR
                           ▼
                    Back to IDLE
```

### State Transitions (code in `task_manager_node.py`)

| From | To | Trigger |
|------|----|---------|
| `IDLE` | `CLAIMED` | Won SHA-256 auction for a task |
| `CLAIMED` | `EN_ROUTE_PICKUP` | A* path computed successfully |
| `EN_ROUTE_PICKUP` | `DELIVERING` | Reached pickup waypoint |
| `DELIVERING` | `COMPLETED` | Reached drop-off waypoint |
| `COMPLETED` | `IDLE` | KPIs logged, task cleared |
| Any driving state | `BLOCKED_AUCTION` | Stuck for > N seconds |
| `BLOCKED_AUCTION` | `IDLE` | CNP handoff complete |

---

## 13. Edge AI — YOLO Vision

### Architecture

```
Camera Frame (640×480 RGB)
         │
         ▼
   ┌───────────┐
   │  YOLOv8n  │  ← Nano model, ONNX runtime
   │  (5 FPS)  │    Throttled to save CPU
   └─────┬─────┘
         │
         ▼
   ┌──────────────────────┐
   │ Detection Categories │
   ├──────────────────────┤
   │ Person    → SAFETY_STOP (motor kill)        │
   │ Backpack  → Cargo type: LIGHT_PAYLOAD       │
   │ Suitcase  → Cargo type: MEDIUM_PAYLOAD      │
   │ Couch     → Cargo type: HEAVY_PAYLOAD       │
   │ Chair     → Obstacle: BLOCKED_AISLE         │
   └──────────────────────┘
```

### Speed Profiles Based on Cargo

| Cargo Type | Max Linear Speed | Turn Speed | Safety Buffer |
|-----------|-----------------|------------|---------------|
| No cargo | 0.3 m/s | 1.0 rad/s | 0.6m |
| Light | 0.25 m/s | 0.8 rad/s | 0.8m |
| Medium | 0.2 m/s | 0.6 rad/s | 1.0m |
| Heavy | 0.15 m/s | 0.4 rad/s | 1.2m |

### Current Limitation

YOLOv8n is trained on **COCO dataset** — it doesn't have "pallet" or "warehouse box" classes. We use proxy mappings (`backpack` → light payload, etc.). For production, this needs **fine-tuning on a warehouse-specific dataset**.

---

## 14. Dashboard & Telemetry

### Tech Stack

- **Backend**: Flask (Python) with Server-Sent Events (SSE)
- **Frontend**: Vanilla HTML/CSS/JS (no framework, no CDN)
- **Data Source**: ROS 2 topics via `rclpy` subscriber
- **Port**: `http://localhost:5050`

### Features

| Feature | Status |
|---------|--------|
| Real-time AMR position display | ✅ |
| Battery percentage per AMR | ✅ |
| Current task state (IDLE/CLAIMED/DELIVERING...) | ✅ |
| Active task details (pickup/dropoff coords) | ✅ |
| KPI metrics (delivery time, distance) | ✅ |
| SSE auto-reconnect on disconnect | ✅ |
| Dark theme with glassmorphism | ✅ |
| Offline-capable (no CDN) | ✅ |
| Live 2D warehouse map with AMR positions | ❌ TODO |
| Historical task timeline | ❌ TODO |
| CSV export for KPI reports | ❌ TODO |

### Running the Dashboard

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/dashboard/app.py
# Open http://localhost:5050
```

---

## 15. How to Build & Run

### Prerequisites

```bash
# Ubuntu 24.04 with ROS 2 Jazzy installed
sudo apt install ros-jazzy-desktop ros-jazzy-gz-sim ros-jazzy-ros-gz-bridge

# Optional: Zenoh P2P middleware
# Without it, system falls back to FastDDS (works identically)
```

### Build

```bash
cd /home/siva/mustang
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Run — Full System

```bash
# Terminal 1: Launch Gazebo + 3 AMRs
./src/run_swarm.sh

# Terminal 2: Launch task managers (one per AMR) — CURRENTLY MANUAL
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 src/task_manager_node.py --ros-args -r __ns:=/amr_1 -r odom:=/amr_1/odom -r cmd_vel:=/amr_1/cmd_vel -r scan:=/amr_1/scan

# Terminal 3: Second AMR task manager
python3 src/task_manager_node.py --ros-args -r __ns:=/amr_2 -r odom:=/amr_2/odom -r cmd_vel:=/amr_2/cmd_vel -r scan:=/amr_2/scan

# Terminal 4: Third AMR task manager
python3 src/task_manager_node.py --ros-args -r __ns:=/amr_3 -r odom:=/amr_3/odom -r cmd_vel:=/amr_3/cmd_vel -r scan:=/amr_3/scan

# Terminal 5: Dashboard
python3 src/dashboard/app.py

# Terminal 6: Inject test tasks
bash src/test_traffic_jam.sh
```

### Run — Dashboard Only

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 src/dashboard/app.py
# Open http://localhost:5050
```

### Inject a Custom Task

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /swarm/tasks/pool std_msgs/msg/String "{data: '{\"task_id\": \"my_task_1\", \"status\": \"AVAILABLE\", \"pickup\": {\"x\": 3.25, \"y\": 5.0}, \"dropoff\": {\"x\": 8.75, \"y\": 10.0}}'}"
```

### Verify Topics Are Running

```bash
ros2 topic list       # Should show /amr_1/odom, /amr_1/cmd_vel, /swarm/tasks/pool, etc.
ros2 topic echo /amr_1/odom --once   # Check AMR position
```

---

## 16. Current Status — What Works ✅

| Feature | Status | Notes |
|---------|--------|-------|
| Gazebo warehouse world (20m × 16m) | ✅ Working | 8 racks, 3 aisles, visual markers |
| 3-AMR fleet spawning | ✅ Working | Namespaced `/amr_1`, `/amr_2`, `/amr_3` |
| Differential drive locomotion | ✅ Working | URDF + gz DiffDrive plugin |
| Topological Graph A* pathfinding | ✅ Working | ~60 nodes, ~100 edges, Manhattan-only |
| Manhattan Segment FSM (ROTATE → DRIVE) | ✅ Working | Zero lateral drift |
| SHA-256 deterministic task claiming | ✅ Working | No central coordinator |
| CNP auction for blocked task handoff | ✅ Working | `cnp_auction_node.py` |
| Priority-based yielding negotiation | ✅ Working | Priority aging prevents starvation |
| LiDAR safety braking (<0.6m) | ✅ Working | Immediate motor kill |
| YOLOv8n person/obstacle detection | ✅ Working | 5 FPS throttled, ONNX runtime |
| Dynamic A* lane changing (+999 penalty) | ✅ Working | Head-on conflict avoidance |
| Zenoh P2P decentralized comms | ✅ Working | No broker, UDP multicast |
| Dual parallel rails per aisle (2.5m wide) | ✅ Working | 35cm shelf clearance |
| Visual floor markers (green dots, bay dividers) | ✅ Working | Debug visualization in Gazebo |
| Flask dashboard (SSE, offline-capable) | ✅ Working | Port 5050 |
| KPI logging (duration, distance, idle time) | ✅ Working | Per-task metrics |
| Git version control | ✅ Working | 30+ meaningful commits |

---

## 17. Current Status — What's Broken / Stuck 🔴

### 🔴 P1 — Critical (Must Fix)

| Issue | Details | File(s) |
|-------|---------|---------|
| **Visual rail markers sometimes cause duplicate SDF model names** | When `generate_visual_markers.py` output is spliced into `sih_warehouse.sdf`, duplicate `<model name="vis_div_0">` entries can crash Gazebo's XML parser. Need to ensure unique names or remove old markers before re-injecting. | `generate_visual_markers.py`, `sih_warehouse.sdf` |
| **AMRs sometimes don't move after task injection** | Task is claimed (visible in logs) but the robot doesn't execute the path. Suspected cause: the `/amr_N/odom` topic hasn't published yet when the task manager starts, so initial position is `(0,0)` and the A* plan goes haywire. Need a "wait for first odom" guard. | `task_manager_node.py` |
| **Task manager nodes must be started manually** | `run_swarm.sh` only launches Gazebo + AMR spawning. Task managers, CNP, priority, and YOLO nodes must be launched in separate terminals. Need to auto-launch all nodes from the launch file. | `run_swarm.sh`, `multi_spawn.launch.py` |

### 🟡 P2 — Important (Should Fix)

| Issue | Details | File(s) |
|-------|---------|---------|
| **YOLO uses COCO proxy classes** | No real "pallet" or "warehouse box" detection. `backpack`/`suitcase`/`couch` are proxies. Need to either fine-tune on warehouse dataset or add custom ONNX model. | `yolo_vision_node.py` |
| **Frictionless front caster is a hack** | Added to prevent camera pitch during acceleration. Proper fix would be a better URDF inertia tensor or a suspension model. | `amr.urdf` |
| **Dashboard doesn't auto-discover AMRs** | Currently hardcoded for 3 AMRs. Should dynamically discover fleet size from Zenoh topics. | `dashboard/app.py` |
| **No return-to-staging logic** | After completing a task, AMRs stay at the delivery location instead of returning to the staging area. | `task_manager_node.py` |
| **XGBoost congestion prediction not yet integrated** | The PRABU.docx describes XGBoost for 10-second-ahead collision prediction, but this is not yet implemented in code. | NEW: `xgboost_predictor_node.py` |
| **Isolation Forest trust network not yet integrated** | The PRABU.docx describes anomaly detection on swarm telemetry, but this is not yet implemented in code. | NEW: `trust_network_node.py` |

### 🟠 P3 — Nice to Have

| Issue | Details |
|-------|---------|
| **No battery simulation** | Battery percentage is simulated as a static value. Should drain based on distance traveled and recharge when at staging. |
| **No multi-floor support** | Current layout is single-floor. SIH problem may require elevator/multi-level. |
| **No RViz visualization** | Could add RViz2 markers for path visualization alongside Gazebo. |
| **No degraded network fallback** | The PRABU.docx describes Wi-Fi dead-zone behavior (cap speed to 40%, local-only nav). Not yet implemented. |
| **No ghost node timeout** | The PRABU.docx describes 2-tier timeout (2s jitter, 6s dead). Not yet implemented. |
| **Dashboard live 2D map** | A top-down warehouse map showing AMR dots in real-time. |

---

## 18. What Needs to Be Built Next 🔨

### Phase 1 — Stabilization (Immediate)

- [ ] **Fix SDF duplicate model name bug** in `generate_visual_markers.py` — ensure all generated model names are globally unique
- [ ] **Add "wait for first odom" guard** in `task_manager_node.py` before accepting tasks
- [ ] **Auto-launch all nodes** — modify `multi_spawn.launch.py` or `run_swarm.sh` to also launch task managers, CNP, priority, and YOLO nodes per AMR (currently manual)
- [ ] **Fix Gazebo launch reliability** — sometimes `gz sim` fails silently; add health checks

### Phase 2 — Feature Completion (PRABU.docx features)

- [ ] **XGBoost Congestion Prediction** — New node `xgboost_predictor_node.py`:
  - Subscribe to `/swarm/trajectories`
  - Predict collision points 10 seconds ahead
  - Trigger proactive rerouting via edge cost penalties
- [ ] **Isolation Forest Trust Network** — New node `trust_network_node.py`:
  - Audit all AMR telemetry for anomalous position jumps
  - Flag/mute glitching robots from swarm pathfinding
  - Cross-verify with YOLO visual confirmation
- [ ] **Battery Drain Simulation** — In `task_manager_node.py`:
  - Decrease battery based on distance traveled
  - Battery Gate: disqualify from auctions when <25%
  - Route to charging station when low
- [ ] **Return-to-staging behavior** — AMRs should navigate back to staging after completing delivery
- [ ] **Degraded Network Fallback** — In `task_manager_node.py`:
  - Detect Zenoh connectivity loss
  - Fall back to local LiDAR/vision navigation only
  - Cap speed to 40%
- [ ] **Ghost Node Timeout** — In `task_manager_node.py` or `cnp_auction_node.py`:
  - 2-second jitter tolerance
  - 6-second dead timeout → auto re-auction uncompleted task
- [ ] **Dynamic fleet scaling** — support 5+ AMRs without code changes

### Phase 3 — Dashboard Enhancements

- [ ] **Live 2D map** with AMR positions plotted on warehouse layout
- [ ] **Historical task timeline** — visual Gantt chart of task execution
- [ ] **Fleet utilization percentage** — real-time metric
- [ ] **Export KPI reports as CSV**
- [ ] **Alert sounds** on SAFETY_STOP events

### Phase 4 — Demo Polish

- [ ] **Fine-tune YOLOv8 on warehouse dataset** — real pallet/box/forklift detection
- [ ] **RViz2 path visualization** — show planned paths as colored lines
- [ ] **Multi-order batching** — AMR picks up multiple items in one trip
- [ ] **Charging station model** — add physical charging pads in Gazebo world
- [ ] **Video recording** — auto-record Gazebo simulation for demo purposes

---

## 19. Key Design Decisions & Rationale

### Why Decentralized (No Central Server)?
- **Resilience**: If any single robot fails, the others keep working. No single point of failure.
- **Scalability**: Adding a new AMR = just spawning another instance. No server config changes.
- **Edge-native**: Each robot is self-sufficient. Works even if network is degraded.

### Why Zenoh Instead of DDS?
- Zenoh is the **next-gen ROS 2 middleware** replacing DDS
- Built-in peer discovery via gossip (no daemon needed)
- Better suited for edge/IoT scenarios
- Lower overhead than Cyclone DDS or Fast-RTPS

### Why Topological Graph Instead of SLAM?
- Warehouse layouts are **known and fixed** — no need for SLAM exploration
- Topological graph gives **semantic meaning** to locations (bay names, not just coordinates)
- A* on a graph is **orders of magnitude faster** than A* on a 2D occupancy grid
- Manhattan-only movement **prevents diagonal drift** that causes shelf collisions

### Why Manhattan Segment FSM?
- Robots on rails should move like trains — **turn in place, then drive straight**
- Prevents the "banana curve" problem where differential drive robots cut corners
- Guarantees robots stay on the rail track with minimal lateral error

### Why SHA-256 Tiebreaker for Task Claiming?
- Two equidistant robots would enter a race condition
- SHA-256 of `(task_id + amr_id)` is **deterministic** — both robots compute the same winner
- No network round-trip needed, no central coordinator
- Hash is uniformly distributed — fair over many tasks

### Why YOLOv8n-nano Instead of Larger Models?
- Runs at **5+ FPS on CPU** — no GPU required per robot
- Nano model is <6MB — fits on edge devices
- Detection quality is sufficient for safety-critical person detection
- Larger models would bottleneck the main navigation loop

---

## 20. KPIs & Success Metrics

| KPI | Target | How Measured |
|-----|--------|-------------|
| Task completion rate | > 95% | `COMPLETED` count / total tasks dispatched |
| Average pick-to-deliver time | < 60 seconds | `completed_at - claimed_at` per task |
| Deadlock resolution time | < 5 seconds | Priority negotiation convergence time |
| Safety stop latency | < 200ms | Time from person detection to motor kill |
| Fleet utilization | > 70% | `active_travel_time / total_time` per AMR |
| CNP auction success rate | > 90% | Successful handoffs / total auction attempts |
| Collision count | 0 | LiDAR-triggered emergency stops |
| Congestion prediction accuracy | > 80% | XGBoost correct predictions / total predictions |
| Anomaly detection precision | > 90% | True positives / (true + false positives) |

---

## 21. Glossary

| Term | Definition |
|------|-----------|
| **AMR** | Autonomous Mobile Robot — a robot that navigates independently |
| **Bay** | A semantic parking position adjacent to a rack shelf (e.g., `r2a_bay_5`) |
| **CNP** | Contract Net Protocol — an auction mechanism for task reallocation |
| **Differential Drive** | A robot driven by two independently controlled wheels |
| **FSM** | Finite State Machine — a model with discrete states and transitions |
| **Manhattan Movement** | Movement restricted to horizontal and vertical directions only (no diagonals) |
| **Priority Aging** | A scheduling technique where priority increases with wait time to prevent starvation |
| **SDF** | Simulation Description Format — Gazebo's world description file format |
| **SSE** | Server-Sent Events — a unidirectional HTTP streaming protocol (server → client) |
| **Staging Area** | The starting/parking area for AMRs (bottom of warehouse, y < 2.0m) |
| **Topological Graph** | A graph of named nodes and edges representing navigable paths |
| **URDF** | Unified Robot Description Format — describes robot geometry and physics |
| **Zenoh** | A pub/sub/query middleware for edge computing (used as ROS 2 RMW layer) |
| **XGBoost** | Gradient boosting ML model used for congestion prediction |
| **Isolation Forest** | Unsupervised anomaly detection algorithm for telemetry auditing |
| **Edge AI** | AI inference running directly on the robot (no cloud round-trip) |
| **P2P** | Peer-to-peer — direct communication between robots without a broker |
| **Cross-track Error** | Perpendicular distance from the robot to the ideal rail line |

---

> **For contributors**: Clone the repo, follow [Section 15](#15-how-to-build--run) to get running, check [Section 17](#17-current-status--whats-broken--stuck-) for what needs fixing, and [Section 18](#18-what-needs-to-be-built-next-) for the roadmap. Each file's purpose is documented in [Section 6](#6-repository-structure-file-by-file).
