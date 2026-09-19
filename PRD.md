# 🏭 Project Mustang — Product Requirements Document (PRD)

> **Decentralized Autonomous Mobile Robot (AMR) Swarm for Smart Warehouse Order Fulfillment**
>
> Last updated: 2026-09-19 | Status: **Active Development (Alpha)**

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Problem Statement](#2-problem-statement)
3. [Tech Stack & Dependencies](#3-tech-stack--dependencies)
4. [Repository Structure](#4-repository-structure)
5. [System Architecture](#5-system-architecture)
6. [Warehouse Layout & Physical Design](#6-warehouse-layout--physical-design)
7. [The Robot (AMR)](#7-the-robot-amr)
8. [Navigation System — Topological Graph A*](#8-navigation-system--topological-graph-a)
9. [Multi-Robot Coordination (Decentralized)](#9-multi-robot-coordination-decentralized)
10. [Task Lifecycle & State Machine](#10-task-lifecycle--state-machine)
11. [Edge AI — YOLO Vision](#11-edge-ai--yolo-vision)
12. [Dashboard & Telemetry](#12-dashboard--telemetry)
13. [How to Build & Run](#13-how-to-build--run)
14. [Current Status — What Works ✅](#14-current-status--what-works-)
15. [Current Status — What's Broken / Stuck 🔴](#15-current-status--whats-broken--stuck-)
16. [What Needs to Be Built Next 🔨](#16-what-needs-to-be-built-next-)
17. [Key Design Decisions & Rationale](#17-key-design-decisions--rationale)
18. [KPIs & Success Metrics](#18-kpis--success-metrics)
19. [Glossary](#19-glossary)

---

## 1. Project Overview

**Mustang** is a fully decentralized warehouse robot swarm simulation. Instead of a central server telling robots what to do, each AMR (Autonomous Mobile Robot) is an independent agent that:

- **Self-assigns tasks** from a shared task pool (no dispatcher)
- **Plans its own routes** using A* on a topological rail graph
- **Negotiates traffic conflicts** peer-to-peer with other AMRs
- **Hands off blocked tasks** via Contract Net Protocol (CNP) auctions
- **Detects humans and obstacles** using onboard YOLOv8 computer vision

The entire fleet runs **without any central controller, master node, or cloud server**. All communication happens via Zenoh peer-to-peer pub/sub middleware.

### The "SIH" Context

This project was built for the **Smart India Hackathon (SIH)** — a national-level hackathon in India. The problem statement requires designing an autonomous warehouse fulfillment system using multi-robot coordination.

---

## 2. Problem Statement

Modern warehouses need fleets of robots to pick and deliver items. The challenge:

| Challenge | Our Solution |
|-----------|-------------|
| Single point of failure (central server dies → all robots stop) | **Fully decentralized** — no master node. Each AMR is autonomous. |
| Traffic jams at intersections | **Priority Aging algorithm** — robots negotiate who yields based on wait time + battery. Prevents starvation. |
| Robot blocked by fallen box/human | **CNP Auction** — blocked robot broadcasts a "help wanted", peers bid, best peer takes over the task. |
| Expensive cloud compute for vision | **Edge AI** — YOLOv8n runs onboard each robot (ONNX Runtime, no GPU cloud needed). |
| Unsafe around humans | **LiDAR safety braking** (<0.6m) + YOLO person detection → instant motor kill. |

---

## 3. Tech Stack & Dependencies

| Layer | Technology | Version |
|-------|-----------|---------|
| **Robot OS** | ROS 2 | Jazzy Jalisco |
| **Simulator** | Gazebo | Harmonic (gz-sim) |
| **Middleware** | Zenoh (rmw_zenoh_cpp) | Latest via ROS 2 |
| **Language** | Python | 3.12+ |
| **Vision AI** | YOLOv8n | Ultralytics ONNX |
| **Dashboard** | Flask + SSE | Python |
| **Build System** | colcon | ROS 2 standard |
| **OS** | Ubuntu | 24.04 LTS |

### Key Python Dependencies
```
rclpy           — ROS 2 Python client library
launch          — ROS 2 launch system
onnxruntime     — ONNX model inference (YOLOv8)
flask           — Dashboard web server
numpy           — Math/array operations
cv2 (opencv)    — Image processing
sensor_msgs     — ROS image/lidar message types
geometry_msgs   — Twist/Pose message types
nav_msgs        — Odometry message types
std_msgs        — String message types
```

### Zenoh Configuration
The middleware is configured for **purely decentralized P2P**:
- No router/broker needed
- UDP multicast discovery (`224.0.0.224:7446`)
- Gossip scouting for peer discovery
- Config file: `src/zenoh_config.json5`

To use Zenoh as the ROS 2 middleware:
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

---

## 4. Repository Structure

```
mustang/
├── package.xml                    # ROS 2 package manifest
├── setup.py                       # Python package installer (colcon)
├── setup.cfg                      # Entry points config
├── .gitignore
├── PRD.md                         # ← You are here
├── X.pdf / Y.pdf / Z.pdf         # Reference PDFs (SIH problem statements)
│
├── src/
│   ├── sih_warehouse.sdf          # 🏭 Gazebo world file (warehouse layout)
│   ├── amr.urdf                   # 🤖 Robot description (chassis, sensors, wheels)
│   ├── multi_spawn.launch.py      # 🚀 ROS 2 launch file (spawns 3 AMRs)
│   ├── run_swarm.sh               # 🏃 One-click launcher script
│   │
│   ├── task_manager_node.py       # 🧠 MAIN ROBOT BRAIN (1153 lines)
│   ├── cnp_auction_node.py        # 🤝 Contract Net Protocol auctions (490 lines)
│   ├── priority_negotiator_node.py# 🚦 Traffic deadlock resolver (476 lines)
│   ├── yolo_vision_node.py        # 👁️ Edge AI vision node (476 lines)
│   │
│   ├── generate_visual_markers.py # 🟢 Generates floor debug markers for Gazebo
│   ├── visual_rails.sdf           # 🟢 Generated SDF snippet (visual markers)
│   ├── zenoh_config.json5         # ⚙️ Zenoh P2P middleware config
│   │
│   ├── test_traffic_jam.sh        # 🧪 Stress test: 3 intersecting tasks
│   ├── benchmark_test.py          # 📊 KPI benchmark: dispatches 5 sequential tasks
│   │
│   └── dashboard/
│       ├── app.py                 # 📊 Flask dashboard server (port 5050)
│       ├── templates/
│       │   └── index.html         # 📊 Dashboard UI (SSE-powered, no CDN)
│       └── static/                # (empty — all CSS/JS is inline in index.html)
│
├── build/                         # colcon build output (gitignored)
├── install/                       # colcon install output (gitignored)
└── log/                           # colcon log output (gitignored)
```

---

## 5. System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                    GAZEBO HARMONIC SIMULATOR                  │
│  ┌──────────────────────────────────────────────────────┐    │
│  │              sih_warehouse.sdf                       │    │
│  │  8 Racks · 80 Semantic Bays · Dual Parallel Rails    │    │
│  │  Floor markers · Obstacle zones · Corridors          │    │
│  └──────────────────────────────────────────────────────┘    │
│         ▲ Physics/Sensors          ▼ cmd_vel                 │
│  ┌──────┴──────┐ ┌──────┴──────┐ ┌──────┴──────┐           │
│  │   AMR #1    │ │   AMR #2    │ │   AMR #3    │           │
│  │  amr.urdf   │ │  amr.urdf   │ │  amr.urdf   │           │
│  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘           │
└─────────┼───────────────┼───────────────┼────────────────────┘
          │               │               │
    ┌─────▼─────┐   ┌─────▼─────┐   ┌─────▼─────┐
    │ros_gz_    │   │ros_gz_    │   │ros_gz_    │
    │bridge     │   │bridge     │   │bridge     │
    │/amr_1/*   │   │/amr_2/*   │   │/amr_3/*   │
    └─────┬─────┘   └─────┬─────┘   └─────┬─────┘
          │               │               │
          ▼               ▼               ▼
    ╔═══════════════════════════════════════════╗
    ║        ZENOH P2P MESH (No Broker)         ║
    ║  Topics:                                  ║
    ║  · swarm/tasks/pool     (task dispatch)   ║
    ║  · swarm/tasks/claim    (task claiming)   ║
    ║  · swarm/telemetry/{id} (position/status) ║
    ║  · swarm/cnp/*          (auction msgs)    ║
    ║  · swarm/priority/*     (negotiation)     ║
    ╚═══════════════════════════════════════════╝
          │               │               │
    ┌─────▼─────┐   ┌─────▼─────┐   ┌─────▼─────┐
    │ PER-AMR   │   │ PER-AMR   │   │ PER-AMR   │
    │ NODE STACK│   │ NODE STACK│   │ NODE STACK│
    │           │   │           │   │           │
    │• TaskMgr  │   │• TaskMgr  │   │• TaskMgr  │
    │• CNP      │   │• CNP      │   │• CNP      │
    │• Priority │   │• Priority │   │• Priority │
    │• YOLO     │   │• YOLO     │   │• YOLO     │
    └───────────┘   └───────────┘   └───────────┘
          │               │               │
          └───────────┬───┘               │
                      ▼                   │
              ┌───────────────┐           │
              │  Flask        │◄──────────┘
              │  Dashboard    │  (SSE stream)
              │  :5050        │
              └───────────────┘
```

### Per-AMR Node Stack (each robot runs ALL of these independently)

| Node | File | Role |
|------|------|------|
| `task_manager_node` | `task_manager_node.py` | Main brain: navigation, task claiming, path planning, motor control, KPI logging |
| `cnp_auction_node` | `cnp_auction_node.py` | Handles task handoff when permanently blocked |
| `priority_negotiator_node` | `priority_negotiator_node.py` | Resolves intersection deadlocks via priority aging |
| `yolo_vision_node` | `yolo_vision_node.py` | Onboard camera AI: person detection → safety stop, obstacle → auction trigger |

---

## 6. Warehouse Layout & Physical Design

### Dimensions
- **Total footprint**: ~16m × 15m
- **Aisles**: 2.5m wide (expanded from original 1.0m to support dual rails)
- **Corridors**: Top and bottom cross-corridors for AMR transit

### Rack Configuration
```
    ┌─────────────────────────────────────┐
    │           TOP CORRIDOR               │
    │  ┌────┐  ┌────┐  ┌────┐  ┌────┐    │ ← STAGING
    │  │R1A │  │R2A │  │R3A │  │R4A │    │   AREA
    │  │Yel │  │Yel │  │Yel │  │Yel │    │   (AMRs
    │  └────┘  └────┘  └────┘  └────┘    │    start
    │  ║ ║     ║ ║     ║ ║     ║ ║       │    here)
    │  ║ ║     ║ ║     ║ ║     ║ ║       │
    │  ║ ║ DUAL RAILS (2 tracks/aisle)    │
    │  ║ ║     ║ ║     ║ ║     ║ ║       │
    │  ┌────┐  ┌────┐  ┌────┐  ┌────┐    │
    │  │R1B │  │R2B │  │R3B │  │R4B │    │
    │  │Blue│  │Blue│  │Blue│  │Blue│    │
    │  └────┘  └────┘  └────┘  └────┘    │
    │           BOTTOM CORRIDOR            │
    └─────────────────────────────────────┘
```

- **8 Racks total**: 4 Yellow (Row A, top) + 4 Blue (Row B, bottom)
- Each rack is **2m × 4m**
- Each rack has **10 semantic parking bays** around its perimeter (80 bays total)
- **Dual Parallel Rails**: 2 tracks per aisle spaced 1.0m apart
  - Rail A (right track): e.g., X = 3.25m
  - Rail B (left track): e.g., X = 4.25m
- **Clearance**: 35cm between robot and shelf edge (no rubbing)
- **Rail switching**: AMRs switch tracks at top/bottom cross-corridors

### Why Dual Rails?
On a single track, overtaking is impossible. With two parallel tracks:
- One robot can pass another that's stopped at a shelf
- Bidirectional traffic flow within the same aisle
- Reduces deadlocks significantly

---

## 7. The Robot (AMR)

**File**: `src/amr.urdf`

### Physical Specs
| Property | Value |
|----------|-------|
| Chassis | 0.5m × 0.4m × 0.15m box |
| Mass | 5.0 kg |
| Drive type | Differential drive (2 wheels) |
| Wheel radius | 0.1m |
| Wheel separation | 0.4m |
| Max linear speed | 0.5 m/s |
| Max angular speed | 1.0 rad/s |

### Sensors
| Sensor | Type | Details |
|--------|------|---------|
| **LiDAR** | GPU Ray (360°) | 640 samples, 12m range, mounted on top |
| **Camera** | RGB Camera | 640×480, 1.05 rad FOV, front-facing, for YOLO |

### Wheels
| Wheel | Purpose | Notes |
|-------|---------|-------|
| Left drive | Locomotion | μ₁ = 1.0 (high grip) |
| Right drive | Locomotion | μ₁ = 1.0 (high grip) |
| Rear caster | Balance | μ₁ = 0.0 (frictionless) |
| Front caster | Anti-pitch | μ₁ = 0.0 (frictionless, prevents camera tilt) |

### ROS 2 Topics (per AMR, namespaced)
```
/amr_N/cmd_vel       — Twist commands (motor control)
/amr_N/odom          — Odometry (position, orientation)
/amr_N/scan          — LaserScan (360° LiDAR)
/amr_N/camera/image  — RGB camera feed
```

---

## 8. Navigation System — Topological Graph A*

**File**: `task_manager_node.py` → class `WarehouseGraph` and `AStarPlanner`

### How It Works

Instead of a free-space 2D grid (like traditional SLAM), we use a **topological graph of virtual rails**:

```
Nodes = Named locations (intersections, bay stops, staging points)
Edges = Rail segments connecting nodes (Manhattan-only: horizontal or vertical)
```

### Graph Nodes (Examples)
```python
"staging_1":  (13.0, 13.0)    # AMR start position
"staging_2":  (13.0, 12.0)
"staging_3":  (13.0, 11.0)
"tc_r1_top":  (3.25, 13.5)    # Top corridor junction for rack 1
"r1a_bay_3":  (3.25, 10.5)    # Bay 3 of rack 1A
"bc_r1_bot":  (3.25, 1.0)     # Bottom corridor junction for rack 1
```

### A* Path Planning
1. Robot receives a task: "Go to `r2a_bay_5`"
2. A* finds shortest path on the topological graph
3. Path is **densified** into 10cm waypoint steps
4. Segments are **collapsed** into Manhattan segments (strictly horizontal or vertical)

### Manhattan Segment FSM (Finite State Machine)
Each segment is executed as:
```
ROTATE phase → Turn in place to face the target direction (±0.15 rad threshold)
DRIVE phase  → Drive straight along the rail (no turning while driving)
```

This prevents diagonal drift — robots move like trains on rails.

### Dynamic Lane Changing
- Each AMR broadcasts its current trajectory on `swarm/telemetry/{id}`
- If a head-on conflict is detected with a higher-priority peer, the AMR dynamically replans
- The blocked edge gets a **+999.0 cost penalty** in A*, forcing reroute to the parallel rail

---

## 9. Multi-Robot Coordination (Decentralized)

### 9.1 Task Claiming (No Central Dispatcher)

**How tasks enter the system:**
```bash
# Publish a task to the shared pool via Zenoh/ROS 2
ros2 topic pub --once /swarm/tasks/pool std_msgs/String \
  '{"data": "{\"task_id\": \"order_42\", \"target_bay\": \"r2a_bay_5\", \"status\": \"IDLE\"}"}'
```

**How robots claim tasks:**
1. All AMRs subscribe to `swarm/tasks/pool`
2. When a new IDLE task appears, each AMR calculates its **Euclidean distance** to the target bay
3. The closest AMR claims it by publishing to `swarm/tasks/claim`
4. **Tiebreaker**: If two AMRs are equidistant, a **SHA-256 hash** of `(task_id + amr_id)` breaks the tie deterministically (no randomness, no race condition)

### 9.2 Priority Negotiation (Intersection Deadlocks)

**File**: `priority_negotiator_node.py`

When two AMRs approach the same intersection:

```
Priority Score = (wait_duration_sec × 10.0)
               + (100.0 - battery_pct)
               + stable_random_tiebreaker(0.01...0.20)
```

- **Wait time dominates**: The longer an AMR has been waiting, the higher its priority
- **Battery bonus**: Low-battery robots get slight priority (to finish and recharge)
- **Tiebreaker**: Prevents oscillation between equally-scored robots
- **Anti-starvation guarantee**: Wait time monotonically increases → every robot eventually wins

The **loser yields** (halts motors) and waits for the winner to clear the intersection.

### 9.3 CNP Auctions (Task Handoff When Blocked)

**File**: `cnp_auction_node.py`

When an AMR is permanently blocked (e.g., fallen box detected by YOLO):

```
1. BLOCKED AMR broadcasts CFP (Call for Proposals) on swarm/cnp/cfp
2. All available peers evaluate:
   - Battery > 25%? (minimum threshold)
   - Current status = IDLE or light workload?
3. Eligible peers reply with a BID:
   bid_score = euclidean_distance_to_target + battery_penalty
4. Blocked AMR waits 500ms for all bids
5. Selects LOWEST bid_score → AWARD sent to winner
6. Winner takes over the task; blocked AMR returns to IDLE
```

---

## 10. Task Lifecycle & State Machine

```
                    ┌──────────┐
                    │   IDLE   │ ← Robot is free, scanning task pool
                    └────┬─────┘
                         │ (new task found, closest AMR wins)
                    ┌────▼─────┐
                    │ CLAIMED  │ ← Task locked to this AMR
                    └────┬─────┘
                         │ (path planned via A*)
              ┌──────────▼──────────┐
              │  EN_ROUTE_PICKUP    │ ← Driving to pickup bay
              └──────────┬──────────┘
                         │ (arrived at bay, simulated pickup)
              ┌──────────▼──────────┐
              │    DELIVERING       │ ← Carrying item to destination
              └──────┬────────┬─────┘
                     │        │
            (success)│        │(permanently blocked)
          ┌──────────▼┐  ┌────▼──────────────────┐
          │ COMPLETED │  │ BLOCKED_AUCTION_REQ    │
          └───────────┘  └────┬──────────────────┘
                              │ (CNP auction triggered)
                         ┌────▼─────┐
                         │ HANDOFF  │ ← Task given to winning bidder
                         └──────────┘
```

### KPI Metrics Tracked (Per Task)
- `task_id`: Unique identifier
- `claimed_at`: Timestamp when claimed
- `completed_at`: Timestamp when delivered
- `total_duration`: End-to-end time
- `distance_traveled`: Odometry-based
- `active_travel_time`: Time wheels were actually moving
- `idle_wait_time`: Time spent yielding at intersections

---

## 11. Edge AI — YOLO Vision

**File**: `yolo_vision_node.py`

### Architecture
```
Camera Feed (640×480) → Resize to 640×640 → YOLOv8n ONNX → Detections
```

### Detection → Action Mapping
| YOLO Detection | Confidence | Action Triggered |
|----------------|-----------|------------------|
| `person` (class 0) | > 0.5 | **SAFETY_STOP** — Immediately kills all motors |
| `backpack` / `suitcase` / `couch` (proxy) | > 0.5 | **BLOCKED_AUCTION_REQUIRED** — Triggers CNP |
| Other objects | any | Logged but ignored |

### Performance
- Throttled to **5 FPS** (every 200ms) to avoid CPU overload
- Uses **ONNX Runtime** (CPU inference, no GPU required)
- Model: `yolov8n.onnx` (~6MB)

### ⚠️ Known Limitation
YOLOv8n is trained on **COCO dataset** which has no "pallet" or "warehouse box" class. We use proxy classes (backpack, suitcase, couch) as stand-ins. **Production deployment needs fine-tuning on a warehouse-specific dataset.**

---

## 12. Dashboard & Telemetry

**Files**: `src/dashboard/app.py`, `src/dashboard/templates/index.html`

### Features
- **Real-time AMR positions** on a 2D warehouse map
- **Task status tracker** (IDLE/CLAIMED/EN_ROUTE/DELIVERING/COMPLETED)
- **CNP auction log** (who bid, who won, bid scores)
- **KPI metrics display** (duration, distance, idle time)
- **Fleet battery levels** (simulated)

### Tech
- Flask server on **port 5050**
- **Server-Sent Events (SSE)** for real-time streaming (no WebSocket complexity)
- **Zero CDN dependencies** — all CSS/JS is inline (works offline)
- Subscribes to Zenoh topics and forwards to browser

### How to Run
```bash
cd src/dashboard && python3 app.py
# Open http://localhost:5050
```

---

## 13. How to Build & Run

### Prerequisites
```bash
# Ubuntu 24.04 with ROS 2 Jazzy + Gazebo Harmonic installed
sudo apt install ros-jazzy-desktop gazebo-harmonic
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim
pip install flask onnxruntime numpy opencv-python ultralytics
```

### Build
```bash
cd /home/siva/mustang
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Run Everything (One Command)
```bash
cd /home/siva/mustang
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./src/run_swarm.sh
```

This launches:
1. Gazebo with the warehouse world (`sih_warehouse.sdf`)
2. 3 AMR models spawned at staging positions
3. ROS-Gazebo bridges for all sensor/actuator topics

### Then, in separate terminals:

**Terminal 2, 3, 4 — AMR Task Managers:**
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# AMR 1
python3 src/task_manager_node.py --ros-args -r __ns:=/amr_1 \
  -r odom:=/amr_1/odom -r cmd_vel:=/amr_1/cmd_vel -r scan:=/amr_1/scan

# AMR 2
python3 src/task_manager_node.py --ros-args -r __ns:=/amr_2 \
  -r odom:=/amr_2/odom -r cmd_vel:=/amr_2/cmd_vel -r scan:=/amr_2/scan

# AMR 3
python3 src/task_manager_node.py --ros-args -r __ns:=/amr_3 \
  -r odom:=/amr_3/odom -r cmd_vel:=/amr_3/cmd_vel -r scan:=/amr_3/scan
```

**Terminal 5 — Inject a Test Task:**
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 topic pub --once /swarm/tasks/pool std_msgs/String \
  '{"data": "{\"task_id\": \"order_1\", \"target_bay\": \"r2a_bay_5\", \"status\": \"IDLE\"}"}'
```

**Terminal 6 — Dashboard:**
```bash
cd src/dashboard && python3 app.py
```

**Terminal 7 — Stress Test (Traffic Jam):**
```bash
bash src/test_traffic_jam.sh
```

---

## 14. Current Status — What Works ✅

| Feature | Status | Notes |
|---------|--------|-------|
| Warehouse SDF world (8 racks, corridors, obstacles) | ✅ Working | Renders in Gazebo Harmonic |
| AMR URDF (diff drive, LiDAR, camera, 4 wheels) | ✅ Working | Stable chassis, no pitch issues |
| 3-AMR spawning with namespaced topics | ✅ Working | `/amr_1/*`, `/amr_2/*`, `/amr_3/*` |
| ROS-Gazebo bridge (cmd_vel, odom, scan, camera) | ✅ Working | All topics bridged |
| Topological Graph A* path planning | ✅ Working | 80 bay nodes + corridor junctions |
| Manhattan Segment FSM (turn-then-drive) | ✅ Working | No diagonal drift |
| Path densification (10cm steps) | ✅ Working | Smooth rail tracking |
| Task pool claiming (distance + SHA-256 tiebreak) | ✅ Working | Deterministic, no race conditions |
| Priority Aging intersection negotiation | ✅ Working | Anti-starvation guaranteed |
| CNP auction task handoff | ✅ Working | 500ms bidding window |
| LiDAR safety braking (<0.6m) | ✅ Working | Immediate motor kill |
| YOLOv8n person/obstacle detection | ✅ Working | 5 FPS throttled, ONNX |
| Dynamic A* lane changing (+999 penalty) | ✅ Working | Head-on conflict avoidance |
| Zenoh P2P decentralized comms | ✅ Working | No broker, UDP multicast |
| Visual floor markers (green dots, bay dividers) | ✅ Working | Debug visualization in Gazebo |
| Flask dashboard (SSE, offline-capable) | ✅ Working | Port 5050 |
| KPI logging (duration, distance, idle time) | ✅ Working | Per-task metrics |
| Git version control | ✅ Working | 30+ meaningful commits |

---

## 15. Current Status — What's Broken / Stuck 🔴

### 🔴 P1 — Critical (Must Fix)

| Issue | Details | File(s) |
|-------|---------|---------|
| **Visual rail markers sometimes cause duplicate SDF model names** | When `generate_visual_markers.py` output is spliced into `sih_warehouse.sdf`, duplicate `<model name="vis_div_0">` entries can crash Gazebo's XML parser. Need to ensure unique names or remove old markers before re-injecting. | `generate_visual_markers.py`, `sih_warehouse.sdf` |
| **AMRs sometimes don't move after task injection** | Task is claimed (visible in logs) but the robot doesn't execute the path. Suspected cause: the `/amr_N/odom` topic hasn't published yet when the task manager starts, so initial position is `(0,0)` and the A* plan goes haywire. Need a "wait for first odom" guard. | `task_manager_node.py` |

### 🟡 P2 — Important (Should Fix)

| Issue | Details | File(s) |
|-------|---------|---------|
| **YOLO uses COCO proxy classes** | No real "pallet" or "warehouse box" detection. `backpack`/`suitcase`/`couch` are proxies. Need to either fine-tune on warehouse dataset or add custom ONNX model. | `yolo_vision_node.py` |
| **Frictionless front caster is a hack** | Added to prevent camera pitch during acceleration. Proper fix would be a better URDF inertia tensor or a suspension model. | `amr.urdf` |
| **Dashboard doesn't auto-discover AMRs** | Currently hardcoded for 3 AMRs. Should dynamically discover fleet size from Zenoh topics. | `dashboard/app.py` |
| **No return-to-staging logic** | After completing a task, AMRs stay at the delivery location instead of returning to the staging area. | `task_manager_node.py` |

### 🟠 P3 — Nice to Have

| Issue | Details |
|-------|---------|
| **No battery simulation** | Battery percentage is simulated as a static value. Should drain based on distance traveled and recharge when at staging. |
| **No multi-floor support** | Current layout is single-floor. SIH problem may require elevator/multi-level. |
| **No RViz visualization** | Could add RViz2 markers for path visualization alongside Gazebo. |

---

## 16. What Needs to Be Built Next 🔨

### Phase 1 — Stabilization (Immediate)
- [ ] **Fix SDF duplicate model name bug** in `generate_visual_markers.py` — ensure all generated model names are globally unique
- [ ] **Add "wait for first odom" guard** in `task_manager_node.py` before accepting tasks
- [ ] **Auto-launch all nodes** — modify `run_swarm.sh` to also launch task managers, CNP, priority, and YOLO nodes per AMR (currently manual)
- [ ] **Fix Gazebo launch reliability** — sometimes `gz sim` fails silently; add health checks

### Phase 2 — Feature Completion
- [ ] **Return-to-staging behavior** — AMRs should navigate back to staging after completing delivery
- [ ] **Battery drain simulation** — decrease battery based on distance; low battery → return to charging station
- [ ] **Dynamic fleet scaling** — support 5+ AMRs without code changes
- [ ] **Fine-tune YOLOv8 on warehouse dataset** — real pallet/box/forklift detection
- [ ] **Dashboard improvements**:
  - Live 2D map with AMR positions plotted
  - Historical task timeline
  - Fleet utilization percentage
  - Export KPI reports as CSV

### Phase 3 — Demo Polish
- [ ] **RViz2 path visualization** — show planned paths as colored lines
- [ ] **Voice/alert system** — dashboard plays alert sound on SAFETY_STOP
- [ ] **Multi-order batching** — AMR picks up multiple items in one trip
- [ ] **Charging station model** — add physical charging pads in Gazebo world
- [ ] **Video recording** — auto-record Gazebo simulation for demo purposes

---

## 17. Key Design Decisions & Rationale

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

---

## 18. KPIs & Success Metrics

| KPI | Target | How Measured |
|-----|--------|-------------|
| Task completion rate | > 95% | `COMPLETED` count / total tasks dispatched |
| Average pick-to-deliver time | < 60 seconds | `completed_at - claimed_at` per task |
| Deadlock resolution time | < 5 seconds | Priority negotiation convergence time |
| Safety stop latency | < 200ms | Time from person detection to motor kill |
| Fleet utilization | > 70% | `active_travel_time / total_time` per AMR |
| CNP auction success rate | > 90% | Successful handoffs / total auction attempts |
| Collision count | 0 | LiDAR-triggered emergency stops |

---

## 19. Glossary

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
| **Staging Area** | The starting/parking area for AMRs (top-right of warehouse) |
| **Topological Graph** | A graph of named nodes and edges representing navigable paths |
| **URDF** | Unified Robot Description Format — describes robot geometry and physics |
| **Zenoh** | A pub/sub/query middleware for edge computing (used as ROS 2 RMW layer) |

---

> **For contributors**: Clone the repo, follow [Section 13](#13-how-to-build--run) to get running, check [Section 15](#15-current-status--whats-broken--stuck-) for what needs fixing, and [Section 16](#16-what-needs-to-be-built-next-) for the roadmap. Each file's purpose is documented in [Section 4](#4-repository-structure).
