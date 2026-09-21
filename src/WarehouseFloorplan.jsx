import React, { useState, useMemo } from "react";

// ═══════════════════════════════════════════════════════════════════════
//  SwarmMonitorMap.jsx
//  SIH26123 — Interactive Expandable Warehouse Floorplan
//
//  Brutalist dark theme  •  Tailwind CSS  •  State-driven AMR positions
//  Toggle [+] Expand / [-] Collapse for fullscreen overlay
// ═══════════════════════════════════════════════════════════════════════

const G = 15; // 15×15 grid cells (1 cell = 1 meter)

// ── Muted Brutalist Palette ────────────────────────────────────────────
const PAL = {
  red:    { bg: "#7A3030", border: "#9A4444" },
  yellow: { bg: "#8B7D2A", border: "#A89530" },
  blue:   { bg: "#3A5A7A", border: "#4A7090" },
  cyan:   "#40C8C8",
  robot:  "#4A90D9",
  robotBorder: "#6AB0FF",
};

// ── Static Obstacles ───────────────────────────────────────────────────
//    { id, row, col, rs (rowSpan), cs (colSpan), type, label }
//    Grid is 1-indexed (matching CSS grid-row / grid-column).
const RACKS = [
  // ▌ Red trap zone — top-left
  { id: "trap",  row: 1,  col: 1, rs: 3, cs: 6, type: "red",    label: "TRAP ZONE" },

  // ▌ Yellow racks — left aisle
  { id: "y1",    row: 4,  col: 2, rs: 2, cs: 5, type: "yellow", label: "Y1" },
  { id: "y2",    row: 7,  col: 2, rs: 2, cs: 5, type: "yellow", label: "Y2" },
  { id: "y3",    row: 10, col: 2, rs: 2, cs: 5, type: "yellow", label: "Y3" },
  { id: "y4",    row: 13, col: 2, rs: 2, cs: 5, type: "yellow", label: "Y4" },

  // ▌ Blue racks — right aisle
  { id: "b1",    row: 4,  col: 9, rs: 2, cs: 5, type: "blue",   label: "B1" },
  { id: "b2",    row: 7,  col: 9, rs: 2, cs: 5, type: "blue",   label: "B2" },
  { id: "b3",    row: 10, col: 9, rs: 2, cs: 5, type: "blue",   label: "B3" },
  { id: "b4",    row: 13, col: 9, rs: 2, cs: 5, type: "blue",   label: "B4" },
];

// ── Initial Robot Positions (top-right open area) ──────────────────────
const INITIAL_ROBOTS = [
  { id: "AMR-1", row: 2, col: 9  },
  { id: "AMR-2", row: 2, col: 11 },
  { id: "AMR-3", row: 2, col: 13 },
];

// ── Waypoint Generator ─────────────────────────────────────────────────
//    Places cyan navigation dots along aisles and corridors,
//    skipping any cell occupied by a rack obstacle.
function buildWaypoints() {
  // Build occupancy lookup
  const blocked = new Set();
  RACKS.forEach((r) => {
    for (let row = r.row; row < r.row + r.rs; row++)
      for (let col = r.col; col < r.col + r.cs; col++)
        blocked.add(`${row},${col}`);
  });

  const pts = new Set();
  const add = (r, c) => {
    if (r >= 1 && r <= G && c >= 1 && c <= G && !blocked.has(`${r},${c}`))
      pts.add(`${r},${c}`);
  };

  // Vertical aisles (columns that run between/around rack blocks)
  const aisleColumns = [1, 7, 8, 14, 15];
  aisleColumns.forEach((c) => {
    for (let r = 1; r <= G; r++) add(r, c);
  });

  // Horizontal corridors (rows between rack tiers)
  const corridorRows = [1, 3, 6, 9, 12, 15];
  corridorRows.forEach((r) => {
    for (let c = 1; c <= G; c++) add(r, c);
  });

  // Bay access columns (edges of rack blocks)
  const bayColumns = [2, 6, 9, 13];
  bayColumns.forEach((c) => {
    for (let r = 1; r <= G; r++) add(r, c);
  });

  return Array.from(pts).map((k) => {
    const [row, col] = k.split(",").map(Number);
    return { row, col };
  });
}

// ═══════════════════════════════════════════════════════════════════════
//  Main Component
// ═══════════════════════════════════════════════════════════════════════
export default function SwarmMonitorMap() {
  const [isExpanded, setIsExpanded] = useState(false);
  const [robots] = useState(INITIAL_ROBOTS);
  const waypoints = useMemo(() => buildWaypoints(), []);

  // CSS grid template (shared by all layers)
  const gridCSS = {
    display: "grid",
    gridTemplateColumns: `repeat(${G}, 1fr)`,
    gridTemplateRows: `repeat(${G}, 1fr)`,
  };

  // ── Container classes: normal panel vs fullscreen overlay ──
  const containerCls = isExpanded
    ? "fixed inset-0 z-50 bg-black p-4 sm:p-6 md:p-8"
    : "border border-white/20 bg-black p-3";

  // ── Dot / indicator sizes scale up when expanded ──
  const dotSize = isExpanded ? 6 : 3;
  const botSize = isExpanded ? 18 : 11;
  const labelSize = isExpanded ? 9 : 6;
  const rackLabelSize = isExpanded ? 12 : 8;

  return (
    <div className={`${containerCls} font-mono select-none flex flex-col`}>
      {/* ── Header ── */}
      <div className="flex items-center justify-between mb-2 border-b border-white/10 pb-1.5 shrink-0">
        <h2 className="text-[10px] md:text-xs font-bold uppercase tracking-[0.15em] text-white/60">
          15×15m Warehouse Floorplan Map
        </h2>
        <button
          onClick={() => setIsExpanded((v) => !v)}
          className="text-[10px] font-bold uppercase tracking-wider text-cyan-400
                     border border-cyan-400/40 px-2 py-0.5 hover:bg-cyan-400/10
                     active:bg-cyan-400/20 cursor-pointer transition-colors"
        >
          {isExpanded ? "[-] Collapse" : "[+] Expand"}
        </button>
      </div>

      {/* ── Map viewport ── */}
      <div className="flex-1 min-h-0 flex items-center justify-center">
        <div className="aspect-square h-full max-w-full relative border-[2.5px] border-white/30 bg-[#030303] overflow-hidden">

          {/* ░░░ Layer 0 — Faint gridlines ░░░ */}
          <div className="absolute inset-0" style={gridCSS}>
            {Array.from({ length: G * G }, (_, i) => (
              <div key={i} className="border border-white/[0.04]" />
            ))}
          </div>

          {/* ░░░ Layer 1 — Center aisle subtle highlight ░░░ */}
          <div className="absolute inset-0" style={gridCSS}>
            {Array.from({ length: G }, (_, r) => (
              <React.Fragment key={`aisle-${r}`}>
                <div
                  className="bg-white/[0.018]"
                  style={{ gridColumn: "7 / span 1", gridRow: `${r + 1} / span 1` }}
                />
                <div
                  className="bg-white/[0.018]"
                  style={{ gridColumn: "8 / span 1", gridRow: `${r + 1} / span 1` }}
                />
              </React.Fragment>
            ))}
          </div>

          {/* ░░░ Layer 2 — Rack / obstacle blocks ░░░ */}
          <div className="absolute inset-0" style={gridCSS}>
            {RACKS.map((rack) => {
              const p = PAL[rack.type];
              return (
                <div
                  key={rack.id}
                  className="flex items-center justify-center"
                  style={{
                    gridColumn: `${rack.col} / span ${rack.cs}`,
                    gridRow: `${rack.row} / span ${rack.rs}`,
                    backgroundColor: p.bg,
                    border: `1.5px solid ${p.border}`,
                  }}
                >
                  <span
                    className="font-bold uppercase tracking-[0.2em] text-white/25"
                    style={{ fontSize: `${rackLabelSize}px` }}
                  >
                    {rack.label}
                  </span>
                </div>
              );
            })}
          </div>

          {/* ░░░ Layer 3 — Waypoint dots (navigation graph) ░░░ */}
          <div className="absolute inset-0" style={gridCSS}>
            {waypoints.map((wp) => (
              <div
                key={`wp-${wp.row}-${wp.col}`}
                className="flex items-center justify-center"
                style={{
                  gridColumn: `${wp.col} / span 1`,
                  gridRow: `${wp.row} / span 1`,
                }}
              >
                <div
                  className="rounded-full"
                  style={{
                    width: `${dotSize}px`,
                    height: `${dotSize}px`,
                    backgroundColor: PAL.cyan,
                    opacity: 0.45,
                  }}
                />
              </div>
            ))}
          </div>

          {/* ░░░ Layer 4 — Robot indicators ░░░ */}
          <div className="absolute inset-0" style={gridCSS}>
            {robots.map((bot) => (
              <div
                key={bot.id}
                className="flex flex-col items-center justify-center"
                style={{
                  gridColumn: `${bot.col} / span 1`,
                  gridRow: `${bot.row} / span 1`,
                  gap: "2px",
                }}
              >
                <div
                  style={{
                    width: `${botSize}px`,
                    height: `${botSize}px`,
                    backgroundColor: PAL.robot,
                    border: `2px solid ${PAL.robotBorder}`,
                  }}
                />
                <span
                  className="font-bold uppercase text-white/40 leading-none"
                  style={{ fontSize: `${labelSize}px`, letterSpacing: "0.05em" }}
                >
                  {bot.id}
                </span>
              </div>
            ))}
          </div>

          {/* ░░░ Layer 5 — Axis labels ░░░ */}
          <div className="absolute inset-0 pointer-events-none" style={gridCSS}>
            {[0, 5, 10].map((m) => (
              <React.Fragment key={`axis-${m}`}>
                {/* X-axis (bottom edge) */}
                <div
                  className="flex items-end justify-center pb-px"
                  style={{
                    gridColumn: `${m + 1} / span 1`,
                    gridRow: `${G} / span 1`,
                  }}
                >
                  <span className="text-white/15 font-mono" style={{ fontSize: "7px" }}>
                    {m}m
                  </span>
                </div>
                {/* Y-axis (left edge) */}
                <div
                  className="flex items-center justify-start pl-px"
                  style={{
                    gridColumn: "1 / span 1",
                    gridRow: `${G - m} / span 1`,
                  }}
                >
                  <span className="text-white/15 font-mono" style={{ fontSize: "7px" }}>
                    {m}
                  </span>
                </div>
              </React.Fragment>
            ))}
          </div>
        </div>
      </div>

      {/* ── Legend bar ── */}
      <div className="mt-2 flex flex-wrap items-center gap-3 md:gap-4 text-[8px] md:text-[9px] text-white/40 uppercase tracking-wider shrink-0">
        <Swatch color={PAL.red.bg} label="Trap Zone" />
        <Swatch color={PAL.yellow.bg} label="Yellow Rack" />
        <Swatch color={PAL.blue.bg} label="Blue Rack" />
        <Swatch color={PAL.robot} label="AMR" />
        <Swatch color={PAL.cyan} label="Waypoint" dot />
      </div>
    </div>
  );
}

// ── Legend swatch ──
function Swatch({ color, label, dot }) {
  return (
    <div className="flex items-center gap-1.5">
      <div
        className={dot ? "rounded-full" : ""}
        style={{
          width: dot ? "6px" : "10px",
          height: dot ? "6px" : "10px",
          backgroundColor: color,
        }}
      />
      <span>{label}</span>
    </div>
  );
}
