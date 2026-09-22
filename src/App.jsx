import React, { useState, useEffect } from 'react';
import WarehouseFloorplan from './WarehouseFloorplan';
import MissionInjector from './MissionInjector';

/**
 * App — Main Wrapper with Brutalist Onboarding Tour for Judges
 */

const TOUR_STEPS = [
  {
    step: 1,
    tab: 'SwarmMonitor',
    title: 'FLOORPLAN MAP',
    content: '15X15M WAREHOUSE FLOORPLAN: Real-time 2D representation of the A* costmap.',
  },
  {
    step: 2,
    tab: 'SwarmMonitor',
    title: 'SWARM TELEMETRY',
    content: 'SWARM TELEMETRY: Live X/Y coordinate and state tracking for the decentralized fleet.',
  },
  {
    step: 3,
    tab: 'SwarmMonitor',
    title: 'CNP AUCTION STREAM',
    content: 'CNP AUCTION STREAM: The brain of the swarm. Watch edge-AI nodes bid on tasks based on Manhattan distance.',
  },
  {
    step: 4,
    tab: 'SwarmMonitor',
    title: 'EXECUTION KPI',
    content: 'EXECUTION KPI: Metrics tracking latency and active travel time.',
  },
  {
    step: 5,
    tab: 'SwarmMonitor',
    title: 'MISSION INJECTOR',
    content: 'MISSION INJECTOR: Enter the control panel to inject manual tasks into the P2P network.',
  },
  {
    step: 6,
    tab: 'MissionInjector',
    title: 'PICKUP BAY',
    content: 'PICKUP: Select the starting bay to load cargo.',
  },
  {
    step: 7,
    tab: 'MissionInjector',
    title: 'DROPOFF BAY',
    content: 'DROPOFF: Select the destination bay.',
  },
  {
    step: 8,
    tab: 'MissionInjector',
    title: 'TASK SCRIPT',
    content: 'TASK SCRIPT: Review the queue and broadcast the job to the edge nodes!',
  }
];

export default function App() {
  const [activeTab, setActiveTab] = useState('SwarmMonitor');
  const [tourStep, setTourStep] = useState(0);

  // Clear active tasks/logs and reset to Step 0 on mount
  useEffect(() => {
    fetch('/api/clear_tasks', { method: 'POST' }).catch(() => {});
    setTourStep(0);
  }, []);

  const handleNextStep = () => {
    if (tourStep === 5) {
      // Crucial: Switch view to Suwetha's Mission Injector tab before setting tourStep to 6
      setActiveTab('MissionInjector');
      setTourStep(6);
    } else if (tourStep >= 8) {
      setTourStep(9); // Done
    } else {
      setTourStep((prev) => prev + 1);
    }
  };

  const handlePrevStep = () => {
    if (tourStep === 6) {
      setActiveTab('SwarmMonitor');
      setTourStep(5);
    } else if (tourStep > 1) {
      setTourStep((prev) => prev - 1);
    }
  };

  const handleSkipTour = () => {
    setTourStep(9);
  };

  const currentStepConfig = TOUR_STEPS.find((s) => s.step === tourStep);

  return (
    <div className="w-full min-h-screen bg-black text-neutral-300 font-mono select-none flex flex-col relative">
      {/* ── Top Header Navigation ── */}
      <header className="border-b border-neutral-800 bg-black p-3 flex justify-between items-center shrink-0">
        <div className="flex items-center gap-4">
          <h1 className="text-sm font-bold tracking-wider text-white uppercase">
            SIH26123 SWARM MONITORING SYSTEM
          </h1>
          <span className="text-xs text-neutral-500 uppercase">
            DECENTRALIZED P2P SIMULATION
          </span>
        </div>

        {/* Tab Switcher */}
        <div className="flex items-center gap-2">
          <button
            onClick={() => setActiveTab('SwarmMonitor')}
            className={`px-3 py-1 text-xs font-bold uppercase border transition-colors ${
              activeTab === 'SwarmMonitor'
                ? 'bg-neutral-200 text-black border-white'
                : 'bg-black text-neutral-400 border-neutral-800 hover:text-white'
            }`}
          >
            SWARM MONITOR
          </button>
          <button
            onClick={() => setActiveTab('MissionInjector')}
            className={`px-3 py-1 text-xs font-bold uppercase border transition-colors ${
              tourStep === 5
                ? 'relative z-50 ring-2 ring-green-500 shadow-[0_0_15px_rgba(34,197,94,0.5)] bg-green-950 text-green-300 border-green-500'
                : activeTab === 'MissionInjector'
                ? 'bg-neutral-200 text-black border-white'
                : 'bg-black text-neutral-400 border-neutral-800 hover:text-white'
            }`}
          >
            MISSION INJECTOR
          </button>
          
          <button
            onClick={() => {
              setActiveTab('SwarmMonitor');
              setTourStep(0);
            }}
            className="ml-4 px-2 py-1 text-[11px] font-bold uppercase border border-green-500/50 text-green-400 hover:bg-green-500/10"
          >
            ⚡ RESTART TOUR
          </button>
        </div>
      </header>

      {/* ── Main Tab Content ── */}
      <main className="flex-1 p-4 relative overflow-auto">
        {activeTab === 'SwarmMonitor' && (
          <div className="grid grid-cols-1 lg:grid-cols-12 gap-4 h-full">
            {/* Left: Floorplan Map */}
            <div
              className={`lg:col-span-6 flex flex-col ${
                tourStep === 1
                  ? 'relative z-50 ring-2 ring-green-500 shadow-[0_0_15px_rgba(34,197,94,0.5)]'
                  : ''
              }`}
            >
              <WarehouseFloorplan />
            </div>

            {/* Right: Telemetry, Auction Stream, KPI */}
            <div className="lg:col-span-6 flex flex-col gap-4">
              {/* Telemetry Panel */}
              <div
                className={`bg-neutral-950 border border-neutral-800 p-3 flex flex-col ${
                  tourStep === 2
                    ? 'relative z-50 ring-2 ring-green-500 shadow-[0_0_15px_rgba(34,197,94,0.5)]'
                    : ''
                }`}
              >
                <div className="text-xs font-bold uppercase text-white border-b border-neutral-800 pb-1 mb-2">
                  SWARM TELEMETRY
                </div>
                <div className="grid grid-cols-3 gap-2 text-xs">
                  <div className="border border-neutral-800 p-2 bg-black">
                    <div className="text-neutral-500 text-[10px]">AMR_1</div>
                    <div className="text-white font-bold">COL 6, ROW 2</div>
                    <div className="text-green-400 text-[10px]">STATUS: IDLE</div>
                  </div>
                  <div className="border border-neutral-800 p-2 bg-black">
                    <div className="text-neutral-500 text-[10px]">AMR_2</div>
                    <div className="text-white font-bold">COL 7, ROW 2</div>
                    <div className="text-green-400 text-[10px]">STATUS: IDLE</div>
                  </div>
                  <div className="border border-neutral-800 p-2 bg-black">
                    <div className="text-neutral-500 text-[10px]">AMR_3</div>
                    <div className="text-white font-bold">COL 8, ROW 2</div>
                    <div className="text-green-400 text-[10px]">STATUS: IDLE</div>
                  </div>
                </div>
              </div>

              {/* CNP Auction Stream Panel */}
              <div
                className={`bg-neutral-950 border border-neutral-800 p-3 flex flex-col flex-1 min-h-[180px] ${
                  tourStep === 3
                    ? 'relative z-50 ring-2 ring-green-500 shadow-[0_0_15px_rgba(34,197,94,0.5)]'
                    : ''
                }`}
              >
                <div className="text-xs font-bold uppercase text-white border-b border-neutral-800 pb-1 mb-2">
                  CNP AUCTION STREAM
                </div>
                <div className="bg-black border border-neutral-900 p-2 flex-1 text-xs text-neutral-400 space-y-1 font-mono">
                  <div className="text-neutral-600">[SYSTEM] Listening on /swarm/tasks/pool...</div>
                  <div className="text-green-400">[AUCTION] All 3 AMR nodes initialized & IDLE.</div>
                </div>
              </div>

              {/* Execution KPI Panel */}
              <div
                className={`bg-neutral-950 border border-neutral-800 p-3 flex flex-col ${
                  tourStep === 4
                    ? 'relative z-50 ring-2 ring-green-500 shadow-[0_0_15px_rgba(34,197,94,0.5)]'
                    : ''
                }`}
              >
                <div className="text-xs font-bold uppercase text-white border-b border-neutral-800 pb-1 mb-2">
                  EXECUTION KPI
                </div>
                <div className="grid grid-cols-4 gap-2 text-center text-xs">
                  <div className="bg-black border border-neutral-800 p-2">
                    <div className="text-[10px] text-neutral-500">TASKS</div>
                    <div className="text-white font-bold text-sm">0</div>
                  </div>
                  <div className="bg-black border border-neutral-800 p-2">
                    <div className="text-[10px] text-neutral-500">AVG LATENCY</div>
                    <div className="text-white font-bold text-sm">0.0s</div>
                  </div>
                  <div className="bg-black border border-neutral-800 p-2">
                    <div className="text-[10px] text-neutral-500">TRAVEL TIME</div>
                    <div className="text-white font-bold text-sm">0.0s</div>
                  </div>
                  <div className="bg-black border border-neutral-800 p-2">
                    <div className="text-[10px] text-neutral-500">EFFICIENCY</div>
                    <div className="text-green-400 font-bold text-sm">100%</div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        )}

        {activeTab === 'MissionInjector' && (
          <MissionInjector
            tourStep={tourStep}
            onPassScript={() => setActiveTab('SwarmMonitor')}
          />
        )}
      </main>

      {/* ── Dark Backdrop Overlay for Active Tour Steps (1-8) ── */}
      {tourStep >= 1 && tourStep <= 8 && (
        <div className="fixed inset-0 bg-black/75 z-40 pointer-events-none transition-opacity" />
      )}

      {/* ── STEP 0: INTRO MODAL ── */}
      {tourStep === 0 && (
        <div className="fixed inset-0 bg-black/85 z-50 flex items-center justify-center p-4">
          <div className="bg-black border-2 border-green-500 max-w-xl w-full p-6 text-left font-mono shadow-[0_0_25px_rgba(34,197,94,0.3)]">
            <div className="flex justify-between items-center border-b border-green-500/40 pb-2 mb-4">
              <span className="text-xs font-bold uppercase tracking-widest text-green-400">
                [ SYSTEM NOTIFICATION ]
              </span>
              <span className="text-[10px] bg-green-950 border border-green-500 text-green-300 px-2 py-0.5">
                SIH26123 SIMULATION
              </span>
            </div>
            
            <p className="text-sm text-neutral-200 uppercase leading-relaxed mb-6">
              SYSTEM NOTIFICATION: This is a web-based virtual simulation designed to visualize the decentralized pathfinding and CNP auction logic of our physical ROS2/Gazebo AMR prototype.
            </p>

            <div className="flex justify-between items-center">
              <button
                onClick={handleSkipTour}
                className="text-xs text-neutral-500 hover:text-white uppercase font-bold px-3 py-2 border border-transparent hover:border-neutral-700 cursor-pointer"
              >
                [ SKIP ]
              </button>

              <button
                onClick={() => setTourStep(1)}
                className="bg-green-500 text-black font-bold uppercase text-xs px-6 py-3 border border-green-400 hover:bg-green-400 active:bg-green-600 transition-colors cursor-pointer"
              >
                [ BEGIN SYSTEM TOUR ]
              </button>
            </div>
          </div>
        </div>
      )}

      {/* ── STEPS 1-8: BRUTALIST TOUR TOOLTIP COMPONENT ── */}
      {tourStep >= 1 && tourStep <= 8 && currentStepConfig && (
        <div className="fixed bottom-8 right-8 z-50 max-w-md w-full bg-black border-2 border-green-500 p-4 font-mono shadow-[0_0_20px_rgba(34,197,94,0.4)]">
          <div className="flex justify-between items-center border-b border-neutral-800 pb-2 mb-3">
            <span className="text-xs font-bold text-green-400 uppercase tracking-wider">
              TOUR STEP {tourStep} OF 8: {currentStepConfig.title}
            </span>
            <span className="text-[10px] text-neutral-500 font-bold">
              [{tourStep}/8]
            </span>
          </div>

          <p className="text-xs text-neutral-200 uppercase leading-relaxed mb-4">
            {currentStepConfig.content}
          </p>

          <div className="flex items-center justify-between pt-2 border-t border-neutral-900">
            <button
              onClick={handleSkipTour}
              className="text-xs text-neutral-500 hover:text-white uppercase font-bold px-2 py-1 border border-transparent hover:border-neutral-700 cursor-pointer"
            >
              [ SKIP ]
            </button>

            <div className="flex items-center gap-2">
              <button
                onClick={handlePrevStep}
                disabled={tourStep === 1}
                className={`text-xs font-bold px-3 py-1.5 uppercase border ${
                  tourStep === 1
                    ? 'border-neutral-900 text-neutral-700 bg-black cursor-not-allowed'
                    : 'border-neutral-700 text-neutral-300 hover:text-white hover:border-neutral-500 bg-neutral-950 cursor-pointer'
                }`}
              >
                [ &lt;- PREV ]
              </button>

              <button
                onClick={handleNextStep}
                className="text-xs font-bold px-4 py-1.5 uppercase bg-green-500 text-black border border-green-400 hover:bg-green-400 active:bg-green-600 cursor-pointer"
              >
                [ NEXT -&gt; ]
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
