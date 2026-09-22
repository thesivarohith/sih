import React, { useState } from 'react';

/**
 * MissionInjector — Brutalist Task Queueing Component for Warehouse AMR Swarm
 * 
 * Column Order: Left (From), Middle (To), Right (Task Script Log)
 */

const BAYS = [
  'r1a_bay_0', 'r1a_bay_1', 'r1a_bay_2', 'r1a_bay_3', 'r1a_bay_4',
  'r1b_bay_0', 'r1b_bay_1', 'r1b_bay_2', 'r1b_bay_3', 'r1b_bay_4',
  'r2a_bay_0', 'r2a_bay_1', 'r2a_bay_2', 'r2a_bay_3', 'r2a_bay_4',
  'r2b_bay_0', 'r2b_bay_1', 'r2b_bay_2', 'r2b_bay_3', 'r2b_bay_4',
  'r3a_bay_0', 'r3a_bay_1', 'r3a_bay_2', 'r3a_bay_3', 'r3a_bay_4',
  'r3b_bay_0', 'r3b_bay_1', 'r3b_bay_2', 'r3b_bay_3', 'r3b_bay_4'
];

export default function MissionInjector({ onPassScript, tourStep }) {
  const [selectedFrom, setSelectedFrom] = useState(null);
  const [step, setStep] = useState('SELECT_FROM');
  const [queuedTasks, setQueuedTasks] = useState([]);
  const [copied, setCopied] = useState(false);

  const handleSelectFrom = (bay) => {
    if (step !== 'SELECT_FROM') return;
    setSelectedFrom(bay);
    setStep('SELECT_TO');
  };

  const handleSelectTo = (toBay) => {
    if (step !== 'SELECT_TO' || !selectedFrom) return;

    const taskId = `task_${Date.now().toString().slice(-6)}`;
    const command = `zenoh put /swarm/tasks/pool '{"msg_type": "AVAILABLE", "task_id": "${taskId}", "pickup": "${selectedFrom}", "drop": "${toBay}"}'`;

    setQueuedTasks((prev) => [
      ...prev,
      {
        id: taskId,
        pickup: selectedFrom,
        drop: toBay,
        command: command,
        timestamp: new Date().toLocaleTimeString()
      }
    ]);

    setSelectedFrom(null);
    setStep('SELECT_FROM');
  };

  const handleClear = () => {
    setQueuedTasks([]);
  };

  const handlePassScript = () => {
    if (queuedTasks.length === 0) return;

    queuedTasks.forEach((t) => {
      fetch('/api/inject_task', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ task_id: t.id, pickup: t.pickup, drop: t.drop })
      }).catch((err) => console.error('Task injection error:', err));
    });

    const fullScript = queuedTasks.map((t) => t.command).join('\n');
    navigator.clipboard?.writeText(fullScript);
    setCopied(true);

    setTimeout(() => {
      if (onPassScript) {
        onPassScript(fullScript, queuedTasks);
      } else {
        window.location.href = '/';
      }
    }, 500);
  };

  return (
    <div className="w-full min-h-screen bg-black text-neutral-300 font-mono p-6 border border-neutral-800 rounded-none flex flex-col select-none">
      {/* Header */}
      <header className="border-b border-neutral-800 pb-4 mb-6 flex justify-between items-center">
        <div>
          <h1 className="text-xl font-bold tracking-tight text-white uppercase">
            MISSION INJECTOR CONTROL
          </h1>
          <p className="text-xs text-neutral-500 uppercase mt-1">
            SWARM TASK QUEUEING SYSTEM / SIH26123
          </p>
        </div>
        <div className="text-xs border border-neutral-800 bg-neutral-950 px-3 py-1 text-neutral-400 font-mono uppercase">
          MODE: {step === 'SELECT_FROM' ? 'PICKUP SELECTION' : 'DROPOFF SELECTION'}
        </div>
      </header>

      {/* Main 3-Column Layout: Left (From), Middle (To), Right (Task Script Log) */}
      <div className="grid grid-cols-1 md:grid-cols-12 gap-6 flex-1">

        {/* LEFT COLUMN: 'From' Grid */}
        <div
          className={`md:col-span-4 bg-neutral-950 border p-4 flex flex-col ${
            tourStep === 6
              ? 'relative z-50 ring-2 ring-green-500 shadow-[0_0_15px_rgba(34,197,94,0.5)] border-green-500 opacity-100 pointer-events-auto'
              : step === 'SELECT_FROM'
              ? 'border-neutral-500'
              : 'border-neutral-900 opacity-40 pointer-events-none'
          }`}
        >
          <div className="flex justify-between items-center border-b border-neutral-800 pb-2 mb-3">
            <span className="text-xs font-bold uppercase text-white">
              FROM: PICKUP BAY
            </span>
            <span
              className={`text-[10px] px-2 py-0.5 font-mono uppercase border ${
                step === 'SELECT_FROM'
                  ? 'border-neutral-600 text-white bg-neutral-900'
                  : 'border-neutral-900 text-neutral-600'
              }`}
            >
              {step === 'SELECT_FROM' ? 'READY' : 'LOCKED'}
            </span>
          </div>

          <div className="grid grid-cols-2 gap-2 overflow-y-auto max-h-[490px] pr-1">
            {BAYS.map((bay) => {
              const isSelected = selectedFrom === bay;
              return (
                <button
                  key={`from_${bay}`}
                  onClick={() => handleSelectFrom(bay)}
                  className={`border p-2.5 text-left text-xs font-mono flex flex-col justify-between ${
                    isSelected
                      ? 'bg-neutral-200 text-black border-white font-bold'
                      : 'bg-black border-neutral-800 hover:border-neutral-600 text-neutral-400 hover:text-white'
                  }`}
                >
                  <span
                    className={`text-[10px] ${
                      isSelected ? 'text-black' : 'text-neutral-600'
                    }`}
                  >
                    BAY
                  </span>
                  <span>{bay}</span>
                </button>
              );
            })}
          </div>
        </div>

        {/* MIDDLE COLUMN: 'To' Grid */}
        <div
          className={`md:col-span-3 bg-neutral-950 border p-4 flex flex-col ${
            tourStep === 7
              ? 'relative z-50 ring-2 ring-green-500 shadow-[0_0_15px_rgba(34,197,94,0.5)] border-green-500 opacity-100 pointer-events-auto'
              : step === 'SELECT_TO'
              ? 'border-neutral-500'
              : 'border-neutral-900 opacity-40 pointer-events-none'
          }`}
        >
          <div className="flex justify-between items-center border-b border-neutral-800 pb-2 mb-3">
            <span className="text-xs font-bold uppercase text-white">
              TO: DROPOFF BAY
            </span>
            <span
              className={`text-[10px] px-2 py-0.5 font-mono uppercase border ${
                step === 'SELECT_TO'
                  ? 'border-neutral-600 text-white bg-neutral-900'
                  : 'border-neutral-900 text-neutral-600'
              }`}
            >
              {step === 'SELECT_TO' ? 'READY' : 'LOCKED'}
            </span>
          </div>

          <div className="grid grid-cols-2 gap-2 overflow-y-auto max-h-[490px] pr-1">
            {BAYS.map((bay) => (
              <button
                key={`to_${bay}`}
                onClick={() => handleSelectTo(bay)}
                className="border border-neutral-800 bg-black hover:border-neutral-600 text-neutral-400 hover:text-white p-2.5 text-left text-xs font-mono flex flex-col justify-between"
              >
                <span className="text-[10px] text-neutral-600">BAY</span>
                <span>{bay}</span>
              </button>
            ))}
          </div>
        </div>

        {/* RIGHT COLUMN: Task Script Log */}
        <div
          className={`md:col-span-5 bg-neutral-950 border p-4 flex flex-col justify-between ${
            tourStep === 8
              ? 'relative z-50 ring-2 ring-green-500 shadow-[0_0_15px_rgba(34,197,94,0.5)] border-green-500'
              : 'border-neutral-800'
          }`}
        >
          <div>
            <div className="flex justify-between items-center border-b border-neutral-800 pb-2 mb-3">
              <span className="text-xs font-bold uppercase text-white">
                TASK SCRIPT LOG ({queuedTasks.length})
              </span>
              {queuedTasks.length > 0 && (
                <button
                  onClick={handleClear}
                  className="text-xs border border-neutral-800 text-neutral-400 hover:text-white px-2 py-0.5 uppercase"
                >
                  CLEAR
                </button>
              )}
            </div>

            {/* Code Output Box */}
            <div className="bg-black border border-neutral-900 p-3 h-[420px] overflow-y-auto font-mono text-xs text-neutral-300 space-y-3 leading-relaxed">
              {queuedTasks.length === 0 ? (
                <div className="text-neutral-600">
                  Select a pickup bay on the left, then a dropoff bay in the middle to append Zenoh task commands.
                </div>
              ) : (
                queuedTasks.map((t, idx) => (
                  <div key={t.id || idx} className="border-b border-neutral-900 pb-2">
                    <div className="text-neutral-600 text-[10px]">
                      Task {idx + 1} [{t.timestamp}] {t.pickup} -&gt; {t.drop}
                    </div>
                    <div className="text-neutral-200 break-all">
                      {t.command}
                    </div>
                  </div>
                ))
              )}
            </div>
          </div>

          {/* Bottom Pass Script Button */}
          <div className="mt-4 pt-3 border-t border-neutral-800">
            <button
              onClick={handlePassScript}
              disabled={queuedTasks.length === 0}
              className={`w-full py-3 font-bold uppercase text-xs border ${
                queuedTasks.length === 0
                  ? 'border-neutral-900 text-neutral-700 bg-black cursor-not-allowed'
                  : 'border-white bg-neutral-200 text-black hover:bg-white'
              }`}
            >
              {copied ? 'SCRIPT COPIED TO CLIPBOARD' : 'PASS SCRIPT TO SWARM'}
            </button>
          </div>
        </div>

      </div>
    </div>
  );
}
