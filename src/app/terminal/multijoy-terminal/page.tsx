"use client";
import { useState, useEffect, useRef } from 'react';
import { useRouter } from 'next/navigation';

export default function MultijoyTerminal() {
  const [logs, setLogs] = useState<string[]>([]);
  const [command, setCommand] = useState('');
  const [isConnected, setIsConnected] = useState(false);
  const logsEndRef = useRef<HTMLDivElement>(null);
  const router = useRouter();
  const sessionName = 'multijoy-terminal';

  // Auto-scroll to bottom when new logs arrive
  const scrollToBottom = () => {
    logsEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [logs]);

  // Fetch logs from tmux session
  const fetchLogs = async () => {
    try {
      const response = await fetch('/api/tmux-logs', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ sessionName }),
      });
      
      if (response.ok) {
        const result = await response.json();
        if (result.success && result.logs) {
          setLogs(result.logs.split('\n').filter((line: string) => line.trim()));
          setIsConnected(true);
        }
      }
    } catch (error) {
      console.error('Error fetching logs:', error);
      setIsConnected(false);
    }
  };

  // Send command to tmux session
  const sendCommand = async () => {
    if (!command.trim()) return;

    try {
      const response = await fetch('/api/tmux-command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ sessionName, command }),
      });

      if (response.ok) {
        setCommand('');
        // Fetch logs immediately after sending command
        setTimeout(fetchLogs, 100);
      }
    } catch (error) {
      console.error('Error sending command:', error);
    }
  };

  // Poll for logs every 500ms
  useEffect(() => {
    const interval = setInterval(fetchLogs, 500);
    fetchLogs(); // Initial fetch
    
    return () => clearInterval(interval);
  }, []);

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter') {
      sendCommand();
    }
  };

  return (
    <div className="min-h-screen bg-black text-green-400 font-mono flex flex-col">
      {/* Header */}
      <div className="bg-gray-900 p-4 border-b border-gray-700 flex items-center justify-between">
        <div className="flex items-center gap-4">
          <button
            onClick={() => router.back()}
            className="bg-red-600 hover:bg-red-700 text-white px-4 py-2 rounded transition-colors"
          >
            ← Back
          </button>
          <h1 className="text-xl font-bold text-white">Multijoy Terminal</h1>
          <div className={`w-3 h-3 rounded-full ${isConnected ? 'bg-green-500' : 'bg-red-500'}`}></div>
          <span className="text-sm text-gray-400">
            Session: {sessionName} ({isConnected ? 'Connected' : 'Disconnected'})
          </span>
        </div>
      </div>

      {/* Terminal Output */}
      <div className="flex-1 p-4 overflow-y-auto">
        <div className="space-y-1">
          {logs.map((log, index) => (
            <div key={index} className="whitespace-pre-wrap break-words">
              {log}
            </div>
          ))}
          <div ref={logsEndRef} />
        </div>
      </div>

      {/* Command Input */}
      <div className="bg-gray-900 p-4 border-t border-gray-700">
        <div className="flex items-center">
          <span className="text-green-400 mr-2">$</span>
          <input
            type="text"
            value={command}
            onChange={(e) => setCommand(e.target.value)}
            onKeyDown={handleKeyPress}
            placeholder="Enter command..."
            className="flex-1 bg-transparent text-green-400 outline-none placeholder-gray-500"
            autoFocus
          />
          <button
            onClick={sendCommand}
            className="ml-4 bg-green-600 hover:bg-green-700 text-white px-4 py-2 rounded transition-colors"
          >
            Send
          </button>
        </div>
      </div>
    </div>
  );
}