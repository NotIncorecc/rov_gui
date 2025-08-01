"use client";
import clsx from 'clsx';
import Image from "next/image";
import React, {useState} from 'react';
import { useRouter } from 'next/navigation';

// Function to execute shell commands
const executeCommand = async (command: string, buttonName: string) => {
  try {
    console.log(`Executing command for ${buttonName}: ${command}`);
    
    const response = await fetch('/api/execute-command', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ command }),
    });
    
    const result = await response.json();
    
    if (result.success) {
      console.log(`${buttonName} command executed successfully:`, result.stdout);
      if (result.stderr) {
        console.warn(`${buttonName} command stderr:`, result.stderr);
      }
    } else {
      console.error(`${buttonName} command failed:`, result.error);
    }
  } catch (error) {
    console.error(`Error executing ${buttonName} command:`, error);
  }
};

// Function to execute multiple commands in the same terminal
const executeMultipleCommands = async (commands: string[], buttonName: string) => {
  try {
    console.log(`Opening terminal with multiple commands for ${buttonName}:`, commands);
    
    const response = await fetch('/api/execute-command', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ 
        type: 'terminal-multi',
        commands,
        buttonName 
      }),
    });
    
    const result = await response.json();
    
    if (result.success) {
      console.log(`${buttonName} terminal opened successfully:`, result.stdout);
    } else {
      console.error(`${buttonName} terminal failed:`, result.error);
    }
  } catch (error) {
    console.error(`Error opening ${buttonName} terminal:`, error);
  }
};



export default function Home() {
  const router = useRouter();

  const [armColor, setArmColor] = useState('gray');
  const [driveColor, setDriveColor] = useState('gray');
  const [cameraColor, setCameraColor] = useState('gray');
  const [multijoyColor, setMultijoyColor] = useState('gray');

  return (
    // Main container - using 'p-2' for minimal overall padding to maximize space
    <div className="min-h-screen bg-gray-100 flex flex-col items-center p-2 font-sans">

      {/* Camera Feeds Section - 4 feeds, bigger, and using full screen width */}
      <div className="w-full grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4 mb-4">
        {/* Camera Feed 1 */}
        <div className="bg-white rounded-xl shadow-md overflow-hidden border-2 border-gray-200">
          <h2 className="text-xl font-semibold text-gray-700 p-3 bg-gray-50 border-b border-gray-200">
            Camera Feed 1
          </h2>
          <div className="relative w-full h-80 bg-gray-200 flex items-center justify-center text-gray-500 text-lg font-medium">
            Placeholder for Camera 1
          </div>
        </div>

        {/* Camera Feed 2 */}
        <div className="bg-white rounded-xl shadow-md overflow-hidden border-2 border-gray-200">
          <h2 className="text-xl font-semibold text-gray-700 p-3 bg-gray-50 border-b border-gray-200">
            Camera Feed 2
          </h2>
          <div className="relative w-full h-80 bg-gray-200 flex items-center justify-center text-gray-500 text-lg font-medium">
            Placeholder for Camera 2
          </div>
        </div>

        {/* Camera Feed 3 */}
        <div className="bg-white rounded-xl shadow-md overflow-hidden border-2 border-gray-200">
          <h2 className="text-xl font-semibold text-gray-700 p-3 bg-gray-50 border-b border-gray-200">
            Camera Feed 3
          </h2>
          <div className="relative w-full h-80 bg-gray-200 flex items-center justify-center text-gray-500 text-lg font-medium">
            Placeholder for Camera 3
          </div>
        </div>

        {/* Camera Feed 4 */}
        <div className="bg-white rounded-xl shadow-md overflow-hidden border-2 border-gray-200">
          <h2 className="text-xl font-semibold text-gray-700 p-3 bg-gray-50 border-b border-gray-200">
            Camera Feed 4
          </h2>
          <div className="relative w-full h-80 bg-gray-200 flex items-center justify-center text-gray-500 text-lg font-medium">
            Placeholder for Camera 4
          </div>
        </div>
      </div>

      {/* Master Button - Remains centered and its size is unchanged */}
      <div className="w-full max-w-2xl px-2 mb-4">
        <button
          className="w-full py-3 px-6 bg-indigo-600 hover:bg-indigo-700 text-white text-xl font-bold rounded-lg shadow-md transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-4 focus:ring-indigo-300"
          onClick={() => executeCommand('gnome-terminal', 'Master Control')}
        >
          Master Control Button
        </button>
      </div>

      {/* New Section: Left-aligned Buttons & Right-aligned Sensor Info */}
      <div className="w-full flex flex-col md:flex-row gap-4 mb-4">

        {/* Left Column: All other buttons, stacked vertically and center-aligned within their groups */}
        {/* The parent column itself is still aligned to the start (left) */}
        <div className="w-full md:w-1/4 flex flex-col gap-4 p-2">

          {/* Button Group (Main + Secondary) - Now center-aligned within its own container */}
          <div className="max-w-sm flex flex-col items-center gap-3 p-4 bg-white rounded-xl shadow-md border border-gray-200 mx-auto"> {/* Changed items-start to items-center, added mx-auto to center block itself */}
            <button
              className="w-full py-2 px-4 bg-blue-600 hover:bg-blue-700 text-white font-medium rounded-md shadow-sm transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-2 focus:ring-blue-300"
              onClick={() => {executeMultipleCommands([
                'echo "Opening multijoy terminal session..."',
                'ls -la',
                'pwd',
                'echo "Terminal ready for multijoy operations"',
              ], 'Multijoy Terminal'); setMultijoyColor('green');}}
            >
              Activate Multijoy
            </button>
            <button
              className={clsx(
                  'w-full py-1 px-2 text-white text-xs rounded-sm shadow-xs transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-1',
                  {
                    'bg-gray-500 hover:bg-gray-600 focus:ring-gray-300': multijoyColor === 'gray',
                    'bg-green-500 hover:bg-green-600 focus:ring-green-300': multijoyColor === 'green',
                  }
                )}
              onClick={() => router.push('/terminal/multijoy-terminal')}
            >
              open terminal
            </button>
          </div>

          {/* Two Horizontal Groups of Buttons Section - Now contained within the left column, still horizontal, and their internal buttons are centered */}
          <div className="w-full grid grid-cols-1 sm:grid-cols-2 gap-4">
            {/* Left Button Group (within horizontal section) */}
            <div className="flex flex-col items-center gap-3 p-4 bg-white rounded-xl shadow-md border border-gray-200 mx-auto"> {/* Changed items-start to items-center, added mx-auto */}
              <button
                className="w-full py-2 px-4 bg-purple-600 hover:bg-purple-700 text-white font-medium rounded-md shadow-sm transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-2 focus:ring-purple-300"
                onClick={() => {executeMultipleCommands([
                  'echo "Setting up ARM development environment..."',
                  'echo "ARM environment ready!"'
                ], 'ARM SSH Terminal'); setArmColor('green');}}
              >
                ARM SSH
              </button>
              <button
                className={clsx(
                  'w-full py-1 px-2 text-white text-xs rounded-sm shadow-xs transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-1',
                  {
                    'bg-gray-500 hover:bg-gray-600 focus:ring-gray-300': armColor === 'gray',
                    'bg-green-500 hover:bg-green-600 focus:ring-green-300': armColor === 'green',
                  }
                )}
                onClick={() => router.push('/terminal/arm-ssh-terminal')}
              >
                open terminal
              </button>
            </div>

            {/* Right Button Group (within horizontal section) */}
            <div className="flex flex-col items-center gap-3 p-4 bg-white rounded-xl shadow-md border border-gray-200 mx-auto"> {/* Changed items-start to items-center, added mx-auto */}
              <button
                className="w-full py-2 px-4 bg-green-600 hover:bg-green-700 text-white font-medium rounded-md shadow-sm transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-2 focus:ring-green-300"
                onClick={() => {executeMultipleCommands([
                  'echo "Initializing drive control system..."',
                  'echo "Drive system status checked. Ready for operations!"'
                ], 'Drive SSH Terminal'); setDriveColor('green');}}
              >
                DRIVE SSH
              </button>
              <button
                className={clsx(
                  'w-full py-1 px-2 text-white text-xs rounded-sm shadow-xs transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-1',
                  {
                    'bg-gray-500 hover:bg-gray-600 focus:ring-gray-300': driveColor === 'gray',
                    'bg-green-500 hover:bg-green-600 focus:ring-green-300': driveColor === 'green',
                  }
                )}
                onClick={() => router.push('/terminal/drive-ssh-terminal')}
              >
                open terminal
              </button>
            </div>
          </div>

          {/* Button Group (Main + Secondary) - Now center-aligned within its own container, at the very bottom */}
          <div className="max-w-sm flex flex-col items-center gap-3 p-4 bg-white rounded-xl shadow-md border border-gray-200 mx-auto"> {/* Changed items-start to items-center, added mx-auto */}
            <button
              className="w-full py-2 px-4 bg-blue-600 hover:bg-blue-700 text-white font-medium rounded-md shadow-sm transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-2 focus:ring-blue-300"
              onClick={() => {executeMultipleCommands([
                'echo "Initializing camera monitoring terminal..."',
                'echo "Camera system diagnostics complete. Terminal ready for monitoring!"'
              ], 'Camera Monitoring Terminal'); setCameraColor('green');}}
            >
              Activate Cameras
            </button>
            <button
              className={clsx(
                  'w-full py-1 px-2 text-white text-xs rounded-sm shadow-xs transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-1',
                  {
                    'bg-gray-500 hover:bg-gray-600 focus:ring-gray-300': cameraColor === 'gray',
                    'bg-green-500 hover:bg-green-600 focus:ring-green-300': cameraColor === 'green',
                  }
                )}
              onClick={() => router.push('/terminal/camera-monitoring-terminal')}
            >
              open terminal
            </button>
          </div>

        </div> {/* End of Left Column: All Buttons */}

        {/* Right Column: Sensor Info Title and extra space */}
        <div className="w-full md:w-3/4 flex flex-col items-center md:items-start p-6 bg-white rounded-xl shadow-md border border-gray-200">
          <h2 className="text-3xl font-bold text-gray-700 mb-4">Sensor Info</h2>
          <div className="text-gray-600 text-lg w-full text-center md:text-left">
            <p>This area provides extra space for future sensor data and other elements.</p>
            <p className="mt-2">You can add charts, graphs, or more detailed information here.</p>
            {/* Add more elements here as needed */}
          </div>
        </div>

      </div> {/* End of New Section: Left-aligned Buttons & Right-aligned Sensor Info */}

    </div>
  );
}
