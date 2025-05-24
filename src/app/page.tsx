"use client";
import Image from "next/image";
import React from 'react';


export default function Home() {
  return (
    // Main container, now aligning items to the start (top)
    <div className="min-h-screen bg-gray-100 flex flex-col items-center pt-8 pb-8 font-sans"> {/* Added pt-8 for top padding */}

      {/* Camera Feeds Section - Remains at the top, adjusted to fit screen horizontally */}
      <div className="w-full max-w-6xl grid grid-cols-1 md:grid-cols-2 gap-6 mb-8 px-4 md:px-0"> {/* Added px-4 for mobile padding */}
        {/* First Camera Feed Container (Empty Placeholder) */}
        <div className="bg-white rounded-xl shadow-xl overflow-hidden border-2 border-gray-200">
          <h2 className="text-2xl font-semibold text-gray-700 p-4 bg-gray-50 border-b border-gray-200">
            Camera Feed 1
          </h2>
          <div className="relative w-full h-64 bg-gray-200 flex items-center justify-center text-gray-500 text-lg font-medium">
            Placeholder for Camera 1
          </div>
        </div>

        {/* Second Camera Feed Container (Empty Placeholder) */}
        <div className="bg-white rounded-xl shadow-xl overflow-hidden border-2 border-gray-200">
          <h2 className="text-2xl font-semibold text-gray-700 p-4 bg-gray-50 border-b border-gray-200">
            Camera Feed 2
          </h2>
          <div className="relative w-full h-64 bg-gray-200 flex items-center justify-center text-gray-500 text-lg font-medium">
            Placeholder for Camera 2
          </div>
        </div>
      </div>

      {/* Master Button - A single, bigger button under the feeds */}
      <div className="w-full max-w-4xl px-4 mb-10"> {/* Container for master button */}
        <button
          className="w-full py-5 px-8 bg-indigo-600 hover:bg-indigo-700 text-white text-2xl font-bold rounded-lg shadow-xl transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-4 focus:ring-indigo-300"
          onClick={() => console.log('Master Button clicked!')}
        >
          Master Control Button
        </button>
      </div>

      {/* Two Groups of Buttons Section - Aligned horizontally */}
      <div className="w-full max-w-6xl grid grid-cols-1 md:grid-cols-2 gap-8 px-4"> {/* Adjusted gap and added px for mobile */}
        {/* Left Button Group */}
        <div className="flex flex-col items-center gap-4 p-6 bg-white rounded-xl shadow-lg border border-gray-200">
          {/* Main Button (e.g., Action Button 1) */}
          <button
            className="w-full py-3 px-6 bg-blue-600 hover:bg-blue-700 text-white font-bold rounded-lg shadow-md transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-4 focus:ring-blue-300"
            onClick={() => console.log('Main Button Left clicked!')}
          >
            Left Main Action
          </button>
          {/* Secondary Button for Left Group */}
          <button
            className="w-full py-2 px-4 bg-gray-500 hover:bg-gray-600 text-white text-sm rounded-md shadow-sm transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-2 focus:ring-gray-300"
            onClick={() => console.log('Secondary Button Left clicked!')}
          >
            Left Secondary Option
          </button>
        </div>

        {/* Right Button Group */}
        <div className="flex flex-col items-center gap-4 p-6 bg-white rounded-xl shadow-lg border border-gray-200">
          {/* Main Button (e.g., Action Button 2) */}
          <button
            className="w-full py-3 px-6 bg-green-600 hover:bg-green-700 text-white font-bold rounded-lg shadow-md transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-4 focus:ring-green-300"
            onClick={() => console.log('Main Button Right clicked!')}
          >
            Right Main Action
          </button>
          {/* Secondary Button for Right Group */}
          <button
            className="w-full py-2 px-4 bg-gray-500 hover:bg-gray-600 text-white text-sm rounded-md shadow-sm transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-2 focus:ring-gray-300"
            onClick={() => console.log('Secondary Button Right clicked!')}
          >
            Right Secondary Option
          </button>
        </div>
      </div>
    </div>
  );
}
