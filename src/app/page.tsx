"use client";
import Image from "next/image";
import React from 'react';


export default function Home() {
  return (
    // Main container - using 'p-2' for minimal overall padding to maximize space
    <div className="min-h-screen bg-gray-100 flex flex-col items-center p-2 font-sans">

      {/* Camera Feeds Section - Now 4 feeds, bigger, and using full screen width */}
      {/* Removed max-w-6xl to allow full width, adjusted grid for 4 columns on large screens */}
      <div className="w-full grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4 mb-4"> {/* Reduced gap and mb */}
        {/* Camera Feed 1 */}
        <div className="bg-white rounded-xl shadow-md overflow-hidden border-2 border-gray-200">
          <h2 className="text-xl font-semibold text-gray-700 p-3 bg-gray-50 border-b border-gray-200"> {/* Smaller title */}
            Camera Feed 1
          </h2>
          {/* Increased height of feed area */}
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

        {/* Camera Feed 3 (New) */}
        <div className="bg-white rounded-xl shadow-md overflow-hidden border-2 border-gray-200">
          <h2 className="text-xl font-semibold text-gray-700 p-3 bg-gray-50 border-b border-gray-200">
            Camera Feed 3
          </h2>
          <div className="relative w-full h-80 bg-gray-200 flex items-center justify-center text-gray-500 text-lg font-medium">
            Placeholder for Camera 3
          </div>
        </div>

        {/* Camera Feed 4 (New) */}
        <div className="bg-white rounded-xl shadow-md overflow-hidden border-2 border-gray-200">
          <h2 className="text-xl font-semibold text-gray-700 p-3 bg-gray-50 border-b border-gray-200">
            Camera Feed 4
          </h2>
          <div className="relative w-full h-80 bg-gray-200 flex items-center justify-center text-gray-500 text-lg font-medium">
            Placeholder for Camera 4
          </div>
        </div>
      </div>

      {/* Master Button - Made smaller */}
      <div className="w-full max-w-2xl px-2 mb-4"> {/* Reduced max-w and mb */}
        <button
          className="w-full py-3 px-6 bg-indigo-600 hover:bg-indigo-700 text-white text-xl font-bold rounded-lg shadow-md transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-4 focus:ring-indigo-300"
          onClick={() => console.log('Master Button clicked!')}
        >
          Master Control Button
        </button>
      </div>

      {/* Button Group (Main + Secondary) - Centered, under Master Button, made smaller */}
      <div className="w-full max-w-md flex flex-col items-center gap-3 p-4 bg-white rounded-xl shadow-md border border-gray-200 mb-4 px-2"> {/* Reduced max-w, gap, p, and mb */}
        <button
          className="w-full py-2.5 px-5 bg-blue-600 hover:bg-blue-700 text-white font-semibold rounded-md shadow-sm transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-2 focus:ring-blue-300"
          onClick={() => console.log('Top Centered Main Button clicked!')}
        >
          Top Centered Main Button
        </button>
        <button
          className="w-full py-1.5 px-3 bg-gray-500 hover:bg-gray-600 text-white text-xs rounded-sm shadow-xs transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-1 focus:ring-gray-300"
          onClick={() => console.log('Top Centered Secondary Button clicked!')}
        >
          Top Centered Secondary Option
        </button>
      </div>

      {/* Two Groups of Buttons Section - Aligned horizontally, made smaller */}
      <div className="w-full max-w-6xl grid grid-cols-1 md:grid-cols-2 gap-4 mb-4 px-2"> {/* Reduced gap and mb */}
        {/* Left Button Group */}
        <div className="flex flex-col items-center gap-3 p-4 bg-white rounded-xl shadow-md border border-gray-200"> {/* Reduced gap and p */}
          {/* Main Button */}
          <button
            className="w-full py-2.5 px-5 bg-purple-600 hover:bg-purple-700 text-white font-semibold rounded-md shadow-sm transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-2 focus:ring-purple-300"
            onClick={() => console.log('Left Horizontal Main Button clicked!')}
          >
            Left Horizontal Main Action
          </button>
          {/* Secondary Button for Left Group */}
          <button
            className="w-full py-1.5 px-3 bg-gray-500 hover:bg-gray-600 text-white text-xs rounded-sm shadow-xs transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-1 focus:ring-gray-300"
            onClick={() => console.log('Left Horizontal Secondary Button clicked!')}
          >
            Left Horizontal Secondary Option
          </button>
        </div>

        {/* Right Button Group */}
        <div className="flex flex-col items-center gap-3 p-4 bg-white rounded-xl shadow-md border border-gray-200"> {/* Reduced gap and p */}
          {/* Main Button */}
          <button
            className="w-full py-2.5 px-5 bg-green-600 hover:bg-green-700 text-white font-semibold rounded-md shadow-sm transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-2 focus:ring-green-300"
            onClick={() => console.log('Right Horizontal Main Button clicked!')}
          >
            Right Horizontal Main Action
          </button>
          {/* Secondary Button for Right Group */}
          <button
            className="w-full py-1.5 px-3 bg-gray-500 hover:bg-gray-600 text-white text-xs rounded-sm shadow-xs transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-1 focus:ring-gray-300"
            onClick={() => console.log('Right Horizontal Secondary Button clicked!')}
          >
            Right Horizontal Secondary Option
          </button>
        </div>
      </div>

      {/* Button Group (Main + Secondary) - Centered, at the very bottom, made smaller */}
      <div className="w-full max-w-md flex flex-col items-center gap-3 p-4 bg-white rounded-xl shadow-md border border-gray-200 px-2"> {/* Reduced max-w, gap, p */}
        <button
          className="w-full py-2.5 px-5 bg-blue-600 hover:bg-blue-700 text-white font-semibold rounded-md shadow-sm transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-2 focus:ring-blue-300"
          onClick={() => console.log('Bottom Centered Main Button clicked!')}
        >
          Bottom Centered Main Button
        </button>
        <button
          className="w-full py-1.5 px-3 bg-gray-500 hover:bg-gray-600 text-white text-xs rounded-sm shadow-xs transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-1 focus:ring-gray-300"
          onClick={() => console.log('Bottom Centered Secondary Button clicked!')}
        >
          Bottom Centered Secondary Option
        </button>
      </div>

    </div>
  );
}
