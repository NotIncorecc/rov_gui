"use client";
import Image from "next/image";
import React from 'react';


export default function Home() {
  return (
    <div className="min-h-screen bg-gray-100 flex flex-col items-center justify-center p-4 font-sans">
      {/* Camera Feeds Section - Adjusted to fit screen horizontally */}
      <div className="w-full grid grid-cols-1 md:grid-cols-2 gap-6 mb-10">
        {/* First Camera Feed Container (Empty Placeholder) */}
        <div className="bg-white rounded-xl shadow-xl overflow-hidden border-2 border-gray-200">
          <h2 className="text-2xl font-semibold text-gray-700 p-4 bg-gray-50 border-b border-gray-200">
            Camera Feed 1
          </h2>
          {/* Empty div for the feed area */}
          <div className="relative w-full h-64 bg-gray-200 flex items-center justify-center text-gray-500 text-lg font-medium">
            Placeholder for Camera 1
          </div>
        </div>

        {/* Second Camera Feed Container (Empty Placeholder) */}
        <div className="bg-white rounded-xl shadow-xl overflow-hidden border-2 border-gray-200">
          <h2 className="text-2xl font-semibold text-gray-700 p-4 bg-gray-50 border-b border-gray-200">
            Camera Feed 2
          </h2>
          {/* Empty div for the feed area */}
          <div className="relative w-full h-64 bg-gray-200 flex items-center justify-center text-gray-500 text-lg font-medium">
            Placeholder for Camera 2
          </div>
        </div>
      </div>

      {/* Buttons Section */}
      <div className="w-full max-w-2xl flex flex-col sm:flex-row justify-around gap-4 p-4 bg-white rounded-xl shadow-lg">
        {/* Button 1 */}
        <button
          className="flex-1 py-3 px-6 bg-blue-600 hover:bg-blue-700 text-white font-bold rounded-lg shadow-md transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-4 focus:ring-blue-300"
          onClick={() => console.log('Button 1 clicked!')}
        >
          Action Button 1
        </button>

        {/* Button 2 */}
        <button
          className="flex-1 py-3 px-6 bg-green-600 hover:bg-green-700 text-white font-bold rounded-lg shadow-md transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-4 focus:ring-green-300"
          onClick={() => console.log('Button 2 clicked!')}
        >
          Action Button 2
        </button>

        {/* Button 3 */}
        <button
          className="flex-1 py-3 px-6 bg-purple-600 hover:bg-purple-700 text-white font-bold rounded-lg shadow-md transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-4 focus:ring-purple-300"
          onClick={() => console.log('Button 3 clicked!')}
        >
          Action Button 3
        </button>
      </div>
    </div>
  );
}
