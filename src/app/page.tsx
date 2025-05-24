"use client";
import Image from "next/image";
import React from 'react';


export default function Home() {
  return (
    <div className="min-h-screen bg-gray-100 flex flex-col items-center justify-center p-4 font-sans">
      {/* Page Title */}
      <h1 className="text-4xl font-extrabold text-gray-800 mb-8 mt-4 rounded-lg p-2 shadow-lg bg-white">
        Camera Feed Interface
      </h1>

      {/* Camera Feeds Section */}
      <div className="w-full max-w-6xl grid grid-cols-1 md:grid-cols-2 gap-6 mb-10">
        {/* First Camera Feed Container */}
        <div className="bg-white rounded-xl shadow-xl overflow-hidden border-2 border-gray-200">
          <h2 className="text-2xl font-semibold text-gray-700 p-4 bg-gray-50 border-b border-gray-200">
            Camera Feed 1
          </h2>
          {/* Placeholder for Video Feed 1 */}
          <div className="relative w-full h-64 bg-gray-300 flex items-center justify-center text-gray-600 text-lg font-medium">
            <video
              className="absolute inset-0 w-full h-full object-cover rounded-b-xl"
              autoPlay
              muted
              loop
              playsInline
              // Using a placeholder video URL. In a real application, this would be a live stream.
              src="https://www.w3schools.com/html/mov_bbb.mp4"
              onError={(e) => {
                console.error("Error loading video 1:", e);
                e.currentTarget.src = "https://placehold.co/640x360/E0E0E0/888888?text=Video+Unavailable";
              }}
            >
              Your browser does not support the video tag.
            </video>
            {/* Fallback text if video doesn't load */}
            <span className="z-10">Live Feed Placeholder</span>
          </div>
        </div>

        {/* Second Camera Feed Container */}
        <div className="bg-white rounded-xl shadow-xl overflow-hidden border-2 border-gray-200">
          <h2 className="text-2xl font-semibold text-gray-700 p-4 bg-gray-50 border-b border-gray-200">
            Camera Feed 2
          </h2>
          {/* Placeholder for Video Feed 2 */}
          <div className="relative w-full h-64 bg-gray-300 flex items-center justify-center text-gray-600 text-lg font-medium">
            <video
              className="absolute inset-0 w-full h-full object-cover rounded-b-xl"
              autoPlay
              muted
              loop
              playsInline
              // Using a placeholder video URL. In a real application, this would be a live stream.
              src="https://www.w3schools.com/html/mov_bbb.mp4"
              onError={(e) => {
                console.error("Error loading video 2:", e);
                e.currentTarget.src = "https://placehold.co/640x360/E0E0E0/888888?text=Video+Unavailable";
              }}
            >
              Your browser does not support the video tag.
            </video>
            {/* Fallback text if video doesn't load */}
            <span className="z-10">Live Feed Placeholder</span>
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
