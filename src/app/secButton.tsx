import React from "react";

export default function SecondaryButton(){
    return (
        <button
          className="w-full py-2 px-4 bg-gray-500 hover:bg-gray-600 text-white text-sm rounded-md shadow-sm transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-2 focus:ring-gray-300"
          onClick={() => console.log('Top Centered Secondary Button clicked!')}
        >
          open terminal
        </button>
    )
}