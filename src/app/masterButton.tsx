import React from "react";

export default function MaterButton(){
    return (
        <div className="w-full max-w-4xl px-4 mb-8"> {/* Container for master button, increased mb */}
        <button
          className="w-full py-5 px-8 bg-indigo-600 hover:bg-indigo-700 text-white text-2xl font-bold rounded-lg shadow-xl transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-4 focus:ring-indigo-300"
          onClick={() => console.log('Master Button clicked!')}
        >
          Master Control Button
        </button>
      </div>
    )
}