import React from "react";

interface MainButtonProps {
    color: string; // Accepts a color string like 'blue', 'purple', etc.
    text: string; // Text to display on the button
}

export default function MainButton(props: MainButtonProps){
    let tailwindClasses  = `w-full py-3 px-6 bg-${props.color}-600 hover:bg-${props.color}-700 text-white font-bold rounded-lg shadow-md transition duration-300 ease-in-out transform hover:scale-105 focus:outline-none focus:ring-4 focus:ring-${props.color}-300`;
    return (
        <button
            className={tailwindClasses}
            onClick={() => console.log(`${props.text} Button clicked!`)}
          >
            {props.text}
          </button>
    )
}  