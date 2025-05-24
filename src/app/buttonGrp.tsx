import React from "react";
import SecondaryButton from "./secButton";
import MainButton from "./mainButton";

interface MainButtonProps {
    color: string; // Accepts a color string like 'blue', 'purple', etc.
    text: string; // Text to display on the button
}

export default function ButtonGrp(props: MainButtonProps){
    return (
        <div className="flex flex-col items-center gap-4 p-6 bg-white rounded-xl shadow-lg border border-gray-200">
          {/* Main Button (e.g., Action Button 1) */}
          <MainButton color={props.color} text={props.text} />
          {/* Secondary Button for Left Group */}
          <SecondaryButton />
        </div>
    )
}