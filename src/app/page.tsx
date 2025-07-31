"use client";
import Image from "next/image";
import React from 'react';
import CameraFeed from "./cameraFeed";
import MasterButton from "./masterButton";
import ButtonGrp from "./buttonGrp";


export default function Home() {
  return (
    // Main container, now aligning items to the start (top)
    <div className="min-h-screen bg-gray-100 flex flex-col items-center pt-8 pb-8 font-sans"> {/* Added pt-8 for top padding */}

      {/* Camera Feeds Section - Remains at the top, adjusted to fit screen horizontally */}
      <div className="w-full max-w-6xl grid grid-cols-1 md:grid-cols-2 gap-6 mb-8 px-4 md:px-0"> {/* Added px-4 for mobile padding */}

        <CameraFeed cameraName="Camera Feed 1" cameraPlaceholder="Placeholder for Camera 1"/>

        <CameraFeed cameraName="Camera Feed 2" cameraPlaceholder="Placeholder for Camera 2"/>

        <CameraFeed cameraName="Camera Feed 3" cameraPlaceholder="Placeholder for Camera 3"/>

        <CameraFeed cameraName="Camera Feed 4" cameraPlaceholder="Placeholder for Camera 4"/>
      </div>

      <MasterButton />

      <ButtonGrp color="blue" text="Multijoy Activation" />

      <div className="w-full max-w-6xl grid grid-cols-1 md:grid-cols-2 gap-8 mb-8 px-4"> {/* Added mb */}
        {/* Left Button Group */}
        <ButtonGrp color="purple" text="ARM SSH" />

        {/* Right Button Group */}
        <ButtonGrp color="green" text="DRIVE SSH" />
      </div>

      <ButtonGrp color="blue" text="Activate Cameras" />

    </div>
  );
}
