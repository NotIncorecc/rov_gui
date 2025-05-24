import React from "react";

interface CameraFeedProps {
    cameraName: string;
    cameraPlaceholder: string;
}

export default function CameraFeed(props: CameraFeedProps) {
    return (
        <div className="bg-white rounded-xl shadow-xl overflow-hidden border-2 border-gray-200">
          <h2 className="text-2xl font-semibold text-gray-700 p-4 bg-gray-50 border-b border-gray-200">
            {props.cameraName}
          </h2>
          <div className="relative w-full h-64 bg-gray-200 flex items-center justify-center text-gray-500 text-lg font-medium">
            {props.cameraPlaceholder}
          </div>
        </div>
    )
}