"use client";
import { useState, useEffect } from 'react';

interface CameraFeedProps {
  topicName: string;
  feedTitle: string;
}

export default function CameraFeed({ topicName, feedTitle }: CameraFeedProps) {
  const [imageSrc, setImageSrc] = useState<string>('');
  const [isConnected, setIsConnected] = useState(false);
  const [error, setError] = useState<string>('');

  useEffect(() => {
    if (!topicName) return;

    const fetchImage = async () => {
      try {
        const response = await fetch('/api/ros-image', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ topic: topicName }),
        });

        if (response.ok) {
          const blob = await response.blob();
          const imageUrl = URL.createObjectURL(blob);
          setImageSrc(imageUrl);
          setIsConnected(true);
          setError('');
        } else {
          setIsConnected(false);
          setError('Failed to fetch image');
        }
      } catch (err) {
        setIsConnected(false);
        setError('Connection error');
      }
    };

    // Fetch image every 100ms for ~10 FPS
    const interval = setInterval(fetchImage, 100);
    fetchImage(); // Initial fetch

    return () => {
      clearInterval(interval);
      if (imageSrc) URL.revokeObjectURL(imageSrc);
    };
  }, [topicName]);

  return (
    <div className="bg-white rounded-xl shadow-md overflow-hidden border-2 border-gray-200">
      <div className="flex items-center justify-between p-3 bg-gray-50 border-b border-gray-200">
        <h2 className="text-xl font-semibold text-gray-700">{feedTitle}</h2>
        <div className="flex items-center gap-2">
          <div className={`w-3 h-3 rounded-full ${isConnected ? 'bg-green-500' : 'bg-red-500'}`}></div>
          <span className="text-xs text-gray-500">{topicName || 'No topic'}</span>
        </div>
      </div>
      <div className="relative w-full h-80 bg-gray-200 flex items-center justify-center">
        {imageSrc ? (
          <img 
            src={imageSrc} 
            alt={feedTitle}
            className="w-full h-full object-cover"
          />
        ) : (
          <div className="text-center">
            <div className="text-gray-500 text-lg font-medium">
              {error || 'Waiting for image...'}
            </div>
            {topicName && (
              <div className="text-gray-400 text-sm mt-2">
                Topic: {topicName}
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
}