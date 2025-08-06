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

        const result = await response.json();
        
        if (response.ok && result.success && result.imageBase64) {
          setIsConnected(true);
          setError('');
          
          // Convert ROS image data to canvas
          const { width, height, encoding, imageBase64 } = result;
          
          // Decode base64 to get raw image data
          const binaryString = atob(imageBase64);
          const bytes = new Uint8Array(binaryString.length);
          for (let i = 0; i < binaryString.length; i++) {
            bytes[i] = binaryString.charCodeAt(i);
          }
          
          // Create canvas to render the image
          const canvas = document.createElement('canvas');
          canvas.width = width;
          canvas.height = height;
          const ctx = canvas.getContext('2d');
          
          if (ctx) {
            const imageData = ctx.createImageData(width, height);
            
            // Handle different encodings
            if (encoding === 'rgb8') {
              // RGB8: 3 bytes per pixel
              for (let i = 0, j = 0; i < bytes.length; i += 3, j += 4) {
                imageData.data[j] = bytes[i];       // R
                imageData.data[j + 1] = bytes[i + 1]; // G
                imageData.data[j + 2] = bytes[i + 2]; // B
                imageData.data[j + 3] = 255;        // A
              }
            } else if (encoding === 'bgr8') {
              // BGR8: 3 bytes per pixel, swap R and B
              for (let i = 0, j = 0; i < bytes.length; i += 3, j += 4) {
                imageData.data[j] = bytes[i + 2];     // R (from B)
                imageData.data[j + 1] = bytes[i + 1]; // G
                imageData.data[j + 2] = bytes[i];     // B (from R)
                imageData.data[j + 3] = 255;        // A
              }
            } else if (encoding === 'mono8' || encoding === 'grayscale') {
              // Grayscale: 1 byte per pixel
              for (let i = 0, j = 0; i < bytes.length; i++, j += 4) {
                imageData.data[j] = bytes[i];     // R
                imageData.data[j + 1] = bytes[i]; // G
                imageData.data[j + 2] = bytes[i]; // B
                imageData.data[j + 3] = 255;     // A
              }
            } else {
              // For other formats, try to display as RGB
              console.warn(`Unsupported encoding: ${encoding}, trying RGB interpretation`);
              for (let i = 0, j = 0; i < bytes.length && j < imageData.data.length; i += 3, j += 4) {
                imageData.data[j] = bytes[i] || 0;
                imageData.data[j + 1] = bytes[i + 1] || 0;
                imageData.data[j + 2] = bytes[i + 2] || 0;
                imageData.data[j + 3] = 255;
              }
            }
            
            ctx.putImageData(imageData, 0, 0);
            
            // Convert canvas to blob URL
            canvas.toBlob((blob) => {
              if (blob) {
                const imageUrl = URL.createObjectURL(blob);
                setImageSrc(prev => {
                  if (prev && prev.startsWith('blob:')) {
                    URL.revokeObjectURL(prev);
                  }
                  return imageUrl;
                });
              }
            }, 'image/png', 1.0);
          }
          
        } else if (response.ok && result.success) {
          setIsConnected(true);
          setError('Connected but no image data received');
        } else {
          setIsConnected(false);
          setError(result.error || 'Unknown error');
        }
      } catch (err) {
        setIsConnected(false);
        setError('Processing error');
        console.error('Image processing error:', err);
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

// WebSocket message handling (to be integrated in the appropriate place) -> done it in route.ts
// ws.on('message', (data) => {
//   try {
//     const message = JSON.parse(data.toString());
    
//     if (message.topic === topic && message.msg) {
//       imageReceived = true;
//       clearTimeout(timeout);
//       const imageMsg = message.msg;

//       // Debug: Log the actual message structure
//       console.log('Image message details:', {
//         width: imageMsg.width,
//         height: imageMsg.height,
//         encoding: imageMsg.encoding,
//         step: imageMsg.step,
//         is_bigendian: imageMsg.is_bigendian,
//         dataType: typeof imageMsg.data,
//         dataLength: imageMsg.data ? imageMsg.data.length : 0,
//         expectedLength: imageMsg.width * imageMsg.height * (imageMsg.encoding === 'rgb8' ? 3 : imageMsg.encoding === 'mono8' ? 1 : 3),
//         firstFewBytes: imageMsg.data ? imageMsg.data.slice(0, 10) : 'no data'
//       });

//       ws.close();
      
//       resolve(NextResponse.json({
//         success: true,
//         topic: topic,
//         imageBase64: Buffer.from(imageMsg.data).toString('base64'),
//         width: imageMsg.width,
//         height: imageMsg.height,
//         encoding: imageMsg.encoding,
//         step: imageMsg.step,
//         is_bigendian: imageMsg.is_bigendian,
//         debug: {
//           dataLength: imageMsg.data.length,
//           expectedLength: imageMsg.width * imageMsg.height * (imageMsg.encoding === 'rgb8' ? 3 : 1)
//         }
//       }));
//     }
//   } catch (parseError) {
//     console.error('Error parsing WebSocket message:', parseError);
//   }
// });