import { NextRequest, NextResponse } from 'next/server';
import WebSocket from 'ws';

export async function POST(request: NextRequest) {
  try {
    const { topic } = await request.json();
    
    if (!topic) {
      return NextResponse.json(
        { error: 'Topic name is required' },
        { status: 400 }
      );
    }

    return new Promise((resolve) => {
      const ws = new WebSocket('ws://localhost:9090');
      let imageReceived = false;
      
      const timeout = setTimeout(() => {
        if (!imageReceived) {
          ws.close();
          resolve(NextResponse.json({ 
            error: 'Timeout waiting for image from rosbridge',
            suggestion: 'Check if rosbridge is running: ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
          }, { status: 408 }));
        }
      }, 5000);

      ws.on('open', () => {
        console.log('Connected to rosbridge');
        
        // Subscribe to the image topic
        const subscribeMsg = {
          op: 'subscribe',
          topic: topic,
          type: 'sensor_msgs/msg/Image',
          throttle_rate: 500, // Limit to 2 FPS for API calls
          queue_length: 1
        };
        
        ws.send(JSON.stringify(subscribeMsg));
      });

      ws.on('message', (data) => {
        try {
          const message = JSON.parse(data.toString());
          
          if (message.topic === topic && message.msg) {
            imageReceived = true;
            clearTimeout(timeout);
            
            // For now, return the raw message data
            // TODO: Convert to actual image format
            ws.close();
            
            resolve(NextResponse.json({
              success: true,
              topic: topic,
              imageData: {
                width: message.msg.width,
                height: message.msg.height,
                encoding: message.msg.encoding,
                dataLength: message.msg.data ? message.msg.data.length : 0
              },
              message: 'Image data received but conversion to viewable format pending'
            }));
          }
        } catch (parseError) {
          console.error('Error parsing WebSocket message:', parseError);
        }
      });

      ws.on('error', (error) => {
        clearTimeout(timeout);
        ws.close();
        console.error('WebSocket error:', error);
        resolve(NextResponse.json({ 
          error: 'Failed to connect to rosbridge',
          details: error.message,
          suggestion: 'Start rosbridge: ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
        }, { status: 500 }));
      });
    });
    
  } catch (error: any) {
    console.error('ROS image WebSocket error:', error);
    
    return NextResponse.json({
      success: false,
      error: error.message
    }, { status: 500 });
  }
}