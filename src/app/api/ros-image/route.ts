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
      // Add WebSocket options to prevent the bufferUtil error
      const ws = new WebSocket('ws://localhost:9090', {
        perMessageDeflate: false,
        skipUTF8Validation: true
      });
      
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
        
        try {
          // Subscribe to the image topic
          const subscribeMsg = {
            op: 'subscribe',
            topic: topic,
            type: 'sensor_msgs/msg/Image',
            throttle_rate: 500,
            queue_length: 1
          };
          
          // Convert to string and send
          const messageString = JSON.stringify(subscribeMsg);
          ws.send(messageString);
          
        } catch (sendError) {
          console.error('Error sending subscribe message:', sendError);
          clearTimeout(timeout);
          ws.close();
          resolve(NextResponse.json({ 
            error: 'Failed to send subscribe message',
            details: typeof sendError === 'object' && sendError !== null && 'message' in sendError ? (sendError as { message: string }).message : String(sendError)
          }, { status: 500 }));
        }
      });

      ws.on('message', (data) => {
        try {
          const message = JSON.parse(data.toString());
          
          if (message.topic === topic && message.msg) {
            imageReceived = true;
            clearTimeout(timeout);
            const imageMsg = message.msg;
      
            // For now, create a simple base64 encoded response
            const imageDataBuffer = Buffer.from(imageMsg.data);
            const base64Image = imageDataBuffer.toString('base64');

            ws.close();
            
            resolve(NextResponse.json({
              success: true,
              topic: topic,
              imageBase64: base64Image,
              width: imageMsg.width,
              height: imageMsg.height,
              encoding: imageMsg.encoding
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

      ws.on('close', () => {
        clearTimeout(timeout);
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