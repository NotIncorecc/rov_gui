import { NextRequest, NextResponse } from 'next/server';
import { exec } from 'child_process';
import { promisify } from 'util';
import fs from 'fs';
import path from 'path';

const execAsync = promisify(exec);

export async function POST(request: NextRequest) {
  try {
    const { topic } = await request.json();
    
    if (!topic) {
      return NextResponse.json(
        { error: 'Topic name is required' },
        { status: 400 }
      );
    }

    // Create a unique filename to avoid conflicts
    const timestamp = Date.now();
    const imageFilename = `camera_feed_${timestamp}.jpg`;
    const imagePath = path.join('/tmp', imageFilename);

    try {
      // Method 1: Try using ros2 topic echo to get raw image data
      const echoCommand = `timeout 3s ros2 topic echo --once ${topic}`;
      
      try {
        const { stdout } = await execAsync(echoCommand);
        
        // Check if we got valid topic data
        if (stdout && stdout.includes('data:')) {
          // For now, return a placeholder response since parsing ROS message is complex
          return NextResponse.json(
            { 
              success: true, 
              message: 'Topic accessible but image parsing not implemented',
              topic: topic,
              dataReceived: true
            }
          );
        }
      } catch (echoError) {
        console.log('Topic echo failed, trying alternative method...');
      }

      // Method 2: Check if topic exists and is publishing
      try {
        const listCommand = `ros2 topic list`;
        const { stdout: topicList } = await execAsync(listCommand);
        
        if (!topicList.includes(topic)) {
          return NextResponse.json(
            { error: `Topic ${topic} not found. Available topics: ${topicList}` },
            { status: 404 }
          );
        }

        // Check topic type
        const typeCommand = `ros2 topic info ${topic}`;
        const { stdout: topicInfo } = await execAsync(typeCommand);
        
        return NextResponse.json({
          success: false,
          error: 'Image processing not yet implemented',
          topic: topic,
          topicExists: true,
          topicInfo: topicInfo,
          message: 'ROS2 topic is accessible but image conversion is pending implementation'
        });

      } catch (infoError) {
        return NextResponse.json(
          { error: `Failed to get topic info: ${infoError}` },
          { status: 500 }
        );
      }

    } catch (commandError) {
      console.error('ROS command error:', commandError);
      
      return NextResponse.json(
        { 
          error: 'ROS2 command execution failed', 
          details: commandError,
          suggestion: 'Make sure ROS2 is sourced and rosbridge is running'
        },
        { status: 500 }
      );
    }
    
  } catch (error: any) {
    console.error('ROS image API error:', error);
    
    return NextResponse.json({
      success: false,
      error: error.message,
      stack: process.env.NODE_ENV === 'development' ? error.stack : undefined
    }, { status: 500 });
  }
}