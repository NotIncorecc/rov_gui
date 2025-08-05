import { NextRequest, NextResponse } from 'next/server';
import { exec } from 'child_process';
import { promisify } from 'util';

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

    // Use ros2 topic to get a single image and convert to base64
    const command = `timeout 5s ros2 topic echo -n 1 ${topic} | python3 -c "
import sys
import base64
import sensor_msgs.msg
import cv_bridge
import rospy
import numpy as np
from sensor_msgs.msg import Image

# This would need actual ROS implementation
# For now, return a placeholder command
print('ROS_IMAGE_DATA_HERE')
"`;

    // Alternative: Use ros2 run to save image and serve it
    const saveImageCommand = `ros2 run image_view image_saver image:=${topic} _filename_format:=/tmp/camera_feed.jpg`;
    
    try {
      // Execute command to save latest image
      await execAsync(`timeout 2s ${saveImageCommand}`);
      
      // Read the saved image file
      const fs = require('fs');
      const imagePath = '/tmp/camera_feed.jpg';
      
      if (fs.existsSync(imagePath)) {
        const imageBuffer = fs.readFileSync(imagePath);
        
        return new NextResponse(imageBuffer, {
          headers: {
            'Content-Type': 'image/jpeg',
            'Cache-Control': 'no-cache',
          },
        });
      } else {
        throw new Error('Image file not found');
      }
      
    } catch (error) {
      // If ROS command fails, return placeholder image
      return NextResponse.json(
        { error: 'Failed to fetch ROS image', topic },
        { status: 500 }
      );
    }
    
  } catch (error: any) {
    console.error('ROS image error:', error);
    
    return NextResponse.json({
      success: false,
      error: error.message
    }, { status: 500 });
  }
}