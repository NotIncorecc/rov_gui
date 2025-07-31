import { NextRequest, NextResponse } from 'next/server';
import { exec } from 'child_process';
import { promisify } from 'util';

const execAsync = promisify(exec);

export async function POST(request: NextRequest) {
  try {
    const { sessionName, command } = await request.json();
    
    if (!sessionName || !command) {
      return NextResponse.json(
        { error: 'Session name and command are required' },
        { status: 400 }
      );
    }

    // Check if session exists
    const checkSessionCommand = `tmux list-sessions | grep -q "^${sessionName}:"`;
    
    try {
      await execAsync(checkSessionCommand);
    } catch (error) {
      return NextResponse.json({
        success: false,
        error: 'Session not found'
      }, { status: 404 });
    }

    // Send command to the tmux session
    const sendCommand = `tmux send-keys -t "${sessionName}" '${command}' Enter`;
    
    await execAsync(sendCommand);
    
    return NextResponse.json({
      success: true,
      message: `Command sent to session ${sessionName}`,
      command: command
    });
    
  } catch (error: any) {
    console.error('Tmux command error:', error);
    
    return NextResponse.json({
      success: false,
      error: error.message
    }, { status: 500 });
  }
}