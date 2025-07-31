import { NextRequest, NextResponse } from 'next/server';
import { exec } from 'child_process';
import { promisify } from 'util';

const execAsync = promisify(exec);

export async function POST(request: NextRequest) {
  try {
    const { sessionName } = await request.json();
    
    if (!sessionName) {
      return NextResponse.json(
        { error: 'Session name is required' },
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
        error: 'Session not found',
        logs: ''
      });
    }

    // Capture the pane content (last 1000 lines)
    const captureCommand = `tmux capture-pane -t "${sessionName}" -p -S -1000`;
    
    const { stdout } = await execAsync(captureCommand);
    
    return NextResponse.json({
      success: true,
      logs: stdout,
      sessionName: sessionName
    });
    
  } catch (error: any) {
    console.error('Tmux logs error:', error);
    
    return NextResponse.json({
      success: false,
      error: error.message,
      logs: ''
    }, { status: 500 });
  }
}