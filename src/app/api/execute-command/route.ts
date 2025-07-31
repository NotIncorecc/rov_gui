import { NextRequest, NextResponse } from 'next/server';
import { exec, spawn } from 'child_process';
import { promisify } from 'util';

const execAsync = promisify(exec);

export async function POST(request: NextRequest) {
  try {
    const { command, commands, type } = await request.json();
    
    // Handle multiple commands in terminal
    if (type === 'terminal-multi' && commands && Array.isArray(commands)) {
      console.log(`Opening tmux session with multiple commands:`, commands);
      
      // Create a unique session name based on timestamp
      const sessionName = `rover-session-${Date.now()}`;
      
      // Create tmux session and execute commands
      const commandString = commands.join('; ');
      const tmuxCommand = `tmux new-session -d -s "${sessionName}" -c "$HOME" bash -c "${commandString}; echo 'Commands completed. Terminal session active.'; exec bash"`;
      
      try {
        await execAsync(tmuxCommand);
        
        // Open the tmux session in a new terminal window
        const openTerminalCommand = `gnome-terminal -- tmux attach-session -t "${sessionName}"`;
        await execAsync(openTerminalCommand);
        
        return NextResponse.json({
          success: true,
          stdout: `Tmux session '${sessionName}' created with commands: ${commands.join(', ')}`,
          stderr: '',
          command: tmuxCommand,
          sessionName: sessionName
        });
      } catch (error: any) {
        console.error('Tmux command execution error:', error);
        return NextResponse.json({
          success: false,
          error: error.message,
          stdout: error.stdout || '',
          stderr: error.stderr || '',
          command: tmuxCommand
        }, { status: 500 });
      }
    }
    
    // Handle single command (existing functionality)
    if (!command) {
      return NextResponse.json(
        { error: 'Command is required' },
        { status: 400 }
      );
    }

    console.log(`Executing command: ${command}`);
    
    // Execute the command
    const { stdout, stderr } = await execAsync(command);
    
    return NextResponse.json({
      success: true,
      stdout,
      stderr,
      command
    });
    
  } catch (error: any) {
    console.error('Command execution error:', error);
    
    return NextResponse.json({
      success: false,
      error: error.message,
      stdout: error.stdout || '',
      stderr: error.stderr || '',
      command: error.cmd || ''
    }, { status: 500 });
  }
}
