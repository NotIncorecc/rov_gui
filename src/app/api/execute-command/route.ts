import { NextRequest, NextResponse } from 'next/server';
import { exec, spawn } from 'child_process';
import { promisify } from 'util';

const execAsync = promisify(exec);

export async function POST(request: NextRequest) {
  try {
    const { command, commands, type, buttonName } = await request.json();
    
    // Handle multiple commands in terminal
    if (type === 'terminal-multi' && commands && Array.isArray(commands)) {
      console.log(`Opening tmux session with multiple commands:`, commands);
      
      // Create session name based on button name, sanitized for tmux
      const sanitizedButtonName = (buttonName || 'rover-session').replace(/[^a-zA-Z0-9-_]/g, '-').toLowerCase();
      const sessionName = sanitizedButtonName;
      
      // Create tmux session and execute commands
      const commandString = commands.join('; ');
      
      // First, create the tmux session with a simple shell
      const createSessionCommand = `tmux new-session -d -s "${sessionName}"`;
      
      // Then send the commands to the session
      const sendCommandsCommand = `tmux send-keys -t "${sessionName}" '${commandString}' Enter`;
      
      // Send a final message
      const sendFinalMessage = `tmux send-keys -t "${sessionName}" 'echo "Commands completed. Terminal session active."' Enter`;
      
      try {
        // Create the session
        await execAsync(createSessionCommand);
        
        // Send the commands
        await execAsync(sendCommandsCommand);
        
        // Send final message
        await execAsync(sendFinalMessage);

        return NextResponse.json({
          success: true,
          stdout: `Tmux session '${sessionName}' created with commands: ${commands.join(', ')}`,
          stderr: '',
          command: createSessionCommand,
          sessionName: sessionName
        });
      } catch (error: any) {
        console.error('Tmux command execution error:', error);
        return NextResponse.json({
          success: false,
          error: error.message,
          stdout: error.stdout || '',
          stderr: error.stderr || '',
          command: createSessionCommand
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
