import { NextRequest, NextResponse } from 'next/server';
import { exec, spawn } from 'child_process';
import { promisify } from 'util';

const execAsync = promisify(exec);

export async function POST(request: NextRequest) {
  try {
    const { command, commands, type } = await request.json();
    
    // Handle multiple commands in terminal
    if (type === 'terminal-multi' && commands && Array.isArray(commands)) {
      console.log(`Opening terminal with multiple commands:`, commands);
      
      // Create a script that opens terminal and executes multiple commands
      // The terminal will stay open with an interactive bash session after commands complete
      const commandString = commands.join('; ');
      const terminalCommand = `gnome-terminal -- bash -c "${commandString}; echo 'Commands completed. Terminal session active.'; exec bash"`;
      
      try {
        await execAsync(terminalCommand);
        return NextResponse.json({
          success: true,
          stdout: `Terminal opened with commands: ${commands.join(', ')}`,
          stderr: '',
          command: terminalCommand
        });
      } catch (error: any) {
        console.error('Terminal command execution error:', error);
        return NextResponse.json({
          success: false,
          error: error.message,
          stdout: error.stdout || '',
          stderr: error.stderr || '',
          command: terminalCommand
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
