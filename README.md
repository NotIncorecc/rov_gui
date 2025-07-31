# Robot Control Interface

A modern web-based GUI wrapper for robot control and monitoring, built with Next.js and React. This application provides an intuitive interface for managing robot operations including camera feeds, terminal sessions, and real-time system monitoring.

## Features

- **Real-time Camera Feeds**: Monitor multiple camera streams simultaneously
- **Terminal Management**: Interactive web-based terminals with tmux session integration
- **System Control**: Direct control of robot subsystems (ARM, Drive, Camera, Multijoy)
- **Live Monitoring**: Real-time logs and system status updates
- **Responsive Design**: Works seamlessly across desktop and mobile devices

## Technology Stack

- **Frontend**: Next.js 14+ with App Router
- **Styling**: Tailwind CSS for responsive design
- **Terminal Integration**: tmux for persistent terminal sessions
- **Real-time Updates**: Server-sent events for live data streaming
- **API**: Next.js API routes for backend functionality

## Prerequisites

Before getting started, ensure you have the following installed on your system:

### 1. Install tmux
```bash
# Ubuntu/Debian
sudo apt update && sudo apt install tmux

# CentOS/RHEL/Fedora
sudo yum install tmux
# or for newer versions:
sudo dnf install tmux

# macOS
brew install tmux

# Arch Linux
sudo pacman -S tmux
```

### 2. Install Node.js and npm
```bash
# Install Node.js (version 18 or higher recommended)
# Visit https://nodejs.org/ or use a package manager
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs
```

## Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/NotIncorecc/rov_gui.git
cd rov_gui
```

### 2. Install Dependencies
```bash
npm install
# or
yarn install
# or
pnpm install
```

### 3. Start the Development Server
```bash
npm run dev
# or
yarn dev
# or
pnpm dev
```

### 4. Access the Application
Open [http://localhost:3000](http://localhost:3000) in your browser to see the rover control interface.

## Usage

### Main Dashboard
- **Camera Feeds**: View live feeds from multiple robot cameras
- **Master Control**: Quick access to primary robot functions
- **System Controls**: Manage individual subsystems (ARM, Drive, Camera, Multijoy)

### Terminal Sessions
- Click "Activate [System]" buttons to initialize robot subsystems
- Click "open terminal" to access web-based terminal interfaces
- Each terminal connects to a persistent tmux session for reliable operation
- Send commands directly through the web interface

### Real-time Monitoring
- Monitor system logs in real-time through the web terminals
- Connection status indicators show tmux session health
- Auto-scrolling terminal output for continuous monitoring

## Project Structure

```
my-rover-app/
├── src/
│   ├── app/
│   │   ├── api/                    # API routes
│   │   │   ├── execute-command/    # Command execution endpoint
│   │   │   ├── tmux-logs/         # Tmux log fetching
│   │   │   └── tmux-command/      # Tmux command sending
│   │   ├── terminal/              # Terminal page routes
│   │   │   ├── multijoy-terminal/
│   │   │   ├── arm-ssh-terminal/
│   │   │   ├── drive-ssh-terminal/
│   │   │   └── camera-monitoring-terminal/
│   │   ├── globals.css            # Global styles
│   │   ├── layout.tsx             # Root layout
│   │   └── page.tsx               # Main dashboard
│   └── components/                # Reusable components
├── public/                        # Static assets
├── package.json
└── README.md
```

## Development

### Available Scripts
```bash
npm run dev          # Start development server
npm run build        # Build for production
npm run start        # Start production server
npm run lint         # Run ESLint
```

### Environment Configuration
Create a `.env.local` file for environment-specific variables:
```env
# Add your environment variables here
NEXT_PUBLIC_API_URL=http://localhost:3000
```


## Troubleshooting

### Common Issues

**tmux sessions not found:**
- Ensure tmux is installed and accessible
- Check that the rover initialization commands have been run
- Verify tmux sessions exist: `tmux list-sessions`

**Terminal not connecting:**
- Refresh the page to reinitialize the connection
- Check browser console for error messages
- Ensure API endpoints are responding correctly

**404 errors on terminal pages:**
- Verify the correct file structure in `src/app/terminal/`
- Ensure each terminal route has a `page.tsx` file
- Restart the development server after file structure changes

**Duplicate sessions**
- kill existing sessions by `tmux kill-server`

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature-name`
3. Make your changes and commit: `git commit -m 'Add feature'`
4. Push to the branch: `git push origin feature-name`
5. Submit a pull request

## License

This project is licensed under the GNU GPLv3 License - see the LICENSE file for details.

## Support

For issues and questions:
- Check the troubleshooting section above
- Review the Next.js documentation: https://nextjs.org/docs
- Create an issue in the repository for bugs