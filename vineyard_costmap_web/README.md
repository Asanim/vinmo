# Vineyard Costmap Web Application

A React-based web application for managing vineyard costmap generation, integrating with ROS2 costmap services for satellite imagery processing and navigation.

## Features

- **Real-time Costmap Visualization**: Interactive display of satellite imagery and generated costmaps
- **Parameter Control**: Web interface for adjusting detection and generation parameters
- **Mission Planning**: Interface for creating and managing costmap generation tasks
- **Progress Monitoring**: Real-time tracking of image processing and costmap generation
- **Batch Processing**: Queue multiple satellite images for processing
- **Export Functionality**: Download generated costmaps in various formats

## Architecture

- **Frontend**: React with TypeScript and Material-UI
- **Backend**: Node.js API server with ROS2 bridge
- **Communication**: WebSocket via rosbridge_server
- **Database**: PostgreSQL for user management and mission storage

## Quick Start

### Prerequisites

- Node.js 18+
- Python 3.10+
- ROS2 Jazzy
- Docker (optional)

### Installation

```bash
# Clone and setup
cd vineyard_costmap_web

# Install frontend dependencies
cd frontend
npm install

# Install backend dependencies
cd ../backend
pip install -r requirements.txt

# Start services
docker-compose up -d
```

### Development

```bash
# Start frontend development server
cd frontend
npm run dev

# Start backend development server
cd backend/src 
python main.py

# Start ROS2 bridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## Project Structure

```
vineyard_costmap_web/
├── frontend/                 # React application
│   ├── src/
│   │   ├── components/      # UI components
│   │   ├── services/        # ROS2 and API services
│   │   ├── utils/           # Utility functions
│   │   ├── types/           # TypeScript definitions
│   │   └── hooks/           # Custom React hooks
│   ├── public/              # Static assets
│   └── package.json
├── backend/                 # Node.js API server
│   ├── src/
│   │   ├── routes/          # API routes
│   │   ├── services/        # Business logic
│   │   ├── models/          # Data models
│   │   └── middleware/      # Express middleware
│   ├── ros_bridge/          # ROS2 integration
│   └── requirements.txt
├── docker-compose.yml       # Development environment
└── README.md
```

## Integration with ROS2

The application integrates with the following ROS2 services:

- `/costmap/generate_costmap` - Generate costmap from satellite imagery
- `/costmap/update_costmap_layer` - Update specific costmap layers
- `/costmap/get_costmap_info` - Get costmap status and information

## API Documentation

See `backend/docs/api.md` for detailed API documentation.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Submit a pull request

## License

MIT License
