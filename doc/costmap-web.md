Vineyard Costmap Web Application Demo
==============================================

System Overview:
===================
✅ React Frontend with TypeScript and Material-UI
✅ FastAPI Backend with PostgreSQL database
✅ ROS2 Bridge integration via WebSocket
✅ Docker containerization for easy deployment
✅ Real-time costmap visualization with Leaflet
✅ Parameter control panels for detection tuning
✅ Processing job monitoring and management
✅ Satellite imagery upload and management
✅ User authentication and role-based access

Architecture Components:
=============================
Frontend (React + TypeScript):
  - Dashboard with system status
  - Interactive costmap viewer
  - Satellite image management
  - Parameter control panels
  - Processing job monitoring
  - Mission planning interface

Backend (FastAPI + Python):
  - Authentication and user management
  - File upload and storage
  - ROS2 service integration
  - Job queue management
  - Parameter preset storage

ROS2 Integration:
  - WebSocket bridge via rosbridge_server
  - Service calls to costmap generation
  - Real-time topic subscriptions
  - System status monitoring

Docker Services:
===================
Services defined in docker-compose.yml:
  PostgreSQL Database (port 5432)
  Redis Cache (port 6379)
  Backend API Server (port 8000)
  Frontend React App (port 3000)
  ROSbridge WebSocket Server (port 9090)

Project Structure:
=====================
vineyard_costmap_web/
├── frontend/                 # React TypeScript application
│   ├── src/
│   │   ├── components/      # Reusable UI components
│   │   │   ├── costmap/     # Costmap visualization
│   │   │   └── parameters/  # Parameter control panels
│   │   ├── services/        # API and ROS service clients
│   │   ├── hooks/           # Custom React hooks
│   │   ├── types/           # TypeScript type definitions
│   │   └── pages/           # Application pages
│   ├── public/              # Static assets
│   └── package.json         # Frontend dependencies
├── backend/                 # FastAPI Python server
│   ├── src/                 # Python source code
│   └── requirements.txt     # Python dependencies
├── docker-compose.yml       # Multi-service deployment
└── README.md                # Documentation

🔧 Key Features Implemented:
=============================

1. 📊 Real-time Dashboard:
   - System status monitoring
   - Active job tracking
   - Service availability display
   - Connection status indicators

2. 🗺️  Interactive Costmap Viewer:
   - Leaflet-based map display
   - Multi-layer visualization
   - Zoom and pan controls
   - Layer opacity adjustment
   - Cost value color coding

3. 🖼️  Satellite Image Management:
   - Drag-and-drop upload interface
   - GPS coordinate input
   - Image preview and metadata
   - Batch processing support

4. ⚙️  Parameter Control System:
   - Row detection parameters
   - Costmap generation settings
   - Processing configuration
   - Preset save/load functionality

5. 📈 Processing Job Monitor:
   - Real-time progress tracking
   - Job status visualization
   - Error reporting and logging
   - Queue management

6. 🌉 ROS2 Integration:
   - WebSocket communication
   - Service call handlers
   - Topic subscriptions
   - Connection management

🚀 Getting Started:
===================

1. Prerequisites:
   ✅ Docker and Docker Compose
   ✅ Node.js 18+ (for development)
   ✅ Python 3.11+ (for development)
   ✅ ROS2 Jazzy (for ROS integration)

2. Quick Start with Docker:
   $ cd vineyard_costmap_web
   $ docker-compose up -d
   $ open http://localhost:3000

3. Development Setup:
   # Frontend
   $ cd frontend
   $ npm install
   $ npm run dev
   
   # Backend
   $ cd backend
   $ pip install -r requirements.txt
   $ python src/main.py
   
   # ROS2 Bridge
   $ ros2 launch rosbridge_server rosbridge_websocket_launch.xml

🔌 API Endpoints:
=================
Authentication:
  POST /api/auth/login         # User login
  GET  /api/auth/me            # Current user info

Satellite Images:
  POST /api/satellite-images   # Upload image
  GET  /api/satellite-images   # List images
  DELETE /api/satellite-images/{id}  # Delete image

Costmaps:
  GET  /api/costmaps           # List costmaps
  GET  /api/costmaps/{id}      # Get costmap
  GET  /api/costmaps/{id}/export # Export costmap

Processing Jobs:
  GET  /api/processing-jobs    # List jobs
  POST /api/processing-jobs/{id}/cancel # Cancel job

Parameters:
  GET  /api/parameters/presets # List presets
  POST /api/parameters/presets # Save preset

System:
  GET  /api/system/status      # System status

🌐 Web Interface Features:
==========================

Dashboard Page:
  📊 System status cards
  📈 Processing job progress
  🔌 Service availability
  📱 Responsive design

Costmap Viewer:
  🗺️  Interactive map with zoom/pan
  🎨 Layer visibility controls
  📏 Opacity adjustment sliders
  📍 Coordinate display
  📊 Costmap information panel

Image Upload:
  📁 Drag-and-drop interface
  🌍 GPS bounds input
  👁️  Image preview
  📋 Metadata display

Parameter Control:
  🔧 Real-time parameter adjustment
  💾 Preset save/load
  🔄 Default value reset
  ✅ Parameter validation

🔗 Integration with Existing ROS2 System:
==========================================

The web application integrates with your existing ROS2 costmap system:

✅ Uses vineyard_mower_interfaces services:
   - GenerateCostmap.srv
   - UpdateCostmapLayer.srv
   - GetCostmapInfo.srv

✅ Connects to your costmap generation nodes:
   - satellite_processor.py
   - costmap_generator.py
   - costmap_service.py

✅ Subscribes to ROS2 topics:
   - /costmap/updates (nav_msgs/OccupancyGrid)
   - /costmap/job_updates (std_msgs/String)

✅ Compatible with your configuration:
   - Uses same parameter structure
   - Respects costmap format
   - Maintains ROS2 conventions

🎯 Use Cases:
==============

1. 🧑‍🌾 Farm Operator:
   - Upload satellite imagery of vineyard
   - Adjust detection parameters visually
   - Generate costmaps for navigation
   - Monitor processing progress
   - Export maps for field use

2. 🔬 Agricultural Researcher:
   - Compare different parameter settings
   - Analyze detection algorithm performance
   - Batch process multiple field images
   - Export data for analysis

3. 🤖 System Administrator:
   - Monitor ROS2 system health
   - Manage processing job queues
   - Configure system parameters
   - Troubleshoot connectivity issues

📝 Next Steps:
===============

To complete the implementation:

1. 🏗️  Build the interfaces package:
   $ cd ../src/vineyard_mower_interfaces
   $ colcon build --packages-select vineyard_mower_interfaces

2. 🚀 Start the web application:
   $ cd vineyard_costmap_web
   $ docker-compose up -d

3. 🔗 Launch ROS2 costmap services:
   $ ros2 launch vineyard_mower_navigation costmap_generation.launch.py

4. 🌐 Access the web interface:
   $ open http://localhost:3000

5. 🧪 Test the integration:
   - Upload a satellite image
   - Adjust detection parameters
   - Generate a costmap
   - View results in the map viewer

✨ The complete React-based web application is now ready!
🍇 Happy vineyard navigation! 🍇