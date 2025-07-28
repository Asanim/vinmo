# Realistic Vineyard Simulation Environment

## Overview
This vineyard simulation provides a realistic testing environment for autonomous navigation systems in agricultural settings. The world includes structured vineyard rows, realistic terrain features, infrastructure, and environmental elements.

## Features Implemented

### 1. Vineyard Structure
- **8 vineyard rows** with 2.5m spacing between rows
- **Vineyard posts** every 8 meters along each row
- **Vine plants** spaced at 1.5m intervals
- **Trellis systems** with multiple wire levels
- **Headland areas** for turning at row ends

### 2. Terrain Features
- **Realistic ground textures** with soil and grass materials
- **Grass strips** between vineyard rows
- **Access roads** for equipment movement
- **Drainage ditches** for water management
- **Headland areas** for end-of-row operations

### 3. Infrastructure
- **Storage building** for equipment and supplies
- **Equipment shed** for tool storage
- **Water tank** for irrigation system
- **Tool shed** for maintenance equipment
- **Compost area** for organic waste management
- **Equipment parking pad** with concrete surface
- **Irrigation control box** for automated watering
- **Weather station** for environmental monitoring

### 4. Environmental Elements
- **Advanced lighting system** with sun, ambient, and spot lights
- **Wind effects system** for realistic environmental conditions
- **Proper shadows** and lighting conditions
- **Realistic sky** with cloud effects
- **Boundary fencing** around the vineyard perimeter

### 5. Navigation Aids
- **GPS reference markers** at key locations
- **Row start/end markers** (green for start, red for end)
- **Waypoint markers** (blue spheres) for autonomous navigation
- **Gate posts** at access points
- **Drainage systems** for obstacle navigation

## Model Dependencies
The world uses custom models located in:
- `vineyard_post`: Trellis posts with wire systems
- `vine_plant`: Realistic vine plants with seasonal variations
- `irrigation_pipe`: Drip irrigation systems
- `storage_building`: Equipment storage structures

## Configuration Parameters

### Vineyard Dimensions
- **Total area**: 200m x 120m
- **Row length**: 100m
- **Row spacing**: 2.5m
- **Plant spacing**: 1.5m within rows
- **Post spacing**: 8m along rows

### Environmental Settings
- **Ambient light**: 0.4 (40% brightness)
- **Wind effects**: Enabled with realistic variations
- **Shadows**: Enabled for realistic lighting
- **Background**: Sky blue (0.7, 0.8, 0.9)

## Usage Instructions

### 1. Launch the Vineyard Simulation

```bash

cd /home/sam/vinmo/src/vineyard_mower_gazebo && python scripts/generate_vineyard_world.py --from-scratch --output realistic_vineyard.world --rows 6 --spacing 2.0 --length 20
```

```bash
ros2 launch vineyard_mower_gazebo vineyard_simulation.launch.py
```

### 2. Available Launch Parameters
- `use_sim_time`: Use simulation time (default: true)
- `x_pose`: Initial robot X position (default: 0.0)
- `y_pose`: Initial robot Y position (default: 0.0)
- `vineyard_season`: Current season (spring/summer/autumn/winter)
- `weather_enabled`: Enable weather effects (default: false)

### 3. Navigation Waypoints
Key navigation points for autonomous operation:
- **Start point**: (0, 0, 0) - Main GPS marker
- **Row 1 start**: (0, -10, 0) - Green marker
- **Row 1 end**: (100, -10, 0) - Red marker
- **Waypoints**: (25, -10), (50, -10), (75, -10) - Blue markers

## Testing Scenarios

### 1. Row Following
- Navigate along vineyard rows using laser scan data
- Test obstacle avoidance with vine plants and posts
- Validate turning at row ends in headland areas

### 2. Obstacle Navigation
- Navigate around irrigation equipment
- Avoid temporary obstacles (weeds, maintenance equipment)
- Handle weather station and control box obstacles

### 3. Infrastructure Interaction
- Navigate to storage buildings for loading/unloading
- Access water tank for refilling operations
- Use parking pad for equipment staging

### 4. Environmental Testing
- Test under different lighting conditions
- Validate performance with wind effects
- Navigate drainage ditches and terrain variations

## Customization Options

### 1. Seasonal Variations
Modify the `vineyard_season` parameter to change:
- Foliage density on vine plants
- Grape presence/absence
- Lighting conditions

### 2. Weather Effects
Enable `weather_enabled` for:
- Wind effects on vegetation
- Variable lighting conditions
- Atmospheric effects

### 3. Adding More Rows
To add additional vineyard rows:
1. Copy existing row model structure
2. Adjust Y position by row_spacing (2.5m)
3. Update model names to avoid conflicts

### 4. Infrastructure Modifications
- Adjust building positions in world file
- Add/remove irrigation systems
- Modify fence boundaries

## Performance Considerations

### Optimization Settings
- **Physics step size**: 0.001s for accuracy
- **Real-time factor**: 1.0 for real-time simulation
- **Update rate**: 250Hz for responsive physics

### Resource Usage
- **Recommended RAM**: 8GB minimum
- **GPU**: Dedicated graphics recommended for complex lighting
- **CPU**: Multi-core processor for physics calculations

## Troubleshooting

### Common Issues
1. **Model loading errors**: Ensure all custom models are built and installed
2. **Performance issues**: Reduce visual complexity or disable shadows
3. **Navigation problems**: Check TF frames and sensor configurations

### Debug Features
- **RViz visualization**: View sensor data and navigation planning
- **Topic monitoring**: Monitor `/scan`, `/cmd_vel`, and other key topics
- **Diagnostic system**: Check system health and sensor status

## Future Enhancements

### Planned Features
- **Dynamic weather system** with rain, fog, and wind variations
- **Seasonal plant growth** simulation
- **Equipment models** (tractors, sprayers, harvesters)
- **Pest and disease** visual indicators
- **Harvest simulation** with grape clusters

### Extension Points
- **Custom sensor models** for specialized agricultural sensors
- **Multi-robot scenarios** for collaborative operations
- **Crop yield simulation** for harvest planning
- **Soil condition modeling** for precision agriculture

## License and Credits
This vineyard simulation environment is part of the Vineyard Mower project and is designed for research and educational purposes in autonomous agricultural robotics.
