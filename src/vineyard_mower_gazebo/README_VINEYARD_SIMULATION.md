# Vineyard Simulation Environment

This package provides a comprehensive and realistic vineyard simulation environment in Gazebo for testing autonomous navigation systems. The simulation includes detailed vineyard structures, terrain features, static obstacles, and environmental elements.

## Features

### 1. Vineyard Structure
- **Configurable vineyard rows**: Adjustable spacing (default: 2.5m between rows)
- **Variable row lengths**: Customizable from 50-200m (default: 100m)
- **Realistic vine models**: Seasonal foliage variations with grape clusters
- **Trellis systems**: Posts with multi-level wire supports
- **End-of-row areas**: Headlands for turning maneuvers

### 2. Terrain Features
- **Realistic ground textures**: Soil, grass, and gravel materials
- **Terrain variations**: Subtle slopes and height changes
- **Multi-material surfaces**: Different friction coefficients
- **Grass strips**: Between vineyard rows for navigation testing

### 3. Static Obstacles
- **Vineyard posts**: Support structures with trellis wires
- **Irrigation systems**: Drip lines and emitters
- **Storage buildings**: Equipment storage facilities
- **Access gates**: Entry/exit points with posts

### 4. Environmental Elements
- **Dynamic lighting**: Configurable day/night conditions
- **Weather effects**: Optional weather simulation
- **GPS reference points**: For navigation testing
- **Visual markers**: Navigation aids and waypoints

## Files Structure

```
vineyard_mower_gazebo/
├── config/
│   └── vineyard_config.yaml          # Configuration parameters
├── launch/
│   ├── vineyard_simulation.launch.py # Main launch file
│   └── robot_simulation_navigation.launch.py # Updated launch file
├── models/
│   ├── vineyard_post/                # Trellis post model
│   ├── vine_plant/                   # Vine plant with foliage
│   ├── irrigation_pipe/              # Irrigation system
│   └── storage_building/             # Storage structures
├── scripts/
│   ├── vineyard_environment_monitor.py # Environment management
│   └── generate_vineyard_world.py    # World generation tool
└── worlds/
    ├── vineyard.world                # Original simple world
    └── realistic_vineyard.world      # Comprehensive vineyard world
```

## Quick Start

### 1. Launch the Vineyard Simulation

```bash
# Basic vineyard simulation with SLAM
ros2 launch vineyard_mower_gazebo vineyard_simulation.launch.py

# With specific season and teleoperation
ros2 launch vineyard_mower_gazebo vineyard_simulation.launch.py \
    vineyard_season:=autumn \
    enable_teleop:=true \
    enable_slam:=true \
    enable_rviz:=true

# Custom robot starting position
ros2 launch vineyard_mower_gazebo vineyard_simulation.launch.py \
    x_pose:=10.0 \
    y_pose:=-5.0
```

### 2. Generate Custom Vineyard Worlds

```bash
# Generate world with default settings
cd /path/to/vineyard_mower_gazebo/scripts
python3 generate_vineyard_world.py -o my_vineyard.world

# Custom vineyard configuration
python3 generate_vineyard_world.py \
    --rows 10 \
    --spacing 3.0 \
    --length 150.0 \
    --output large_vineyard.world

# Using configuration file
python3 generate_vineyard_world.py \
    --config ../config/vineyard_config.yaml \
    --output configured_vineyard.world
```

## Configuration

### Vineyard Parameters (`vineyard_config.yaml`)

```yaml
vineyard:
  num_rows: 8              # Number of vineyard rows
  row_spacing: 2.5         # Distance between rows (meters)
  row_length: 100.0        # Length of each row (meters)
  headland_width: 8.0      # Width of turning areas (meters)
  plant_spacing: 1.5       # Distance between plants (meters)
  slope: 2.0              # Vineyard slope (degrees)

infrastructure:
  post_spacing: 8.0        # Distance between posts (meters)
  trellis_levels: 3        # Number of wire levels
  irrigation_enabled: true # Enable irrigation system
  irrigation_spacing: 15.0 # Distance between irrigation points

seasons:
  current_season: "summer" # spring, summer, autumn, winter
  foliage_density:
    spring: 0.6
    summer: 1.0
    autumn: 0.8
    winter: 0.2
  grape_presence:
    spring: false
    summer: true
    autumn: true
    winter: false
```

### Environment Settings

```yaml
environment:
  ambient_light: 0.4       # Ambient lighting (0.0-1.0)
  shadows: true           # Enable shadows
  background_color: [0.7, 0.8, 0.9, 1.0]  # Sky color [R,G,B,A]
  wind_enabled: false     # Wind effects

navigation:
  gps_markers:            # GPS reference points
    - name: "start_point"
      position: [0, 0, 0]
    - name: "row_1_start"
      position: [0, -10, 0]
  visual_markers_enabled: true
  marker_spacing: 25.0    # Distance between markers
```

## Launch File Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_sim_time` | `true` | Use simulation clock |
| `x_pose` | `0.0` | Robot starting X position |
| `y_pose` | `0.0` | Robot starting Y position |
| `enable_teleop` | `true` | Enable teleoperation |
| `enable_slam` | `true` | Enable SLAM mapping |
| `enable_rviz` | `true` | Enable RViz visualization |
| `enable_navigation` | `false` | Enable Nav2 navigation |
| `vineyard_season` | `summer` | Current season |
| `weather_enabled` | `false` | Enable weather effects |

## Models and Components

### Vineyard Post Model
- **Dimensions**: 0.1m diameter, 2.0m height
- **Materials**: Wood-textured cylinder
- **Features**: Three trellis wire levels at 0.5m, 1.0m, and 1.5m heights
- **Collision**: Accurate for navigation planning

### Vine Plant Model
- **Components**: Trunk, foliage clusters, seasonal grape bunches
- **Variations**: Seasonal changes in foliage density and grape presence
- **Materials**: Realistic textures for trunk (brown) and foliage (green)
- **Size**: Variable height 0.6-1.2m depending on season

### Irrigation System
- **Components**: Main pipe, drip emitters, support stakes
- **Layout**: Parallel to vine rows with regular spacing
- **Materials**: Black plastic pipes with metallic emitters
- **Function**: Provides realistic obstacles for navigation

### Storage Buildings
- **Structure**: Foundation, walls, roof, door frame
- **Materials**: Concrete foundation, wooden walls, metal roof
- **Size**: 8m x 6m x 3m (configurable)
- **Purpose**: Realistic farm infrastructure for testing

## Environment Monitoring

The `vineyard_environment_monitor.py` script provides:

- **Seasonal management**: Automatic foliage and grape adjustments
- **Weather simulation**: Optional weather state publishing
- **Environment status**: Real-time parameter monitoring
- **ROS topics**: 
  - `/vineyard/season` - Current season
  - `/vineyard/foliage_density` - Foliage density factor
  - `/vineyard/grape_presence` - Grape availability
  - `/vineyard/weather` - Weather state

## Navigation Testing Features

### GPS Integration
- Reference coordinates for absolute positioning
- Realistic GPS coordinate simulation
- Integration with navigation systems

### Visual Markers
- Orange navigation markers at regular intervals
- Visible landmarks for visual navigation
- Configurable spacing and placement

### Row Detection
- Clear row structure for line-following algorithms
- Consistent spacing for navigation planning
- Visual and LiDAR-detectable features

### Obstacle Avoidance
- Posts, irrigation equipment, and buildings as obstacles
- Varying obstacle densities for different difficulty levels
- Realistic collision geometries

## Troubleshooting

### Common Issues

1. **Models not loading**
   - Ensure `GAZEBO_MODEL_PATH` includes the models directory
   - Check model.config and model.sdf files are present

2. **Performance issues**
   - Reduce number of vine plants for better performance
   - Adjust physics update rate in world file
   - Disable shadows if needed

3. **Sensor integration**
   - Verify ros_gz_bridge is properly configured
   - Check topic remappings in launch file
   - Ensure sensor frame IDs match robot URDF

### Performance Optimization

```bash
# Reduce visual complexity
export GZ_SIM_RESOURCE_PATH=/path/to/low_poly_models

# Adjust physics parameters
# Edit realistic_vineyard.world:
<max_step_size>0.002</max_step_size>  # Increase for better performance
<real_time_factor>0.8</real_time_factor>  # Reduce if needed
```

## Extension Points

### Adding New Models
1. Create model directory in `models/`
2. Add `model.config` and `model.sdf`
3. Include in world generation script
4. Update launch file if needed

### Seasonal Variations
1. Modify `vineyard_environment_monitor.py`
2. Add seasonal parameters to config file
3. Implement model variations in SDF files
4. Update visualization in RViz

### Weather Effects
1. Enable weather in configuration
2. Implement weather state machine
3. Add atmospheric effects to world file
4. Integrate with sensor simulation

## Contributing

When adding new features:
1. Update configuration schema
2. Maintain backward compatibility
3. Add documentation and examples
4. Test with different vineyard configurations

## License

This vineyard simulation is part of the vineyard mower project and follows the same license terms.
