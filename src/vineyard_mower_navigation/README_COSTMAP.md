# Vineyard Costmap Generation System

## Overview

The Vineyard Costmap Generation System is an intelligent navigation mapping solution that processes satellite imagery to create ROS2-compatible costmaps for autonomous vineyard navigation. The system combines computer vision techniques with ROS2 navigation stack integration to provide accurate, up-to-date navigation maps for agricultural robots.

## Features

### 1. Satellite Imagery Processing
- **Google Maps API Integration**: Fetch high-resolution satellite imagery using GPS coordinates
- **Local Image Support**: Process local aerial/satellite images 
- **Image Preprocessing**: Automatic contrast enhancement, noise reduction, and filtering
- **Multi-resolution Support**: Handle different image qualities and zoom levels

### 2. Vineyard Structure Detection
- **Vine Row Detection**: Computer vision algorithms to identify parallel vine rows
- **Orientation Calculation**: Determine row orientations and spacing measurements
- **Obstacle Detection**: Identify buildings, equipment, and non-navigable areas
- **Headland Recognition**: Detect turning and access areas at row ends

### 3. Costmap Generation
- **ROS2 Compatibility**: Generate standard OccupancyGrid messages
- **Multi-layer System**: Separate layers for vine rows, obstacles, headlands, and inflation
- **Dynamic Resolution**: Configurable costmap resolution and dimensions
- **Real-time Updates**: Support for live costmap modifications

### 4. Service Interface
- **ROS2 Services**: Generate, update, and query costmaps via standard ROS2 services
- **Parameter Management**: Runtime configuration of detection and generation parameters
- **Status Monitoring**: Real-time status updates and error reporting

## Architecture

```
┌─────────────────────┐    ┌──────────────────────┐    ┌─────────────────────┐
│   Satellite API     │───▶│  Image Processing    │───▶│  Vineyard Detection │
│   (Google Maps)     │    │  & Preprocessing     │    │   (Computer Vision) │
└─────────────────────┘    └──────────────────────┘    └─────────────────────┘
                                                                     │
┌─────────────────────┐    ┌──────────────────────┐                │
│   ROS2 Navigation   │◀───│  Costmap Generation  │◀───────────────┘
│      Stack          │    │   (Multi-layer)      │
└─────────────────────┘    └──────────────────────┘
                                                   
┌─────────────────────┐    ┌──────────────────────┐
│  Service Interface  │◀───│   Configuration      │
│  (ROS2 Services)    │    │   Management         │
└─────────────────────┘    └──────────────────────┘
```

## Installation

### Prerequisites

1. **ROS2** (Humble or later)
2. **Python 3.8+** with the following packages:
   - opencv-python
   - numpy
   - requests
   - pillow
   - pyyaml
3. **Google Maps API Key** (optional, for satellite imagery)

### Build Instructions

```bash
# Navigate to your ROS2 workspace
cd /path/to/your/ros2_ws

# Build the interfaces package first
colcon build --packages-select vineyard_mower_interfaces

# Source the workspace
source install/setup.bash

# Build the navigation package
colcon build --packages-select vineyard_mower_navigation

# Source again
source install/setup.bash
```

## Configuration

### Main Configuration File

The system is configured via `config/costmap_config.yaml`:

```yaml
# Costmap dimensions and resolution
resolution: 0.1  # meters per pixel
width: 1000      # cells
height: 1000     # cells

# Cost values (0-100)
vine_row_cost: 90
obstacle_cost: 100
headland_cost: 30

# Detection parameters
vineyard_detection:
  hough_threshold: 50
  min_row_length: 50
  pixel_to_meter_ratio: 0.1
```

### Environment Variables

```bash
# Google Maps API key (optional)
export GOOGLE_MAPS_API_KEY="your_api_key_here"
```

## Usage

### 1. Launch the Costmap Generation System

```bash
ros2 launch vineyard_mower_navigation costmap_generation.launch.py
```

Parameters:
- `google_api_key`: Your Google Maps API key
- `config_file`: Path to configuration file
- `resolution`: Costmap resolution (meters per pixel)
- `save_costmaps`: Whether to save generated costmaps

### 2. Generate Costmap from GPS Coordinates

```bash
# Using the test client
ros2 run vineyard_mower_navigation test_costmap_client \
  --mode coordinates \
  --lat 37.7749 \
  --lon -122.4194 \
  --zoom 18
```

### 3. Generate Costmap from Local Image

```bash
# First generate a test image
ros2 run vineyard_mower_navigation generate_test_images \
  --output-dir /tmp/test_images \
  --type realistic

# Then process it
ros2 run vineyard_mower_navigation test_costmap_client \
  --mode local \
  --image /tmp/test_images/realistic_vineyard.jpg
```

### 4. Service Interface

The system provides several ROS2 services:

- **`/generate_costmap`**: Generate new costmap from imagery
- **`/get_costmap_info`**: Get current costmap information
- **`/update_costmap_layer`**: Update specific costmap layers
- **`/clear_costmap`**: Clear the current costmap
- **`/enable_auto_update`**: Enable/disable automatic updates

#### Example Service Calls

```bash
# Get costmap information
ros2 service call /get_costmap_info vineyard_mower_interfaces/srv/GetCostmapInfo

# Generate costmap from coordinates
ros2 service call /generate_costmap vineyard_mower_interfaces/srv/GenerateCostmap \
  "{center_latitude: 37.7749, center_longitude: -122.4194, zoom_level: 18, use_local_image: false, image_path: ''}"

# Clear current costmap
ros2 service call /clear_costmap std_srvs/srv/Empty
```

## Topics

### Published Topics

- **`/global_costmap`** (`nav_msgs/OccupancyGrid`): Global costmap for path planning
- **`/local_costmap`** (`nav_msgs/OccupancyGrid`): Local costmap for obstacle avoidance
- **`/costmap_status`** (`std_msgs/String`): System status updates

### Subscribed Topics

The system can be extended to subscribe to:
- Sensor data for real-time obstacle detection
- GPS coordinates for automatic map updates
- Camera feeds for visual odometry

## Testing

### Unit Tests

```bash
# Run unit tests
cd /path/to/ros2_ws
python3 src/vineyard_mower_navigation/test/test_costmap_generation.py
```

### Integration Testing

```bash
# Generate test images
ros2 run vineyard_mower_navigation generate_test_images --type all

# Test with different image types
for image in /tmp/test_vineyard_images/*.jpg; do
  echo "Testing with $image"
  ros2 run vineyard_mower_navigation test_costmap_client \
    --mode local --image "$image"
done
```

## Parameter Tuning

### Detection Parameters

1. **Hough Transform Parameters**:
   - `hough_threshold`: Lower values detect more lines (default: 50)
   - `hough_min_line_length`: Minimum line length in pixels (default: 100)
   - `hough_max_line_gap`: Maximum gap in line segments (default: 20)

2. **Edge Detection**:
   - `canny_low_threshold`: Lower edge detection threshold (default: 50)
   - `canny_high_threshold`: Higher edge detection threshold (default: 150)

3. **Row Grouping**:
   - `angle_tolerance`: Tolerance for parallel line grouping (default: 0.17 rad)
   - `max_row_spacing`: Maximum expected row spacing (default: 5.0m)

### Costmap Parameters

1. **Resolution**: Balance between detail and computational cost
2. **Cost Values**: Adjust based on robot safety requirements
3. **Inflation Radius**: Set based on robot dimensions and safety margins

## Troubleshooting

### Common Issues

1. **No lines detected**:
   - Increase image preprocessing
   - Lower Hough transform thresholds
   - Check image quality and contrast

2. **Too many false positives**:
   - Increase Hough thresholds
   - Improve edge detection parameters
   - Use bilateral filtering

3. **Incorrect row spacing**:
   - Calibrate `pixel_to_meter_ratio`
   - Verify GPS coordinates accuracy
   - Check image zoom level

4. **API errors**:
   - Verify Google Maps API key
   - Check network connectivity
   - Monitor API quota usage

### Debug Mode

Enable detailed logging:

```bash
ros2 launch vineyard_mower_navigation costmap_generation.launch.py \
  --ros-args --log-level debug
```

## Performance Considerations

### Computational Requirements

- **CPU**: Multi-core recommended for image processing
- **Memory**: 4GB+ RAM for large images
- **Storage**: Space for cached images and costmaps

### Optimization Tips

1. **Image Size**: Use appropriate zoom levels
2. **Update Frequency**: Balance between accuracy and performance
3. **Caching**: Cache processed images and results
4. **Parallel Processing**: Use multiple threads for large areas

## Integration with Navigation Stack

The generated costmaps are compatible with:

- **Nav2 Stack**: Direct integration with global and local costmaps
- **Move Base**: Classic ROS navigation stack
- **Custom Planners**: Standard OccupancyGrid format

### Example Nav2 Integration

```yaml
# nav2_params.yaml
global_costmap:
  global_costmap:
    ros__parameters:
      # ... other parameters ...
      plugins: ["static_layer", "vineyard_layer"]
      vineyard_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
        map_topic: "/global_costmap"
```

## Future Enhancements

### Planned Features

1. **Machine Learning Integration**: Deep learning for improved detection
2. **Seasonal Adaptation**: Handle growth patterns and seasonal changes
3. **Multi-spectral Imagery**: Support for NDVI and other agricultural indices
4. **3D Mapping**: Integration with LiDAR for elevation mapping
5. **Collaborative Mapping**: Multi-robot map sharing and updates

### API Extensions

1. **Batch Processing**: Handle multiple fields simultaneously
2. **Temporal Analysis**: Track changes over time
3. **Weather Integration**: Adjust for weather conditions
4. **Crop Monitoring**: Integrate with precision agriculture systems

## Contributing

### Development Setup

1. Fork the repository
2. Create a feature branch
3. Follow ROS2 coding standards
4. Add tests for new functionality
5. Submit a pull request

### Code Style

- Follow PEP 8 for Python code
- Use type hints where appropriate
- Document all public functions
- Include unit tests for new features

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Support

For issues and questions:

1. Check the troubleshooting section
2. Review existing GitHub issues
3. Create a new issue with detailed description
4. Include log files and configuration details

## References

1. [ROS2 Navigation Stack](https://navigation.ros.org/)
2. [OpenCV Documentation](https://docs.opencv.org/)
3. [Google Maps Static API](https://developers.google.com/maps/documentation/maps-static)
4. [Agricultural Robotics](https://www.springer.com/journal/11119)
