#!/usr/bin/env python3

"""
Vineyard Environment Monitor
Manages seasonal variations, weather effects, and environmental parameters
for the vineyard simulation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class VineyardEnvironmentMonitor(Node):
    def __init__(self):
        super().__init__('vineyard_environment_monitor')
        
        # Declare parameters
        self.declare_parameter('vineyard_config_file', '')
        self.declare_parameter('current_season', 'summer')
        self.declare_parameter('weather_enabled', False)
        self.declare_parameter('update_rate', 1.0)
        
        # Get parameters
        config_file = self.get_parameter('vineyard_config_file').get_parameter_value().string_value
        self.current_season = self.get_parameter('current_season').get_parameter_value().string_value
        self.weather_enabled = self.get_parameter('weather_enabled').get_parameter_value().bool_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        
        # Load vineyard configuration
        self.vineyard_config = self.load_vineyard_config(config_file)
        
        # Publishers
        self.season_publisher = self.create_publisher(String, '/vineyard/season', 10)
        self.weather_publisher = self.create_publisher(String, '/vineyard/weather', 10)
        self.foliage_density_publisher = self.create_publisher(Float32, '/vineyard/foliage_density', 10)
        self.grape_presence_publisher = self.create_publisher(String, '/vineyard/grape_presence', 10)
        
        # Create timer for periodic updates
        self.timer = self.create_timer(1.0 / update_rate, self.update_environment)
        
        # Environment state
        self.weather_state = "clear"
        self.wind_speed = 0.0
        self.temperature = 20.0
        
        self.get_logger().info(f'Vineyard Environment Monitor started - Season: {self.current_season}')
    
    def load_vineyard_config(self, config_file):
        """Load vineyard configuration from YAML file"""
        try:
            if config_file and os.path.exists(config_file):
                with open(config_file, 'r') as file:
                    config = yaml.safe_load(file)
                    self.get_logger().info(f'Loaded vineyard configuration from {config_file}')
                    return config
            else:
                # Default configuration
                self.get_logger().warn('Using default vineyard configuration')
                return self.get_default_config()
        except Exception as e:
            self.get_logger().error(f'Error loading config file: {e}')
            return self.get_default_config()
    
    def get_default_config(self):
        """Return default vineyard configuration"""
        return {
            'vineyard': {
                'num_rows': 8,
                'row_spacing': 2.5,
                'row_length': 100.0,
                'plant_spacing': 1.5
            },
            'seasons': {
                'current_season': 'summer',
                'foliage_density': {
                    'spring': 0.6,
                    'summer': 1.0,
                    'autumn': 0.8,
                    'winter': 0.2
                },
                'grape_presence': {
                    'spring': False,
                    'summer': True,
                    'autumn': True,
                    'winter': False
                }
            },
            'environment': {
                'ambient_light': 0.4,
                'shadows': True,
                'wind_enabled': False
            }
        }
    
    def update_environment(self):
        """Update environment parameters and publish status"""
        try:
            # Publish current season
            season_msg = String()
            season_msg.data = self.current_season
            self.season_publisher.publish(season_msg)
            
            # Publish foliage density based on season
            if 'seasons' in self.vineyard_config and 'foliage_density' in self.vineyard_config['seasons']:
                foliage_density = self.vineyard_config['seasons']['foliage_density'].get(self.current_season, 1.0)
                foliage_msg = Float32()
                foliage_msg.data = foliage_density
                self.foliage_density_publisher.publish(foliage_msg)
            
            # Publish grape presence
            if 'seasons' in self.vineyard_config and 'grape_presence' in self.vineyard_config['seasons']:
                grape_present = self.vineyard_config['seasons']['grape_presence'].get(self.current_season, False)
                grape_msg = String()
                grape_msg.data = "present" if grape_present else "absent"
                self.grape_presence_publisher.publish(grape_msg)
            
            # Publish weather information if enabled
            if self.weather_enabled:
                weather_msg = String()
                weather_msg.data = self.weather_state
                self.weather_publisher.publish(weather_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error updating environment: {e}')
    
    def change_season(self, new_season):
        """Change the current season"""
        valid_seasons = ['spring', 'summer', 'autumn', 'winter']
        if new_season.lower() in valid_seasons:
            self.current_season = new_season.lower()
            self.get_logger().info(f'Season changed to: {self.current_season}')
        else:
            self.get_logger().warn(f'Invalid season: {new_season}. Valid options: {valid_seasons}')
    
    def set_weather(self, weather_state):
        """Set the current weather state"""
        valid_weather = ['clear', 'cloudy', 'rain', 'fog', 'wind']
        if weather_state.lower() in valid_weather:
            self.weather_state = weather_state.lower()
            self.get_logger().info(f'Weather changed to: {self.weather_state}')
        else:
            self.get_logger().warn(f'Invalid weather state: {weather_state}. Valid options: {valid_weather}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VineyardEnvironmentMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in vineyard environment monitor: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
