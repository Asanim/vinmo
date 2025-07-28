#!/usr/bin/env python3

"""
Vineyard World Generator
Generates customized vineyard world files based on configuration parameters.
This script takes an existing world file with infrastructure and environmental 
features, removes any existing vineyard rows, and inserts new parametrically 
generated vineyard rows based on user configuration.

Usage:
    python generate_vineyard_world.py --config vineyard_config.yaml --output custom_vineyard.world
    python generate_vineyard_world.py --rows 10 --spacing 3.0 --length 120
"""

import os
import yaml
import argparse
import re
from pathlib import Path

class VineyardWorldGenerator:
    def __init__(self, config_file=None, base_world_file=None):
        self.config = self.load_config(config_file)
        self.base_world_file = base_world_file or self.find_base_world_file()
        self.models_path = self.find_models_path()
        
    def find_base_world_file(self):
        """Find the base world file with all infrastructure"""
        script_dir = Path(__file__).parent
        world_file = script_dir.parent / "worlds" / "realistic_vineyard.world"
        if world_file.exists():
            return str(world_file)
        return None
    
    def find_models_path(self):
        """Find the absolute path to the models directory"""
        script_dir = Path(__file__).parent
        models_dir = script_dir.parent / "models"
        return str(models_dir.absolute())
        
    def load_config(self, config_file):
        """Load configuration from file or use defaults"""
        if config_file and os.path.exists(config_file):
            with open(config_file, 'r') as f:
                return yaml.safe_load(f)
        else:
            return self.get_default_config()
    
    def get_default_config(self):
        """Default vineyard configuration"""
        return {
            'vineyard': {
                'num_rows': 8,
                'row_spacing': 2.5,
                'row_length': 100.0,
                'plant_spacing': 1.5,
                'headland_width': 8.0,
                'slope': 2.0
            },
            'infrastructure': {
                'post_spacing': 8.0,
                'irrigation_enabled': True,
                'irrigation_spacing': 5.0,
                'storage_buildings': 1,
                'equipment_sheds': 2,
                'water_tank': True,
                'tool_shed': True,
                'compost_area': True,
                'parking_pad': True,
                'weather_station': True,
                'irrigation_control': True,
                'boundary_fence': True
            },
            'environment': {
                'ambient_light': 0.4,
                'shadows': True,
                'background_color': [0.7, 0.8, 0.9, 1.0],
                'wind_enabled': True,
                'spot_lights': True,
                'advanced_lighting': True
            },
            'terrain': {
                'headlands': True,
                'access_roads': True,
                'drainage_ditches': True,
                'grass_strips': True,
                'terrain_size': [200, 120]
            },
            'navigation': {
                'gps_markers': True,
                'row_markers': True,
                'waypoints': True,
                'gate_posts': True
            }
        }
    
    def load_base_world(self):
        """Load the base world file content"""
        if not self.base_world_file or not os.path.exists(self.base_world_file):
            raise FileNotFoundError(f"Base world file not found: {self.base_world_file}")
        
        with open(self.base_world_file, 'r') as f:
            return f.read()
    
    def remove_existing_vineyard_rows(self, world_content):
        """Remove any existing vineyard row definitions from world content"""
        # Pattern to match vineyard row models and their includes
        patterns_to_remove = [
            r'<!--.*?Vineyard Row.*?-->.*?</model>\s*',
            r'<!--.*?Generated Vineyard Rows.*?-->.*?<!--.*?End Generated Vineyard Rows.*?-->\s*',
            r'<model name="vineyard_row_\d+".*?</model>\s*',
            r'<!--.*?vineyard.*?row.*?-->.*?(?=<!--|\s*<model|\s*</world>)',
            r'<include>.*?vineyard_post.*?</include>\s*',
            r'<include>.*?vine_plant.*?</include>\s*',
            r'<include>.*?irrigation_pipe.*?</include>\s*'
        ]
        
        cleaned_content = world_content
        for pattern in patterns_to_remove:
            cleaned_content = re.sub(pattern, '', cleaned_content, flags=re.DOTALL | re.IGNORECASE)
        
        return cleaned_content
    
    def generate_vineyard_rows(self):
        """Generate vineyard rows with posts and plants based on configuration"""
        vineyard_config = self.config['vineyard']
        infrastructure_config = self.config['infrastructure']
        
        num_rows = vineyard_config['num_rows']
        row_spacing = vineyard_config['row_spacing']
        row_length = vineyard_config['row_length']
        post_spacing = infrastructure_config['post_spacing']
        plant_spacing = vineyard_config['plant_spacing']
        
        rows_xml = "\n    <!-- Generated Vineyard Rows -->\n"
        start_y = -(num_rows - 1) * row_spacing / 2
        
        for row_num in range(num_rows):
            y_pos = start_y + row_num * row_spacing
            rows_xml += self._generate_single_row(row_num, y_pos, row_length, 
                                                 post_spacing, plant_spacing, 
                                                 infrastructure_config)
        
        rows_xml += "    <!-- End Generated Vineyard Rows -->\n\n"
        return rows_xml
    
    def _generate_single_row(self, row_num, y_pos, row_length, post_spacing, 
                           plant_spacing, infrastructure_config):
        """Generate a single vineyard row"""
        row_xml = f'''    <!-- Vineyard Row {row_num + 1} -->
    <model name="vineyard_row_{row_num + 1}">
      <static>true</static>
      <pose>0 {y_pos} 0 0 0 0</pose>
      
'''
        
        # Generate posts
        row_xml += self._generate_posts(row_num, row_length, post_spacing)
        
        # Generate plants
        row_xml += self._generate_plants(row_num, row_length, plant_spacing)
        
        # Generate irrigation if enabled
        if infrastructure_config.get('irrigation_enabled', False):
            row_xml += self._generate_irrigation(row_num, row_length, 
                                               infrastructure_config['irrigation_spacing'])
        
        row_xml += "    </model>\n\n"
        return row_xml
    
    def _generate_posts(self, row_num, row_length, post_spacing):
        """Generate posts for a row"""
        posts_xml = ""
        num_posts = int(row_length / post_spacing) + 1
        
        for post_num in range(num_posts):
            x_pos = post_num * post_spacing
            if x_pos <= row_length:
                posts_xml += f'''      <include>
        <uri>model://{self.models_path}/vineyard_post</uri>
        <pose>{x_pos} 0 1.0 0 0 0</pose>
        <name>post_{row_num + 1}_{post_num + 1}</name>
      </include>
      
'''
        return posts_xml
    
    def _generate_plants(self, row_num, row_length, plant_spacing):
        """Generate plants for a row"""
        plants_xml = ""
        num_plants = int(row_length / plant_spacing)
        
        for plant_num in range(num_plants):
            x_pos = plant_num * plant_spacing + plant_spacing/2
            if x_pos <= row_length:
                plants_xml += f'''      <include>
        <uri>model://{self.models_path}/vine_plant</uri>
        <pose>{x_pos} 0 0 0 0 0</pose>
        <name>vine_{row_num + 1}_{plant_num + 1}</name>
      </include>
      
'''
        return plants_xml
    
    def _generate_irrigation(self, row_num, row_length, irrigation_spacing):
        """Generate irrigation for a row"""
        irrigation_xml = ""
        num_irrigation = int(row_length / irrigation_spacing)
        
        for irr_num in range(num_irrigation):
            x_pos = irr_num * irrigation_spacing + irrigation_spacing/2
            if x_pos <= row_length:
                irrigation_xml += f'''      <include>
        <uri>model://{self.models_path}/irrigation_pipe</uri>
        <pose>{x_pos} -0.3 0 0 0 0</pose>
        <name>irrigation_{row_num + 1}_{irr_num + 1}</name>
      </include>
      
'''
        return irrigation_xml
    
    def find_insertion_point(self, world_content):
        """Find the best insertion point for vineyard rows in the world content"""
        # Look for common patterns where we can insert the vineyard rows
        insertion_patterns = [
            r'(<!-- Vineyard terrain -->.*?</model>\s*)',
            r'(</model>\s*(?=\s*<!-- Navigation markers -->))',
            r'(</model>\s*(?=\s*<!-- Physics))',
            r'(</light>\s*(?=\s*<!-- Physics))',
            r'(</plugin>\s*(?=\s*<!-- Navigation))'
        ]
        
        for pattern in insertion_patterns:
            match = re.search(pattern, world_content, re.DOTALL)
            if match:
                return match.end()
        
        # Fallback: insert before the closing world tag
        match = re.search(r'(\s*</world>)', world_content)
        if match:
            return match.start()
        
        # If no good spot found, append at the end
        return len(world_content)
    
    def generate_world(self, output_file):
        """Generate complete world file"""
        # Try to generate from scratch first, fallback to modifying base world
        try:
            return self.generate_complete_world(output_file)
        except Exception as e:
            print(f"Failed to generate complete world: {e}")
            if self.base_world_file:
                print("Falling back to modifying base world file...")
                return self.generate_world_from_base(output_file)
            else:
                raise FileNotFoundError("Cannot generate world: no base world file available")
    
    def generate_complete_world(self, output_file):
        """Generate a complete world file from scratch with all components"""
        print("Generating complete vineyard world from scratch...")
        
        # Generate all world components
        header = self.generate_world_header()
        terrain = self.generate_terrain()
        vineyard_rows = self.generate_vineyard_rows()
        infrastructure = self.generate_infrastructure()
        navigation_aids = self.generate_navigation_aids()
        footer = self.generate_world_footer()
        
        # Combine all components
        complete_world = (header + 
                         terrain + 
                         vineyard_rows + 
                         infrastructure + 
                         navigation_aids + 
                         footer)
        
        # Write the complete world file
        with open(output_file, 'w') as f:
            f.write(complete_world)
        
        print(f"Generated complete vineyard world file: {output_file}")
        return output_file
    
    def generate_world_from_base(self, output_file):
        """Generate world file by modifying the base world (fallback method)"""
        if not self.base_world_file:
            raise FileNotFoundError("No base world file specified")
        
        # Load the base world content
        world_content = self.load_base_world()
        
        # Remove any existing vineyard rows
        cleaned_content = self.remove_existing_vineyard_rows(world_content)
        
        # Generate new vineyard rows
        vineyard_rows = self.generate_vineyard_rows()
        
        # Find insertion point and insert the new rows
        insertion_point = self.find_insertion_point(cleaned_content)
        new_content = (cleaned_content[:insertion_point] + 
                      vineyard_rows + 
                      cleaned_content[insertion_point:])
        
        # Write the new world file
        with open(output_file, 'w') as f:
            f.write(new_content)
        
        print(f"Generated vineyard world file: {output_file}")
        print(f"Based on: {self.base_world_file}")
        return output_file

    def generate_world_header(self):
        """Generate the world file header with plugins and lighting"""
        env_config = self.config['environment']
        
        header = '''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="realistic_vineyard_world">
    <!-- Physics settings optimized for outdoor simulation -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Required system plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
    </plugin>
    
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands">
    </plugin>
    
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu">
    </plugin>
'''
        
        # Add wind system if enabled
        if env_config.get('wind_enabled', False):
            header += '''
    <!-- Wind system for realistic environmental effects -->
    <plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
      <horizontal>
        <magnitude>
          <time_for_rise>10</time_for_rise>
          <sin>
            <amplitude_percent>0.05</amplitude_percent>
            <period>60</period>
          </sin>
        </magnitude>
        <direction>
          <time_for_rise>30</time_for_rise>
          <sin>
            <amplitude>5</amplitude>
            <period>20</period>
          </sin>
        </direction>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.0002</stddev>
        </noise>
      </vertical>
    </plugin>
'''
        
        # Add lighting
        header += '''
    <!-- Lighting setup for vineyard environment -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Additional ambient lighting -->
    <light type="directional" name="ambient_light">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.2 0.2 0.3 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0 0 -1</direction>
    </light>
'''
        
        # Add spot lights if enabled
        if env_config.get('spot_lights', False):
            header += '''
    <!-- Spot lights for key areas -->
    <light type="spot" name="vineyard_spotlight_1">
      <pose>0 -10 8 0 1.2 0</pose>
      <diffuse>0.6 0.6 0.4 1</diffuse>
      <specular>0.3 0.3 0.2 1</specular>
      <direction>0 0 -1</direction>
      <spot>
        <inner_angle>0.6</inner_angle>
        <outer_angle>1.0</outer_angle>
        <falloff>1</falloff>
      </spot>
      <cast_shadows>true</cast_shadows>
    </light>

    <light type="spot" name="vineyard_spotlight_2">
      <pose>50 0 8 0 1.2 0</pose>
      <diffuse>0.6 0.6 0.4 1</diffuse>
      <specular>0.3 0.3 0.2 1</specular>
      <direction>0 0 -1</direction>
      <spot>
        <inner_angle>0.6</inner_angle>
        <outer_angle>1.0</outer_angle>
        <falloff>1</falloff>
      </spot>
      <cast_shadows>true</cast_shadows>
    </light>
'''
        
        return header

    def generate_terrain(self):
        """Generate terrain with grass strips, headlands, and roads"""
        vineyard_config = self.config['vineyard']
        terrain_config = self.config['terrain']
        
        num_rows = vineyard_config['num_rows']
        row_spacing = vineyard_config['row_spacing']
        row_length = vineyard_config['row_length']
        terrain_size = terrain_config.get('terrain_size', [200, 120])
        
        terrain = f'''
    <!-- Vineyard terrain with realistic features -->
    <model name="vineyard_terrain">
      <static>true</static>
      <link name="terrain_link">
        <!-- Main ground -->
        <collision name="ground_collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{terrain_size[0]} {terrain_size[1]}</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.7</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="ground_visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{terrain_size[0]} {terrain_size[1]}</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.4 0.3 0.2 1</ambient>
            <diffuse>0.5 0.4 0.3 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
'''
        
        # Add grass strips if enabled
        if terrain_config.get('grass_strips', True):
            start_y = -(num_rows - 1) * row_spacing / 2
            for i in range(num_rows - 1):
                y_pos = start_y + (i + 0.5) * row_spacing
                terrain += f'''
        <visual name="grass_strip_{i+1}">
          <pose>0 {y_pos} 0.01 0 0 0</pose>
          <geometry>
            <box>
              <size>{row_length} 1.5 0.02</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.6 0.3 1</ambient>
            <diffuse>0.4 0.8 0.4 1</diffuse>
          </material>
        </visual>'''
        
        # Add headlands if enabled
        if terrain_config.get('headlands', True):
            headland_width = vineyard_config.get('headland_width', 8.0)
            terrain += f'''
        
        <!-- Headland areas at row ends -->
        <visual name="headland_west">
          <pose>{-headland_width} 0 0.005 0 0 0</pose>
          <geometry>
            <box>
              <size>{headland_width * 2} {(num_rows - 1) * row_spacing + 5} 0.01</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.4 0.3 1</ambient>
            <diffuse>0.6 0.5 0.4 1</diffuse>
          </material>
        </visual>

        <visual name="headland_east">
          <pose>{row_length + headland_width} 0 0.005 0 0 0</pose>
          <geometry>
            <box>
              <size>{headland_width * 2} {(num_rows - 1) * row_spacing + 5} 0.01</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.4 0.3 1</ambient>
            <diffuse>0.6 0.5 0.4 1</diffuse>
          </material>
        </visual>'''
        
        # Add access roads if enabled
        if terrain_config.get('access_roads', True):
            terrain += f'''
        
        <!-- Access roads -->
        <visual name="access_road_main">
          <pose>{-headland_width - 10} 0 0.01 0 0 0</pose>
          <geometry>
            <box>
              <size>10 3 0.02</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>

        <visual name="access_road_north">
          <pose>{row_length/2} {-(num_rows * row_spacing)/2 - 5} 0.01 0 0 1.57</pose>
          <geometry>
            <box>
              <size>10 3 0.02</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>

        <visual name="access_road_south">
          <pose>{row_length/2} {(num_rows * row_spacing)/2 + 5} 0.01 0 0 1.57</pose>
          <geometry>
            <box>
              <size>10 3 0.02</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>'''
        
        # Add drainage ditches if enabled
        if terrain_config.get('drainage_ditches', True):
            terrain += f'''
        
        <!-- Drainage ditches -->
        <visual name="drainage_north">
          <pose>0 {-(num_rows * row_spacing)/2 - 2.5} -0.1 0 0 0</pose>
          <geometry>
            <box>
              <size>{row_length} 0.5 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.3 0.4 1</ambient>
            <diffuse>0.3 0.4 0.5 1</diffuse>
          </material>
        </visual>

        <visual name="drainage_south">
          <pose>0 {(num_rows * row_spacing)/2 + 2.5} -0.1 0 0 0</pose>
          <geometry>
            <box>
              <size>{row_length} 0.5 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.3 0.4 1</ambient>
            <diffuse>0.3 0.4 0.5 1</diffuse>
          </material>
        </visual>'''
        
        terrain += '''
      </link>
    </model>
'''
        return terrain
    
    def generate_infrastructure(self):
        """Generate vineyard infrastructure including buildings and equipment"""
        infrastructure_config = self.config['infrastructure']
        infrastructure_xml = ""
        
        # Storage buildings
        if infrastructure_config.get('storage_buildings', 0) > 0:
            for i in range(infrastructure_config['storage_buildings']):
                x_pos = -20 - i * 15
                y_pos = 0
                infrastructure_xml += f'''
    <!-- Storage Building {i+1} -->
    <include>
      <uri>model://{self.models_path}/storage_building</uri>
      <pose>{x_pos} {y_pos} 0 0 0 0</pose>
      <name>storage_building_{i+1}</name>
    </include>
'''
        
        # Equipment sheds
        if infrastructure_config.get('equipment_sheds', 0) > 0:
            for i in range(infrastructure_config['equipment_sheds']):
                x_pos = -20 - i * 15
                y_pos = 15 + i * 10
                infrastructure_xml += f'''
    <!-- Equipment Shed {i+1} -->
    <include>
      <uri>model://{self.models_path}/storage_building</uri>
      <pose>{x_pos} {y_pos} 0 0 0 1.57</pose>
      <name>equipment_shed_{i+1}</name>
    </include>
'''
        
        # Water tank
        if infrastructure_config.get('water_tank', False):
            infrastructure_xml += '''
    <!-- Water tank for irrigation -->
    <model name="water_tank">
      <static>true</static>
      <pose>-25 -8 0 0 0 0</pose>
      <link name="tank_link">
        <collision name="tank_collision">
          <geometry>
            <cylinder>
              <radius>2</radius>
              <length>4</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="tank_visual">
          <geometry>
            <cylinder>
              <radius>2</radius>
              <length>4</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.5 1</ambient>
            <diffuse>0.4 0.4 0.7 1</diffuse>
            <specular>0.2 0.2 0.3 1</specular>
          </material>
        </visual>
      </link>
    </model>
'''
        
        # Tool shed
        if infrastructure_config.get('tool_shed', False):
            infrastructure_xml += '''
    <!-- Tool shed -->
    <model name="tool_shed">
      <static>true</static>
      <pose>-30 5 0 0 0 0</pose>
      <link name="shed_link">
        <collision name="shed_collision">
          <geometry>
            <box>
              <size>4 3 2.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="shed_visual">
          <geometry>
            <box>
              <size>4 3 2.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.3 1</ambient>
            <diffuse>0.8 0.6 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''
        
        # Compost area
        if infrastructure_config.get('compost_area', False):
            infrastructure_xml += '''
    <!-- Compost area -->
    <model name="compost_pile">
      <static>true</static>
      <pose>-20 -15 0 0 0 0</pose>
      <link name="compost_link">
        <collision name="compost_collision">
          <geometry>
            <box>
              <size>3 2 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="compost_visual">
          <geometry>
            <box>
              <size>3 2 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.2 0.1 1</ambient>
            <diffuse>0.4 0.3 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''
        
        # Equipment parking pad
        if infrastructure_config.get('parking_pad', False):
            infrastructure_xml += '''
    <!-- Equipment parking area -->
    <model name="parking_pad">
      <static>true</static>
      <pose>-35 0 0 0 0 0</pose>
      <link name="parking_link">
        <collision name="parking_collision">
          <geometry>
            <box>
              <size>8 6 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="parking_visual">
          <geometry>
            <box>
              <size>8 6 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''
        
        # Weather station
        if infrastructure_config.get('weather_station', False):
            infrastructure_xml += '''
    <!-- Weather station for environmental monitoring -->
    <model name="weather_station">
      <static>true</static>
      <pose>10 -15 0 0 0 0</pose>
      <link name="station_base">
        <collision name="base_collision">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.5</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="base_visual">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.5</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.9 0.9 0.9 1</diffuse>
          </material>
        </visual>
      </link>
      
      <link name="station_pole">
        <pose>0 0 1.5 0 0 0</pose>
        <collision name="pole_collision">
          <geometry>
            <cylinder>
              <radius>0.03</radius>
              <length>3.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="pole_visual">
          <geometry>
            <cylinder>
              <radius>0.03</radius>
              <length>3.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
      
      <joint name="base_to_pole" type="fixed">
        <parent>station_base</parent>
        <child>station_pole</child>
      </joint>
    </model>
'''
        
        # Irrigation control box
        if infrastructure_config.get('irrigation_control', False):
            infrastructure_xml += '''
    <!-- Irrigation control box -->
    <model name="irrigation_control">
      <static>true</static>
      <pose>-10 -8 0 0 0 0</pose>
      <link name="control_box">
        <collision name="box_collision">
          <geometry>
            <box>
              <size>0.8 0.6 1.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="box_visual">
          <geometry>
            <box>
              <size>0.8 0.6 1.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.5 0.2 1</ambient>
            <diffuse>0.3 0.7 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''
        
        # Boundary fence
        if infrastructure_config.get('boundary_fence', False):
            vineyard_config = self.config['vineyard']
            row_length = vineyard_config['row_length']
            num_rows = vineyard_config['num_rows']
            row_spacing = vineyard_config['row_spacing']
            
            infrastructure_xml += f'''
    <!-- Boundary fence sections -->
    <model name="fence_north">
      <static>true</static>
      <pose>{row_length/2} {-(num_rows * row_spacing)/2 - 10} 0 0 0 0</pose>
      <link name="fence_link">
        <collision name="fence_collision">
          <geometry>
            <box>
              <size>{row_length} 0.1 1.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="fence_visual">
          <geometry>
            <box>
              <size>{row_length} 0.1 1.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.3 0.2 1</ambient>
            <diffuse>0.6 0.4 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="fence_south">
      <static>true</static>
      <pose>{row_length/2} {(num_rows * row_spacing)/2 + 10} 0 0 0 0</pose>
      <link name="fence_link">
        <collision name="fence_collision">
          <geometry>
            <box>
              <size>{row_length} 0.1 1.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="fence_visual">
          <geometry>
            <box>
              <size>{row_length} 0.1 1.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.3 0.2 1</ambient>
            <diffuse>0.6 0.4 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''
        
        return infrastructure_xml

    def generate_navigation_aids(self):
        """Generate navigation markers and aids"""
        navigation_config = self.config['navigation']
        vineyard_config = self.config['vineyard']
        nav_xml = ""
        
        # GPS markers
        if navigation_config.get('gps_markers', False):
            nav_xml += '''
    <!-- Navigation markers -->
    <model name="gps_marker_start">
      <static>true</static>
      <pose>0 0 0.5 0 0 0</pose>
      <link name="marker_link">
        <visual name="marker_visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 0.5 0 1</ambient>
            <diffuse>1 0.7 0 1</diffuse>
            <emissive>0.3 0.15 0 1</emissive>
          </material>
        </visual>
      </link>
    </model>
'''
        
        # Row markers
        if navigation_config.get('row_markers', False):
            row_length = vineyard_config['row_length']
            start_y = -(vineyard_config['num_rows'] - 1) * vineyard_config['row_spacing'] / 2
            
            nav_xml += f'''
    <!-- Row start/end markers -->
    <model name="nav_marker_row1_start">
      <static>true</static>
      <pose>0 {start_y} 0.3 0 0 0</pose>
      <link name="marker_link">
        <visual name="marker_visual">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.6</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
            <emissive>0 0.3 0 1</emissive>
          </material>
        </visual>
      </link>
    </model>

    <model name="nav_marker_row1_end">
      <static>true</static>
      <pose>{row_length} {start_y} 0.3 0 0 0</pose>
      <link name="marker_link">
        <visual name="marker_visual">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.6</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
            <emissive>0.3 0 0 1</emissive>
          </material>
        </visual>
      </link>
    </model>
'''
        
        # Waypoints
        if navigation_config.get('waypoints', False):
            row_length = vineyard_config['row_length']
            start_y = -(vineyard_config['num_rows'] - 1) * vineyard_config['row_spacing'] / 2
            waypoint_positions = [row_length * 0.25, row_length * 0.5, row_length * 0.75]
            
            for i, x_pos in enumerate(waypoint_positions, 1):
                nav_xml += f'''
    <!-- Waypoint {i} -->
    <model name="waypoint_{i}">
      <static>true</static>
      <pose>{x_pos} {start_y} 0.2 0 0 0</pose>
      <link name="waypoint_link">
        <visual name="waypoint_visual">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
            <emissive>0 0 0.2 1</emissive>
          </material>
        </visual>
      </link>
    </model>
'''
        
        # Gate posts
        if navigation_config.get('gate_posts', False):
            row_length = vineyard_config['row_length']
            nav_xml += f'''
    <!-- Gate posts at access points -->
    <model name="gate_1">
      <static>true</static>
      <pose>{row_length/2} -12 0 0 0 0</pose>
      <link name="gate_post_1">
        <collision name="gate_post_1_collision">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>2.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="gate_post_1_visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>2.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
      
      <link name="gate_post_2">
        <pose>0 3 1.0 0 0 0</pose>
        <collision name="gate_post_2_collision">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>2.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="gate_post_2_visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>2.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''
        
        return nav_xml

    def generate_world_footer(self):
        """Generate world file footer with physics and scene configuration"""
        env_config = self.config['environment']
        bg_color = env_config['background_color']
        ambient = env_config['ambient_light']
        shadows = env_config['shadows']
        
        return f'''
    <!-- Physics configuration -->
    <physics name="default_physics" default="0" type="ode">
      <gravity>0 0 -9.8066</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <sor>1.4</sor>
          <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.0</contact_max_correcting_vel>
          <contact_surface_layer>0.01</contact_surface_layer>
        </constraints>
      </ode>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
      <magnetic_field>6.0e-06 2.3e-05 -4.2e-05</magnetic_field>
    </physics>

    <!-- Scene configuration -->
    <scene>
      <ambient>{ambient} {ambient} {ambient} 1</ambient>
      <background>{bg_color[0]} {bg_color[1]} {bg_color[2]} {bg_color[3]}</background>
      <shadows>{"true" if shadows else "false"}</shadows>
      <sky>
        <clouds>true</clouds>
      </sky>
    </scene>

    <!-- GUI configuration -->
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>-20 -20 15 0 0.5 0.7</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

  </world>
</sdf>'''

def main():
    parser = argparse.ArgumentParser(description='Generate customized vineyard world files')
    parser.add_argument('--config', '-c', help='Configuration file (YAML)')
    parser.add_argument('--base-world', '-b', help='Base world file to modify (optional)')
    parser.add_argument('--output', '-o', help='Output world file', 
                       default='custom_vineyard.world')
    parser.add_argument('--from-scratch', action='store_true', 
                       help='Generate world from scratch instead of modifying base world')
    parser.add_argument('--rows', '-r', type=int, help='Number of vineyard rows')
    parser.add_argument('--spacing', '-s', type=float, help='Row spacing in meters')
    parser.add_argument('--length', '-l', type=float, help='Row length in meters')
    parser.add_argument('--post-spacing', '-p', type=float, help='Post spacing in meters')
    parser.add_argument('--plant-spacing', type=float, help='Plant spacing in meters')
    
    args = parser.parse_args()
    
    # Create generator
    generator = VineyardWorldGenerator(args.config, args.base_world)
    
    # Override config with command line arguments
    if args.rows:
        generator.config['vineyard']['num_rows'] = args.rows
    if args.spacing:
        generator.config['vineyard']['row_spacing'] = args.spacing
    if args.length:
        generator.config['vineyard']['row_length'] = args.length
    if args.post_spacing:
        generator.config['infrastructure']['post_spacing'] = args.post_spacing
    if args.plant_spacing:
        generator.config['vineyard']['plant_spacing'] = args.plant_spacing
    
    # Generate world file - force method if specified
    if args.from_scratch:
        generator.generate_complete_world(args.output)
    elif args.base_world:
        generator.generate_world_from_base(args.output) 
    else:
        generator.generate_world(args.output)
    
    print("Configuration used:")
    print(f"  Rows: {generator.config['vineyard']['num_rows']}")
    print(f"  Row spacing: {generator.config['vineyard']['row_spacing']}m")
    print(f"  Row length: {generator.config['vineyard']['row_length']}m")
    print(f"  Post spacing: {generator.config['infrastructure']['post_spacing']}m")
    print(f"  Plant spacing: {generator.config['vineyard']['plant_spacing']}m")

if __name__ == '__main__':
    main()
