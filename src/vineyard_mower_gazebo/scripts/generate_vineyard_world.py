#!/usr/bin/env python3

"""
Vineyard World Generator
Generates customized vineyard world files based on configuration parameters.
"""

import os
import yaml
import argparse
from ament_index_python.packages import get_package_share_directory

class VineyardWorldGenerator:
    def __init__(self, config_file=None):
        self.config = self.load_config(config_file)
        
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
                'headland_width': 8.0,
                'plant_spacing': 1.5,
                'slope': 2.0
            },
            'infrastructure': {
                'post_spacing': 8.0,
                'irrigation_enabled': True,
                'irrigation_spacing': 15.0,
                'storage_buildings': 1,
                'equipment_sheds': 1
            },
            'environment': {
                'ambient_light': 0.4,
                'shadows': True,
                'background_color': [0.7, 0.8, 0.9, 1.0]
            }
        }
    
    def generate_world_header(self):
        """Generate the world file header"""
        return '''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="generated_vineyard_world">
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

    <!-- Lighting setup for vineyard environment -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

'''
    
    def generate_terrain(self):
        """Generate terrain model"""
        vineyard_config = self.config['vineyard']
        num_rows = vineyard_config['num_rows']
        row_spacing = vineyard_config['row_spacing']
        row_length = vineyard_config['row_length']
        
        terrain_width = num_rows * row_spacing + 20  # Add margins
        terrain_length = row_length + 20
        
        terrain = f'''    <!-- Vineyard terrain -->
    <model name="vineyard_terrain">
      <static>true</static>
      <link name="terrain_link">
        <collision name="ground_collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{terrain_length} {terrain_width}</size>
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
              <size>{terrain_length} {terrain_width}</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.4 0.3 0.2 1</ambient>
            <diffuse>0.6 0.4 0.3 1</diffuse>
          </material>
        </visual>
'''
        
        # Add grass strips between rows
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
        
        terrain += '''
      </link>
    </model>

'''
        return terrain
    
    def generate_vineyard_rows(self):
        """Generate vineyard rows with posts and plants"""
        vineyard_config = self.config['vineyard']
        infrastructure_config = self.config['infrastructure']
        
        num_rows = vineyard_config['num_rows']
        row_spacing = vineyard_config['row_spacing']
        row_length = vineyard_config['row_length']
        post_spacing = infrastructure_config['post_spacing']
        plant_spacing = vineyard_config['plant_spacing']
        
        rows_xml = ""
        start_y = -(num_rows - 1) * row_spacing / 2
        
        for row_num in range(num_rows):
            y_pos = start_y + row_num * row_spacing
            
            rows_xml += f'''    <!-- Vineyard Row {row_num + 1} -->
    <model name="vineyard_row_{row_num + 1}">
      <static>true</static>
      <pose>0 {y_pos} 0 0 0 0</pose>
      
'''
            
            # Generate posts along the row
            num_posts = int(row_length / post_spacing) + 1
            for post_num in range(num_posts):
                x_pos = post_num * post_spacing
                if x_pos <= row_length:
                    rows_xml += f'''      <include>
        <uri>model://vineyard_post</uri>
        <pose>{x_pos} 0 1.0 0 0 0</pose>
        <name>post_{row_num + 1}_{post_num + 1}</name>
      </include>
      
'''
            
            # Generate vine plants
            num_plants = int(row_length / plant_spacing)
            for plant_num in range(num_plants):
                x_pos = plant_num * plant_spacing + plant_spacing/2
                if x_pos <= row_length:
                    rows_xml += f'''      <include>
        <uri>model://vine_plant</uri>
        <pose>{x_pos} 0 0 0 0 0</pose>
        <name>vine_{row_num + 1}_{plant_num + 1}</name>
      </include>
      
'''
            
            # Add irrigation if enabled
            if infrastructure_config['irrigation_enabled']:
                irrigation_spacing = infrastructure_config['irrigation_spacing']
                num_irrigation = int(row_length / irrigation_spacing)
                for irr_num in range(num_irrigation):
                    x_pos = irr_num * irrigation_spacing + irrigation_spacing/2
                    if x_pos <= row_length:
                        rows_xml += f'''      <include>
        <uri>model://irrigation_pipe</uri>
        <pose>{x_pos} -0.3 0 0 0 0</pose>
        <name>irrigation_{row_num + 1}_{irr_num + 1}</name>
      </include>
      
'''
            
            rows_xml += '''    </model>

'''
        
        return rows_xml
    
    def generate_buildings(self):
        """Generate storage buildings and equipment sheds"""
        infrastructure_config = self.config['infrastructure']
        buildings_xml = ""
        
        # Storage buildings
        num_storage = infrastructure_config.get('storage_buildings', 1)
        for i in range(num_storage):
            x_pos = -20 - i * 15
            y_pos = 0
            buildings_xml += f'''    <!-- Storage Building {i+1} -->
    <include>
      <uri>model://storage_building</uri>
      <pose>{x_pos} {y_pos} 0 0 0 0</pose>
      <name>storage_building_{i+1}</name>
    </include>

'''
        
        # Equipment sheds
        num_sheds = infrastructure_config.get('equipment_sheds', 1)
        for i in range(num_sheds):
            x_pos = -20 - i * 15
            y_pos = 15 + i * 10
            buildings_xml += f'''    <!-- Equipment Shed {i+1} -->
    <include>
      <uri>model://storage_building</uri>
      <pose>{x_pos} {y_pos} 0 0 0 1.57</pose>
      <name>equipment_shed_{i+1}</name>
    </include>

'''
        
        return buildings_xml
    
    def generate_world_footer(self):
        """Generate world file footer"""
        env_config = self.config['environment']
        ambient = env_config['ambient_light']
        bg_color = env_config['background_color']
        shadows = env_config['shadows']
        
        return f'''    <!-- Navigation markers -->
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
    
    def generate_world(self, output_file):
        """Generate complete world file"""
        world_content = ""
        world_content += self.generate_world_header()
        world_content += self.generate_terrain()
        world_content += self.generate_vineyard_rows()
        world_content += self.generate_buildings()
        world_content += self.generate_world_footer()
        
        with open(output_file, 'w') as f:
            f.write(world_content)
        
        print(f"Generated vineyard world file: {output_file}")
        return output_file

def main():
    parser = argparse.ArgumentParser(description='Generate customized vineyard world files')
    parser.add_argument('--config', '-c', help='Configuration file (YAML)')
    parser.add_argument('--output', '-o', help='Output world file', 
                       default='custom_vineyard.world')
    parser.add_argument('--rows', '-r', type=int, help='Number of vineyard rows')
    parser.add_argument('--spacing', '-s', type=float, help='Row spacing in meters')
    parser.add_argument('--length', '-l', type=float, help='Row length in meters')
    
    args = parser.parse_args()
    
    # Create generator
    generator = VineyardWorldGenerator(args.config)
    
    # Override config with command line arguments
    if args.rows:
        generator.config['vineyard']['num_rows'] = args.rows
    if args.spacing:
        generator.config['vineyard']['row_spacing'] = args.spacing
    if args.length:
        generator.config['vineyard']['row_length'] = args.length
    
    # Generate world file
    generator.generate_world(args.output)
    
    print(f"Configuration used:")
    print(f"  Rows: {generator.config['vineyard']['num_rows']}")
    print(f"  Spacing: {generator.config['vineyard']['row_spacing']}m")
    print(f"  Length: {generator.config['vineyard']['row_length']}m")

if __name__ == '__main__':
    main()
