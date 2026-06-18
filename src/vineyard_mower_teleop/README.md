# Vineyard Mower Teleoperation

Keyboard teleoperation package for the vineyard mower differential drive robot.

## Features

- Direct keyboard control publishing to `/diff_drive_controller/cmd_vel`
- Adjustable linear and angular speeds
- Emergency stop capability
- Clean terminal interface

## Installation

```bash
cd /home/sam/vinmo
colcon build --packages-select vineyard_mower_teleop
source install/setup.bash
```

## Usage

### Run the keyboard teleop node directly:

```bash
ros2 run vineyard_mower_teleop keyboard_teleop
```

### Run with launch file:

```bash
ros2 launch vineyard_mower_teleop keyboard_teleop.launch.py
```

### With custom speeds:

```bash
ros2 launch vineyard_mower_teleop keyboard_teleop.launch.py linear_speed:=0.7 angular_speed:=1.5
```

## Controls

```
Moving around:
   q    w    e
   a    x    d
   z    s    c

w/s : forward/backward
a/d : turn left/right
q/e : forward + turn left/right
z/c : backward + turn left/right
x   : force stop

i/k : increase/decrease linear speed by 10%
j/l : increase/decrease angular speed by 10%

SPACE : emergency stop
ESC/CTRL-C : quit
```

## Parameters

- `linear_speed` (default: 0.5): Default linear velocity in m/s
- `angular_speed` (default: 1.0): Default angular velocity in rad/s
- `linear_increment` (default: 0.05): Linear speed adjustment increment
- `angular_increment` (default: 0.1): Angular speed adjustment increment
- `publish_rate` (default: 10.0): Command publishing rate in Hz
- `use_sim_time` (default: true): Use simulation time

## Topics

### Published Topics

- `/diff_drive_controller/cmd_vel` (geometry_msgs/TwistStamped): Velocity commands for the differential drive controller

## Integration with Launch Files

To integrate with your main launch file:

```python
from launch_ros.actions import Node

keyboard_teleop_node = Node(
    package='vineyard_mower_teleop',
    executable='keyboard_teleop',
    name='keyboard_teleop',
    output='screen',
    parameters=[{
        'use_sim_time': True,
        'linear_speed': 0.5,
        'angular_speed': 1.0,
    }],
    prefix='xterm -e',  # Run in separate terminal
)
```

## Troubleshooting

### Robot doesn't move

1. Check that the controller is running:
   ```bash
   ros2 control list_controllers
   ```

2. Verify the topic is correct:
   ```bash
   ros2 topic echo /diff_drive_controller/cmd_vel
   ```

3. Check that diff_drive_controller is loaded:
   ```bash
   ros2 topic list | grep diff_drive
   ```

### Terminal issues

If you encounter terminal input issues, ensure your terminal supports raw mode input. The xterm prefix in the launch file helps with this.
