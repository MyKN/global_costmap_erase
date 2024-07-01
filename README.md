# Global Costmap Erase Plugin for TurtleBot3

This repository contains a plugin developed for use with the TurtleBot3 project. The plugin is designed to update and manage obstacles in the global costmap that are outside a 2-meter radius from the robot.

## Features

- Utilizes the TurtleBot3 platform.
- Updates the costmap to erase obstacles outside a 2-meter radius around the robot.

## Installation

### Prerequisites

- **Operating System**: Ubuntu 20.04
- **ROS Distribution**: ROS Noetic

### Installation Steps

1. **Setup Sources and Install ROS Noetic:**
   ```sh
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   sudo apt update
   sudo apt install ros-noetic-desktop-full
   sudo rosdep init
   rosdep update
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
2. **Setup Sources and Install Turtlebot3:**
   ```sh
   sudo apt install ros-noetic-turtlebot3
   sudo apt install ros-noetic-turtlebot3-simulations
   echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
   source ~/.bashrc

3. **For More Detail Information:**
   ```sh
   https://wiki.ros.org/noetic/Installation/Ubuntu
   https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
   

   

## Configuration

The `global_costmap.params` file is configured as follows:

```yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link

  update_frequency: 10.0
  publish_frequency: 0.5
  transform_tolerance: 0.5

  static_map: true
  rolling_window: false

  plugins:
    - {name: static_layer, type: "costmap_2d::StaticLayer"}
    - {name: costmap_erase_layer, type: "costmap_2d::CostmapErasePlugin"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}

  costmap_erase_layer:
    global_frame: map
    robot_base_frame: base_link
    rolling_window: false
    width: 10.0
    height: 10.0
    resolution: 0.05
    enabled: true
    erase_radius: 2.0
    observation_sources: scan
    scan:
      sensor_frame: base_scan
      data_type: LaserScan
      topic: /scan
      marking: true
      clearing: true


  inflation_layer:
    enabled: true
    inflation_radius: 0.55
    cost_scaling_factor: 10.0
    track_unknown_space: true
    lethal_cost: 253
    inflate_unknown: true


  static_layer:
    enabled: true
