# Global Costmap Erase Plugin for TurtleBot3

This repository contains a plugin developed for use with the TurtleBot3 project. The plugin is designed to update and manage obstacles in the global costmap that are outside a 2-meter radius from the robot.

## Features

- Utilizes the TurtleBot3 platform.
- Updates the costmap to erase obstacles outside a 2-meter radius around the robot.

## Configuration

The `global_costmap.params` file is configured as follows:

```yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link

  update_frequency: 10.0  # 3 seconds = 0.33 Hz
  publish_frequency: 1.0
  transform_tolerance: 0.5

  static_map: true
  rolling_window: false  # true

  plugins:
    - {name: static_layer, type: "costmap_2d::StaticLayer"}
    - {name: costmap_erase_layer, type: "costmap_2d::CostmapErasePlugin"}
    #- {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}

  obstacle_layer:
    global_frame: odom
    robot_base_frame: base_link
    static_map: true
    rolling_window: false  # true
    width: 10.0
    weight: 10.0
    resolution: 0.05
    enabled: true
    map_type: costmap
    observation_sources: scan
    scan:
      sensor_frame: base_scan
      data_type: LaserScan
      topic: /scan
      marking: true
      clearing: true
      #observation_persistence: 5.0  # Obstacle persistence time

  costmap_erase_layer:
    global_frame: odom
    robot_base_frame: base_link
    rolling_window: false  # true
    width: 10.0
    weight: 10.0
    resolution: 0.05
    map_type: costmap
    enabled: true
    erase_radius: 2.0
    observation_sources: scan
    scan:
      sensor_frame: base_scan
      data_type: LaserScan
      topic: /scan
      marking: true
      clearing: true
      #observation_persistence: 5.0  # Obstacle persistence time

  inflation_layer:
    enabled: true
    inflation_radius: 2.0
    cost_scaling_factor: 2.0
    map_type: costmap
    observation_sources: scan
    scan:
      sensor_frame: base_scan
      data_type: LaserScan
      topic: /scan
      marking: true
      clearing: true

  static_layer:
    global_frame: map
    enabled: true

  always_send_full_costmap: True
