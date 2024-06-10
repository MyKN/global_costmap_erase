#include <costmap_erase_plugin/costmap_erase_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/obstacle_layer.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <vector>

PLUGINLIB_EXPORT_CLASS(costmap_2d::CostmapErasePlugin, costmap_2d::Layer)

namespace costmap_2d
{
  CostmapErasePlugin::CostmapErasePlugin() : clear_obstacles_(false), erase_radius_(2.0), robot_x_(0.0), robot_y_(0.0), observation_persistence_(5.0), tolerance_(0.5)
  {
  }

  void CostmapErasePlugin::onInitialize()
  {
    ObstacleLayer::onInitialize();
    ros::NodeHandle nh("~/" + name_);
    
    clear_costmap_sub_ = nh.subscribe("/move_base/clear_costmaps", 1, &CostmapErasePlugin::clearCostmapCallback, this);
    odom_sub_ = nh.subscribe("/odom", 1, &CostmapErasePlugin::odomCallback, this);
    laser_scan_sub_ = nh.subscribe("/scan", 1, &CostmapErasePlugin::laserScanCallback, this);

    nh.param("observation_persistence", observation_persistence_, observation_persistence_);
    nh.param("tolerance", tolerance_, 0.5); // Tolerance for the difference between laser data

    ROS_INFO("CostmapErasePlugin initialized with observation_persistence: %f and tolerance: %f", observation_persistence_, tolerance_);
  }

  void CostmapErasePlugin::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
  {
    ObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  }

  void CostmapErasePlugin::updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    if (clear_obstacles_)
    {
      if (!previous_scan_.empty() && !current_scan_.empty())
      {
        bool is_same = true;
        for (size_t i = 0; i < current_scan_.size(); ++i)
        {
          if (std::abs(current_scan_[i] - previous_scan_[i]) > tolerance_)
          {
            is_same = false;
            break;
          }
        }

        if (is_same)
        {
          ROS_INFO("Laser data within 2-meter radius is the same, costmap not updated.");
          return;
        }
      }

      double wx, wy;
      ROS_INFO("Starting obstacle clearing process...");
      for (int i = min_i; i < max_i; ++i)
      {
        for (int j = min_j; j < max_j; ++j)
        {
          master_grid.mapToWorld(i, j, wx, wy);
          double distance = std::sqrt(std::pow(wx - robot_x_, 2) + std::pow(wy - robot_y_, 2));
          if (distance > erase_radius_)
          {
            if (master_grid.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE)
            {
              master_grid.setCost(i, j, costmap_2d::FREE_SPACE);
              ROS_INFO("Cleared cell at (%f, %f) with distance %f", wx, wy, distance);
            }
          }
        }
      }
      ROS_INFO("Obstacle clearing process completed.");
      clear_obstacles_ = false;
    }
    else
    {
      ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
    }
  }

  void CostmapErasePlugin::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    if (previous_scan_.empty())
    {
      // Store the first set of data
      previous_scan_ = scan->ranges;
      return;
    }

    // Store the new set of data
    current_scan_ = scan->ranges;

    // Compare the data
    bool is_same = true;
    for (size_t i = 0; i < current_scan_.size(); ++i)
    {
      if (std::abs(current_scan_[i] - previous_scan_[i]) > tolerance_)
      {
        is_same = false;
        break;
      }
    }

    if (is_same)
    {
      ROS_INFO("Laser data within 2-meter radius is the same.");
    }
    else
    {
      ROS_INFO("Laser data within 2-meter radius is different.");
    }

    // Store the current data as the previous data
    previous_scan_ = current_scan_;
  }

  void CostmapErasePlugin::clearCostmapCallback(const std_msgs::Empty& msg)
  {
    ROS_INFO("Clearing obstacle layer costmap outside radius");
    clear_obstacles_ = true;
  }

  void CostmapErasePlugin::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    //ROS_INFO("Updated robot position: (%f, %f)", robot_x_, robot_y_);
  }
} // end namespace costmap_2d
