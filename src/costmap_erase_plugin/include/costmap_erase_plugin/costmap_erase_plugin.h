#ifndef COSTMAP_ERASE_PLUGIN_H_
#define COSTMAP_ERASE_PLUGIN_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/obstacle_layer.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <map>
#include <tf/transform_listener.h>

namespace costmap_2d
{
  struct ObjectData {
    ros::Time last_seen;
    float x, y;  // Nesnenin son bilinen koordinatları
  };

  class CostmapErasePlugin : public ObstacleLayer
  {
  public:
    CostmapErasePlugin();
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void checkObjectPersistence();

    bool clear_obstacles_;
    double erase_radius_;
    double robot_x_;
    double robot_y_;
    double observation_persistence_;
    double tolerance_;

    tf::TransformListener tf_listener_;

    ros::Subscriber clear_costmap_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber laser_scan_sub_;

    std::map<std::pair<float, float>, ObjectData> observed_objects;  // Nesne koordinatlarına göre nesne takibi
  };
} // end namespace costmap_2d

#endif // COSTMAP_ERASE_PLUGIN_H_
