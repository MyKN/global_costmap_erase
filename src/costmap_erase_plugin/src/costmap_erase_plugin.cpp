#include <costmap_erase_plugin/costmap_erase_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/obstacle_layer.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <ros/ros.h>
#include <map>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(costmap_2d::CostmapErasePlugin, costmap_2d::Layer)

namespace costmap_2d {
  CostmapErasePlugin::CostmapErasePlugin() : clear_obstacles_(true), erase_radius_(2.0), robot_x_(0.0), robot_y_(0.0), tolerance_(1.0), object_persistence_time_(5.0) {
    
    ros::NodeHandle nh("~/" + name_);

    odom_sub_ = nh.subscribe("/odom", 1, &CostmapErasePlugin::odomCallback, this);
    laser_scan_sub_ = nh.subscribe("/scan", 1, &CostmapErasePlugin::laserScanCallback, this);
  }

  void CostmapErasePlugin::onInitialize() {
    ObstacleLayer::onInitialize();
  }

  void CostmapErasePlugin::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) {
    ObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y); 
  }

  void CostmapErasePlugin::updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    auto now = ros::Time::now();
    ROS_INFO("Observed objects map size before UPDATECOST 1: %lu", observed_objects.size());

    bool sayacFlag = true;


    for (auto it = observed_objects.begin(); it != observed_objects.end();) {
        unsigned int mx, my;
        mx = it->first.first;
        my = it->first.second;
        
        double wx, wy;
        master_grid.mapToWorld(mx, my, wx, wy);
        
        double distance = std::hypot(wx - robot_x_, wy - robot_y_);

        // Nesne 2 metre çapı dışında ve gözlemlenme süresi dolunca sil
        // Object was stayed out of radius, update it after persistence time
        if (distance > erase_radius_ && (now - it->second).toSec() > 5.0) {
            it = observed_objects.erase(it);
        } else {
            // Nesne 2 metre çapı içinde ise engel olarak işaretle
            // If object stays in 2 metres radius of robot, define it as Obstacle
            if (distance <= erase_radius_) {
                master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
            }
            ++it;
        } 
    }
    

    // Static layer'ı bozmadan haritanın tamamını güncelley
    // Update map without static layer

    ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);



    //ROS_INFO("Observed objects map size after UPDATECOST 2: %lu", observed_objects.size());
  }

  void CostmapErasePlugin::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    laser_geometry::LaserProjection projector_;
    sensor_msgs::PointCloud cloud;

    try {
        tf_listener_.waitForTransform("map", scan->header.frame_id, scan->header.stamp, ros::Duration(1.0));
        projector_.transformLaserScanToPointCloud("map", *scan, cloud, tf_listener_);
    } 
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("Error transforming laser scan into the map frame: %s", ex.what());
        return;
    }

    // Tanımlanan nesneyi obstacle ya da değil olarak güncelle
    // Update Object as obstacle or not

    for (const auto& point : cloud.points) {
        double ox = point.x;
        double oy = point.y;
        unsigned int mx, my;
        if (layered_costmap_->getCostmap()->worldToMap(ox, oy, mx, my)) {
            observed_objects[{mx, my}] = ros::Time::now();
        }
    }
  }

  void CostmapErasePlugin::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
  }
} // end namespace costmap_2d
