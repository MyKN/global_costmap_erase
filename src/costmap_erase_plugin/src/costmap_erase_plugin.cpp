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
    // Check Object and remove necessary that
    // Obje varlığını kontrol et ve gerekli olanları sil
    checkObjectPersistence(); 
    //ROS_INFO("Observed objects map size before UPDATECOST 1: %lu", observed_objects.size());
    for (const auto& obj : observed_objects) {
        unsigned int mx, my;

        mx = obj.first.first;
        my = obj.first.second;

        double wx, wy;
        master_grid.mapToWorld(mx, my, wx, wy);

        double distance = std::hypot(wx - robot_x_, wy - robot_y_);
        // If object stays in 2 metres radius of robot, define it as Obstacle
        // Obje 2 metre çap içinde ise engel olarak işaretle
        if (distance <= erase_radius_) {
            master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
        }
    }
    // Update map without static layer
    // Static layer'a dokunmadan haritanın tamamını güncelle

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

    // Update Object as obstacle or not
    // Tanımlanan objeyi obstacle ya da değil olarak güncelle
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

  void CostmapErasePlugin::checkObjectPersistence() {
    auto now = ros::Time::now(); 
    //ROS_INFO("Observed objects map size before CHECKOBJECTPERSISTENCE: %lu", observed_objects.size());
    for (auto it = observed_objects.begin(); it != observed_objects.end();) {
        unsigned int mx = it->first.first;
        unsigned int my = it->first.second;
        double wx, wy;
        layered_costmap_->getCostmap()->mapToWorld(mx, my, wx, wy);
        double distance = std::hypot(wx - robot_x_, wy - robot_y_);
        // Nesne 2 metre çapı dışında ise veya belirli bir süre boyunca gözlemlenmediyse sil
        if (distance > erase_radius_ && (now - it->second).toSec() > object_persistence_time_) {

            it = observed_objects.erase(it);
        } else {
            ++it;
        }
    }
  }
} // end namespace costmap_2d
