#include <costmap_erase_plugin/costmap_erase_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/obstacle_layer.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <ros/ros.h>
#include <unordered_map>

PLUGINLIB_EXPORT_CLASS(costmap_2d::CostmapErasePlugin, costmap_2d::Layer)

namespace costmap_2d {
  CostmapErasePlugin::CostmapErasePlugin() : clear_obstacles_(true), erase_radius_(2.0), robot_x_(0.0), robot_y_(0.0), observation_persistence_(5.0), tolerance_(0.5) {
    
    ros::NodeHandle nh("~/" + name_);

    // Subscribing to Odom and Laser Topic
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
    checkObjectPersistence(); // Nesnelerin varlığını kontrol et ve gerekli olanları sil
    for (const auto& obj : observed_objects) {
        unsigned int mx, my;
        if (master_grid.worldToMap(obj.second.x, obj.second.y, mx, my)) {
            double distance = std::hypot(obj.second.x - robot_x_, obj.second.y - robot_y_);
            // Nesne 2 metre çap içinde ise engel olarak işaretle
            if (distance <= erase_radius_) {
                master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
            } else {
                // Nesne 2 metre çap dışında ise serbest alan olarak işaretle
                master_grid.setCost(mx, my, costmap_2d::FREE_SPACE);
            }
        }
    }
    // Static layer'ı bozmadan haritanın tamamını güncelle
    ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
  }

  void CostmapErasePlugin::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    laser_geometry::LaserProjection projector_;
    sensor_msgs::PointCloud cloud;

    try {
        // LaserScan verilerini harita çerçevesine dönüştür
        tf_listener_.waitForTransform("map", scan->header.frame_id, scan->header.stamp, ros::Duration(1.0));
        projector_.transformLaserScanToPointCloud("map", *scan, cloud, tf_listener_);
    } 
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("Error transforming laser scan into the map frame: %s", ex.what());
        return;
    }

    // Tanımlanan nesneyi obstacle ya da değil olarak güncelle
    for (const auto& point : cloud.points) {
        double ox = point.x;
        double oy = point.y;
        observed_objects[{ox, oy}] = {ros::Time::now(), ox, oy};
    }
  }

  void CostmapErasePlugin::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Robotun anlık konumunu al
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
  }

  void CostmapErasePlugin::checkObjectPersistence() {
    auto now = ros::Time::now(); 
    for (auto it = observed_objects.begin(); it != observed_objects.end();) {
        double distance = std::hypot(it->second.x - robot_x_, it->second.y - robot_y_);
        // Nesne 2 metre çapı dışında ise sil
        if (distance > erase_radius_) {
            it = observed_objects.erase(it);
        } else {
            ++it;
        }
    }
  }
} // end namespace costmap_2d
