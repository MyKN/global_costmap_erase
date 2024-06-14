#ifndef COSTMAP_ERASE_PLUGIN_H
#define COSTMAP_ERASE_PLUGIN_H

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/obstacle_layer.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <unordered_map>
#include <utility>

// Özel hash fonksiyonunu tanımlayalım
namespace std {
    template <>
    struct hash<std::pair<double, double>> {
        size_t operator()(const std::pair<double, double>& p) const {
            auto hash1 = std::hash<double>{}(p.first);
            auto hash2 = std::hash<double>{}(p.second);
            return hash1 ^ hash2; // Basit bir XOR kombinasyonu kullanıyoruz
        }
    };
}

namespace costmap_2d {

class CostmapErasePlugin : public ObstacleLayer {
public:
    CostmapErasePlugin();
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void checkObjectPersistence();

    ros::Subscriber odom_sub_;
    ros::Subscriber laser_scan_sub_;
    tf::TransformListener tf_listener_;

    bool clear_obstacles_;
    double erase_radius_;
    double robot_x_;
    double robot_y_;
    double observation_persistence_;
    double tolerance_;

    struct Object {
        ros::Time last_seen;
        double x, y;
    };
    
    std::unordered_map<std::pair<double, double>, Object> observed_objects;
};

} // end namespace costmap_2d

#endif // COSTMAP_ERASE_PLUGIN_H
