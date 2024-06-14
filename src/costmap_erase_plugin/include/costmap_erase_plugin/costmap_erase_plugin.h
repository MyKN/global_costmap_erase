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
#include <map>
#include <utility>


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
    
    double object_persistence_time_; // Object persistence time, 
    double tolerance_;  
    // Tolerance value, this parameter was created for laser scan data, because data comes as differently each other. 
    // Therefore, just coming data from specific range can be acceptable as a obstacle.

 
    struct Object {
        ros::Time last_seen;  // Last seen time
        double x, y;          // Position of the object
    };
    

    // Map of observed objects, this function holds obstacle values in there
    std::map<std::pair<double, double>, Object> observed_objects;
};

} // end namespace costmap_2d

#endif // COSTMAP_ERASE_PLUGIN_H
