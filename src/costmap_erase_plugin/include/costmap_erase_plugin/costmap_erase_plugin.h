#ifndef COSTMAP_ERASE_PLUGIN_H_
#define COSTMAP_ERASE_PLUGIN_H_

#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/inflation_layer.h>
#include <pluginlib/class_list_macros.h>

#include <tf/transform_listener.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <laser_geometry/laser_geometry.h>
#include <set>
#include <map>
#include <vector>

namespace costmap_2d
{
    class CostmapErasePlugin : public costmap_2d::ObstacleLayer
    {
    public:
        CostmapErasePlugin();
        virtual ~CostmapErasePlugin();

        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
        virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    private:
        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void initializeStaticCells();

        std::vector<std::pair<unsigned int, unsigned int>> static_cells_;
        std::map<std::pair<unsigned int, unsigned int>, ros::Time> dynamic_cells_;

        double robot_x_, robot_y_;
        laser_geometry::LaserProjection projector_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_2;
        tf::TransformListener tf_listener_1;

        ros::Subscriber laser_scan_sub_;
        ros::Subscriber odom_sub_;

        costmap_2d::InflationLayer* inflation_layer_;
    };
}

#endif // COSTMAP_ERASE_PLUGIN_H_
