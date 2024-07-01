#include <costmap_erase_plugin/costmap_erase_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/inflation_layer.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <set>
#include <vector>
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(costmap_2d::CostmapErasePlugin, costmap_2d::Layer)

namespace costmap_2d {

CostmapErasePlugin::CostmapErasePlugin() : tf_listener_2(tf_buffer_) {
    ros::NodeHandle nh("~/" + name_);
    laser_scan_sub_ = nh.subscribe("/scan", 1, &CostmapErasePlugin::laserScanCallback, this);
    odom_sub_ = nh.subscribe("/odom", 1, &CostmapErasePlugin::odomCallback, this);
}

void CostmapErasePlugin::onInitialize() {
    ObstacleLayer::onInitialize();
    //ROS_INFO("CostmapErasePlugin::onInitialize started");

    // Statik hücreleri başlat
    initializeStaticCells();

    // Inflation layer'ı başlat
    inflation_layer_ = new costmap_2d::InflationLayer();
    inflation_layer_->initialize(layered_costmap_, name_ + "_inflation", &tf_buffer_);

    //ROS_INFO("CostmapErasePlugin::onInitialize finished");
}




void CostmapErasePlugin::initializeStaticCells() {
    unsigned int size_x = layered_costmap_->getCostmap()->getSizeInCellsX();
    unsigned int size_y = layered_costmap_->getCostmap()->getSizeInCellsY();
    std::cout << "Static layer size: " << size_x << " x " << size_y << std::endl;

    if (size_x > 0 && size_y > 0) {
        static_cells_.clear();
        for (unsigned int i = 0; i < size_x; ++i) {
            for (unsigned int j = 0; j < size_y; ++j) {
                if (layered_costmap_->getCostmap()->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE) {
                    static_cells_.push_back(std::make_pair(i, j));
                }
            }
        }
        //std::cout << "Static cells initialized with size: " << static_cells_.size() << std::endl;
    } else {
        //std::cout << "Static layer size is zero, initialization skipped." << std::endl;
    }
}

void CostmapErasePlugin::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) {
    ObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
    inflation_layer_->updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void CostmapErasePlugin::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    // Statik hücreleri koru
    for (const auto& cell : static_cells_) {
        master_grid.setCost(cell.first, cell.second, costmap_2d::LETHAL_OBSTACLE);
    }

    // Dinamik hücreleri güncelle ve görünmeyenleri sil
    ros::Time current_time = ros::Time::now();
    for (auto it = dynamic_cells_.begin(); it != dynamic_cells_.end(); ) {
        if ((current_time - it->second).toSec() > 2.0) { // 2 saniyeden uzun süredir görünmüyorsa
            master_grid.setCost(it->first.first, it->first.second, costmap_2d::FREE_SPACE); // Hücreyi temizle
            it = dynamic_cells_.erase(it); // Dinamik hücre listesinden sil
        } else {
            master_grid.setCost(it->first.first, it->first.second, costmap_2d::LETHAL_OBSTACLE);
            ++it;
        }
    }

    // Inflation layer'ı güncelle
    inflation_layer_->updateCosts(master_grid, min_i, min_j, max_i, max_j);
}

void CostmapErasePlugin::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    sensor_msgs::PointCloud cloud;

    try {
        tf_listener_1.waitForTransform(layered_costmap_->getGlobalFrameID(), scan->header.frame_id, scan->header.stamp, ros::Duration(1.0));
        projector_.transformLaserScanToPointCloud(layered_costmap_->getGlobalFrameID(), *scan, cloud, tf_listener_1);
    } catch (tf::TransformException &ex) {
        //ROS_ERROR("Error transforming laser scan into the global frame: %s", ex.what());
        return;
    }

    // Dinamik objeleri güncelle
    //ROS_INFO("Laser scan callback started. Observed objects size before clearing: %lu", dynamic_cells_.size());
    
    for (const auto& point : cloud.points) {
        unsigned int mx, my;
        if (layered_costmap_->getCostmap()->worldToMap(point.x, point.y, mx, my)) {
            std::pair<unsigned int, unsigned int> cell_id(mx, my);
            if (std::find(static_cells_.begin(), static_cells_.end(), cell_id) == static_cells_.end()) {
                dynamic_cells_[cell_id] = ros::Time::now();
            }
        }
    }

    //ROS_INFO("Laser scan callback executed. Observed objects size after updating: %lu", dynamic_cells_.size());
}




void CostmapErasePlugin::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Odom verilerini güncelle
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
}

CostmapErasePlugin::~CostmapErasePlugin() {
    if (inflation_layer_ != nullptr) {
        delete inflation_layer_;
        inflation_layer_ = nullptr;
    }
}

} // end namespace costmap_2d
