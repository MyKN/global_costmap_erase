# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "costmap_2d;pluginlib;roscpp;nav_msgs;std_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lcostmap_erase_plugin".split(';') if "-lcostmap_erase_plugin" != "" else []
PROJECT_NAME = "costmap_erase_plugin"
PROJECT_SPACE_DIR = "/home/muhammedyasin/catkin_ws/install"
PROJECT_VERSION = "0.0.0"