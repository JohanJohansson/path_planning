#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ras_arduino_msgs/Odometry.h>
#include <robot_msgs/detectedObject.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include "path_planning.hpp"

//Not an actual node yet, just for testing the path planner

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_planing");
    ros::NodeHandle nh;

    nav_msgs::OccupancyGrid map;
    map.info.height = 10;
    map.info.width = 10;
    map.data.resize(10*10);
    map.data[50] = 100;
    map.data[51] = 100;
    map.data[52] = 100;
    map.data[53] = 100;
    map.data[54] = 100;
    map.data[55] = 100;
    map.data[56] = 100;
    map.data[57] = 100;
    map.data[58] = 100;
    map.data[59] = 0;
    map.data[39] = 100;
    map.data[38] = 100;
    map.data[37] = 100;
    map.data[36] = 100;
    map.data[35] = 0;
    map.data[34] = 100;
    map.data[33] = 100;
    map.data[70] = -1;

    geometry_msgs::Pose start, goal;
    start.position.x = 0;
    start.position.y = 0;
    goal.position.x = 9;
    goal.position.y = 9;

    nav_msgs::Path path;

    planPathUnexplored(map, start, path);

    std::cout << "Path size: " << path.poses.size() << std::endl;

    ros::spin();
    return 0;
}
