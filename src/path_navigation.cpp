#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <ras_arduino_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <robot_msgs/useMazeFollower.h>
#include "path_planning.hpp"
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <ras_arduino_msgs/Encoders.h>
#include <robot_msgs/MakeTurn.h>
#include <robot_msgs/FollowWall.h>
#include <math.h>
#include <iostream>

enum {LEFT_TURN = 1, RIGHT_TURN = 2};

#define RATE 20.0


class Navigation {

public:
    ros::NodeHandle n;
    ros::Subscriber path_sub;
    ros::Subscriber pose_sub;
    ros::Publisher turn_pub;
    ros::Publisher twist_pub;
    ros::Subscriber cost_map_sub;
    ros::ServiceClient turn_client;
    ros::ServiceServer path_server;
    ros::Publisher marker_pub;
    ros::Publisher costMap_publisher;
    ros::Subscriber distance_sub;

    Navigation() {
        n = ros::NodeHandle();
        //path_sub = n.subscribe("/path", 1, &Navigation::pathCallback, this);
        //pose_sub = n.subscribe("/arduino/odometry", 1, &Navigation::odometryCallback, this);
        pose_sub = n.subscribe("/loc/pose", 1, &Navigation::odometryCallback, this);
        twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
        turn_client = n.serviceClient<robot_msgs::MakeTurn>("/make_turn");
        cost_map_sub = n.subscribe("/costmap", 1, &Navigation::costMapCallback, this);
        path_server = n.advertiseService("/use_path_follower", &Navigation::findAndFollowPath, this);
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        costMap_publisher = n.advertise<nav_msgs::OccupancyGrid>("/test_costmap", 1);
        distance_sub = n.subscribe("/ir_sensor_cm", 1, &Navigation::IrCallback, this);

        look_ahead = 0.2;
        goal_reached = false;
        resolution = 0.02;
        center_x = 5.0;
        center_y = 5.0;

        /*path.poses.resize(150);
        geometry_msgs::PoseStamped point;

        for (int i = 0; i < 50; i++) {
            point.pose.position.x = 0/100.0;
            point.pose.position.y = i/100.0;
            path.poses[i] = point;
            ROS_INFO("Point x: %f y: %f", point.pose.position.x, point.pose.position.y);
        }
        for (int i = 0; i < 50; i++) {
            point.pose.position.x = i/100.0;
            point.pose.position.y = 49/100.0;
            path.poses[i+50] = point;
            ROS_INFO("Point x: %f y: %f", point.pose.position.x, point.pose.position.y);
        }
        for (int i = 0; i < 50; i++) {
            point.pose.position.x = 49/100.0;
            point.pose.position.y = 49/100.0 - i/100.0;
            path.poses[i+100] = point;
            ROS_INFO("Point x: %f y: %f", point.pose.position.x, point.pose.position.y);
        }*/
    }

    ~Navigation() {}

    /*void pathCallback(const nav_msgs::Path &msg) {
        path = msg;
    }*/

    //Callback for using IR sensor values
    void IrCallback(const ras_arduino_msgs::ADConverterConstPtr &msg) {
        front_left = msg->ch1;
        front_right = msg->ch2;
        back_left = msg->ch3;
        back_right = msg->ch4;
        forward_right = msg->ch5;
        forward_left = msg->ch6;
    }

    void odometryCallback(const ras_arduino_msgs::Odometry &msg) {
        pose = msg;
        //ROS_INFO("Pose x: %f y: %f theta: %f", pose.x, pose.y, pose.theta);
    }

    void costMapCallback(const nav_msgs::OccupancyGrid &msg) {
        cost_map = msg;
        cost_map.info.height = 500;
        cost_map.info.width = 500;
    }

    void followPath() {
        int index = findCarrot();
        int index_closest = findClosest();

        if (index == -1) {
            ROS_INFO("Out of track");
            return;
        }

        double x = path.poses[index].pose.position.x - path.poses[index_closest].pose.position.x;
        double y = path.poses[index].pose.position.y - path.poses[index_closest].pose.position.y;

        //Same here as description below
        double angle = atan2(path.poses[index].pose.position.y * resolution - pose.y,
                             path.poses[index].pose.position.x * resolution - pose.x);
        double angle_v = atan2(y, x);

        double rotate;

        //TODO Check if it makes sense by taking path position * resolution - center_
        //At least know when using the pose from /arduino/odometry since (0,0) for robot is at (5.0, 5.0) in map.
        //Will be different if subscribing to localization if that pose is with origin (0,0) in map.
        double dist = sqrt(pow(path.poses[index_closest].pose.position.x * resolution - pose.x,2)
                           + pow(path.poses[index_closest].pose.position.y * resolution - pose.y,2));
        if (dist > 0.05) {
           rotate = pose.theta + M_PI_2 - angle;
        } else
           rotate = pose.theta + M_PI_2 - angle_v;
        if (rotate > M_PI) {
            rotate -= 2*M_PI;
        } else if (rotate < -M_PI) {
            rotate += 2*M_PI;
        }

        ROS_INFO("Carrot x: %f y: %f", path.poses[index].pose.position.x, path.poses[index].pose.position.y);
        ROS_INFO("Angle to path: %f \n Angle to rotate: %f \n Theta: %f", angle, rotate, pose.theta);

        if (rotate >=M_PI_2*0.97 || rotate <= -M_PI_2*0.97) {
            ROS_INFO("Make large turn");
            makeTurn(-rotate);

        } else {
	    
            double alpha = 2;
            twist.linear.x = 0.1;
            double temp = wallTooClose();
            if (temp != 0) {
                ROS_INFO("Wall too close %f", temp);
                twist.angular.z = temp;
            } else if (fabs(rotate) < 0.16*2) {
                ROS_INFO("Corrects heading");
            	twist.angular.z = alpha*(-rotate);
            }
            else {twist.angular.z = 0;}

            ROS_INFO("Rotation speed: %f", twist.angular.z);
            twist_pub.publish(twist);
        }
    }

    int findClosest() {
        int index = -1;
        double dist = 10000;
        for (int i = 0; i < path.poses.size(); i++) {
            double distance = sqrt(pow(pose.x-path.poses[i].pose.position.x, 2) + pow(pose.y-path.poses[i].pose.position.y, 2));
            if (distance < dist) {
                index = i;
                dist = distance;
            }
        }

        if(index > 0) return index;
        else return -1;

    }

    int findCarrot() {

        int index = -1;
        for (int i = 0; i < path.poses.size(); i++) {
            double distance = sqrt(pow(pose.x-path.poses[i].pose.position.x, 2) + pow(pose.y-path.poses[i].pose.position.y, 2));
            if (distance <= look_ahead) {
                index = i;
            }
        }
                ROS_INFO("Index %d, x: %f y: %f", index, path.poses[index].pose.position.x, path.poses[index].pose.position.y);
        if(sqrt(pow(pose.x-path.poses[path.poses.size()-1].pose.position.x, 2) + pow(pose.y-path.poses[path.poses.size()-1].pose.position.y, 2)) <= 0.05)
                goal_reached = true;

	return index;

    }

    double wallTooClose() {
        if ((front_left < 9 && front_left > 0 && back_left < 9 && back_left > 0) &&
                front_right < 9 && front_right > 0 && back_right < 9 && back_right > 0)
            return 0.0;
        else if (front_left < 9 && front_left > 0 && back_left < 9 && back_left > 0) {
            return -0.314;
        } else if (front_right < 9 && front_right > 0 && back_right < 9 && back_right > 0) {
            return 0.314;
        }
    }

    void makeTurn(double angle) {
        robot_msgs::MakeTurn turn_srv;
        if (angle < 0) {
            turn_srv.request.state = RIGHT_TURN;
        } else {
            turn_srv.request.state = LEFT_TURN;
        }
        turn_srv.request.degrees = fabs(angle)*180/M_PI;
        ROS_INFO("Degrees to rotate: %f", angle);

        if (turn_client.call(turn_srv)) {
            ROS_INFO("Succesfully called turn service");
          }
          else
          {
            ROS_ERROR("Failed to call turn service in maze_follower");
          }
    }

    bool reachedGoal() {
        return goal_reached;
    }

    void stop() {
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        twist_pub.publish(twist);
    }

    //Finds the path between start and goal if possible and then drives there
    bool findAndFollowPath(robot_msgs::useMazeFollower::Request &req, robot_msgs::useMazeFollower::Response &res) {

        if (req.go == true) {
            ROS_INFO("Wants to plan a path.");
            geometry_msgs::Pose goal, start;
            goal.position.x = 250;
            goal.position.y = 250;
            start.position.x = floor((center_x + pose.x)/resolution);
            start.position.y = floor((center_y + pose.y)/resolution);
            ROS_INFO("Start x: %f y: %f \n Goal x: %f y: %f", start.position.x, start.position.y, goal.position.x, goal.position.y);

            //costMap_publisher.publish(temp);
            planPathGoal(cost_map, start, goal, path);
            //ROS_INFO("Length of path %lu", path.poses.size());
            visualizePath();

            //Follows path until it reaches goal point
            ros::Rate loop_rate(RATE);
            while (!reachedGoal()) {
                ros::spinOnce();
                followPath();
                loop_rate.sleep();
            }

            stop();
        } else ROS_INFO("Don't plan a path");

        return true;
    }

    //Publishes the path to be visualized in Rviz
    void visualizePath() {

        visualization_msgs::Marker points;
        points.header.frame_id = "map";
        points.header.stamp = ros::Time::now();
        points.ns = "path";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.type = visualization_msgs::Marker::LINE_STRIP;
        points.scale.x = 0.02;
        points.color.r = 1.0;
        points.color.a = 1.0;
        points.lifetime = ros::Duration(0);
        points.pose.position.x = 5;
        points.pose.position.y = 5;

        ROS_INFO("Length of path %lu", path.poses.size());
        for (int i = 0; i < path.poses.size(); i++) {
            geometry_msgs::Point p;
            p.x = path.poses[i].pose.position.x * resolution - center_x;
            p.y = path.poses[i].pose.position.y * resolution - center_y;
            p.z = 0;
            //ROS_INFO("Cells x: %f y: %f", p.x, p.y);
            points.points.push_back(p);
        }
        marker_pub.publish(points);

    }

private:
    nav_msgs::Path path;
    ras_arduino_msgs::Odometry pose;
    double look_ahead;
    bool goal_reached;
    geometry_msgs::Twist twist;
    nav_msgs::OccupancyGrid cost_map;
    double resolution;
    double center_x;
    double center_y;
    int front_left, front_right, back_left, back_right, forward_left, forward_right, state;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "path_navigation");

    Navigation nav;

    //ros::Duration(1.0).sleep();
    //ros::Rate loop_rate(RATE);
    /*while (!nav.reachedGoal()) {
        ros::spinOnce();
        nav.followPath();
        loop_rate.sleep();
    }

    nav.stop();
*/
    ros::spin();
    return 0;
}
