#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <ras_arduino_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <robot_msgs/usePathFollower.h>
#include "path_planning.hpp"
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <ras_arduino_msgs/Encoders.h>
#include <robot_msgs/MakeTurn.h>
#include <robot_msgs/FollowWall.h>
#include <math.h>
#include <iostream>
#include <robot_msgs/pathSucceeded.h>


enum {LEFT_TURN = 1, RIGHT_TURN = 2};

#define RATE 20.0
#define ANGULAR_AVOIDANCE 0.5


class Navigation {

public:
    bool callFindandFollow;
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
    ros::ServiceClient succeed_path_client;

    Navigation() {
        n = ros::NodeHandle();
        //path_sub = n.subscribe("/path", 1, &Navigation::pathCallback, this);
        pose_sub = n.subscribe("/arduino/odometry", 1, &Navigation::odometryCallback, this);
        //pose_sub = n.subscribe("/loc/pose", 1, &Navigation::odometryCallback, this);
        twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
        turn_client = n.serviceClient<robot_msgs::MakeTurn>("/make_turn");
        cost_map_sub = n.subscribe("/costmap", 1, &Navigation::costMapCallback, this);
        path_server = n.advertiseService("/use_path_follower", &Navigation::findAndFollowPathCallback, this);
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        costMap_publisher = n.advertise<nav_msgs::OccupancyGrid>("/test_costmap", 1);
        distance_sub = n.subscribe("/ir_sensor_cm", 1, &Navigation::IrCallback, this);
        succeed_path_client = n.serviceClient<robot_msgs::pathSucceeded>("/simple");
	
	callFindandFollow=false;
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

        geometry_msgs::Pose pose_closest, pose_carrot;
        pose_closest.position.x = path.poses[index_closest].pose.position.x * resolution - center_x;
        pose_closest.position.y = path.poses[index_closest].pose.position.y * resolution - center_y;
        pose_carrot.position.x = path.poses[index].pose.position.x * resolution - center_x;
        pose_carrot.position.y = path.poses[index].pose.position.y * resolution - center_y;

        double x = pose_carrot.position.x - pose_closest.position.x;
        double y = pose_carrot.position.y - pose_closest.position.y;

        double angle = atan2(pose_carrot.position.y - pose.y, pose_carrot.position.x - pose.x);
        double angle_v = atan2(y, x);

        double rotate;

        //TODO Check if it makes sense by taking path position * resolution - center_
        //At least know when using the pose from /arduino/odometry since (0,0) for robot is at (5.0, 5.0) in map.
        //Will be different if subscribing to localization if that pose is with origin (0,0) in map.
        double dist = sqrt(pow(pose_closest.position.x - pose.x,2) + pow(pose_closest.position.y - pose.y,2));
        if (dist > 0.05) {
           rotate = pose.theta + M_PI_2 - angle;
        } else
           rotate = pose.theta + M_PI_2 - angle_v;
        if (rotate > M_PI) {
            rotate -= 2*M_PI;
        } else if (rotate < -M_PI) {
            rotate += 2*M_PI;
        }

        ROS_INFO("Robot x: %f y: %f", pose.x, pose.y);
        ROS_INFO("Closest x: %f y: %f", pose_closest.position.x, pose_closest.position.y);
        ROS_INFO("Carrot x: %f y: %f", pose_carrot.position.x, pose_carrot.position.y);
        //ROS_INFO("Angle to path: %f \n Angle to rotate: %f \n Theta: %f", angle, rotate, pose.theta);
        int tresh_front = 15;//17;

	//Test med closest och carrot är med än eller lika med pi halva
        if (rotate >= M_PI_2*0.94 || rotate <= -M_PI_2*0.94) {
            ROS_INFO("Make large turn");
            makeTurn(-rotate);
	
	} else if ((forward_left < tresh_front &&
                forward_left >= 0) ||
                (forward_right < tresh_front && forward_right >= 0) ||
                (forward_left > 40 && forward_right < 20) ||
                (forward_right > 40 && forward_left < 20)) {
		makeTurn(-rotate);
   	
	} else if (front_right < front_left &&
                      back_right < back_left &&
                      front_right < 7 &&
                      back_right < 7) {
		twist.linear.x = 0.1;
		twist.angular.z = ANGULAR_AVOIDANCE;
		ROS_INFO("Too close to right wall");		

	} else if (front_right < front_left &&
                      back_right < back_left &&
                      front_right < 7 &&
                      back_right < 7) {
		twist.linear.x = 0.1;
		twist.angular.z = -ANGULAR_AVOIDANCE;
		ROS_INFO("Too close to left wall");		
        } else {
	    
            double alpha = 4;
            twist.linear.x = 0.1;
	    
            if (std::abs(rotate) < 0.16*2) {
                ROS_INFO("Corrects heading");
            	twist.angular.z = alpha*(-rotate);
            }
            else {
	        twist.angular.z = 0;
		ROS_INFO("Angle too big, goes forward instead.");
	    }

            ROS_INFO("Rotation speed: %f", twist.angular.z);
            twist_pub.publish(twist);
        }
    }

    int findClosest() {
        int index = -1;
        double dist = 10000;
        for (int i = 0; i < path.poses.size(); i++) {
            double distance = sqrt(pow(pose.x-(path.poses[i].pose.position.x*resolution - center_x), 2) + pow(pose.y-(path.poses[i].pose.position.y*resolution - center_y), 2));
            if (distance < dist) {
                index = i;
                dist = distance;
            }
        }
        //ROS_INFO("Index closeset %d, x: %f y: %f", index, path.poses[index].pose.position.x*resolution - center_x, path.poses[index].pose.position.y*resolution - center_y);
        if(index >= 0) return index;
        else return -1;

    }

    int findCarrot() {

        int index = -1;
        for (int i = 0; i < path.poses.size(); i++) {
            double distance = sqrt(pow(pose.x-(path.poses[i].pose.position.x*resolution - center_x), 2) + pow(pose.y-(path.poses[i].pose.position.y*resolution - center_y), 2));
            if (distance <= look_ahead) {
                index = i;
            }
        }
        //ROS_INFO("Index carrot %d, x: %f y: %f", index, path.poses[index].pose.position.x*resolution - center_x, path.poses[index].pose.position.y*resolution - center_y);
        if(sqrt(pow(pose.x-(path.poses[path.poses.size()-1].pose.position.x*resolution - center_x), 2) + pow(pose.y-(path.poses[path.poses.size()-1].pose.position.y*resolution - center_y), 2)) <= 0.05)
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
        turn_srv.request.degrees = std::abs(angle)*180/M_PI;
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
        robot_msgs::pathSucceeded succeed_srv;
        succeed_srv.request.succeeded = true;

	ROS_INFO("Path-Navigation try to Stop");
        if(succeed_path_client.call(succeed_srv)){
            ROS_INFO("Succesfully called succeed_srv to masternode");
        }
        else{
            ROS_ERROR("Failed to call Succeeded to Master Node");
        }

    }
    //Finds the path between start and goal if possible and then drives there
    bool findAndFollowPathCallback(robot_msgs::usePathFollower::Request &req, robot_msgs::usePathFollower::Response &res) {
	if(req.go){
	    currentRequest=req;
	    callFindandFollow=true;
            STOP=false;
	}
	else{
	    STOP=true;
	}
	return true;

}
    void findAndFollowPath(){
	    callFindandFollow=false;
            ROS_INFO("Wants to plan a path.");
            geometry_msgs::Pose goal, start;

            start.position.x = floor((center_x + pose.x)/resolution);
            start.position.y = floor((center_y + pose.y)/resolution);
	    path.poses.clear();
            if(currentRequest.goal.x<0 && currentRequest.goal.y<0){
                ROS_INFO("Explore unexplored Area");
                planPathUnexplored(cost_map,start,path);
            }
            else{
                goal.position.x = currentRequest.goal.x; //Goal needs to be in cell indexes
                goal.position.y = currentRequest.goal.y; //Goal needs to be in cell indexes
                ROS_INFO("Start x: %f y: %f \n Goal x: %f y: %f", start.position.x, start.position.y, goal.position.x, goal.position.y);

                //costMap_publisher.publish(temp);
                planPathGoal(cost_map, start, goal, path);
            }
            ROS_INFO("Length of path %lu", path.poses.size());
            visualizePath();

            //Follows path until it reaches goal point
            ros::Rate loop_rate(RATE);
            while (!reachedGoal() && !STOP) {

                followPath();
                ros::spinOnce();

                loop_rate.sleep();
            }
	    if(STOP) ROS_INFO("Left While-loop,  got an Stop msg");
	    if(reachedGoal()) ROS_INFO("Left While-loop, reached the Goal ");
            stop();


        return;
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
        geometry_msgs::Point p;
        for (int i = 0; i < path.poses.size(); i++) {
            p.x = path.poses[i].pose.position.x * resolution - center_x;
            p.y = path.poses[i].pose.position.y * resolution - center_y;
            p.z = 0;
            //ROS_INFO("Cells x: %f y: %f", p.x, p.y);
            points.points.push_back(p);
        }
        marker_pub.publish(points);

        points.points.clear();
        points.id = 1;
        points.type = visualization_msgs::Marker::SPHERE;
        points.scale.x = 0.04;
        points.scale.y = 0.04;
        points.color.b = 1.0;
        points.color.a = 1.0;
        points.points.push_back(p);
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
    bool STOP;
    robot_msgs::usePathFollower::Request currentRequest;
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "path_navigation");

    Navigation nav;

    //ros::Duration(1.0).sleep();
    ros::Rate loop_rate(RATE);
    /*while (!nav.reachedGoal()) {
        ros::spinOnce();
        nav.followPath();
        loop_rate.sleep();
    }

    nav.stop();
*/
    while(ros::ok()){
	ros::spinOnce();
        if(nav.callFindandFollow){
	    nav.findAndFollowPath();
	}
	loop_rate.sleep();
    }
//    ros::spin();
    return 0;
}
