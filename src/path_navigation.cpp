#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <ras_arduino_msgs/Odometry.h>

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
    ros::ServiceClient turn_client;
    Navigation() {
        n = ros::NodeHandle();
        path_sub = n.subscribe("/path", 1, &Navigation::pathCallback, this);
        pose_sub = n.subscribe("/arduino/odometry", 1, &Navigation::odometryCallback, this);
        twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
        turn_client = n.serviceClient<robot_msgs::MakeTurn>("/make_turn");
        look_ahead = 0.2;
        goal_reached = false;

        path.poses.resize(100);
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

      /*pose.x = 8;
        pose.y = -2;
        pose.theta = 1.3;*/
    }

    ~Navigation() {}

    void pathCallback(const nav_msgs::Path &msg) {
        path = msg;
    }

    void odometryCallback(const ras_arduino_msgs::Odometry &msg) {
        pose = msg;
        ROS_INFO("Pose x: %f y: %f theta: %f", pose.x, pose.y, pose.theta);
    }

    void followPath() {
        int index = findCarrot();
        //int index_closest = findClosest();
        ROS_INFO("Look ahead coordinates x: %f y: %f", path.poses[index].pose.position.x, path.poses[index].pose.position.y);

        double angle = atan2(path.poses[index].pose.position.y-pose.y, path.poses[index].pose.position.x-pose.x);
        double used_angle = pose.theta + M_PI_2 - angle;
        ROS_INFO("ANGLE: %f USED_ANGLE: %f", angle, used_angle);

        double alpha = 1;
        //TODO 1 try out with the circular movement, maybe add pi/2 to the pose angle
        double delta_x = (path.poses[index].pose.position.x-pose.x)*cos(used_angle) + (path.poses[index].pose.position.y-pose.y)*sin(used_angle);
        double gamma = -2*delta_x/pow(look_ahead,2);

        ROS_INFO("Delta x: %f", delta_x);
        ROS_INFO("Radius r: %f Gamma g: %f", 1/gamma, gamma);
        //TODO try with pid controller
        //while {
        geometry_msgs::Twist twist;
        if (abs(angle)>=M_PI_2) {
            makeTurn(angle);
        } else {
            twist.linear.x = 0.1;
            twist.angular.z = alpha*gamma;
            //Try twist.angular.z = 2*M_PI/gamma*RATE;
            ROS_INFO("Angular velocity: %f %f", alpha*gamma, -0.000005*2*M_PI/gamma*RATE);
            twist_pub.publish(twist);
        }
        //}
    }

    int findClosest() {
        int index = -1;
        double dist = 10000;
        for (int i = 0; i < path.poses.size(); i++) {
            double distance = sqrt(pow(pose.x-path.poses[i].pose.position.x, 2) + pow(pose.y-path.poses[i].pose.position.y, 2));
            if (distance < dist) {
                //ROS_INFO("Index %d, x: %f y: %f", i, path.poses[i].pose.position.x, path.poses[i].pose.position.y);
                //ROS_INFO("Angle %f", atan2(path.poses[i].pose.position.y-pose.y, path.poses[i].pose.position.x)-pose.theta);
                index = i;
                dist = distance;
                //return i;
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
                //ROS_INFO("Index %d, x: %f y: %f", i, path.poses[i].pose.position.x, path.poses[i].pose.position.y);
                //ROS_INFO("Angle %f", atan2(path.poses[i].pose.position.y-pose.y, path.poses[i].pose.position.x)-pose.theta);
                index = i;
                //return i;
            }
        }

        if(sqrt(pow(pose.x-path.poses[path.poses.size()-1].pose.position.x, 2) + pow(pose.y-path.poses[path.poses.size()-1].pose.position.y, 2) <= look_ahead))
                goal_reached = true;

        if(index > 0) return index;
        else return -1;

    }

    void makeTurn(int angle) {
        robot_msgs::MakeTurn turn_srv;
        if (angle < 0) {
            turn_srv.request.state = RIGHT_TURN;
        } else {
            turn_srv.request.state = LEFT_TURN;
        }
        turn_srv.request.degrees = abs(angle);

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

private:
    nav_msgs::Path path;
    ras_arduino_msgs::Odometry pose;
    double look_ahead;
    bool goal_reached;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "path_navigation");

    Navigation nav;

    ros::Rate loop_rate(RATE);
    while (!nav.reachedGoal()) {
        ros::spinOnce();
        nav.followPath();
        loop_rate.sleep();
    }

    return 0;
}
