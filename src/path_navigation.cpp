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

//enum {LEFT_TURN = 1, RIGHT_TURN = 2};

#define RATE 20.0

#define INVALID 1000

enum {FORWARD = 0, LEFT_TURN = 1, RIGHT_TURN = 2, FOLLOW_LEFT = 3, FOLLOW_RIGHT = 4, TWO_LEFT = 5, SMALL_EDGE_TURN = 6};

int front_left, front_right, back_left, back_right, forward_left, forward_right, state;

class MazeController {

public:

    ros::NodeHandle n;
    ros::Publisher twist_pub;
    ros::Subscriber distance_sub;
    ros::Subscriber encoder_sub;
    int previous_state;
    int previous_sensor_reading[2];

    //Constructor
    MazeController() {
        n = ros::NodeHandle();
        distance_sub = n.subscribe("/ir_sensor_cm", 1, &MazeController::MazeCallback, this);
        twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
        encoder_sub = n.subscribe("/arduino/encoders", 1, &MazeController::EncoderCallback, this);
        turn_client = n.serviceClient<robot_msgs::MakeTurn>("/make_turn");
        follow_client = n.serviceClient<robot_msgs::FollowWall>("/follow_wall");
        previous_state = FORWARD;
        previous_sensor_reading[0] = 0;
        previous_sensor_reading[1] = 0;

    }

    //Destructor
    ~MazeController() {}

    //Callback for using IR sensor values
    void MazeCallback(const ras_arduino_msgs::ADConverterConstPtr &msg) {
        front_left = msg->ch1;
        front_right = msg->ch2;
        back_left = msg->ch3;
        back_right = msg->ch4;
        forward_right = msg->ch5;
        forward_left = msg->ch6;
    }

    //Callback for the encoders
    void EncoderCallback(const ras_arduino_msgs::EncodersConstPtr &msg) {
        delta_encoder_left = msg->delta_encoder2;
        delta_encoder_right = msg->delta_encoder1;
        //ROS_INFO("Delta left: %d Delta right: %d", delta_encoder_left, delta_encoder_right);
    }

    //Method to make the robot drive forward
    void forward() {

        msg.linear.x = 0.13; //TODO Try increased speed forward and turn. Try 0.15 linear x
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;

        twist_pub.publish(msg);
    }

    //Method for driving the robot a specific distance
    void forward(double distance) {
        ROS_INFO("Drives forward %f cm", distance);
        double wheel_radius = 5.0;

        double distance_per_wheel = 2*M_PI*wheel_radius;
        double distance_per_tick = distance_per_wheel/360;

        int ticks = nearbyint(distance/distance_per_tick);

        msg.linear.x = 0.13;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;

        int left_encoder = 0;
        int right_encoder = 0;

        int tresh_front = 17;

        ros::Rate loop_rate(20);
        while (abs(left_encoder) < ticks && abs(right_encoder) < ticks) {

            ros::spinOnce();

            if ((forward_left < tresh_front &&
                forward_left > 0) ||
                (forward_right < tresh_front &&
                forward_right > 0)) break;

            left_encoder += delta_encoder_left;
            right_encoder += delta_encoder_right;

            twist_pub.publish(msg);

            loop_rate.sleep();
        }
    }

    //Sends service message through chosen client to perform desired task
    void setClientCall(int state) {
        if (state == LEFT_TURN || state == RIGHT_TURN) {
            srv_turn.request.state = state;
            srv_turn.request.degrees = 87;//85;
            if (turn_client.call(srv_turn)) {
                ROS_INFO("Succesfully called turn service");
              }
              else
              {
                ROS_ERROR("Failed to call turn service in maze_follower");
              }
        }
        else if (state == FOLLOW_LEFT || state == FOLLOW_RIGHT) {
            srv_follow.request.state = state;
            if (follow_client.call(srv_follow)) {
                ROS_INFO("Succesfully called follow wall service");
              }
              else
              {
                ROS_ERROR("Failed to call follow service in maze_follower");
              }
        }
    }

    //Outputs if the state changes
    void changeState(int s) {
    msg.linear.x = 0;
    msg.angular.z = 0;
    twist_pub.publish(msg);
        std::cout << "want to change the state " << s << std::endl;
        std::cin.ignore();
        state = s;
    }

    //Check if there is a rapid change in distance measured by ir sensors on the left side
    bool checkSensorsDistanceLeft() {
        if (abs(front_left - previous_sensor_reading[0]) > 8) {
            return false;
        }

        return true;
    }

   //Check if there is a rapid change in distance measured by ir sensors on the right side
    bool checkSensorsDistanceRight() {
        if (abs(front_right - previous_sensor_reading[0]) > 8) {
            return false;
        }

        return true;
    }

    //Checks whenever the two sensors have registered an edge of wall
    void checkSensorsTurn() {
        bool back = false;
        bool front = false;
        ros::Rate loop_rate(10);

        while (!back || !front) {
            ros::spinOnce();
            //if (front_left < 25 && back_left < 25) break; //Necessary?
            if (wallInFront()) {
                if (front == true && back == false) {
                    setClientCall(RIGHT_TURN);
                }
                break;
            } else {
                if (front_left < 20) front = true; //TODO Try these new values for wall edge detection. Old value was 15
                if (back_left < 20) back = true;
                ROS_INFO("front: %d back: %d", front, back);
                forward();
            }
            loop_rate.sleep();
        }
    }

    void checkBackSensorTurn(int prev_state) {

        int back_sensor, front_sensor;
    ros::spinOnce();
        if (prev_state == RIGHT_TURN) {
        ROS_INFO("Want to detect wall with back left sensor");
            previous_sensor_reading[1] = back_left;
            back_sensor = back_left;
            front_sensor = front_left;
        } else {
            ROS_INFO("Want to detect wall with back right sensor");
            previous_sensor_reading[1] = back_right;
            back_sensor = back_right;
            front_sensor = front_right;
        }
        bool back = false;
        ros::Rate loop_rate(10);

    ROS_INFO("back sensor: %d", back_sensor);

        while (!back) {
            ros::spinOnce();
            if (wallInFront()) return;
            if (prev_state == RIGHT_TURN) {
                back_sensor = back_left;
                front_sensor = front_left;
            } else {
                back_sensor = back_right;
                front_sensor = front_right;
            }
            if (abs(back_sensor - previous_sensor_reading[1]) > 20) back = true; //TODO Try different values for wall edge detection. Old value was 15
            ROS_INFO("back: %d", back);
            ROS_INFO("Value: %d", back_sensor);
            forward();
            loop_rate.sleep();
        }

        if (front_sensor > 25) {
            forward(15.0);
            if(prev_state == LEFT_TURN) setClientCall(RIGHT_TURN);
            else setClientCall(LEFT_TURN);
            forward(20);
        }
    }

    bool wallInFront() {
        int tresh_front = 15;//17;
        if ((forward_left < tresh_front &&
                forward_left >= 0) ||
                (forward_right < tresh_front && forward_right >= 0) ||
                (forward_left > 40 && forward_right < 20) ||
                (forward_right > 40 && forward_left < 20)) {
            return true;
        }
        return false;
    }

    double wallTooClose() {
        if ((front_left < 9 && front_left > 0 && back_left < 9 && back_left > 0) &&
                front_right < 9 && front_right > 0 && back_right < 9 && back_right > 0)
            return 0.0;
        else if (front_left < 9 && front_left > 0 && back_left < 9 && back_left > 0) {
            return -0.314*0.7;
        } else if (front_right < 9 && front_right > 0 && back_right < 9 && back_right > 0) {
            return 0.314*0.7;
        } else {
	    return 0.0;
	}
    }

    //Checks the ir sensors which decides what state to use
    int chooseState() {
        int s;

        //Checks if robot is close to a wall do turn, else follow wall or go forward
        if (wallInFront()) {
                /*if (front_left > front_right ||
                       back_left > back_right) {
                    s = LEFT_TURN;
                } else
                    s = RIGHT_TURN;*/
        if (front_right > front_left ||
                       back_right > back_left) {
                    s = RIGHT_TURN;
                } else
                    s = LEFT_TURN;
        } else { //If no wall seen in front of robot
             if (front_left < front_right &&
                   back_left < back_right &&
                   front_left < 25 &&
                   back_left < 25) {
                s = FOLLOW_LEFT;
        } else if (front_right < front_left &&
                      back_right < back_left &&
                      front_right < 25 &&
                      back_right < 25) {
                s = FOLLOW_RIGHT;
        } else
                s = FORWARD;
        }
    //ROS_INFO("State: %d", s);
        //Decides the state depending on the state transition
        if (previous_state == FOLLOW_LEFT && s == FORWARD) {
             s = TWO_LEFT;
        } else if (previous_state == LEFT_TURN && s != FOLLOW_RIGHT) {//(s == FORWARD || s == FOLLOW_RIGHT)) {
            ros::spinOnce();
            ROS_INFO("SMALL_EDGE_TURN left");
            if (front_right > 25 || back_right > 25) {
                s = SMALL_EDGE_TURN;
            }
        } else if (previous_state == RIGHT_TURN && s != FOLLOW_LEFT) {//(s == FORWARD || s == FOLLOW_LEFT)) {
            ros::spinOnce();
            ROS_INFO("SMALL_EDGE_TURN right");
            if (front_left > 25 || back_left > 25) {
                s = SMALL_EDGE_TURN;
            }
        }

        /*if (s != previous_state) {
            changeState(s);
        }*/

    ROS_INFO("State: %d", s);

        return s;
    }

    bool robotReady() {
        if (front_left != 0 ||
                front_right != 0 ||
                back_left != 0 ||
                back_right != 0 ||
                forward_left != 0 ||
                forward_right != 0) return true;
        else false;
    }

private:
    ros::ServiceClient turn_client;
    ros::ServiceClient follow_client;
    robot_msgs::MakeTurn srv_turn;
    robot_msgs::FollowWall srv_follow;

    geometry_msgs::Twist msg;
    int delta_encoder_left;
    int delta_encoder_right;
};

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
    MazeController mc;
    ros::Publisher costMap_publisher;

    Navigation() {
        n = ros::NodeHandle();
        path_sub = n.subscribe("/path", 1, &Navigation::pathCallback, this);
        pose_sub = n.subscribe("/arduino/odometry", 1, &Navigation::odometryCallback, this);
        twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
        turn_client = n.serviceClient<robot_msgs::MakeTurn>("/make_turn");
        cost_map_sub = n.subscribe("/costmap", 1, &Navigation::costMapCallback, this);
        path_server = n.advertiseService("/use_path_follower", &Navigation::findAndFollowPath, this);
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        costMap_publisher = n.advertise<nav_msgs::OccupancyGrid>("/test_costmap", 1);

        mc = MazeController();

        look_ahead = 0.2;
        goal_reached = false;

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

    void pathCallback(const nav_msgs::Path &msg) {
        path = msg;
    }

    void odometryCallback(const ras_arduino_msgs::Odometry &msg) {
        pose = msg;
        //ROS_INFO("Pose x: %f y: %f theta: %f", pose.x, pose.y, pose.theta);
    }

    void costMapCallback(const nav_msgs::OccupancyGrid &msg) {
        cost_map = msg;
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

        double angle = atan2(path.poses[index].pose.position.y-pose.y, path.poses[index].pose.position.x-pose.x);
        double angle_v = atan2(y, x);

        double rotate;

        double dist = sqrt(pow(path.poses[index_closest].pose.position.x-pose.x,2) + pow(path.poses[index_closest].pose.position.y-pose.y,2));
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
            double temp = mc.wallTooClose();
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

    bool findAndFollowPath(robot_msgs::useMazeFollower::Request &req, robot_msgs::useMazeFollower::Response &res) {

        if (req.go == true) {
            ROS_INFO("Wants to plan a path.");
            geometry_msgs::Pose goal, start;
            std::vector<geometry_msgs::Pose> path;
            //nav_msgs::Path path;
            double resolution = 0.02;
            double center_x = 5.0;
            double center_y = 5.0;
            goal.position.x = 250;
            goal.position.y = 250;
            start.position.x = floor((center_x + pose.x)/resolution);
            start.position.y = floor((center_y + pose.y)/resolution);
            ROS_INFO("Start x: %f y: %f \n Goal x: %f y: %f", start.position.x, start.position.y, goal.position.x, goal.position.y);

            nav_msgs::OccupancyGrid temp = cost_map;
            temp.info.height = 500;
            temp.info.width = 500;

            costMap_publisher.publish(temp);
            planPathGoal(temp, start, goal, path);

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

            ROS_INFO("Length of path %lu", path.size());
            for (int i = 0; i < path.size(); i++) {
                geometry_msgs::Point p;
                p.x = path[i].position.x*resolution-5.0;
                p.y = path[i].position.y*resolution-5.0;
                p.z = 0;
                ROS_INFO("Cells x: %f y: %f", p.x, p.y);
                points.points.push_back(p);
            }
            ros::Rate loop_rate(RATE);
            while (!reachedGoal()) {
                ros::spinOnce();
                nav.followPath();
                loop_rate.sleep();
            }

            stop();
            //ROS_INFO("Number of points %lu", points.points.size());
            marker_pub.publish(points);
        } else ROS_INFO("Don't plan a path");

        return true;
    }

private:
    nav_msgs::Path path;
    ras_arduino_msgs::Odometry pose;
    double look_ahead;
    bool goal_reached;
    geometry_msgs::Twist twist;
    nav_msgs::OccupancyGrid cost_map;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "path_navigation");

    Navigation nav;

    ros::Duration(2.0).sleep();
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
