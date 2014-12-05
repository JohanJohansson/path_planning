#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ras_arduino_msgs/Odometry.h>
#include <robot_msgs/detectedObject.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

enum {WALL = 300000, NOTHING = 0, ROBOT = 300001, GOAL = 1};
//int width = 6, height = 6;
//int map[6][6]= {{0,0,0,0,0,0},
//                {0,0,0,0,0,0},
//                {0,0,0,0,0,0},
//                {300000,300000,300000,300000,0,0},
//                {0,0,0,0,0,0},
//                {0,0,0,0,0,0}};
//int goal_x=0, goal_y=3, robot_x=5, robot_y=3, map_x, map_y, count;
//int map2[36]= {0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,300000,300000,300000,300000,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//int map[6][6];
int goal_x=150, goal_y=150, robot_x=250, robot_y=250, map_x, map_y, count;
int width = 500, height = 500;
int map[500][500];
int minimum_node = 300000, reset_node = 300000, minimum_location = 300000;
int new_state = 1, old_state = 1;
int tmp_x, tmp_y, steps, i;
int robo_path_ori_x[250000], robo_path_ori_y[250000];
nav_msgs::OccupancyGrid map2;

/*void RoboCallback(const ras_arduino_msgs::Odometry &msg)
{
    robot_x = msg.x;
    robot_y = msg.y;
}*/
//robot_msgs::detectedObject
/*void GoalCallback(const robot_msgs::detectedObject &msg)
{
goal_x = msg.x;
goal_y = msg.y;
}*/

void MapCallback(const nav_msgs::OccupancyGrid &msg)
{
    map2.data = msg.data;
    for(tmp_x=0;tmp_x<height;tmp_x++)
        for(tmp_y=0;tmp_y<width;tmp_y++)
        {
            map[tmp_x][tmp_y]=map2.data[tmp_x*width+tmp_y];
            if(map[tmp_x][tmp_y]==150 || map[tmp_x][tmp_y]==-1)
                map[tmp_x][tmp_y]=WALL;
        }
    ROS_INFO("Map called");
}

/*void print_map()
{
    for(tmp_x=0;tmp_x<height;tmp_x++)
    {
        for(tmp_y=0;tmp_y<width;tmp_y++)
        {
            if(map[tmp_x][tmp_y]==WALL)
                printf("W ");
            else if(map[tmp_x][tmp_y]==ROBOT)
                printf("R ");
            else if(map[tmp_x][tmp_y]==GOAL)
                printf("G ");
            else
                printf("%d ", map[tmp_x][tmp_y]);
        }
        printf("\n");
    }
    printf("\n");
}*/

int min_neighbour(int map_x, int map_y)
{
    //down
    minimum_node = reset_node;
    if(map_x < (width-1))
        if(map[map_x+1][map_y]<minimum_node && map[map_x+1][map_y]!=NOTHING)
        {
            minimum_node = map[map_x+1][map_y];
            minimum_location = 1;
        }
    //up
    if(map_x > 0)
        if(map[map_x-1][map_y]<minimum_node && map[map_x-1][map_y]!=NOTHING)
        {
            minimum_node = map[map_x-1][map_y];
            minimum_location = 2;
        }
    //right
    if(map_y < (height-1))
        if(map[map_x][map_y+1]<minimum_node && map[map_x][map_y+1]!=NOTHING)
        {
            minimum_node = map[map_x][map_y+1];
            minimum_location = 3;
        }
    //left
    if(map_y > 0)
        if(map[map_x][map_y-1]<minimum_node && map[map_x][map_y-1]!=NOTHING)
        {
            minimum_node = map[map_x][map_y-1];
            minimum_location = 4;
        }
    return minimum_node;
}

void unpropagate_wavefront(int robot_x, int robot_y)
{
//    printf("before unpropogate:\n");
//    print_map();
    for(map_x=0; map_x<height; map_x++)
        for(map_y=0; map_y<width; map_y++)
            if(map[map_x][map_y]!=WALL && map[map_x][map_y]!=GOAL)
                map[map_x][map_y] = NOTHING;
    map[robot_x][robot_y] = ROBOT;
 //   printf("Unpropogate:\n");
//    print_map();
}

int propagate_wavefront(int robot_x, int robot_y)
{
    unpropagate_wavefront(robot_x, robot_y);
    map[goal_x][goal_y] = GOAL;
    count = 0;
    while(count<250000)
    {
        map_x = 0;
        map_y = 0;
        while(map_x<height && map_y<width)
        {
            if(map[map_x][map_y]!=WALL && map[map_x][map_y]!=GOAL)
            {
                if(min_neighbour(map_x,map_y)<reset_node && map[map_x][map_y]==ROBOT) //finished pathing
                {
                    return minimum_location;
                }
                else if(minimum_node != reset_node)
                    map[map_x][map_y] = minimum_node + 1;
            }
            map_y++;
            if(map_y==width && map_x!=height)
            {
                map_x++;
                map_y = 0;
            }
        }
        // printf("Sweep #: %d\n",count+1);
        // print_map();
        count++;
    }
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planning");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);
    // ros::Subscriber robo_sub = n.subscribe("/arduino/odometry_estimate", 1, RoboCallback);
    // ros::Subscriber goal_sub = n.subscribe("/destination", 1, GoalCallback);
    ros::Subscriber map_sub = n.subscribe("/costmap", 1, MapCallback);
    ros::Publisher mark_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  //  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/path", 1);
    visualization_msgs::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = ros::Time();
    line.ns = "path";
    //line.id = 0;

    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;

    line.pose.orientation.x = 0.0;
    line.pose.orientation.y = 0.0;
    line.pose.orientation.z = 0.0;
    line.pose.orientation.w = 1.0;

    line.pose.position.x = 0;
    line.pose.position.y = 0;
    line.pose.position.z = 0;

    line.scale.x = 0.1;
    line.scale.y = 0.1;
    line.scale.z = 0.1;
    line.color.a = 1.0;
    line.color.r = 0.0;
    line.color.g = 1.0;
    line.color.b = 0.0;
   // vis_pub.publish(line);

   // print_map();
    steps = 0;
    robo_path_ori_x[0] = robot_x;
    robo_path_ori_y[0] = robot_y;
    while(ros::ok())
    {
        while(map[robot_x][robot_y]!=GOAL)
        {
            new_state=propagate_wavefront(robot_x,robot_y);
            switch(new_state)
            {
            case 1:
                robot_x++; //down
                break;
            case 2:
                robot_x--; //up
                break;
            case 3:
                robot_y++; //right
                break;
            case 4:
                robot_y--; //left
            }
            old_state = new_state;
            steps++;
            robo_path_ori_x[steps] = robot_x;
            robo_path_ori_y[steps] = robot_y;
        }
      //  nav_msgs::Path path;
      //  path.poses.resize(steps);
        int robo_path_x[steps], robo_path_y[steps];
        for(i=0;i<=steps;i++)
        {
            robo_path_x[i] = robo_path_ori_x[i];
            robo_path_y[i] = robo_path_ori_y[i];
            // geometry_msgs::Pose point;
            // path.poses.pose.position.x = robo_path_ori_x[i];
            // path.poses.pose.position.y = robo_path_ori_y[i];
            // path.poses[i].pose.position.x = robo_path_ori_x[i];
            // path.poses[i].pose.position.y = robo_path_ori_y[i];
            printf("x: %d, y: %d\n", robo_path_x[i], robo_path_y[i]);
            geometry_msgs::Point p;
            line.id = i;
            p.x = robo_path_x[i];
            p.y = robo_path_y[i];

            line.points.push_back(p);
        }
        mark_pub.publish(line);
       // path_pub.publish(path);
        ros::spinOnce();
        loop_rate.sleep();
    return 0;
}
}
