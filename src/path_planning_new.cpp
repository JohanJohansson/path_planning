#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ras_arduino_msgs/Odometry.h>
#include <robot_msgs/detectedObject.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>

class pathPlanner
{
public:
    ros::NodeHandle n;
    ros::Subscriber costMap_subscriber;
    ros::Publisher newMap_publisher;

    enum {WALL = 300000, NOTHING = 0, ROBOT = 300001, GOAL = 1};
    long map_[500][500];
    int width_map, height_map;
    long minimum_node, reset_node, minimum_location;
    int new_state, old_state;
    long steps, i;
    int robo_path_ori_x[250000], robo_path_ori_y[250000];
    int goal_x, goal_y, robot_X, robot_Y, map_x, map_y, count;
    bool haveMap;

    nav_msgs::OccupancyGrid newMap, pathMap;

    pathPlanner()
    {
        n = ros::NodeHandle();
    }

    ~pathPlanner()
    {}

    void init()
    {
        costMap_subscriber = n.subscribe("/costmap", 1, &pathPlanner::mapCallback,this);
        newMap_publisher = n.advertise<nav_msgs::OccupancyGrid>("/testMap", 1);
        width_map = 500;
        height_map = 500;
        minimum_node = 300000;
        reset_node = 300000;
        minimum_location = 300000;
        new_state = 1;
        old_state = 1;
        goal_x = 250;
        goal_y = 240;
        robot_X = 250;
        robot_Y = 250;
        haveMap = false;

    }

    void mapCallback(const nav_msgs::OccupancyGrid &map_msg)
    {
        if(haveMap==false){
        newMap = map_msg;
        haveMap = true;

        for(int tmp_x = 0; tmp_x <= height_map; tmp_x++)
        {
            for(int tmp_y = 0; tmp_y <= width_map; tmp_y++)
            {
                map_[tmp_x][tmp_y] = int(newMap.data[tmp_x+width_map*tmp_y]);

                if(map_[tmp_x][tmp_y]==150 || map_[tmp_x][tmp_y]==-1)
                    map_[tmp_x][tmp_y]=300000;
            }
        }
//        for(int i = 0; i <= height_map; i++)
//            for(int j = 0; j <= width_map; j++)
//            {
//                map_[i][]
//            }

        //newMap_publisher.publish(newMap);
        getMap();

        }


    }

    int min_neighbour(int map_x, int map_y)
    {
        //down
        minimum_node = reset_node;
        if(map_x < (width_map-1))
            if(map_[map_x+1][map_y]<minimum_node && map_[map_x+1][map_y]!=NOTHING)
            {
                minimum_node = map_[map_x+1][map_y];
                minimum_location = 1;
            }
        //up
        if(map_x > 0)
            if(map_[map_x-1][map_y]<minimum_node && map_[map_x-1][map_y]!=NOTHING)
            {
                minimum_node = map_[map_x-1][map_y];
                minimum_location = 2;
            }
        //right
        if(map_y < (height_map-1))
            if(map_[map_x][map_y+1]<minimum_node && map_[map_x][map_y+1]!=NOTHING)
            {
                minimum_node = map_[map_x][map_y+1];
                minimum_location = 3;
            }
        //left
        if(map_y > 0)
            if(map_[map_x][map_y-1]<minimum_node && map_[map_x][map_y-1]!=NOTHING)
            {
                minimum_node = map_[map_x][map_y-1];
                minimum_location = 4;
            }
        return minimum_node;
    }
    void unpropagate_wavefront(int robot_x, int robot_y)
    {
        for(map_x=0; map_x<height_map; map_x++)
            for(map_y=0; map_y<width_map; map_y++)
                if(map_[map_x][map_y]!=WALL && map_[map_x][map_y]!=GOAL)
                    map_[map_x][map_y] = NOTHING;

        map_[robot_x][robot_y] = ROBOT;
    }

    int propagate_wavefront(int robot_x, int robot_y)
    {
        unpropagate_wavefront(robot_x, robot_y);
        map_[goal_x][goal_y] = GOAL;

        count = 0;
        while(count<250000)
        {
            map_x = 0;
            map_y = 0;
            while(map_x<height_map && map_y<width_map)
            {
                if(map_[map_x][map_y]!=WALL && map_[map_x][map_y]!=GOAL)
                {
                    if(min_neighbour(map_x,map_y)<reset_node && map_[map_x][map_y]==ROBOT) //finished pathing
                    {
                        return minimum_location;
                    }
                    else if(minimum_node != reset_node)
                        map_[map_x][map_y] = minimum_node + 1;
                }
                map_y++;
                if(map_y==width_map && map_x!=height_map)
                {
                    map_x++;
                    map_y = 0;
                }
            }
            count++;
        }
        return 0;
    }

    void getMap()
    {
        steps = 0;
        robo_path_ori_x[0] = robot_X;
        robo_path_ori_y[0] = robot_Y;
        while(map_[robot_X][robot_Y]!=GOAL)
        {
            new_state=propagate_wavefront(robot_X,robot_Y);
            switch(new_state)
            {
            case 1:
                robot_X++; //down
                break;
            case 2:
                robot_X--; //up
                break;
            case 3:
                robot_Y++; //right
                break;
            case 4:
                robot_Y--; //left
            }
            old_state = new_state;
            steps++;
            robo_path_ori_x[steps] = robot_X;
            robo_path_ori_y[steps] = robot_Y;
        }
        //nav_msgs::Path path;
        //path.poses.resize(steps);

        int robo_path_x[steps], robo_path_y[steps];
        for(int i = 0; i <= steps; i++)
        {

            robo_path_x[i] = robo_path_ori_x[i];
            robo_path_y[i] = robo_path_ori_y[i];
//            newMap.data[robo_path_ori_x[i] + robo_path_ori_y[i]*width_map] = 110;

//            path.poses[i].pose.position.x = robo_path_ori_x[i];
//            path.poses[i].pose.position.y = robo_path_ori_y[i];
            printf("x: %d, y: %d\n", robo_path_x[i], robo_path_y[i]);
        }
        //newMap_publisher.publish(newMap);
    }

private:

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planning_new");
    ros::NodeHandle n;

    pathPlanner plan;

    plan.init();

    ros::Rate loop_rate(20.0);

    while(plan.n.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}

