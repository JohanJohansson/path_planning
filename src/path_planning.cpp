#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

enum {WALL = 300000, NOTHING = 0, ROBOT = 300001, GOAL = 1};
//int width = 6, height = 6;
/*int map[6][6]=	{{0,0,0,0,0,0},
                 {0,0,0,0,0,0},
                 {0,0,0,0,0,0},
                 {300000,300000,300000,300000,0,0},
                 {0,0,0,0,0,0},
                 {0,0,0,0,0,0}};*/

//int goal_x=0, goal_y=3, robot_x=5, robot_y=3, map_x, map_y, count;
int goal_x, goal_y, robot_x, robot_y, map_x, map_y, count;
int width = 500, height = 500;
int minimum_node = 300000, reset_node = 300000, minimum_location = 300000;
int new_state = 1, old_state = 1;
int tmp_x, tmp_y;
int map[500][500];
//int map2[36]=	{0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,300000,300000,300000,300000,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//int map[6][6];

nav_msgs::OccupancyGrid map2;

void MapCallback(const nav_msgs::OccupancyGrid &msg)
{
    map2.data = msg.data;
    for(tmp_x=0;tmp_x<height;tmp_x++)
        for(tmp_y=0;tmp_y<width;tmp_y++)
        {
            map[tmp_x][tmp_y]=map2.data[tmp_x*width+tmp_y];
            if(map[tmp_x][tmp_y]==150 || map[tmp_x][tmp_y]==-1)
                map[tmp_x][tmp_y]=300000;
        }
}

void print_map()
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
}

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
    printf("before unpropogate:\n");
    print_map();
    for(map_x=0; map_x<height; map_x++)
        for(map_y=0; map_y<width; map_y++)
            if(map[map_x][map_y]!=WALL && map[map_x][map_y]!=GOAL)
                map[map_x][map_y] = NOTHING;

    map[robot_x][robot_y] = ROBOT;
    printf("Unpropogate:\n");
    print_map();
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
        printf("Sweep #: %d\n",count+1);
        print_map();
        count++;
    }
    return 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planning");
    ros::NodeHandle n;
    ros::Subscriber map_sub = n.subscribe("/gridmap", 1, MapCallback);
 /*   for(tmp_x=0;tmp_x<6;tmp_x++)
          for(tmp_y=0;tmp_y<6;tmp_y++)
              map[tmp_x][tmp_y]=map2[tmp_x*6+tmp_y];*/

    print_map();
    while(map[robot_x][robot_y]!=GOAL)
    {
        new_state=propagate_wavefront(robot_x,robot_y);
        switch(new_state)
        {
        case 1:
            robot_x++;
            break;
        case 2:
            robot_x--;
            break;
        case 3:
            robot_y++;
            break;
        case 4:
            robot_y--;
        }
        old_state = new_state;
    }
    return 0;
}
