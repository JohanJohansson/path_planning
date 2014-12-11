#include "ros/ros.h"
#include "math.h"
#include "nav_msgs/OccupancyGrid.h"
#include "vector"
#include "visualization_msgs/Marker.h"
#include "robot_msgs/detectedObject.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

class GoalFinder
{
public:
    ros::NodeHandle n;
    ros::Subscriber costmap_subscriber;
    //ros::Subscriber object_subscriber;
    ros::Publisher new_obj_publisher;
    ros::Publisher obj_test_publisher;
    std::vector<robot_msgs::detectedObject> objects;
    robot_msgs::detectedObject::ConstPtr obj;
    nav_msgs::OccupancyGrid costmap, testmap;
    robot_msgs::detectedObject new_obj_pose;
    std::vector<signed char> cost_vector;
    int pos_cell_x, pos_cell_y, max_cells;
    double resolution, width_map;
    int obj_cell_x, obj_cell_y;
    int count;
    int cellNumber;
    int findGoal;

    GoalFinder()
    {
        n = ros::NodeHandle();
    }

    ~GoalFinder()
    {}
    void init()
    {
        costmap_subscriber = n.subscribe("/costmap", 1, &GoalFinder::getMapCallback, this);
        //object_subscriber = n.subscribe("/allObjects", 1, &GoalFinder::getObjects, this);
        //pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/loc/pose", 1);
        new_obj_publisher = n.advertise<robot_msgs::detectedObject>("/goal_obj", 1);
        obj_test_publisher = n.advertise<nav_msgs::OccupancyGrid>("/objInMap", 1);

        resolution = 0.02; //[m]
        width_map = 500;
        cellNumber = 500*500;
        cost_vector = std::vector<signed char>(cellNumber);
        findGoal = 0;

    }

    void getObjects()
    {

        rosbag::Bag bag;
        bag.open("/home/ras/catkin_ws/src/mapping/bagfiles/map_test_2top.bag", rosbag::bagmode::Read);

        rosbag::View obj_view(bag, rosbag::TopicQuery("/allObjects"));

        for (rosbag::View::iterator m = obj_view.begin(); m != obj_view.end(); m++)
        {
            obj = (*m).instantiate<robot_msgs::detectedObject>();
            objects.push_back(*obj);
        }
        ROS_INFO("Received %ld objects", objects.size());
        bag.close();
    }

    void getMapCallback(const nav_msgs::OccupancyGrid &map_msg)
    {        
        costmap = map_msg;
        //obj_test_publisher.publish(costmap);
        cost_vector = costmap.data;
        if(findGoal == 0)
            findGoalForPath();
    }

    void findGoalForPath()
    {
        double temp_x, temp_y;
        testmap.data = costmap.data;
        for (std::vector<robot_msgs::detectedObject>::iterator it = objects.begin(); it != objects.end(); ++it)
        {
            robot_msgs::detectedObject Obj = *it;
            temp_x = Obj.position.x/resolution;
            temp_y = Obj.position.y/resolution;
            pos_cell_x = floor(temp_x);
            pos_cell_y = floor(temp_y);
            ROS_INFO("object in x: %d", pos_cell_x);
            ROS_INFO("object in y: %d", pos_cell_y);
            if(cost_vector[pos_cell_x + width_map*pos_cell_y] != 0)
            {
                count = 1;
                while(count <= 10)
                {
                    if(cost_vector[(pos_cell_x+count) + width_map*(pos_cell_y)] == 0)
                    {
                        obj_cell_x = pos_cell_x + count;
                        obj_cell_y = pos_cell_y;
                        break;
                    } //1
                    else if(cost_vector[(pos_cell_x+count) + width_map*(pos_cell_y+count)] == 0)
                    {
                        obj_cell_x = pos_cell_x + count;
                        obj_cell_y = pos_cell_y + count;
                        break;
                    } //2
                    else if(cost_vector[(pos_cell_x) + width_map*(pos_cell_y+count)] == 0)
                    {
                        obj_cell_x = pos_cell_x;
                        obj_cell_y = pos_cell_y + count;
                        break;
                    } //3
                    else if(cost_vector[(pos_cell_x-count) + width_map*(pos_cell_y+count)] == 0)
                    {
                        obj_cell_x = pos_cell_x - count;
                        obj_cell_y = pos_cell_y + count;
                        break;
                    } //4
                    else if(cost_vector[(pos_cell_x-count) + width_map*(pos_cell_y)] == 0)
                    {
                        obj_cell_x = pos_cell_x - count;
                        obj_cell_y = pos_cell_y;
                        break;
                    } //5
                    else if(cost_vector[(pos_cell_x-count) + width_map*(pos_cell_y-count)] == 0)
                    {
                        obj_cell_x = pos_cell_x - count;
                        obj_cell_y = pos_cell_y - count;
                        break;
                    } //6
                    else if(cost_vector[(pos_cell_x) + width_map*(pos_cell_y-count)] == 0)
                    {
                        obj_cell_x = pos_cell_x;
                        obj_cell_y = pos_cell_y - count;
                        break;
                    } //7
                    else if(cost_vector[(pos_cell_x+count) + width_map*(pos_cell_y-count)] == 0)
                    {
                        obj_cell_x = pos_cell_x + count;
                        obj_cell_y = pos_cell_y - count;
                        break;
                    } //8
                    else
                    {
                        count++;
                    }
                }
            }
            else
            {
                obj_cell_x = pos_cell_x;
                obj_cell_y = pos_cell_y;
            }
            new_obj_pose.object_id = Obj.object_id;

            //Visualiing where closest cell is
            testmap.data[obj_cell_x + width_map*obj_cell_y] = 110;
            new_obj_pose.header.frame_id = "map";
            new_obj_pose.position.x = obj_cell_x;
            new_obj_pose.position.y = obj_cell_y;
            ROS_INFO("Object: %s", new_obj_pose.object_id.c_str());
            ROS_INFO("closest cell_x: %f", new_obj_pose.position.x );
            ROS_INFO("closest cell_y: %f", new_obj_pose.position.y );
            new_obj_publisher.publish(new_obj_pose);

        }
        findGoal = 1;
        obj_test_publisher.publish(testmap);

    }

    void testmapInit()
    {
        testmap.header.frame_id = "map";
        testmap.header.stamp = ros::Time(0);
        testmap.info.height = width_map;
        testmap.info.width = width_map;
        testmap.info.resolution = resolution;
    }

private:


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_finding");

    GoalFinder find;
    find.init();
    find.getObjects();
    find.testmapInit();

    ros::Rate loop_rate(20.0);

    while(find.n.ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
