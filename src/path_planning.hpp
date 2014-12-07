#ifndef PATH_PLANNING_HPP
#define PATH_PLANNING_HPP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ras_arduino_msgs/Odometry.h>
#include <robot_msgs/detectedObject.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <queue>

struct cell {
    int x, y;
    int cost;

    cell() : x(0), y(0), cost(0) {}
    cell(int y, int x, int cost) : x(x), y(y), cost(cost) {}
    cell(int y, int x) : x(x), y(y), cost(0) {}

    cell operator+(const cell& other) const {
        return cell(y + other.y, x + other.x, cost + other.cost);
    }
};

enum {WALL = 100, UNEXPLORED = -1, UP = 0, RIGHT = 1, DOWN = 2, LEFT = 3};
cell up(-1, 0);
cell right(0, 1);
cell down(1, 0);
cell left(0, -1);
cell directions[] = {up, right, down, left};

bool withinBounds(int rows, int cols, const cell& c) {
    return c.x >= 0 && c.x < cols && c.y >= 0 && c.y < rows;
}

void flood(std::vector<std::vector<int> >& map, int rows, int cols, const cell& goal) {
    map[goal.y][goal.x] = 2;
    std::queue<cell> floodQue;
    floodQue.push(goal);

    while(!floodQue.empty()) {
        cell& cc = floodQue.front();

        if(cc.y > 0 && map[cc.y-1][cc.x] == 0) {
            map[cc.y-1][cc.x] = cc.cost + 1;
            floodQue.push(cell(cc.y-1, cc.x, cc.cost+1));
        }
        if(cc.y < rows-1 && map[cc.y+1][cc.x] == 0) {
            map[cc.y+1][cc.x] = cc.cost + 1;
            floodQue.push(cell(cc.y+1, cc.x, cc.cost+1));
        }
        if(cc.x > 0 && map[cc.y][cc.x-1] == 0) {
            map[cc.y][cc.x-1] = cc.cost + 1;
            floodQue.push(cell(cc.y, cc.x-1, cc.cost+1));
        }
        if(cc.x < cols-1 && map[cc.y][cc.x+1] == 0) {
            map[cc.y][cc.x+1] = cc.cost + 1;
            floodQue.push(cell(cc.y, cc.x+1, cc.cost+1));
        }
        floodQue.pop();
    }
}

cell nextCell(const std::vector<std::vector<int> >& map, int rows, int cols, const cell& currentCell, cell& direction) {
    cell pref = currentCell + direction;
    if(withinBounds(rows, cols, pref) && map[pref.y][pref.x] == currentCell.cost-1) {
        pref.cost = currentCell.cost-1;
        return pref;
    }

    for(int d = 0; d < 4; d++) {
        cell next = currentCell + directions[d];
        if(withinBounds(rows, cols, next) && map[next.y][next.x] == currentCell.cost-1) {
            next.cost = currentCell.cost-1;
            direction = directions[d];
            return next;
        }
    }

    ROS_WARN("No next cell found, this shouldn't happen!");
    return currentCell;
}

void backTrackPath(const std::vector<std::vector<int> >& map, int rows, int cols, cell start, std::vector<cell>& outPath) {
    start.cost = map[start.y][start.x];
    outPath.push_back(start);
    cell direction = up;

    while(outPath.back().cost != 2) {
        cell next = nextCell(map, rows, cols, outPath.back(), direction);
        outPath.push_back(next);
    }

}

void planPathGoal(const nav_msgs::OccupancyGrid& map,
                  const geometry_msgs::Pose& start,
                  const geometry_msgs::Pose& goal,
                  std::vector<geometry_msgs::Pose> outPath) {
    int rows = map.info.height;
    int cols = map.info.width;
    std::vector<std::vector<int> > floodMap(rows, std::vector<int>(cols));

    for(int y = 0; y < rows; ++y) {
        int index = y*cols;
        for(int x = 0; x < cols; ++x) {
            floodMap[y][x] = map.data[index + x] == WALL || map.data[index + x] == UNEXPLORED;
        }
    }

    std::vector<cell> path;
    flood(floodMap, rows, cols, cell(goal.position.y, goal.position.x));

    //Test if there is a possible path before backtracking
    if(floodMap[start.position.y][start.position.x] > 0) {
        backTrackPath(floodMap, rows, cols, cell(start.position.y, start.position.x), path);
        geometry_msgs::Pose pose;

        for(size_t i = 0; i < path.size(); ++i) {
            pose.position.x = path[i].x;
            pose.position.y = path[i].y;
            outPath.push_back(pose);
        }
    }
}

#endif // PATH_PLANNING_HPP
