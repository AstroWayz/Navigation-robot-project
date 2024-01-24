#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/GetModelState.h"
#include <iostream>
#include <cmath>
#include <list>
#include <algorithm>
#include <deque>
#include <stdlib.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>


#define MAP_WIDTH 20
#define MAP_LENGTH 20
#define TEN_OFFSET 10
#define START_X 10
#define START_Y 10
#define END_X 5
#define END_Y 5
#define LASER_SAMPLES 720
#define MAX_ANGLE 1
#define MIN_ANGLE -1

geometry_msgs::Twist robot_msg;
std_msgs::Float32 p_time;
std_msgs::Float32 r_time;
sensor_msgs::LaserScan laser_scan;

struct quarternion
{
    double w;
    double x;
    double y;
    double z;
};

geometry_msgs::Twist move_func(double x, double y, quarternion quar, double dest_x, double dest_y);

bool SCAN_FLAG = false;
bool MOVE_FLAG = false;
bool SEARCH_FLAG = false;
bool DONE_FLAG = false;

char GLOBAL_MAP[MAP_WIDTH][MAP_LENGTH] = {{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}};



//this class represents a point (square) on the map
class Point {

public:

    //constructor
    Point(int a, int b) { x = a; y = b; }

    Point() { x = 0; y = 0; }

    //equal operator between this point and another
    bool operator ==(const Point& o) { return o.x == x && o.y == y; }

    //addition operator between this point and another
    Point operator +(const Point& o) { return Point(o.x + x, o.y + y); }

    bool operator < (const Point& p) { return y < p.y; }

    //coordinates
    int x, y;
};


//this class represents the map
class Map {

public:

    //constructor of the map, initialize every point to zero
    Map() {
        for (int i = 0; i++; i < MAP_WIDTH)
            for (int j = 0; j++; j < MAP_LENGTH)
                points[i][j] = 0;
    }

    //method for reseting the map points to zero
    void map_clear() {
        for (int i = 0; i++; i < MAP_WIDTH)
            for (int j = 0; j++; j < MAP_LENGTH)
                points[i][j] = 0;
    };

    char points[MAP_WIDTH][MAP_LENGTH];

};


//this class represents a state of the map including, each state contains the cost and the distance from the end point.
//we also have the point we are currently in and the parent point
class Node {

public:

    bool operator == (const Node& o) { return pos == o.pos; }
    bool operator == (const Point& o) { return pos == o; }
    bool operator < (const Node& o) { return dist  < o.dist ; }

    Point pos, parent;
    int dist;

};



class SAP {

public:

    SAP() {

        publisher_1 = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        publisher_2 = nh.advertise<std_msgs::Float32>("process_time", 10);
        publisher_3 = nh.advertise<std_msgs::Float32>("run_time", 10);
        subscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &SAP::callback, this);
        laser_subscriber = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &SAP::scan, this);
        
    }

    SAP(ros::NodeHandle nodehandle) {

        publisher_1 = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        publisher_2 = nh.advertise<std_msgs::Float32>("process_time", 10);
        publisher_3 = nh.advertise<std_msgs::Float32>("run_time", 10);
        subscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &SAP::callback, this);
        laser_subscriber = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &SAP::scan, this);
        this->nh = nodehandle;
    }

    void set_dest(Point next_point) {

        this->path.x = next_point.x - TEN_OFFSET;
        this->path.y = next_point.y - TEN_OFFSET;

        MOVE_FLAG = true;
    }

    void callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {

        quarternion robot_quar;
        robot_quar.w = msg->pose.back().orientation.w;
        robot_quar.x = msg->pose.back().orientation.x;
        robot_quar.y = msg->pose.back().orientation.y;
        robot_quar.z = msg->pose.back().orientation.z;
        this->quart = robot_quar;
        double dest_x = path.x;
        double dest_y = path.y;
        this->x_pos = msg->pose.back().position.x;
        this->y_pos = msg->pose.back().position.y;

        if (MOVE_FLAG) {

            robot_msg = move_func(-y_pos, x_pos, robot_quar, dest_x, dest_y);
            publisher_1.publish(robot_msg);
            

        }
        else if (!SCAN_FLAG) {

            robot_msg = adjust();
            publisher_1.publish(robot_msg);
            
        }
        else 
            return;
    }


    geometry_msgs::Twist adjust() {

        update_orientation();

//        int dest_x = std::round(this->x_pos);
//        int dest_y = std::round(this->y_pos);
//        double dist = std::sqrt(pow((dest_x - this->x_pos), 2) + pow((dest_y - this->y_pos), 2));

//        double dest_angle;
//        if (this->orientation == 0)
//            dest_angle = 0;
//        else if (this->orientation == 1)
//            dest_angle = M_PI_2;
//        else if (this->orientation == 2)
//            dest_angle = M_PI;
//        else
//            dest_angle = 3 * M_PI_2;

        

//        double cross_product = cos(this->robot_angle) * (dest_y - this->y_pos) - sin(this->robot_angle) * (dest_x - this->x_pos);
//        double dot_product = cos(this->robot_angle) * (dest_x - this->x_pos) + sin(this->robot_angle) * (dest_y - this->y_pos);
//        double w = 2 * std::acos(dot_product / std::sqrt(pow(dest_x - this->x_pos, 2) + pow(dest_y - this->y_pos, 2)));
//        double angle_diff = std::abs(this->robot_angle - dest_angle);
//        double vel = 0.8 / (w + 1);
        double x = - y_pos;
        double y = x_pos;
        int dest_x = std::round(x);
        int dest_y = std::round(y);
        double dist = std::sqrt(pow((dest_x - x), 2) + pow((dest_y - y), 2));

        double dest_angle;
        if (this->orientation == 0)
            dest_angle = 0;
        else if (this->orientation == 1)
            dest_angle = M_PI_2;
        else if (this->orientation == 2)
            dest_angle = M_PI;
        else
            dest_angle = 3 * M_PI_2;



        double cross_product = cos(this->robot_angle) * (dest_y - y) - sin(this->robot_angle) * (dest_x - x);
        double dot_product = cos(this->robot_angle) * (dest_x - x) + sin(this->robot_angle) * (dest_y - y);
        double w = 2 * std::acos(dot_product / std::sqrt(pow(dest_x - x, 2) + pow(dest_y - y, 2)));
        double angle_diff = std::abs(this->robot_angle - dest_angle);
        double vel = 0.8 / (w + 1);

        //std::cout << "orientation #" << this->orientation << " adjusting to angle: " << dest_angle << " from: " << this->robot_angle << " difference is: " << angle_diff << std::endl;




        if (angle_diff < 0.01) {
            if (dist < 0.1) {
                robot_msg.linear.x = 0;
                robot_msg.angular.z = 0;
            }
            else {
                robot_msg.angular.z = 0;
                robot_msg.linear.x = 0.1;
            }
        }
        else {
            robot_msg.linear.x = 0;
            if (cross_product < 0)
                robot_msg.angular.z = -w / 5;
            else
                robot_msg.angular.z = w / 5;
        }


        /*if (dist < 0.01) {
            robot_msg.linear.x = 0;
        }
        else {
            robot_msg.linear.x = vel / 5;
        }

        if (angle_diff < 0.01)
            robot_msg.angular.z = 0;
        else {
            if (cross_product < 0)
                robot_msg.angular.z = w/2;
            else
                robot_msg.angular.z = -w/2;
        }*/

        if (dist < 0.1 && angle_diff < 0.01)
            SCAN_FLAG = true;

        //std::cout << this->x_pos << "," << this->y_pos << " at angle: " << this->robot_angle <<std::endl;
        
        return robot_msg;
    }



    void scan(const sensor_msgs::LaserScan::ConstPtr& laser_msg) {

        if (SCAN_FLAG) {

            SCAN_FLAG = false;
            std::vector<float> ranges = laser_msg->ranges;
            std::vector<Point> obstacles;
            std::vector<Point> available_points;

            std::vector<double> x_hits;
            std::vector<double> y_hits;

            double ray_density = MAX_ANGLE - MIN_ANGLE;
            ray_density = ray_density / (LASER_SAMPLES - 1);


            //first we take the data from the scan and for each angle we calc the x and y distance
            //using sin and cos
            for (unsigned long i = 0; i < ranges.size(); i++) {
                double angle = MIN_ANGLE + ray_density * i;

                double x_target = std::cos(M_PI_2 + angle) * ranges[i];
                double y_target = std::sin(M_PI_2 + angle) * ranges[i];

                x_hits.push_back(x_target);
                y_hits.push_back(y_target);

            }


            //using the data we look for obstacles
            int jump = 10;
            int next_pack = 0;
            for (int i = 0; i < LASER_SAMPLES / jump; i++) {

                double first_x = x_hits[next_pack];
                double last_x = x_hits[next_pack + jump - 1];
                double first_y = y_hits[next_pack];
                double last_y = y_hits[next_pack + jump - 1];

                Point p = ConvergencePoint(first_x, last_x, first_y, last_y);
                if (!contains(obstacles, p) && p.x != 1000 && p.y != 1000 && p.y <= 7)
                    obstacles.push_back(p);
                next_pack = next_pack + jump;
            }

            //update the map with the obstacles we found
            update_global_map_with_obstacles(obstacles);


            //now again, using the data analyze which points are free to move to
            int height = 7;
            std::vector<Point> free_road;
            while (height > 0) {

                for (int horz_coor = -height ; horz_coor <= height; horz_coor++) {
                    double angle = 0;

                    if (horz_coor != 0) {
                        angle = std::atan(height / horz_coor);
                        if (angle > 0)
                            angle = angle - M_PI_2;
                        else
                            angle = M_PI_2 + angle;
                    }

                    //std::cout << "check destination: (" << horz_coor << "," << height << ") at angle: " << angle << std::endl;
                    double distance = std::sqrt(pow(height, 2) + pow(horz_coor, 2));
                    //std::cout << "(" << horz_coor << "," << height << ") at angle: " << angle << ", and distance: " << distance << std::endl;
                    for (unsigned long i = 0; i < ranges.size(); i++) {
                        double current = MIN_ANGLE + ray_density * i;
                        if (current > angle) {
                            //std::cout << "found current: " << current << "with distance: " << ranges[i] << std::endl;
                            if (ranges[i] > distance && ranges[i - 1] > distance && !contains(obstacles, Point(horz_coor, height))) {
                                free_road.push_back(Point(horz_coor, height));
                            }
                            break;
                        }
                    }
                }

                height--;

            }

            for (int i = 0; i < obstacles.size(); i++) {
                std::cout << obstacles[i].x << "," << obstacles[i].y << std::endl;
            }
            


            //from the free points elimnate the cubies (points that are surrounded by obstacles (dead end)
            for (int i = 0; i < free_road.size(); i++) {
                Point point_on_map = MapPoint(free_road[i]);
                if (IsCuby(point_on_map)) {
                    GLOBAL_MAP[point_on_map.x][point_on_map.y] = 2;
                    free_road.erase(free_road.begin() + i);
                    //std::cout << "found a cuby at: " << point_on_map.x << "," << point_on_map.y << std::endl;
                    i--;
                }
            }


            //now mark potential points on the map with 3
            update_global_map_with_potential_free_points(free_road);

            std::cout << "-----------------------------------------" << std::endl;
            for (int j = MAP_LENGTH - 1; j >= 0; j--) {
                for (int i = 0; i < MAP_WIDTH; i++)
                    std::cout << int(GLOBAL_MAP[i][j]) << " ";

                std::cout << std::endl;
            }

            std::cout << "-----------------------------------------" << std::endl;
            //check if any on these free point is the end point, if so then mark
            //all of them as free
            for (int i = 0; i < free_road.size(); i++) {
                Point point_on_map = MapPoint(free_road[i]);
                if (point_on_map.x == END_X && point_on_map.y == END_Y) {
                    std::cout << "found the exit at one of the 3 mark points" << std::endl;
                    Turn_potential_points_to_free_points();
                }
            }

            EleminatePotentialDeadEnds();
            Turn_potential_points_to_free_points();

            for (int j = MAP_LENGTH - 1; j >= 0; j--) {
                for (int i = 0; i < MAP_WIDTH; i++)
                    std::cout << int(GLOBAL_MAP[i][j]) << " ";

                std::cout << std::endl;
            }
            std::cout << "-----------------------------------------" << std::endl;
            std::cout << "finished scanning, turn search on" << std::endl;
            SCAN_FLAG = false;
            SEARCH_FLAG = true;
        }

        return;
    }

    void update_global_map_with_obstacles(std::vector<Point> obstacles) {

        //go over obstacle points and mark them on the global map
        for (int i = 0; i < obstacles.size(); i++) {
            Point obstacle = MapPoint(obstacles[i]);
            GLOBAL_MAP[obstacle.x][obstacle.y] = 2;
        } 
    }
    
    void update_global_map_with_potential_free_points(std::vector<Point> free_points) {
        std::cout << "potential free points: " << std::endl;
        for (int i = 0; i < free_points.size(); i++) {
            Point point = MapPoint(free_points[i]);
            std::cout << point.x << "," << point.y << std::endl;
            if (!isValid(point))
                continue;
            else if (point.x == END_X && point.y == END_Y) {
                GLOBAL_MAP[point.x][point.y] = 0;
            }
            else {
                GLOBAL_MAP[point.x][point.y] = 3;
                threes_on_map.push_back(point);
            }
        }
        return;
    }

    void Turn_potential_points_to_free_points() {
        for (int i = 0; i < MAP_WIDTH; i++) {
            for (int j = 0; j < MAP_LENGTH; j++) {
                if (GLOBAL_MAP[i][j] == 3)
                    GLOBAL_MAP[i][j] = 0;
            }
        }
        threes_on_map.clear();
        return;
    }

    /*take a point from the POV of the robot and map it to 
    the actual point on the Global Map*/
    Point MapPoint(Point point) {

        int x;
        int y;

        if (this->orientation == 0) {
            x = point.y;
            y = -point.x;
        }
        else if (this->orientation == 1) {
            x = point.x;
            y = point.y;
        }
        else if (this->orientation == 2) {
            x = -point.y;
            y = point.x;
        }
        else {
            x = -point.x;
            y = -point.y;
        }

        x = x - std::round(this->y_pos) + TEN_OFFSET;
        y = y + std::round(this->x_pos) + TEN_OFFSET;

        return Point(x, y);
    }



    void EleminatePotentialDeadEnds() {
        
        int corner = 0;
        bool redo = false;
        bool success = false;
        for (int i = 0; i < threes_on_map.size(); i++) {

            if (IsCuby(threes_on_map[i])) {
                GLOBAL_MAP[threes_on_map[i].x][threes_on_map[i].y] = 2;
                continue;
            }
            corner = isCorner(threes_on_map[i]);
            if (corner > 0) {
                std::cout << "corner type: " << corner << " at: " << threes_on_map[i].x << "," << threes_on_map[i].y << std::endl;
                if (!Eliminate(threes_on_map[i].x, threes_on_map[i].y, corner))
                    redo = true;
                else
                    success = true;
            }
        }

        //for (int x = 0; x < MAP_WIDTH; x++) {
        //    for (int y = 0; y < MAP_LENGTH; y++) {
        //        //std::cout << "here" << std::endl;
        //        if (GLOBAL_MAP[x][y] == 3) {
        //            //std::cout << "found potential free point at: " << x << "," << y << std::endl;
        //            if (IsCuby(Point(x, y))) {
        //                GLOBAL_MAP[x][y] = 2;
        //                //std::cout << "found cuby at: " << x << "," << y << std::endl;
        //                continue;
        //            }
        //            corner = isCorner(Point(x, y));
        //            if (corner > 0) {
        //                //std::cout << "found corner at: " << x << "," << y << " type: " << corner << std::endl;
        //                if (!Eliminate(x, y, corner))
        //                    redo = true;
        //                else
        //                    success = true;
        //            }

        //        }
        //    }
        //}

        if (redo && success)
            void EleminatePotentialDeadEnds();
        else
            return;

    }


    bool EliminateNorthDeadEnd(int x, int y, int cornerType) {

        int x_next = x;
        int lineLength = 1;

        while (true) {
            if (cornerType == 1)
                x_next = x_next - 1;
            else
                x_next = x_next + 1;

            int newcornerType = isCorner(Point(x_next, y));
            if (GLOBAL_MAP[x_next][y] == 3 && ((newcornerType == 2 && cornerType == 1) || (newcornerType == 1 && cornerType == 2))) {
                lineLength++;
                break;
            }

            if (GLOBAL_MAP[x_next][y + 1] == 0 || GLOBAL_MAP[x_next][y + 1] == 1)
                return false;

            else if (GLOBAL_MAP[x_next][y + 1] == 2) {
                lineLength++;
                continue;
            }
        }

        int it = 0;
        while (lineLength > 0) {

            GLOBAL_MAP[x + it][y] = 2;
            if (cornerType == 1)
                it =  it - 1;
            else
                it = it + 1;

            lineLength--;
        }

        return true;
    }


    bool EliminateSouthDeadEnd(int x, int y, int cornerType) {

        int x_next = x;
        int lineLength = 1;
        std::cout << "starting at: " << x << "," << y << std::endl;
        while (true) {
            if (cornerType == 3)
                x_next = x_next - 1;
            else
                x_next = x_next + 1;


            int newcornerType = isCorner(Point(x_next, y));

            std::cout << "next item at: " << x_next << "," << y << " type: " << newcornerType << std::endl;

            if (GLOBAL_MAP[x_next][y] == 3 && ((newcornerType == 4 && cornerType == 3) || (newcornerType == 3 && cornerType == 4))) {
                lineLength++;
                std::cout << "oposite corner at : " << x_next << "," << y << " length is: " << lineLength << std::endl;
                break;
            }

            if (GLOBAL_MAP[x_next][y - 1] == 0 || GLOBAL_MAP[x_next][y - 1] == 1)
                return false;

            else if (GLOBAL_MAP[x_next][y - 1] == 2) {
                lineLength++;
                continue;
            }
        }



        int it = 0;
        while (lineLength > 0) {
            
            GLOBAL_MAP[x + it][y] = 2;
            if (cornerType == 3)
                it = it - 1;
            else
                it = it + 1;

            lineLength--;
        }


        return true;
    }


    bool EliminateEastDeadEnd(int x, int y, int cornerType) {

        int y_next = y;
        int lineLength = 1;

        while (true) {
            if (cornerType == 1)
                y_next = y_next - 1;
            else
                y_next = y_next + 1;

            int newcornerType = isCorner(Point(x, y_next));
            if (GLOBAL_MAP[x][y_next] == 3 && ((newcornerType == 3 && cornerType == 1) || (newcornerType == 1 && cornerType == 3))) {
                lineLength++;
                break;
            }

            if (GLOBAL_MAP[x + 1][y_next] == 0 || GLOBAL_MAP[x + 1][y_next] == 1)
                return false;

            else if (GLOBAL_MAP[x][y_next - 1] == 2) {
                lineLength++;
                continue;
            }
        }

        int it = 0;
        while (lineLength > 0) {

            GLOBAL_MAP[x][y + it] = 2;
            if (cornerType == 1)
                it = it - 1;
            else
                it = it + 1;

            lineLength--;
        }

        return true;
    }


    bool EliminateWestDeadEnd(int x, int y, int cornerType) {

        int y_next = y;
        int lineLength = 1;

        while (true) {
            if (cornerType == 2)
                y_next = y_next - 1;
            else
                y_next = y_next + 1;

            int newcornerType = isCorner(Point(x, y_next));
            if (GLOBAL_MAP[x][y_next] == 3 && ((newcornerType == 4 && cornerType == 2) || (newcornerType == 2 && cornerType == 4))) {
                lineLength++;
                break;
            }

            if (GLOBAL_MAP[x - 1][y_next] == 0 || GLOBAL_MAP[x - 1][y_next] == 1)
                return false;

            else if (GLOBAL_MAP[x][y_next - 1] == 2) {
                lineLength++;
                continue;
            }
        }

        int it = 0;
        while (lineLength  > 0) {

            GLOBAL_MAP[x][y + it] = 2;
            if (cornerType == 2)
                it = it - 1;
            else
                it = it + 1;

            lineLength--;
        }

        return true;
    }

    bool Eliminate(int x, int y, int cornerType) {

        if (cornerType == 1) {
            bool first_try = EliminateNorthDeadEnd(x, y, cornerType);
            if (!first_try)
                return EliminateEastDeadEnd(x, y, cornerType);
            else
                return true;
            
            }
        
        if (cornerType == 2) {
            bool first_try = EliminateNorthDeadEnd(x, y, cornerType);
            if (!first_try)
                return EliminateWestDeadEnd(x, y, cornerType);
            else
                return true;
        }

        if (cornerType == 3) {
            std::cout << "eliminating corner type 3" << std::endl;
            bool first_try = EliminateSouthDeadEnd(x, y, cornerType);
            
            if (!first_try) {
                return EliminateEastDeadEnd(x, y, cornerType);

            }
            else
                return true;
        }

        if (cornerType == 4) {
            bool first_try = EliminateSouthDeadEnd(x, y, cornerType);
            if (!first_try)
                return EliminateWestDeadEnd(x, y, cornerType);
            else
                return true;
        }

    }

 
    //check if point is valid
    bool isValid(Point p) {

        return (p.x > -1 && p.y > -1 && p.x < MAP_WIDTH && p.y < MAP_LENGTH);

    }

    int isCorner(Point point) {


        //eastern neighbour
        Point Neightbour_1 = point + Point(1, 0);
        //northern neighbour
        Point Neightbour_2 = point + Point(0, 1);
        //western neighbour
        Point Neightbour_3 = point + Point(-1, 0);
        //southern neighbour
        Point Neightbour_4 = point + Point(0, -1);


        //check for north-east corner
        if ((!isValid(Neightbour_1) || GLOBAL_MAP[Neightbour_1.x][Neightbour_1.y] == 2) && (!isValid(Neightbour_2) || GLOBAL_MAP[Neightbour_2.x][Neightbour_2.y] == 2))
            return 1;

        //check for north-west corner
        if ((!isValid(Neightbour_3) || GLOBAL_MAP[Neightbour_3.x][Neightbour_3.y] == 2) && (!isValid(Neightbour_2) || GLOBAL_MAP[Neightbour_2.x][Neightbour_2.y] == 2))
            return 2;

        //check for south-east corner
        if ((!isValid(Neightbour_1) || GLOBAL_MAP[Neightbour_1.x][Neightbour_1.y] == 2) && (!isValid(Neightbour_4) || GLOBAL_MAP[Neightbour_4.x][Neightbour_4.y] == 2))
            return 3;

        //check for south-west corner
        if ((!isValid(Neightbour_3) || GLOBAL_MAP[Neightbour_3.x][Neightbour_3.y] == 2) && (!isValid(Neightbour_4) || GLOBAL_MAP[Neightbour_4.x][Neightbour_4.y] == 2))
            return 4;

        else return 0;

    }

    bool IsCuby(Point p) {

        Point Neightbour_1 = p + Point(1, 0);
        Point Neightbour_2 = p + Point(0, 1);
        Point Neightbour_3 = p + Point(-1, 0);
        Point Neightbour_4 = p + Point(0, -1);

        int blockedneighbourCount = 0;

        if (!isValid(Neightbour_1) || GLOBAL_MAP[Neightbour_1.x][Neightbour_1.y] == 2)
            blockedneighbourCount++;

        if (!isValid(Neightbour_2) || GLOBAL_MAP[Neightbour_2.x][Neightbour_2.y] == 2)
            blockedneighbourCount++;

        if (!isValid(Neightbour_3) || GLOBAL_MAP[Neightbour_3.x][Neightbour_3.y] == 2)
            blockedneighbourCount++;

        if (!isValid(Neightbour_4) || GLOBAL_MAP[Neightbour_4.x][Neightbour_4.y] == 2)
            blockedneighbourCount++;

        if (blockedneighbourCount == 3 && p.x != END_X && p.y != END_Y)
            return true;

        else
            return false;

    }


    Point ConvergencePoint(double first_x, double last_x, double first_y, double last_y) {
        int x_coor;
        int y_coor;

        double x_dev = first_x - last_x;
        double y_dev = first_y - last_y;

        //corner, ignore
        if (x_dev < 0.03 && x_dev >-0.03 && y_dev < 0.03 && y_dev >-0.03) {
            return Point(1000, 1000);
        }

        //intesection between two boxes, ignore
        if(std::round(first_x) != std::round(last_x) || std::round(first_y) != std::round(last_y))
            return Point(1000, 1000);


        //x converges on a point
        else if (x_dev < 0.03 && x_dev >-0.03) {

            //its on the east of the robot
            if (first_x > 0 && last_x > 0)
                x_coor = std::ceil(first_x);
            //its on the west of the robot
            else if (first_x < 0 && last_x < 0)
                x_coor = std::floor(first_x);
            else {
                return Point(1000, 1000);
            }

            //check if y is going up or down
            //if ((first_y - last_y) < 0)
                //y_coor = std::floor(first_y);
            //else
                //y_coor = std::ceil(first_y);
            y_coor = std::round(first_y);

            std::cout << "x convergence: " << x_coor << "," << y_coor << std::endl;

            if (x_coor == 2 && y_coor == 1) {
                std::cout << "(" << first_x << "," << first_y << ")----->(" << last_x << "," << last_y << ")" << std::endl;
            }

        }


        //y converges on a point
        else if (y_dev < 0.03 && y_dev >-0.03) {
            if (first_y > 0)
                y_coor = std::ceil(first_y);
            else
                y_coor = std::floor(first_y);

            
            x_coor = std::round(first_x);
            /*if((first_x - std::floor(first_x) < 0)
                x_coor = std::floor(first_x);
            else {
                std::cout << "1" << std::endl;
                x_coor = std::ceil(first_x);
            }*/
            std::cout << "y convergence: " << x_coor << "," << y_coor << std::endl;
        }

        else {
            return Point(1000, 1000);
        }

        Point convergence_point = Point(x_coor, y_coor);
        return convergence_point;
    }

    bool contains(std::vector<Point> points, Point point) {

        for (int i = 0; i < points.size(); i++) {
            if (points[i].x == point.x && points[i].y == point.y)
                return true;
        }

        return false;

    }

    void update_orientation() {

        double orientaion_angle;

        orientaion_angle = std::atan2(2 * (this->quart.w * this->quart.z + this->quart.x * this->quart.y), 1 - (2 * (pow(this->quart.y, 2) + pow(this->quart.z, 2))));
        if (orientaion_angle < -1 * M_PI_2)
            orientaion_angle = 5 * M_PI_2 + orientaion_angle;

        else
            orientaion_angle = orientaion_angle + M_PI_2;





        this->robot_angle = orientaion_angle;

        //east orientaion
        if ((0 < this->robot_angle && this->robot_angle < M_PI_4) || ((2 * M_PI - M_PI_4) < this->robot_angle && this->robot_angle < 2 * M_PI))
            this->orientation = 0;


        //north orientaion
        else if ((M_PI_2 - M_PI_4) < this->robot_angle && this->robot_angle < (M_PI_2 + M_PI_4))
            this->orientation = 1;



        //west orientaion
        else if ((M_PI - M_PI_4) < this->robot_angle && this->robot_angle < (M_PI + M_PI_4))
            this->orientation = 2;



        //south orientaion
        else if ((3*M_PI_2 - M_PI_4) < this->robot_angle && this->robot_angle < (3*M_PI_2 + M_PI_4))
            this->orientation = 3;

    }



    ros::NodeHandle nh;
    ros::Publisher publisher_1;
    ros::Publisher publisher_2;
    ros::Publisher publisher_3;
    ros::Subscriber subscriber;
    ros::Subscriber laser_subscriber;

    Point path;
    double x_pos;
    double y_pos;
    std::vector<Point> threes_on_map;
    double robot_angle;
    int orientation = 1;
    quarternion quart;


};




//this class is the A* algorithm
class A_Star {

  public:

    //constructor: if we dont receive a location then assume we start at (x,y)=(0,0)
    A_Star(Point start, Point end) {

        this->start = start;
        this->end = end;
        this->current_state.pos = Point(start);
        this->current_state.parent = Point();
        this->current_state.dist = calc_dist(this->start);
        this->reached_points.push_back(current_state);
        neighbours[0] = Point(0, 1);  neighbours[1] = Point(1, 0);
        neighbours[2] = Point(-1, 0);  neighbours[3] = Point(0, -1);

    }

    //calculate the distance to the end point'
    double calc_dist(Point p) {

        //double dist = sqrt(pow(end.x-p.x, 2)+pow(end.y-p.y, 2));
        double dist = std::sqrt(pow((end.x - p.x),2) + pow((end.y - p.y),2));
        //double dist = (p.x - end.x) + (p.y - end.y);
        return dist;
        

    };

    //check if point is valid
    bool isValid(Point p) {

        return (p.x > -1 && p.y > -1 && p.x < MAP_WIDTH&& p.y < MAP_LENGTH);

    }

    //check if point exists in the closed queue, if so check if the current cost is lower than the previous signature then return true else remove it amd return false
    // then check the same in the open queue
    //return the path itself
    bool existPoint(Point& p, double cost) {

        std::list<Node>::iterator i;

        i = std::find(closed.begin(), closed.end(), p);
        if (i != closed.end()) {
            if ((*i).dist < cost){
                return true;
            }
            else {
                closed.erase(i);
                return false;
            }

        }
 

        i = std::find(open.begin(), open.end(), p);
        if (i != open.end()) {
            //std::cout << "found " << p.x << "," << p.y << " in open " << std::endl;
            if ((*i).dist < cost){
                return true;
            }
            else {
                open.erase(i);
                return false;
            }

        }
        return false;

    }

    //fill the open queue
    bool fillOpen(Node& n) {

        double dist;
        Point neighbour;

        for (int x = 0; x < 4; x++) {

            neighbour = n.pos + neighbours[x];
            //std::cout << "neighbour #" << x << " at: " << neighbour.x << "," <<neighbour.y;
            if (neighbour == end){
                std::cout << "found the exit" << std::endl;
                return true;
            };

            if (isValid(neighbour) && GLOBAL_MAP[neighbour.x][neighbour.y] < 1) {
                if (GLOBAL_MAP[n.pos.x][neighbour.y] < 1 && GLOBAL_MAP[neighbour.x][n.pos.y] < 1) {
                    //std::cout << " is valid to move to" << std::endl;
                    dist = calc_dist(neighbour);
                    //std::cout << "checking neighbour: " << neighbour.x << "," << neighbour.y << std::endl;
                    if (!existPoint(neighbour, dist)) {
                        //std::cout << x << " neighbour: " << neighbour.x << "," << neighbour.y << " is new" << std::endl;
                        Node child_node;
                        child_node.dist = dist;
                        child_node.pos = neighbour;
                        child_node.parent = n.pos;
                        open.push_back(child_node);
                    }
                }
            }
            //else
            //std::cout << " is NOT valid to move to" << std::endl;
        };


        return false;
    }

    //search for the optimal path
    void search() {
        
        std::cout << "search, currently at node (" << current_state.pos.x << "," << current_state.pos.y << ")" << std::endl;
        open.clear();
        //std::cout << "reached points: " << std::endl;
        //for (int i = 0; i < reached_points.size(); i++)
            //std::cout << reached_points[i].pos.x << "," << reached_points[i].pos.y << std::endl;
        if (fillOpen(current_state)) {
            DONE_FLAG = true;
            std::cout << "found the exit" << std::endl;
            return;
        }
        int open_size = open.size();
        if (open_size > 0)
            open.sort();
        //std::cout << "open list:" << std::endl;
        //std::list<Node>::iterator it = open.begin();
        //for(it = open.begin(); it == open.end(); it++)
          //std::cout << (*it).pos.x << "," << (*it).pos.y << std::endl;
        int i = 0;
        while (i < open_size) {
            Node m = open.front();
            open.pop_front();
            std::cout << "looking into (" << m.pos.x << "," << m.pos.y << ")" << std::endl;
            if (!IsPointReached(m)) {
                std::cout << "not reached, go there" << std::endl;
                reached_points.push_back(m);
                closed.push_back(m);
                current_state = m;
                open.clear();
                sap.set_dest(m.pos);
                SEARCH_FLAG = false;
                SCAN_FLAG = false;
                return;
            }
            
            i++;

        }

        if (i == open_size) {
            std::cout << "retract" << std::endl;
            reached_points.clear();
            closed.clear();
            reached_points.push_back(current_state);
            SCAN_FLAG = false;
            MOVE_FLAG = false;
            SEARCH_FLAG = false;
            return;
            //current_state = current_state.parent; //TODO change to Node
        }

    }
     

    
    bool IsPointReached(Node point) {
        for (int i = 0; i < reached_points.size(); i++){
            if (reached_points[i] ==  point)
                return true;
        }
        return false;
    }



    //construct a path using the closed q
//    int path_construct() {
//        path.push_front(end);
//        int cost = 1 + closed.back().cost;
//        path.push_front(closed.back().pos);
//        Point parent = closed.back().parent;

//        for(std::list<Node>::reverse_iterator i = closed.rbegin(); i != closed.rend(); i++){
//          if((*i).pos == parent && !((*i).pos == start)){

//            path.push_front((*i).pos);
//            parent = (*i).parent;
//          }

//        }
//        path.push_front(start);
//        return cost;
//    }


    Point neighbours[4];
    Point start;
    Point end;
    Node current_state;
    std::list<Node> closed;
    std::list<Node> open;
    std::vector<Node> reached_points;
    std::deque<Point> path;
    SAP sap;


};





geometry_msgs::Twist move_func(double x, double y, quarternion quar, double dest_x, double dest_y) {
    //std::cout << "(" << x << "," << y << ") --- > (" << dest_x << "," << dest_y << ")" << std::endl;
        quarternion q = quar;

        double angle;
        double vel;
        double w;
        double dest_angle = 0;

        //angle = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - (2 * (pow(q.y, 2) + pow(q.z, 2)))) + M_PI_2;
        angle = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - (2 * (pow(q.y, 2) + pow(q.z, 2))));

        if (angle < - 1*M_PI_2){
          angle = 5* M_PI_2 + angle;

        }
        else {
          angle = angle + M_PI_2;

        }


        double delta_y = dest_y - y;
        double delta_x = dest_x -x;

        if(delta_y > 0 && delta_x > 0)
            dest_angle = std::atan(delta_y/delta_x);

        else if (delta_y > 0 && delta_x < 0)
            dest_angle = M_PI + std::atan(delta_y/ delta_x);

        else if (delta_y < 0 && delta_x < 0)
            dest_angle = M_PI + std::atan(delta_y/ delta_x);

        else if (delta_y < 0 && delta_x > 0)
            dest_angle = 2*M_PI + std::atan(delta_y/ delta_x);



        double cross_product = cos(angle) * (dest_y - y) - sin(angle) * (dest_x - x);

        double dot_product = cos(angle) * (dest_x - x) + sin(angle) * (dest_y - y);
        double dist = std::sqrt(pow((dest_x - x), 2) + pow((dest_y - y), 2));


        w = 2 * std::acos(dot_product / std::sqrt(pow(dest_x - x, 2) + pow(dest_y - y, 2)));

        vel = 0.8 / (w + 1);
        //ROS_INFO_STREAM("dest_angle:" << dest_angle);
        //ROS_INFO_STREAM("robot_angle:" << angle);
        //ROS_INFO_STREAM("w:" << w);
        //ROS_INFO_STREAM("dest_point:" << dest_x << "," << dest_y);
        //ROS_INFO_STREAM("pos:" << x << "," << y << " distance:" << dist);

        if (dist < 0.1) {
            std::cout << "reached (" << dest_x << "," << dest_y << ")" << std::endl;
            robot_msg.linear.x = 0;
            robot_msg.angular.z = 0;
            MOVE_FLAG = false;

        }
        else {
              robot_msg.linear.x = vel-vel/5;

              if (cross_product < 0) {
                  robot_msg.angular.z = w;
              }
              else {
                  robot_msg.angular.z = -w;
              }

        }

        return robot_msg;
}



int main(int argc, char** argv) {

    Point starting_point = Point(START_X, START_Y);
    Point ending_point = Point(END_X, END_Y);

    ros::init(argc, argv, "navigation_robot_viz");
    ros::Time::init();
    ros::Rate loop_rate(50);
    

    A_Star my_a_star(starting_point, ending_point);
    
    while(ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();
        
        if (SEARCH_FLAG && !DONE_FLAG) {
            std::cout << "we scanned and we can search" << std::endl;
            my_a_star.search();   
        }
        else if (DONE_FLAG) {
            robot_msg = my_a_star.sap.adjust();
            my_a_star.sap.publisher_1.publish(robot_msg);

        }

    };


    return 0;
}




