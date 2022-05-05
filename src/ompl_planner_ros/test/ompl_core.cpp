#include "ompl_planner_ros/ompl_planner.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h> 
#include <nav_msgs/OccupancyGrid.h>

nav_msgs::OccupancyGridConstPtr globalMap;
geometry_msgs::PoseWithCovarianceStamped start;
geometry_msgs::PoseStamped goal;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg) {
    ROS_INFO("map received.");
    globalMap = mapMsg;
}

void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& s) {
    ROS_INFO("start pose received.");
    start = *s;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& s) {
    ROS_INFO("goal pose received.");
    goal = *s;
}


int main(int argc, char** argv)
{
    // init ROS node
    ros::init(argc, argv, "ompl_core_test");

    // create node handler
    ros::NodeHandle nodeHandle("~");

    double st = 0.05;
    ompl_planner::Planner2D planner_(st, globalMap);

    // setup the ROS loop rate
    ros::Rate loop_rate(1);

    // occupancy map subscriber
    ros::Subscriber map_sub = nodeHandle.subscribe("/map", 1, mapCallback);
    ros::Subscriber start_sub = nodeHandle.subscribe("/initialpose", 1, startCallback);
    ros::Subscriber goal_sub = nodeHandle.subscribe("/move_base_simple/goal", 1, goalCallback);

    // planned path publisher
    ros::Publisher path_pub = nodeHandle.advertise<nav_msgs::Path>("planned_path", 1000);

    while (ros::ok()){        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}