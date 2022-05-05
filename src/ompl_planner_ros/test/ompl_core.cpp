#include "ompl_planner_ros/ompl_planner.h"

#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h> 
#include <nav_msgs/OccupancyGrid.h>

class StartGoalUpdater
{
public:
    StartGoalUpdater( ros::NodeHandle* nh, geometry_msgs::PoseStamped* start, 
        geometry_msgs::PoseStamped* goal, nav_msgs::OccupancyGrid* map)
      : nh_(nh), start_(start), goal_(goal), map_(map),
       receivedStart_(false), receivedGoal_(false), receivedMap_(false)
    {
        startSub_ = nh_->subscribe( "/initialpose", 1, &StartGoalUpdater::startCallback, this);
        goalSub_ = nh_->subscribe( "/move_base_simple/goal", 1, &StartGoalUpdater::goalCallback, this);
        mapSub_ = nh_->subscribe( "/map", 1, &StartGoalUpdater::mapCallback, this);
    }

    void startCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
        if (!receivedStart_)
            receivedStart_ = true;

        start_->header = msg.header;
        start_->pose = msg.pose.pose;
    }

    void goalCallback(const geometry_msgs::PoseStamped& msg) {
        if (!receivedGoal_)
            receivedGoal_ = true;

        *goal_ = msg;
    }

    void mapCallback(const nav_msgs::OccupancyGrid& mapMsg) {
        if (!receivedMap_)
            receivedMap_ = true;
        ROS_INFO("map received.");
        *map_ = mapMsg;
    }


    bool receivedStart() { return receivedStart_; }

    bool receivedGoal() { return receivedGoal_; }

    bool receivedStartAndGoal() { return (receivedStart_ && receivedGoal_); }

    bool allReady() { return(receivedStart_ && receivedGoal_ && receivedMap_); }

private:
    ros::NodeHandle* nh_;
    ros::Subscriber startSub_;
    ros::Subscriber goalSub_;
    ros::Subscriber mapSub_;

    geometry_msgs::PoseStamped* start_;
    geometry_msgs::PoseStamped* goal_;
    nav_msgs::OccupancyGrid* map_;

    bool receivedStart_;
    bool receivedGoal_;
    bool receivedMap_;
};


int main(int argc, char** argv)
{
    // init ROS node
    ros::init(argc, argv, "ompl_core_test");

    // create node handler
    ros::NodeHandle nh("~");

    ros::Publisher startpose_pub = nh.advertise<geometry_msgs::PoseStamped>("start_pose", 1);
    ros::Publisher goalpose_pub = nh.advertise<geometry_msgs::PoseStamped>("goal_pose", 1);
    // planned path publisher
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1000);

    geometry_msgs::PoseStamped start, goal;
    start.header.frame_id = goal.header.frame_id = "map";

    nav_msgs::OccupancyGrid g_map;

    StartGoalUpdater sgu(&nh, &start, &goal, &g_map);

    double st = 0.05;
    ompl_planner::Planner2D planner_(st, &g_map);

    // setup the ROS loop rate
    ros::Rate loop_rate(1);

    while (ros::ok()){
        if (sgu.receivedStart()) startpose_pub.publish(start);

        if (sgu.receivedGoal()) goalpose_pub.publish(goal);

        if(sgu.allReady()) {
            std::vector<geometry_msgs::PoseStamped> planned_path;
            
            planner_.plan(start, goal, planned_path);
            
            // construct the nav_msgs for publish
            nav_msgs::Path pub_path;

            pub_path.header = (start.header.stamp > goal.header.stamp) ? start.header : goal.header;
            pub_path.poses = planned_path;

            path_pub.publish(pub_path);
        } else {
            ROS_INFO("waiting for start and goal pose...");
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}