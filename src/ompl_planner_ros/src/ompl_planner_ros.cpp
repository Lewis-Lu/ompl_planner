#include "ompl_planner_ros/ompl_planner.h"

int main(int argc, char** argv) {

    double step_size = 0.05;
    ompl_planner_ros::Limit x_bound{-5.0, 5.0};
    ompl_planner_ros::Limit y_bound{-5.0, 5.0};
    
    // init ROS node
    ros::init(argc, argv, "ompl_planner_ros");

    // create node handler
    ros::NodeHandle nh("~");

    ompl_planner_ros::Planner2D planner(nh, step_size, x_bound, y_bound);

    ros::Subscriber map_sub = nh.subscribe("/map", 1, &ompl_planner_ros::Planner2D::mapCallback, &planner);
    ros::Subscriber start_sub = nh.subscribe("/initialpose", 1000, &ompl_planner_ros::Planner2D::mapCallback, &planner);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1000, &ompl_planner_ros::Planner2D::mapCallback, &planner);
    
    // setup the ROS loop rate
    ros::Rate loop_rate(1);

    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
