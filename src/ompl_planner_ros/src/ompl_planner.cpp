#include "ompl_planner_ros/ompl_planner.h"

#include <iostream>

using namespace std;
using namespace ros;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_planner {

nav_msgs::OccupancyGridConstPtr occupancy_map;

Planner2D::Planner2D(double& max_step_length, int& dim, const navConfig& nc) 
        : max_step_(max_step_length), dim_(dim), initialized_(false) {

    ROS_INFO("Planner2D Configuring...");

    start_.push_back(nc.sgconf_[0]);
    start_.push_back(nc.sgconf_[1]);
    goal_.push_back(nc.sgconf_[2]);
    goal_.push_back(nc.sgconf_[3]);
    
    bound_x_.push_back(nc.bound_[0]);
    bound_x_.push_back(nc.bound_[1]);
    bound_y_.push_back(nc.bound_[2]);
    bound_y_.push_back(nc.bound_[3]);

    initialize();
}

Planner2D::Planner2D(double& max_step_length, nav_msgs::OccupancyGridConstPtr& map, const navConfig& nc)
        : max_step_(max_step_length), grid_map_(map) {
    if(!nc.sgconf_) {
        ROS_ERROR("configuration does not contain start and goal configuration.");
    } else {
        ROS_INFO("Planner2D Configuring...");
        double width = map->info.width, height = map->info.height;
        double origin_x = map->info.origin.position.x;
        double origin_y = map->info.origin.position.y;

        occupancy_map = map;

        bound_x_.push_back(origin_x);
        bound_x_.push_back(width + origin_x);
        bound_y_.push_back(origin_y);
        bound_y_.push_back(height + origin_y);

        start_.push_back(nc.sgconf_[0]);
        start_.push_back(nc.sgconf_[1]);
        goal_.push_back(nc.sgconf_[2]);
        goal_.push_back(nc.sgconf_[3]);
    }
}

Planner2D::~Planner2D() { }

void Planner2D::initialize(){
    if(initialized_){
        ROS_WARN("Planner2D is initialized.");
    } else {

        ROS_INFO("Space dimension is %d", dim_);

        assert(start_.size() == dim_);
        assert(goal_.size() == dim_);
        assert(bound_x_.size() == dim_);
        assert(bound_y_.size() == dim_);

        auto space(std::make_shared<ob::RealVectorStateSpace>());
        // auto space(std::make_shared<ob::RealVectorStateSpace>(dim_)); // set start and goal later, respectively.

        space->addDimension(bound_x_[0], bound_x_[1]);
        space->addDimension(bound_y_[0], bound_y_[1]);

        si_ompl_ = std::make_shared<ob::SpaceInformation>(space);

        ob::ScopedState<> start(space);
        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_[0];
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_[1];
    
        // Set our robot's goal state to be the top-right corner of the
        // environment, or (1,1).
        ob::ScopedState<> goal(space);
        goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_[0];
        goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_[1];

        pdef_ompl_ = std::make_shared<ob::ProblemDefinition>(si_ompl_);
        pdef_ompl_->setStartAndGoalStates(start, goal);

        ROS_INFO("initialized!");
        initialized_ = true;

    }
}

#define OCCUPIED 100
#define FREE 0

bool isStateValid(const ob::State* state) {
    const double wx = (double)state->as<ob::RealVectorStateSpace::StateType>()->values[0];
    const double wy = (double)state->as<ob::RealVectorStateSpace::StateType>()->values[1];
    double origin_x = occupancy_map->info.origin.position.x;
    double origin_y = occupancy_map->info.origin.position.y;
    
    if( wx < origin_x || wy < origin_y ) {
        return false;
    }

    double resln = occupancy_map->info.resolution;
    unsigned int mx = (int)((wx - origin_x) / resln);
    unsigned int my = (int)((wy - origin_y) / resln);

    size_t mIndex = mx + occupancy_map->info.width * my;

    unsigned int cells_x = occupancy_map->info.width / resln;
    unsigned int cells_y = occupancy_map->info.height / resln;

    if(mx < cells_x && my < cells_y) {
        if(OCCUPIED == occupancy_map->data[mIndex]) {
            return false;
        }
    } else {
        return false;
    }

    return true;
}

mapInfo& Planner2D::getMapInformation() {
    return mi_;
}

bool Planner2D::plan() {

    si_ompl_->setStateValidityChecker(isStateValid);
    si_ompl_->setStateValidityCheckingResolution(max_step_);
    
    auto planner(std::make_shared<og::RRTConnect>(si_ompl_));
    planner->setRange(max_step_);
    planner->setProblemDefinition(pdef_ompl_);
    planner->setup();
    
    double time_limit = 2.0; // solely the time the solver allowed.

    ob::PlannerStatus solved = planner->ob::Planner::solve(time_limit);

    nav_msgs::Path plannedPath;
    if (solved) {// if cussess
       return true;
    }
    return false;
}

nav_msgs::Path Planner2D::extractPath(ob::ProblemDefinition* pdef) {
    nav_msgs::Path plannedPath;
    return plannedPath;
}

} /* namespace */


using namespace ompl_planner;

int main(int argc, char const *argv[])
{
    int dimension = 2;
    navConfig config( dimension );
    
    config.sgconf_[0] = -3.0;
    config.sgconf_[1] = -3.0;
    config.sgconf_[2] = 3.0; 
    config.sgconf_[3] = 3.0;

    config.bound_[0] = -5.0;
    config.bound_[1] = 5.0;
    config.bound_[2] = -5.0;
    config.bound_[3] = 5.0;
    
    double step_ = 0.01;
    
    ros::Duration rate(10);

    Planner2D planner(step_, dimension, config);

    while(ros::ok()) {
        rate.sleep();

        ros::spin();
    }
    ROS_INFO("main started.");
    return 0;
}
