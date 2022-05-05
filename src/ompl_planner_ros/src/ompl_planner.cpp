#include "ompl_planner_ros/ompl_planner.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include "boost/bind.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_planner {

Planner2D::Planner2D(double max_step_length, nav_msgs::OccupancyGridConstPtr& map)
        : dim_(2), max_step_(max_step_length), grid_map_(map), frame_name_("map"), bounds_(dim_)
{
    ROS_INFO("Planner constructed.");
}


Planner2D::~Planner2D() {}


bool Planner2D::plan(const geometry_msgs::PoseWithCovariance& start, 
                    const geometry_msgs::PoseStamped& goal, 
                    std::vector<geometry_msgs::PoseStamped>& path) 
{
    auto space(std::make_shared<ob::RealVectorStateSpace>(dim_));
    
    // setup bounds IN meter
    bounds_.setLow(grid_map_->info.origin.position.x);
    bounds_.setLow(grid_map_->info.origin.position.y);
    bounds_.setHigh(grid_map_->info.width * grid_map_->info.resolution + grid_map_->info.origin.position.x);
    bounds_.setHigh(grid_map_->info.height * grid_map_->info.resolution + grid_map_->info.origin.position.y);
    
    space->setBounds(bounds_);

    // initialize the simple setup
    ss_ = std::make_shared<og::SimpleSetup>(space);
    
    ss_->setStateValidityChecker( boost::bind(&Planner2D::isStateValid, this, _1) );

    // sampling resoltion
    ss_->getSpaceInformation()->setStateValidityCheckingResolution(grid_map_->info.resolution); 

    ob::ScopedState<> start_ompl(space);
    start_ompl[0] = start.pose.position.x;
    start_ompl[1] = start.pose.position.y;

    ob::ScopedState<> goal_ompl(space);
    goal_ompl[0] = goal.pose.position.x;
    goal_ompl[1] = goal.pose.position.y;

    ss_->setStartAndGoalStates(start_ompl, goal_ompl);

    // ss_->setPlanner(std::make_shared<og::RRTConnect>(ss_->getSpaceInformation()));

    ss_->solve();

    if (ss_->haveSolutionPath()) {
        const std::size_t numSln = ss_->getProblemDefinition()->getSolutionCount();
        ROS_INFO("Found %d solutions", numSln);
        og::PathGeometric& p = ss_->getSolutionPath();
        // auto& stateVec = p.getState();
        // for(auto& s : stateVec) {
        //     geometry_msgs::PoseStamped tmpPose;
        //     state2pose(s, tmpPose);
        //     path.poses.push_back(tmpPose);
        // }
        return true;
    } else {
        return false;
    }
}

void Planner2D::state2pose(const ob::State* state, geometry_msgs::PoseStamped& pose) {
    const ob::RealVectorStateSpace::StateType *s = state->as<ob::RealVectorStateSpace::StateType>();
    pose.pose.position.x = s->values[0];
    pose.pose.position.y = s->values[1];
    // pose.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    pose.header.frame_id = frame_name_;
    pose.header.stamp = ros::Time::now();
}

bool Planner2D::isStateValid(const ob::State* state) {
    // extract the state value as two-dimensional realVectorStateSpace 
    const ob::RealVectorStateSpace::StateType* s = state->as<ob::RealVectorStateSpace::StateType>();

    const double wx = s->values[0];
    const double wy = s->values[1];

    double origin_x = grid_map_->info.origin.position.x;
    double origin_y = grid_map_->info.origin.position.y;
    
    // the boundary check could also be carried out using native OMPL boundary check class;
    if( wx < origin_x || wy < origin_y ) {
        ROS_WARN("the state is out of the space boundary! ");
        return false;
    }

    double resln = grid_map_->info.resolution;

    unsigned int mx = (int)((wx - origin_x) / resln);
    unsigned int my = (int)((wy - origin_y) / resln);

    size_t mIndex = mx + grid_map_->info.width * my;

    unsigned int cells_x = grid_map_->info.width / resln;
    unsigned int cells_y = grid_map_->info.height / resln;

    if(mx < cells_x && my < cells_y) {
        if(OCCUPIED == grid_map_->data[mIndex]) {
            return false;
        }
    } else {
        return false;
    }

    return true;
}

} /* namespace */
