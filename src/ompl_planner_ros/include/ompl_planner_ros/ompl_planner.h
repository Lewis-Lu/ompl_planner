/**
 * @file ompl_planner.h
 * @author lewis (luh.lewis@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef OMPL_PLANNER_H_
#define OMPL_PLANNER_H_

#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>


#include <ros/ros.h>

namespace ompl_planner {

struct mapInfo {
    double resolution_;
    double origin_x_;
    double origin_y_;
    uint32_t map_width_;
    uint32_t map_height_;
    unsigned int size_x_;
    unsigned int size_y_;
}; // struct mapInfo

typedef std::vector<double> Point;
typedef std::vector<double> Limit;

struct navConfig {
    int dim_;
    double* sgconf_; // start and goal config
    double* bound_; // boundary config
    navConfig(int dim): dim_(dim) {
        sgconf_ = (double*)malloc(2 * dim_ * sizeof(double));
        bound_ = (double*)malloc(2 * dim_ * sizeof(double));
    }
    ~navConfig() {
        free(sgconf_);
        free(bound_);
    }
}; // struct navConfig

#define OCCUPIED 100
#define FREE 0

class ValidityChecker : public ompl::base::StateValidityChecker {
public:
    ValidityChecker(const ompl::base::SpaceInformationPtr& si, nav_msgs::OccupancyGrid* mi)
        : ompl::base::StateValidityChecker(si), grid_map_(mi) { }

    bool isValid(const ompl::base::State* state) const override {
        const double wx = (double)state->as<ob::RealVectorStateSpace::StateType>()->values[0];
        const double wy = (double)state->as<ob::RealVectorStateSpace::StateType>()->values[1];
        double origin_x = grid_map_->info.origin.position.x;
        double origin_y = grid_map_->info.origin.position.y;
        
        if( wx < origin_x || wy < origin_y ) {
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
private:
    nav_msgs::OccupancyGrid* grid_map_;

}; // validity checker class

class Planner2D {
public:
    Planner2D(double& max_step_length, int& dim, const navConfig& nc);

    Planner2D(double& max_step_length, nav_msgs::OccupancyGridConstPtr& map, const navConfig& nc);

    virtual ~Planner2D();

    /// configure node
    void initialize();

    bool plan();

    nav_msgs::Path extractPath(ompl::base::ProblemDefinition* pdef);

    mapInfo& getMapInformation();

private:
    bool initialized_;

    int dim_;

    double max_step_;

    ompl::base::SpaceInformationPtr si_ompl_;

    ompl::base::ProblemDefinitionPtr pdef_ompl_;

    mapInfo mi_;

    Point start_, goal_;

    Limit bound_x_, bound_y_;

    nav_msgs::OccupancyGridConstPtr grid_map_;

}; // class Planner2D

} // namespace

#endif