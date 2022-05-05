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

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ompl/geometric/SimpleSetup.h>

#define OCCUPIED 100
#define FREE 0

namespace ompl_planner {

class Planner2D {
public:
    Planner2D(double max_step_length, nav_msgs::OccupancyGridConstPtr& map);

    virtual ~Planner2D();

    bool plan(const geometry_msgs::PoseWithCovariance& start, 
            const geometry_msgs::PoseStamped& goal, 
            std::vector<geometry_msgs::PoseStamped>& path);

protected:
    bool isStateValid(const ompl::base::State* state);

    void state2pose(const ompl::base::State* state, geometry_msgs::PoseStamped& pose);

private:

    unsigned int dim_;

    double max_step_;

    std::string frame_name_;

    ompl::geometric::SimpleSetupPtr ss_;

    ompl::base::RealVectorBounds bounds_;

    nav_msgs::OccupancyGridConstPtr grid_map_;

    tf::TransformListener* tf_listener_; 

}; // class Planner2D

} // namespace

#endif