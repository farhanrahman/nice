#ifndef NICE_GLOBAL_PLANNER_ROS_HPP
#define NICE_GLOBAL_PLANNER_ROS_HPP
#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <string>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nice_global_planner/RRTPlanner.hpp>
#include <nice_global_planner/Costmap2DSampler.hpp>
#include <utils/IStamper.hpp>

#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include "CostCalculatorDelegate.hpp"

namespace nice_global_planner{

class NiceGlobalPlannerROS : public nav_core::BaseGlobalPlanner, CostCalculatorDelegate {
public:
	/**
	 * @brief Given a goal pose in the world, compute the plan
	 * @param start The start pose
	 * @param goal The goal pose
	 * @param plan the Plan... filled by the planner
	 * @return True if a valid plan was found, false otherwise
	 */
	bool makePlan(const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

	/**
	 * @brief Initialization function for the BaseGlobalPlanner
	 * @param name The name of the planner
	 * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning*/
	 void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);


	 /**
	  * @brief Default constructor
	  */
	 NiceGlobalPlannerROS(void);

	~NiceGlobalPlannerROS(void);

private:
	std::string nodeName;

	costmap_2d::Costmap2DROS* costmap_ros;
	costmap_2d::Costmap2D costmap;

	bool initialised;

	Costmap2DSampler *sampler;
	RRTPlanner *planner;
	utils::IStamper *stamper;

	double step_size_, min_dist_from_robot_;
    base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use
    double inscribed_radius_, circumscribed_radius_, inflation_radius_; 

      /**
       * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
       * @param x_i The x position of the robot 
       * @param y_i The y position of the robot 
       * @param theta_i The orientation of the robot
       * @return 
       */
    double footprintCost(double x_i, double y_i, double theta_i);

    std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief The footprint specification of the robot

};

}

#endif