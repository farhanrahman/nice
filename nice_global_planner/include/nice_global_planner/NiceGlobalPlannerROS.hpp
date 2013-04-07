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

namespace nice_global_planner{

class NiceGlobalPlannerROS : public nav_core::BaseGlobalPlanner {
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

	~NiceGlobalPlannerROS(void) {}

private:
	std::string nodeName;

	costmap_2d::Costmap2DROS* costmap_ros;

	bool initialised;

};

}

#endif