#ifndef VFH_LOCAL_PLANNER_ROS_HPP
#define VFH_LOCAL_PLANNER_ROS_HPP

#include <string>

#include <ros/ros.h>

/*Definition for BaseLocalPlanner*/
#include <nav_core/base_local_planner.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <tf/transform_listener.h>

#include <boost/thread/mutex.hpp>

#include <nav_msgs/Odometry.h>

namespace vfh_local_planner{
class VFHLocalPlannerROS : public nav_core::BaseLocalPlanner {
public:
	VFHLocalPlannerROS(void);

	~VFHLocalPlannerROS(void){}

	/**
	 * @brief given the Pose and Odometry data of the robot, 
	 * compute the velocity commands to send to the robot
	 * @param cmd_vel command velocity to be updated for 
	 * the robot's base
	 * @return True if a valid path is found
	 */
	bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

	/**
	 *@brief compute if the robot has reached its goal
	 *@return True if the robot has reached its goal
	 */
	bool isGoalReached(void);

	/**
	* @brief Set the plan that the controller is following
	* @param plan The plan to pass to the controller
	* @return True if the plan was updated successfully, false otherwise
	*/
	bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

	/*
	 * @brief initialise the VFH object with the required parameters
	 * @param name the name of the node
	 * @param tf pointer to a transform listener
	 * @param costmap_ros wrapper for a costmap
	 */
	void initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS* costmap_ros);

private:
	void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometryMsg);


	bool initialised;
	costmap_2d::Costmap2DROS *costmap_ros;
	costmap_2d::Costmap2D costmap;

	tf::TransformListener *tf;

	std::string name;

	/*Variables related to odometry*/
	boost::mutex odomMutex;
	ros::Subscriber odomSub;
	nav_msgs::Odometry baseOdom;

	/*Current plan*/
	std::vector<geometry_msgs::PoseStamped> globalPlan;


	/*Parameters for the planner*/
	double rot_stopped_vel;
	double trans_stopped_vel;
	double xy_goal_tolerance;
	double yaw_goal_tolerance;	

};

}



#endif
