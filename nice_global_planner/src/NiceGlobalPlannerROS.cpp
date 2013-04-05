#include <nice_global_planner/NiceGlobalPlannerROS.hpp>
#include <pluginlib/class_list_macros.h>
#include <sstream>
#include <string>


//Declare the NiceGlobalPlannerROS as a nav_core::BaseGlobalPlanner class
PLUGINLIB_DECLARE_CLASS(nice_global_planner, NiceGlobalPlannerROS, nice_global_planner::NiceGlobalPlannerROS, nav_core::BaseGlobalPlanner); 

namespace nice_global_planner{

NiceGlobalPlannerROS::NiceGlobalPlannerROS(void){
	initialised = false;
}

bool NiceGlobalPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
	const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
	// ROS_INFO(start);
	// ROS_INFO(goal);
	std::stringstream ss;
	std::cout << "goal.pose.position.x: " << goal.pose.position.x
	<< ", goal.pose.position.y: " << goal.pose.position.y
	<< ", goal.pose.position.z: " << goal.pose.position.z;

	std::string str;
	ss >> str;

	std::cout << str << std::endl;

	//ROS_INFO(str.c_str());

	return false;
}

void NiceGlobalPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	if(initialised){
		ROS_WARN("NiceGlobalPlannerROS already initialised");
		return;
	}

	/*Copy over the pointer to the costmap_ros*/
	this->costmap_ros = costmap_ros;

	this->nodeName = name;

	/*Flag the initialised parameter to true*/
	this->initialised = true;
}

}