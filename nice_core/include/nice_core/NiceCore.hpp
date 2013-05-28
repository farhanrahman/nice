#ifndef NICE_CORE_HPP
#define NICE_CORE_HPP
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <nav_msgs/GetPlan.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Empty.h>

#include <actionlib_msgs/GoalID.h>

#include <geometry_msgs/PoseStamped.h>
#include <boost/thread/mutex.hpp>

#include <vector>

namespace nice_core
{

class NiceCore
{
public:
	NiceCore(
		unsigned glbs = 1000,
		unsigned gpbs = 1000,
		unsigned gsbs = 1000,
		unsigned gcbs = 1000,
		unsigned rate = 10
	);
	~NiceCore(){}

	/**
	 *@brief Main segment of the code that will get
	 * executed as part of a ros node
	 */
	void nodeLoop(void);

private:
	/**
	 *@brief Callback function for a new goal published from an
	 *  external node. Will forward the goal if necessary to
	 *  move_base node.
	 *@param msg Goal point being sent on the topic
	 */
	void GoalListenerCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg);

	/**
	 *@brief Callback function for goal status
	 *@param msg Message published from an external node
	 * containing information about the status of previous
	 * goals published
	 */
	void GoalStatusListenerCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

	/**
	 * @brief Thread safe method to retrieve the last saved goal
	 * @return The last updated goal position assigned to the robot
	 */
	geometry_msgs::PoseStamped getGoal(void);

	ros::Subscriber goalListener;
	ros::Publisher coreGoalPublisher;
	ros::Publisher goalCanceller;

	ros::Subscriber goalStatusListener; 

	ros::ServiceClient planService;
	ros::ServiceClient clearUnknownService;
	ros::ServiceClient clearCostmapService;

	ros::NodeHandle nh;

	unsigned goalListenerBufferSize;
	unsigned goalPublisherBufferSize;
	unsigned goalCancellerBufferSize;
	unsigned goalStatusListenerBufferSize;
	unsigned rate;

	boost::mutex goalLock;
	geometry_msgs::PoseStamped goal;

	boost::mutex goalStatusLock;
	std::vector<actionlib_msgs::GoalStatus> goalInfos;

};

}

#endif