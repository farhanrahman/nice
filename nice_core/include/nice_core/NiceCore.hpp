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
#include <boost/thread.hpp>

#include <vector>

namespace nice_core
{

class NiceCore
{
public:
	NiceCore(
		unsigned glbs = 1000,
		unsigned gpbs = 1000,
		unsigned gcbs = 1000,
		unsigned rplbs = 1000,
		unsigned rate = 10
	);
	~NiceCore();

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
	void GoalListenerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

	/**
	 * @brief Thread safe method to retrieve the last saved goal
	 * @return The last updated goal position assigned to the robot
	 */
	geometry_msgs::PoseStamped getGoal(void);

	/**
	 * @brief Thread safe method to update the goal
	 * @param g New goal position to be set
	 */
	void setGoal(const geometry_msgs::PoseStamped::ConstPtr& msg);
	/**
	 * @brief Updates the feedback about the position of the robot base
	 *  by listening to the robot base information published on the feedback
	 *  topic by move_base
	 * @param msg Feedback message published by move_base
	 */
	void robotPositionListenerCallBack(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);


	/**
	 * @brief Function that sends a goal position to the move_base
	 * @param goal New goal position to be sent to the move_base module
	 */
	void sendGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);

	/**
	 * @brief Function that sends a goal position to the move_base
	 * @param goal New goal position of type geometry_msgs::PoseStamped
	 */
	void sendGoal(const geometry_msgs::PoseStamped& goal);

	/**
	 * @brief Cancels a goal with a particular ID
	 * @param goalID ID of the goal to cancel
	 **/
	void cancelGoal(const actionlib_msgs::GoalID goalId);

	/**
	 * @brief Returns the most upto date value of the robot's base
	 *  as reported by the move_base module
	 * @return The most upto date robot position
	 */
	geometry_msgs::PoseStamped getRobotPosition(void);

	ros::Subscriber goalListener;
	ros::Publisher coreGoalPublisher;
	ros::Publisher goalCanceller;

	ros::Subscriber robotPositionListener;

	ros::ServiceClient planService;
	ros::ServiceClient clearUnknownService;
	ros::ServiceClient clearCostmapService;

	ros::NodeHandle nh;

	unsigned goalListenerBufferSize;
	unsigned goalPublisherBufferSize;
	unsigned goalCancellerBufferSize;
	unsigned robotPositionListenerBufferSize;
	unsigned rate;

	boost::mutex goalLock;
	geometry_msgs::PoseStamped goal;

	move_base_msgs::MoveBaseActionFeedback feedback;

	bool newGoalPublished;
	boost::mutex newGoalLock;

};

}

#endif