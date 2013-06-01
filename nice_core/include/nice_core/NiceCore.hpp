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

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>

#include <costmap_update/tfMessageParser.hpp>
#include <costmap_update/ParserMessage.hpp>

#include <nice_detector/Detection.h>

#include <queue>

namespace nice_core
{

typedef enum CoreMode{
	IDLE = 0,
	FOLLOWING = 1,
	PLANNING = 2
} CoreMode;

class NiceCore
{
public:
	NiceCore(
		ros::NodeHandle& n,
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


	/**
	 * @brief Callback function for tf messages published on the
	 *  openni_tracker topic
	 * @param msg An array of TransformStamped messages
	 **/
	void tfCallback(const tf::tfMessage::ConstPtr& msg);

	/**
	 * @brief Thread safe getter function
	 * @return Returns a copy of the Core Mode
	 **/
	CoreMode getCoreMode(void);

	/**
	 * @brief Thread safe method to update the core module mode
	 * @param c New mode to be set
	 **/
	void setCoreMode(CoreMode c);

	/**
	 * @brief Thread safe method to update the following agent number
	 * @param fa The agent number to be set that the base_link should follow
	 **/
	void setFollowingAgent(int fa);	

	/**
	 * @brief Calculates the squared distance in the Euclidian space
	 * @return Returns the squared Euclidian distance
	 */
	double getDistanceSq(const geometry_msgs::Transform& transform);

	/**
	 * @brief Checks whether a particular agent is facing the goal
	 * @param transform Transform of the agent
	 * @return True if the agent with transform is facing the goal
	 */
	bool facingGoal(const geometry_msgs::Transform& transform);


	/**
	 * @brief Method for retreiving the following agent number
	 * @return Returns the following agent number in a thread safe method
	 **/
	int getFollowingAgent(void);

	/**/
	void detectionCallback(const nice_core::DetectionListConstPtr& detects);

	ros::Subscriber goalListener;
	ros::Publisher coreGoalPublisher;
	ros::Publisher goalCanceller;

	ros::Subscriber robotPositionListener;

	ros::ServiceClient planService;
	ros::ServiceClient clearUnknownService;
	ros::ServiceClient clearCostmapService;

	ros::NodeHandle* nhPtr;

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

	ros::Subscriber tfListener;

	boost::mutex cmMutex;
	CoreMode coreMode;

	tf::TransformListener tfl;

	int followingAgent;

	ros::Publisher pub;

	ros::Subscriber detection;

	boost::mutex dqMutex;
	std::queue<geometry_msgs::PointStamped> detectionQueue;
};

}

#endif