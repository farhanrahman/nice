#ifndef NICE_CORE
#define NICE_CORE
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <boost/thread/mutex.hpp>
#include <actionlib/action_definition.h>

#include <queue>

#include <limits>

namespace nice_core
{

typedef enum CoreMode{
	IDLE = 0,
	PLANNING = 1,
	FOLLOWING = 2
} CoreMode;

class NiceCore
{
public:
	NiceCore(
		ros::NodeHandle& n,
		double rate = 10
	);
	~NiceCore();

	void nodeLoop(void);

private:
	double rate;


	void mainGoalListenerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void setMainGoal(const geometry_msgs::PoseStamped& poseStamped);
	move_base_msgs::MoveBaseGoal getMainGoal(void);

	ros::NodeHandle* nodeHandlePtr;


	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	MoveBaseClient* ac;
	void doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
	boost::function<void (const actionlib::SimpleClientGoalState&, const move_base_msgs::MoveBaseResultConstPtr&) > f;
	void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
	boost::function<void (const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) > f2;

	void updateBasePosition(const geometry_msgs::PoseStamped& base_position){
		boost::mutex::scoped_lock(baseUpdateMutex);
		this->base_position = base_position;
	}

	geometry_msgs::PoseStamped getBasePosition(void){
		boost::mutex::scoped_lock(baseUpdateMutex);
		geometry_msgs::PoseStamped ret = this->base_position;
		return ret;		
	}

	boost::mutex baseUpdateMutex;
	geometry_msgs::PoseStamped base_position;

	boost::mutex mainGoalMutex;
	move_base_msgs::MoveBaseGoal mainGoal;
	ros::Subscriber mainGoalListener;

	ros::Subscriber kalmanListener;
	void kalmanCallback(const move_base_msgs::MoveBaseGoal::ConstPtr& msg);
	boost::mutex kalmanMutex;
	std::queue<move_base_msgs::MoveBaseGoal> kalmanPointQueue;


	boost::mutex cmMutex;
	CoreMode coreMode;

	void setCoreMode(CoreMode cm){
		boost::mutex::scoped_lock(cmMutex);
		this->coreMode = cm;
	}

	CoreMode getCoreMode(void){
		boost::mutex::scoped_lock(cmMutex);
		CoreMode ret = this->coreMode;
		return ret;
	}

};
}

#endif