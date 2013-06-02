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

namespace nice_core
{

typedef enum CoreMode{
	IDLE = 0,
	PLANNING = 1,
	FOLLOWING
} CoreMode;

class NiceCore
{
public:
	NiceCore(
		ros::NodeHandle& n,
		double rate = 10
	);
	~NiceCore(){}

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

	boost::mutex mainGoalMutex;
	move_base_msgs::MoveBaseGoal mainGoal;
	ros::Subscriber mainGoalListener;


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