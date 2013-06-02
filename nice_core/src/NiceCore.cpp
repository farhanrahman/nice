#include <nice_core/NiceCore.hpp>

namespace nice_core
{
NiceCore::NiceCore(
	ros::NodeHandle& n,
	double rate
) : 
	nodeHandlePtr(&n),
	rate(rate)
{
	ac = new MoveBaseClient("move_base", true);

	mainGoalListener = n.subscribe(
						"nice_core/main_goal",
						10,
						&NiceCore::mainGoalListenerCallback,
						this
				);

	coreMode = IDLE;
}	
	
void NiceCore::setMainGoal(const geometry_msgs::PoseStamped& poseStamped){
	boost::mutex::scoped_lock(mainGoalMutex);
	mainGoal.target_pose.header.seq = poseStamped.header.seq;
	mainGoal.target_pose.header.stamp = poseStamped.header.stamp;
	mainGoal.target_pose.header.frame_id = poseStamped.header.frame_id;

	mainGoal.target_pose.pose.position.x = poseStamped.pose.position.x;
	mainGoal.target_pose.pose.position.y = poseStamped.pose.position.y;
	mainGoal.target_pose.pose.position.z = poseStamped.pose.position.z;

	mainGoal.target_pose.pose.orientation.x = poseStamped.pose.orientation.x;
	mainGoal.target_pose.pose.orientation.y = poseStamped.pose.orientation.y;
	mainGoal.target_pose.pose.orientation.z = poseStamped.pose.orientation.z;
	mainGoal.target_pose.pose.orientation.w = poseStamped.pose.orientation.w;
}

move_base_msgs::MoveBaseGoal NiceCore::getMainGoal(void){
	boost::mutex::scoped_lock(mainGoalMutex);
	move_base_msgs::MoveBaseGoal ret = this->mainGoal;
	return ret;
}

void NiceCore::doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
	ROS_INFO("DONE");
	this->setCoreMode(IDLE);
}

void NiceCore::mainGoalListenerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	this->setMainGoal(*msg);
	this->setCoreMode(PLANNING);

	ROS_INFO("Sending goal");
	boost::function<void (const actionlib::SimpleClientGoalState&, const move_base_msgs::MoveBaseResultConstPtr&) > f;

	f = boost::bind(&NiceCore::doneCallback, this, _1, _2);

	ac->sendGoal(this->getMainGoal(), f);

	// ac->waitForResult();
	// this->setCoreMode(IDLE);
}


void NiceCore::nodeLoop(void){
	ros::Rate r(this->rate);

	while(ros::ok()){

		ros::spinOnce();
		r.sleep();
	}
}

}