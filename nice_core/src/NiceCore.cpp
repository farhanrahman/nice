#include <nice_core/NiceCore.hpp>

namespace nice_core
{

NiceCore::NiceCore(
		unsigned glbs,
		unsigned gpbs,
		unsigned gcbs,
		unsigned rplbs,
		unsigned rate
) : goalListenerBufferSize(glbs),
	goalPublisherBufferSize(gpbs),
	goalCancellerBufferSize(gcbs),
	robotPositionListenerBufferSize(rplbs),
	rate(rate)
{
	/*Initialise the subscribers and publishers*/
	goalListener = nh.subscribe(
						"move_base_simple/goal", 
						goalListenerBufferSize, 
						&NiceCore::GoalListenerCallback, 
						this
					);

	coreGoalPublisher = nh.advertise<geometry_msgs::PoseStamped>(
						"move_base_simple/filtered_goal",
						goalPublisherBufferSize
					);

	
	goalCanceller = nh.advertise<actionlib_msgs::GoalID>(
						"actionlib_msgs/GoalID",
						goalCancellerBufferSize
					);


	robotPositionListener = nh.subscribe(
						"move_base/feedback",
						robotPositionListenerBufferSize,
						&NiceCore::robotPositionListenerCallBack,
						this
					);

	/*Initialise the clients*/
	planService = nh.serviceClient<nav_msgs::GetPlan> ("make_plan");
	clearUnknownService = nh.serviceClient<std_srvs::Empty>("clear_unknown_space");
	clearCostmapService = nh.serviceClient<std_srvs::Empty>("clear_costmaps");

}

NiceCore::~NiceCore(void){
}

void NiceCore::nodeLoop(void){
	ros::Rate r((*this).rate);
	ros::NodeHandle n;
	while(n.ok()){
		

		ros::spinOnce();
		r.sleep();
	}
}


geometry_msgs::PoseStamped NiceCore::getGoal(void){
	boost::mutex::scoped_lock(goalLock);
	geometry_msgs::PoseStamped ret = this->goal;
	return ret;
}

void NiceCore::setGoal(const geometry_msgs::PoseStamped::ConstPtr& msg){
	boost::mutex::scoped_lock(goalLock);
	this->goal = (*msg);
}

geometry_msgs::PoseStamped NiceCore::getRobotPosition(void){
	boost::mutex::scoped_lock(feedbackLock);
	geometry_msgs::PoseStamped ret = (*this).feedback.feedback.base_position;
	return ret;
}


void NiceCore::sendGoal(const geometry_msgs::PoseStamped::ConstPtr& goal){
	(*this).coreGoalPublisher.publish(*goal);
}

void NiceCore::sendGoal(const geometry_msgs::PoseStamped& goal){
	(*this).coreGoalPublisher.publish(goal);
}

/*Callback functions*/
void NiceCore::GoalListenerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	/*Set the new goal*/
	(*this).setGoal(msg);
	/*Send plan to move_base to start off the base*/
	(*this).sendGoal((*this).getGoal());
	/*If planner thread is inactive then resume it*/

}

void NiceCore::robotPositionListenerCallBack(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg){
	boost::mutex::scoped_lock(feedbackLock);
	this->feedback = (*msg);
}


	
}