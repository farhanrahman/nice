#include <nice_core/NiceCore.hpp>

namespace nice_core
{

NiceCore::NiceCore(
		unsigned glbs,
		unsigned gpbs,
		unsigned gsbs,
		unsigned gcbs,
		unsigned rate
) : goalListenerBufferSize(glbs),
	goalPublisherBufferSize(gpbs),
	goalStatusListenerBufferSize(gsbs),
	goalCancellerBufferSize(gcbs),
	rate(rate)
{
	/*Initialise the subscribers and publishers*/
	goalListener = nh.subscribe(
						"move_base/goal", 
						goalListenerBufferSize, 
						&NiceCore::GoalListenerCallback, 
						this
					);


	coreGoalPublisher = nh.advertise<move_base_msgs::MoveBaseActionGoal>(
						"move_base/refined_goal",
						goalPublisherBufferSize
					);

	goalStatusListener = nh.subscribe(
						"move_base/status",
						goalStatusListenerBufferSize,
						&NiceCore::GoalStatusListenerCallback,
						this						
					);

	goalCanceller = nh.advertise<actionlib_msgs::GoalID>(
						"actionlib_msgs/GoalID",
						goalCancellerBufferSize
					);


	/*Initialise the clients*/
	planService = nh.serviceClient<nav_msgs::GetPlan> ("make_plan");
	clearUnknownService = nh.serviceClient<std_srvs::Empty>("clear_unknown_space");
	clearCostmapService = nh.serviceClient<std_srvs::Empty>("clear_costmaps");

}

void NiceCore::nodeLoop(void){
	while(ros::ok()){
				
	}
}

geometry_msgs::PoseStamped NiceCore::getGoal(void){
	boost::mutex::scoped_lock(goalLock);
	geometry_msgs::PoseStamped ret = this->goal;
	return ret;
}

void NiceCore::GoalListenerCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg){
	boost::mutex::scoped_lock(goalLock);
	this->goal = (*msg).goal.target_pose;
}

void NiceCore::GoalStatusListenerCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
	boost::mutex::scoped_lock(goalStatusLock);
	unsigned size = (*msg).status_list.size();
	for (unsigned i = 0; i < size; ++i){
		(*this).goalInfos.push_back((*msg).status_list[i]);
	}
}

	
}