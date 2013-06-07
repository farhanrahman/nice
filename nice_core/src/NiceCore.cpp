#include <nice_core/NiceCore.hpp>
#include <iostream>

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

	n.param("nice_core/follow_threshold", followThreshold, 1.5);
	n.param("nice_core/follow_goal_discard", followGoalDiscard, 2.0);

	n.param("nice_core/delta_distance_init_time", deltaDistanceInitTime, 3.0);

	ROS_INFO("followThreshold: %f", followThreshold);

	followThresholdSq = followThreshold*followThreshold;
	followGoalDiscardSq = followGoalDiscard*followGoalDiscard;

	coreMode = IDLE;

	kalmanListener = n.subscribe(
				"kalman_points",
				100,
				&NiceCore::kalmanCallback,
				this
			);

	f = boost::bind(&NiceCore::doneCallback, this, _1, _2);
	f2 = boost::bind(&NiceCore::feedbackCallback, this, _1);

	deltaDistanceSq = 0.0;
}

NiceCore::~NiceCore(void){
	delete ac;
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

void NiceCore::setSubGoal(const geometry_msgs::PoseStamped& poseStamped){
	boost::mutex::scoped_lock(subGoalMutex);
	subGoal.target_pose.header.seq = poseStamped.header.seq;
	subGoal.target_pose.header.stamp = poseStamped.header.stamp;
	subGoal.target_pose.header.frame_id = poseStamped.header.frame_id;

	subGoal.target_pose.pose.position.x = poseStamped.pose.position.x;
	subGoal.target_pose.pose.position.y = poseStamped.pose.position.y;
	subGoal.target_pose.pose.position.z = poseStamped.pose.position.z;

	subGoal.target_pose.pose.orientation.x = poseStamped.pose.orientation.x;
	subGoal.target_pose.pose.orientation.y = poseStamped.pose.orientation.y;
	subGoal.target_pose.pose.orientation.z = poseStamped.pose.orientation.z;
	subGoal.target_pose.pose.orientation.w = poseStamped.pose.orientation.w;	
}

move_base_msgs::MoveBaseGoal NiceCore::getSubGoal(void){
	boost::mutex::scoped_lock(subGoalMutex);
	move_base_msgs::MoveBaseGoal ret = this->subGoal;
	return ret;
}


void NiceCore::doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
	std::string stateStr = state.toString();
	if( (stateStr == "SUCCEEDED") || (stateStr == "REJECTED") || (stateStr == "ABORTED") ){
		CoreMode cmCopy = this->getCoreMode();
		if(cmCopy == FOLLOWING){
			this->setCoreMode(PLANNING);
			move_base_msgs::MoveBaseGoal mg = this->getMainGoal();
			ROS_INFO("Sending main goal: (%f, %f) after resuming PLANNING because sub goal was %s", 
				mg.target_pose.pose.position.x, 
				mg.target_pose.pose.position.y,
				stateStr.c_str());
			ac->sendGoal(mg, f, MoveBaseClient::SimpleActiveCallback(), f2);
		} else {
			ROS_INFO("DONE");
			this->setCoreMode(IDLE);
		}
	}
}

void NiceCore::feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){
	this->updateBasePosition(feedback->base_position);
	CoreMode coreModeCopy = this->getCoreMode();
	if(coreModeCopy == FOLLOWING){
		move_base_msgs::MoveBaseGoal sg = this->getSubGoal();
		double dx = (*feedback).base_position.pose.position.x - sg.target_pose.pose.position.x;
		double dy = (*feedback).base_position.pose.position.y - sg.target_pose.pose.position.y;

		double distanceToSubGoalSq = dx*dx + dy*dy;

		if(distanceToSubGoalSq < followGoalDiscardSq){
			this->setCoreMode(PLANNING);
			move_base_msgs::MoveBaseGoal mg = this->getMainGoal();
			ROS_INFO("Robot is near to sub goal (%f,%f), so resuming main goal (%f,%f)",
				sg.target_pose.pose.position.x, sg.target_pose.pose.position.y,
				mg.target_pose.pose.position.x, mg.target_pose.pose.position.y);
			ac->sendGoal(this->getMainGoal(), f, MoveBaseClient::SimpleActiveCallback(), f2);
		}
	}
}

void NiceCore::mainGoalListenerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	this->setMainGoal(*msg);
	this->setCoreMode(PLANNING);

	ROS_INFO("Sending main goal (%f,%f)", (*msg).pose.position.x, (*msg).pose.position.y);

	ac->sendGoal(this->getMainGoal(), f, MoveBaseClient::SimpleActiveCallback(), f2);
}

void NiceCore::kalmanCallback(const move_base_msgs::MoveBaseGoal::ConstPtr& msg){
	boost::mutex::scoped_lock(kalmanMutex);
	(*this).kalmanPointQueue.push(*msg);
}


void NiceCore::nodeLoop(void){
	ros::Rate r(this->rate);

	while(ros::ok()){
		CoreMode cmCopy = this->getCoreMode();
		if(cmCopy == PLANNING) {
			std::queue<move_base_msgs::MoveBaseGoal> qCopy;
			{
				boost::mutex::scoped_lock(kalmanMutex);
				double now = ros::Time::now().toSec();
				if(kalmanPointQueue.size() != 0){
					move_base_msgs::MoveBaseGoal latestData = kalmanPointQueue.back();
					double diff = latestData.target_pose.header.stamp.toSec() - now;
					if(diff >= deltaDistanceInitTime)
						deltaDistanceSq = 0.0;
				}

				for(unsigned i = 0; i < kalmanPointQueue.size(); ++i){
					move_base_msgs::MoveBaseGoal personPoint = kalmanPointQueue.front();
					geometry_msgs::PoseStamped bp = this->getBasePosition();
					double dx = personPoint.target_pose.pose.position.x - bp.pose.position.x;
					double dy = personPoint.target_pose.pose.position.y - bp.pose.position.y;
					deltaDistanceSq = (dx*dx + dy*dy) - deltaDistanceSq;
					qCopy.push(kalmanPointQueue.front());
					kalmanPointQueue.pop();
				}
				// ROS_INFO("deltaDistanceSQ: %f", deltaDistanceSq);
			}

			if(qCopy.size() != 0){
				double maxDistanceSq = 0.0;
				double distanceSqToGoal = 0.0;
				geometry_msgs::PoseStamped bp = this->getBasePosition();
				move_base_msgs::MoveBaseGoal goalCopy = this->getMainGoal();
				double dx = bp.pose.position.x - goalCopy.target_pose.pose.position.x;
				double dy = bp.pose.position.y - goalCopy.target_pose.pose.position.y;
				distanceSqToGoal = dx*dx + dy*dy;
				bool personFound = false;
				move_base_msgs::MoveBaseGoal personToFollow;

				if(distanceSqToGoal < followThresholdSq) {
					ROS_INFO("Main goal (%f,%f) too close to follow a person as distanceSqToGoal(%f) < followThresholdSq(%f)",
						goalCopy.target_pose.pose.position.x,
						goalCopy.target_pose.pose.position.y,
						distanceSqToGoal, followThresholdSq);
					for(unsigned i = 0; i < qCopy.size(); ++i){
						qCopy.pop();
					}
				} else if (deltaDistanceSq < 0){
					ROS_INFO("Sub goal approaching so not following it");
					for(unsigned i = 0; i < qCopy.size(); ++i){
						qCopy.pop();
					}
				} else {

					for(unsigned i = 0; i < qCopy.size(); ++i){
						double personDistanceSqToGoal = 0.0;
						move_base_msgs::MoveBaseGoal person = qCopy.front();
						qCopy.pop();
						double pdx = goalCopy.target_pose.pose.position.x - person.target_pose.pose.position.x;
						double pdy = goalCopy.target_pose.pose.position.y - person.target_pose.pose.position.y;
						personDistanceSqToGoal = pdx*pdx + pdy*pdy;
						if(personDistanceSqToGoal < distanceSqToGoal){
							if(personDistanceSqToGoal > maxDistanceSq){
								maxDistanceSq = personDistanceSqToGoal;
								personFound = true;
								personToFollow = person;
							}
						}
					}
				}

				if(personFound){
					this->setCoreMode(FOLLOWING);
					ROS_INFO("Sending human sub goal (%f,%f)", 
						personToFollow.target_pose.pose.position.x,
						personToFollow.target_pose.pose.position.y
					);
					this->setSubGoal(personToFollow.target_pose);
					ac->sendGoal(personToFollow, f, MoveBaseClient::SimpleActiveCallback(), f2);
				}
			}
		} else {
			boost::mutex::scoped_lock(kalmanMutex);
			for(unsigned i = 0; i < kalmanPointQueue.size(); ++i){				
				kalmanPointQueue.pop();
			}
			deltaDistanceSq = 0.0;
		}

		ros::spinOnce();
		r.sleep();
	}
}

}