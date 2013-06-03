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

	ROS_INFO("followThreshold: %f", followThreshold );

	followThresholdSq = followThreshold*followThreshold;

	coreMode = IDLE;

	kalmanListener = n.subscribe(
				"kalman_points",
				100,
				&NiceCore::kalmanCallback,
				this
			);

	f = boost::bind(&NiceCore::doneCallback, this, _1, _2);
	f2 = boost::bind(&NiceCore::feedbackCallback, this, _1);
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

void NiceCore::doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
	std::string stateStr = state.toString();
	if( (stateStr == "SUCCEEDED") || (stateStr == "REJECTED") || (stateStr == "ABORTED") ){
		CoreMode cmCopy = this->getCoreMode();
		if(cmCopy == FOLLOWING){
			this->setCoreMode(PLANNING);
			ROS_INFO("Sending main goal after resuming PLANNING");
			ac->sendGoal(this->getMainGoal(), f, MoveBaseClient::SimpleActiveCallback(), f2);
		} else {
			ROS_INFO("DONE");
			this->setCoreMode(IDLE);
		}
	}
}

void NiceCore::feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){
	this->updateBasePosition(feedback->base_position);
}

void NiceCore::mainGoalListenerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	this->setMainGoal(*msg);
	this->setCoreMode(PLANNING);

	ROS_INFO("Sending main goal");

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
		if(cmCopy != IDLE){
			std::queue<move_base_msgs::MoveBaseGoal> qCopy;
			{
				boost::mutex::scoped_lock(kalmanMutex);
				for(unsigned i = 0; i < kalmanPointQueue.size(); ++i){
					qCopy.push(kalmanPointQueue.front());
					kalmanPointQueue.pop();
				}
			}

			if(qCopy.size() != 0){
				/*#TODO for now just get the latest point.
				 * Later will analyse points with goal point
				 * to see whether the person is coming towards
				 * or going away*/
				double minDistanceSq = std::numeric_limits<double>::max();
				double distanceSqToGoal = 0.0;
				geometry_msgs::PoseStamped bp = this->getBasePosition();
				move_base_msgs::MoveBaseGoal goalCopy = this->getMainGoal();
				double dx = bp.pose.position.x - goalCopy.target_pose.pose.position.x;
				double dy = bp.pose.position.y - goalCopy.target_pose.pose.position.y;
				distanceSqToGoal = dx*dx + dy*dy;
				bool personFound = false;
				move_base_msgs::MoveBaseGoal personToFollow;

				if(distanceSqToGoal < followThresholdSq){
					ROS_INFO("Goal too close to follow a person");
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
							if(personDistanceSqToGoal < minDistanceSq){
								personFound = true;
								personToFollow = person;
							}
						}
					}
				}

				if(personFound){
					this->setCoreMode(FOLLOWING);
					ROS_INFO("Sending human sub goal");
					ac->sendGoal(personToFollow, f, MoveBaseClient::SimpleActiveCallback(), f2);
				}
			}
		}

		ros::spinOnce();
		r.sleep();
	}
}

}