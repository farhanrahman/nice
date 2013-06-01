#include <nice_core/NiceCore.hpp>
#include <limits>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <string>

namespace nice_core
{

NiceCore::NiceCore(
		ros::NodeHandle& nh,
		unsigned glbs,
		unsigned gpbs,
		unsigned gcbs,
		unsigned rplbs,
		unsigned rate
) : 
	nhPtr(&nh),
	goalListenerBufferSize(glbs),
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

	detection = nh.subscribe(
						"detections"
						1,
						&NiceCore::detectionCallback,
						this
					);

	// coreGoalPublisher = nh.advertise<geometry_msgs::PoseStamped>(
	// 					"move_base_simple/filtered_goal",
	// 					goalPublisherBufferSize
	// 				);

	
	// goalCanceller = nh.advertise<actionlib_msgs::GoalID>(
	// 					"actionlib_msgs/GoalID",
	// 					goalCancellerBufferSize
	// 				);


	// robotPositionListener = nh.subscribe(
	// 					"move_base/feedback",
	// 					robotPositionListenerBufferSize,
	// 					&NiceCore::robotPositionListenerCallBack,
	// 					this
	// 				);

	// tfListener = nh.subscribe(
	// 					"tf",
	// 					1000,
	// 					&NiceCore::tfCallback,
	// 					this
	// 				);

	// pub = nh.advertise<geometry_msgs::Twist>(
 //                		"cmd_vel",
 //                		10
 //                	);

	/*Initialise the clients*/
	planService = nh.serviceClient<nav_msgs::GetPlan> ("make_plan");
	clearUnknownService = nh.serviceClient<std_srvs::Empty>("clear_unknown_space");
	clearCostmapService = nh.serviceClient<std_srvs::Empty>("clear_costmaps");

	coreMode = IDLE;
	followingAgent = -1;




}

NiceCore::~NiceCore(void){
}


void NiceCore::nodeLoop(void){
	ros::Rate r((*this).rate);
	ros::NodeHandle n;

    const double frequency = this->rate;
    const double timeToUser = 6; //Time for robot to reach to user's position
    const double minDistance = 0.4; //30 cm
    const double minAngle = 0.09; //~5deg
    const double maxLinearSpeed = 0.3;// in metres per second
    const double maxAngularSpeed = 0.52; //~30 deg per second

    // Maximum time for transform to be available
    const double timeout = 0.1;

    const std::string destFrame = "base_link";

	while(n.ok()){

		CoreMode cmCopy = this->getCoreMode();

		if (cmCopy == FOLLOWING){
			ROS_INFO("FOLLOWING");
			std::stringstream ss;
			std::string originFrame;
			int fa = this->getFollowingAgent();
			ss << "torso_" << fa << std::endl;
			ss >> originFrame;

	        bool okay = tfl.waitForTransform(
	                    destFrame,
	                    originFrame,
	                    ros::Time::now() - ros::Duration(timeout), //Need recent tf
	                    ros::Duration(timeout)
	                    );

	        double linearSpeed = 0.0;
        	double angularSpeed = 0.0;

	        if(okay){
	            geometry_msgs::PointStamped input;
	            input.header.frame_id = originFrame;
	            input.header.stamp = ros::Time(0);

	            input.point.x = 0.0;
	            input.point.y = 0.0;
	            input.point.z = 0.0;

	            geometry_msgs::PointStamped output;

	            // TODO: Do you understand what this function is doing?
	            tfl.transformPoint( destFrame, input, output );


	            // README: Now we use the transformed point to calculate the
	            // required translational and rotational speeds.

	            // TODO: Compute the euclidian distance between the torso and
	            // the robot.
	            // HINT: To make things simpler, we project the point to the plane
	            // of the robot (in other words, we ignore the z value)
	            double distance = std::sqrt(
	                        output.point.x * output.point.x +
	                        output.point.y * output.point.y
	                        );

	            // TODO: Do you understand what this line of code is doing?
	            // HINT: http://www.cplusplus.com/reference/clibrary/cmath/atan2/
	            double angle = std::atan2(output.point.y, output.point.x);

	            // README: The two blocks of code make sure we don't go faster
	            // than the limits specified at the beginning of the file.
	            // WARNING: These blocks are safety-critical, please do not modify.
	            if ( std::abs(angle) > minAngle )
	            {
	                angularSpeed = std::abs( angle*frequency / timeToUser );

	                if (angularSpeed > maxAngularSpeed)
	                    angularSpeed = maxAngularSpeed;


	                if ( angle < 0 )
	                    angularSpeed = -angularSpeed;

	            }
	            if ( distance > minDistance )
	            {
	                linearSpeed =  distance * frequency /timeToUser ;

	                if (linearSpeed > maxLinearSpeed)
	                    linearSpeed = maxLinearSpeed;

	            }	        	
	        } else {
				this->setCoreMode(PLANNING);
				this->sendGoal((*this).getGoal());
				this->setFollowingAgent(-1);
	        }
		}

		ros::spinOnce();
		r.sleep();
	}
}

CoreMode NiceCore::getCoreMode(void){
	CoreMode ret = IDLE;
	{
		boost::mutex::scoped_lock(cmMutex);
		ret = coreMode;
	}
	return ret;
}

void NiceCore::setCoreMode(CoreMode c){
	{
		boost::mutex::scoped_lock(cmMutex);
		coreMode = c;
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

	CoreMode cmCopy = this->getCoreMode();

	if(cmCopy == IDLE){
		cmCopy = PLANNING;
		this->setCoreMode(cmCopy);
	}

	if(cmCopy == PLANNING){
		/*Send plan to move_base to start off the base*/
		(*this).sendGoal((*this).getGoal());
	}

}

void NiceCore::robotPositionListenerCallBack(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg){
	boost::mutex::scoped_lock(feedbackLock);
	this->feedback = (*msg);
}

void NiceCore::setFollowingAgent(int fa){
	boost::mutex::scoped_lock(faLock);
	followingAgent = fa;
}

int NiceCore::getFollowingAgent(void){
	boost::mutex::scoped_lock(faLock);
	int ret = this->followingAgent;
	return ret;
}


double NiceCore::getDistanceSq(const geometry_msgs::Transform& transform){
	double x = transform.translation.x;
	double y = transform.translation.y;
	double z = transform.translation.z;

	double gx = goal.pose.position.x;
	double gy = goal.pose.position.y;
	double gz = goal.pose.position.z;

	double dx = gx - x;
	double dy = gy - y;
	double dz = gz - z;

	return dx*dx + dy*dy + dz*dz;
}

bool NiceCore::facingGoal(const geometry_msgs::Transform& transform){
	double x = transform.translation.x;
	double y = transform.translation.y;
	double z = transform.translation.z;

	double gx = goal.pose.position.x;
	double gy = goal.pose.position.y;
	double gz = goal.pose.position.z;

	double dotProd = x*gx + y*gy + z*gz;

	return dotProd >= 0.0;
}

void NiceCore::cancelGoal(const actionlib_msgs::GoalID goalId){
	goalCanceller.publish(goalId);
}


void NiceCore::tfCallback(const tf::tfMessage::ConstPtr& msg){
	
	CoreMode cmCopy = this->getCoreMode();
	if(cmCopy == PLANNING){
		double minDistance = std::numeric_limits<double>::max();
		int agentNo = -1;

		for(unsigned i = 0; i < (*msg).transforms.size(); ++i){
			tfparser::message m;
			try{
				m = tfMessageParser::parseFrame((std::string) (*msg).transforms.at(i).child_frame_id);
			} catch(const FrameFormatNotCorrect &e){
				continue;
			}

			if ((m.part == "/torso")){
				geometry_msgs::Transform transform = (*msg).transforms.at(i).transform;
				double agentDistance = this->getDistanceSq(transform);
				bool facingGoal = this->facingGoal(transform);
				if((facingGoal) && (minDistance < agentDistance)){
					agentNo = m.user;
					minDistance = agentDistance;
				 }
			}
		}

		if(agentNo != -1){
			this->setCoreMode(FOLLOWING);
			geometry_msgs::PoseStamped goalCopy = this->getGoal();
			actionlib_msgs::GoalID goalId;
			goalId.stamp = goalCopy.header.stamp;
			goalId.id = goalCopy.header.frame_id;
			this->cancelGoal(goalId);
			this->setFollowingAgent(agentNo);
		}

	}
}

void detectionCallback(const nice_core::DetectionListConstPtr& detects){
	
}

	
}