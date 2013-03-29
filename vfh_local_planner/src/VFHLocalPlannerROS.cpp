#include <vfh_local_planner/VFHLocalPlannerROS.hpp>
#include <pluginlib/class_list_macros.h>
#include <boost/bind.hpp>
#include <base_local_planner/goal_functions.h>

//Declare the VFHLocalPlannerROS as a nav_core::BaseLocalPlanner class
PLUGINLIB_DECLARE_CLASS(vfh_local_planner, VFHLocalPlannerROS, vfh_local_planner::VFHLocalPlannerROS, nav_core::BaseLocalPlanner); 

namespace vfh_local_planner{

VFHLocalPlannerROS::VFHLocalPlannerROS(void) : 
	tf(NULL), 
	costmap_ros(NULL), 
	initialised(false)
{
	/*Default parameters for now. Will convert them to
	 *integrate with dynamic reconfiguration later on*/
	rot_stopped_vel = 1e-2;
	trans_stopped_vel = 1e-2;
	xy_goal_tolerance = 0.1;
	yaw_goal_tolerance = 0.05;	

}

bool VFHLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
	ROS_INFO("VFHLocalPlannerROS::computeVelocityCommands");
	return false;
}

bool VFHLocalPlannerROS::isGoalReached(void){
	if(!initialised){
		ROS_WARN("VFHLocalPlannerROS has not been initialised");
		return false;
	}

    //copy over the odometry information
    nav_msgs::Odometry odomCopy;
    {
      boost::recursive_mutex::scoped_lock(odomMutex);
      odomCopy = baseOdom;
    }

    return base_local_planner::isGoalReached(*tf, globalPlan, *costmap_ros, costmap_ros->getGlobalFrameID(), odomCopy, 
        rot_stopped_vel, trans_stopped_vel, xy_goal_tolerance, yaw_goal_tolerance);
}

bool VFHLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
	if(!initialised){
		ROS_WARN("VFHLocalPlannerROS has not been initialised");
		return false;
	}

	/*Clear the current global plan and assign the new plan*/
	globalPlan.clear();
	globalPlan = plan;

	return true;
}

void VFHLocalPlannerROS::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS* costmap_ros){
	ROS_INFO("VFHLocalPlannerROS::initialise");
	if(initialised){
		ROS_WARN("VFHLocalPlannerROS::initialised function called more than once");
	} else {
		/*Assign the variables of the object that are being
		 *passed as arguments in this function.*/
		this->name = name;
		this->tf = tf;
		this->costmap_ros = costmap_ros;

		/*Get a copy of the costmap*/
		(*this).costmap_ros->getCostmapCopy(this->costmap);

		ros::NodeHandle nh;

		this->odomSub = nh.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&VFHLocalPlannerROS::odometryCallback, this, _1));

		this->initialised = true;
	}
}


void VFHLocalPlannerROS::odometryCallback(const nav_msgs::Odometry::ConstPtr& odometryMsg){
    /*Get a lock on the mutual exclusion and assign
	 *the base odom to the new message
	 */
    boost::mutex::scoped_lock(odomMutex);
    baseOdom.twist.twist.linear.x = odometryMsg->twist.twist.linear.x;
    baseOdom.twist.twist.linear.y = odometryMsg->twist.twist.linear.y;
    baseOdom.twist.twist.angular.z = odometryMsg->twist.twist.angular.z;
}

}