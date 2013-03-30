#include <vfh_local_planner/VFHLocalPlannerROS.hpp>
#include <pluginlib/class_list_macros.h>
#include <boost/bind.hpp>
#include <base_local_planner/goal_functions.h>
#include <tf/transform_datatypes.h>
#include <math.h>

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

	/*For now set the window size to 30x30*/
	windowSize = 30;

	/*Initialise the magnitude and direction vectors*/
	m = new double *[windowSize];
	beta = new double *[windowSize];	

	for(unsigned i = 0; i < windowSize; ++i){
		m[i] = new double[windowSize];
		beta[i] = new double[windowSize];
	}

	/*Assign a, b and dmax*/
	/*a and b are selected in such a way such that
	 *a/b = dmax*/
	a = 1.0; /*Assigning a to 1.0 for now*/
	/*magnitude is proportional to (a - d[i][j]*b)
	 *therefore with this value for b, the cell
	 *farthest away will have the lowest magnitude*/
	b = sqrt(2.0)*(windowSize - 1.0)/2.0;

}

VFHLocalPlannerROS::~VFHLocalPlannerROS(void){
	for(unsigned i = 0; i < windowSize; ++i){
		delete[] m[i];
		delete[] beta[i];
	}

	delete[] m;
	delete[] beta;
}

bool VFHLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
	if (!initialised){
		ROS_WARN("VFHLocalPlannerROS not initialised");
		return false;
	}

	/*Get the global Pose of the robot in the map*/
	tf::Stamped<tf::Pose> globalPose;
	if(!costmap_ros->getRobotPose(globalPose)){
		/*Return false if the costmap was not
		 *able to retrieve the robot's global pose*/
		return false;
	}

	/*Get a copy of the active window around the robot*/
	(*this).costmap_ros->getCostmapWindowCopy(windowSize, windowSize, this->costmap);


	double cij = 0;
	double dij = 0;
	for(unsigned i = 0; i < windowSize; ++i)
		for(unsigned j = 0; j < windowSize; ++j){
			/*Get the points on the costmap*/
			double xi = costmap.getOriginX()+i;
			double yj = costmap.getOriginY()+j;
			
			/*Compute the robot's x and y positions*/
			double xo = globalPose.getOrigin().getY();
			double yo = globalPose.getOrigin().getY();

			/*Calculate the differences*/
			double xdiff = xi - xo;
			double ydiff = yj - yo; 

			/*Get the cost of cell*/
			cij = costmap.getCost(xi,yj);
			/*compute the beta or direction values*/
			beta[i][j] =  atan2( 
								ydiff 
							,	xdiff
							);

			/*Compute the distance between the two co-ordinates*/
			dij = sqrt(xdiff*xdiff + ydiff*ydiff);
			/*Compute the magnitude values for each cell*/
			m[i][j] = cij*cij*(a - b*dij);
	}



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