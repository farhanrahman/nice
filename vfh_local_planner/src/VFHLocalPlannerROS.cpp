#include <vfh_local_planner/VFHLocalPlannerROS.hpp>
#include <pluginlib/class_list_macros.h>
#include <boost/bind.hpp>
#include <base_local_planner/goal_functions.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <utility>
#include <algorithm>
#include <vector>
#include <limits.h>

#define PI 3.14159265359

#define MAX_COST 254

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

	/*For now set the window size to 100x100*/
	windowSize = 33;

	/*Initialise alpha to 5 for now*/
	alpha = 5;
	unsigned h_size = (unsigned) (360.0/(alpha*1.0));

	/*Allocate space for the polar obstacle density*/
	h = new double[h_size];


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
	b = a/(sqrt(2.0)*(windowSize - 1.0)/2.0);


	/*Create the active map*/
	active_window = new unsigned char *[windowSize];
	for(unsigned i = 0; i < windowSize; ++i){
		active_window[i] = new unsigned char[windowSize];
	}

	/*Initialise smax to 18 for now*/
	smax = 5;

	/*Default value of 32 assigned to threshold (one eights of MAX_COST)*/
	threshold = 0.0;//128.0*128.0;//256.0*256.0*10.0/8.0;

}

VFHLocalPlannerROS::~VFHLocalPlannerROS(void){
	for(unsigned i = 0; i < windowSize; ++i){
		delete[] m[i];
		delete[] beta[i];
		delete[] active_window[i];
	}

	delete[] m;
	delete[] beta;
	delete[] h;
	delete[] active_window;
}

void VFHLocalPlannerROS::initialiseActiveWindow(void){
	for(unsigned i = 0; i < windowSize; ++i)
		for(unsigned j = 0; j < windowSize; ++j){
			active_window[i][j] = (unsigned char) MAX_COST;
	}
}

void VFHLocalPlannerROS::initialisePOD(void){
	unsigned limit = (unsigned) (360.0/(alpha*1.0));
	for(unsigned k = 0; k < limit; ++k){
		h[k] = 0.0;
	}
}

double VFHLocalPlannerROS::getRotation(const tf::Stamped<tf::Pose>& point){

    double x = point.getRotation().getX();
    double y = point.getRotation().getY();
    double z = point.getRotation().getZ();

    double angle = 0;

    double w = point.getRotation().getW();

	double scale = sqrt(x * x + y * y + z * z);
	x = x / scale;
	y = y / scale;
	z = z / scale;
	angle = acos(w) * 2.0f;    

	return angle*180.0/PI;
	// ROS_INFO("angle: %f",angle*180.0/PI);
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

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    //get the global plan in our frame
    if(!base_local_planner::transformGlobalPlan(*tf, globalPlan, *costmap_ros, costmap_ros->getGlobalFrameID(), transformed_plan)){
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }

     //we also want to clear the robot footprint from the costmap we're using
    costmap_ros->clearRobotFootprint();   

    /*Extract the target position from the global goal point*/
    tf::Stamped<tf::Pose> goal_point;
    tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
    //we assume the global goal is the last point in the global plan
    double goal_x = goal_point.getOrigin().getX();
    double goal_y = goal_point.getOrigin().getY();

    double yaw = tf::getYaw(goal_point.getRotation());

    double robot_target_angle = atan2(
    							goal_y - globalPose.getOrigin().getY(),
    							goal_x - globalPose.getOrigin().getX()
    							) * 180.0/PI;
    robot_target_angle += 90.0;

	/*Get a copy of the active window around the robot*/
	(*this).costmap_ros->getCostmapCopy(costmap);

	/*Initialise the polar obstacle density array*/
	this->initialisePOD();

	double cij = 0;
	double dij = 0;
	double centreOffset = (windowSize - 1.0) / 2.0;
	for(unsigned i = 0; i < windowSize; ++i)
		for(unsigned j = 0; j < windowSize; ++j){
			/*Compute the robot's x and y positions*/
			double xo = globalPose.getOrigin().getY();
			double yo = globalPose.getOrigin().getY();

			/*Get the points on the costmap*/
			double xi = xo + (i-centreOffset);//costmap.getOriginX()+i;
			double yj = yo + (j-centreOffset);//costmap.getOriginY()+j;
			
			/*Calculate the differences*/
			double xdiff = xi - xo;
			double ydiff = yj - yo;

			/*Get the cost of cell, taking into account whether
			 *the xi and yi values are greater than 0.0*/
			double wx = xi;
			double wy = yj;

			unsigned int mx = 0;
			unsigned int my = 0;

			bool success = costmap.worldToMap(wx,wy,mx,my);

			//if(success){
				cij = costmap.getCost(mx,my);//xi >= 0.0 && yj >= 0.0 ? costmap.getCost(xi,yj) : std::numeric_limits<double>::max();
			//}
			//else {
			//	ROS_INFO("Reached unsuccessful for wx: %f, wy: %f, mx: %u, my: %u", wx, wy, mx, my);
			//	cij = MAX_COST;
			//}
			/*compute the beta or direction values*/
			
			double betaAngle = atan2( 
								ydiff 
							,	xdiff
							) * 180.0/(PI);
			beta[i][j] = betaAngle < 0.0 ? betaAngle + 360.0 : betaAngle;


			/*Compute the Euclidian distance between the two co-ordinates*/
			dij = sqrt(xdiff*xdiff + ydiff*ydiff);
			/*Compute the magnitude values for each cell*/
			m[i][j] = cij*cij*(a - b*dij);

			/*Calculate the index to address the polar obstacle
			 *density array*/
			unsigned k = (unsigned) (beta[i][j] / ((double) alpha) );
			/*Compute the polar obstacle density of each sector*/
			h[k] += m[i][j];
	}

	unsigned limit = (unsigned) (360.0/(alpha*1.0));
	for(unsigned k = 0; k < limit; ++k){
		ROS_INFO("h[%d]: %f", k, h[k]);
	}	

	/*Find the candidate valleys*/
	unsigned sectorSize = (unsigned) (360.0/(alpha*1.0));
	
	std::vector<std::pair<unsigned,unsigned> > candidateValleys;
	std::vector<std::pair<unsigned,unsigned> > wideValleys, narrowValleys;

	unsigned kstart = 0;
	unsigned kend = 0;

	for(unsigned k = 0; k < sectorSize; ++k){
		/*If the sector has a POD value less
		 *than or equal to the threshold then see if the
		 *width of the valley is equal to smax. If 
		 *that is the case then store
		 *the pair in the candidate valleys. If the
		 *width of the valley does not fit the wide
		 *valley criteria then store them in the candidate
		 *valleys anyways and continue on with the search.*/		
		if(h[k] <= threshold){
			if( (kend - kstart) < smax ){
				++kend;
			}else {
				std::pair<unsigned, unsigned> p = std::make_pair(kstart, kend);
				candidateValleys.push_back(p);
				kstart = ++kend;
			}
		}else {
			if(kstart != kend){
				std::pair<unsigned, unsigned> p = std::make_pair(kstart,kend);
				candidateValleys.push_back(p);
			}
			kstart = ++kend;
		}
	}

	/*If no POD value below the threshold value exists
	 *then return false*/
	if(candidateValleys.size() == 0){
		return false;
	}

	/*Fill up the narrow and wide valley data structures*/
	for(unsigned i = 0; i < candidateValleys.size(); ++i){
		unsigned width = candidateValleys[i].second - candidateValleys[i].first;
		if(width >= smax){
			wideValleys.push_back(candidateValleys[i]);
		} else {
			narrowValleys.push_back(candidateValleys[i]);
		}
	}

	/*Initialise the command_vel for now to 0*/
	cmd_vel.linear.x = 0.0;
	cmd_vel.linear.y = 0.0;

	/*Check if any valid paths can be found
	 *from the candidate valleys*/
	// ROS_INFO("size: %d", wideValleys.size());
	if(wideValleys.size() != 0){
		unsigned ktemp = 0;
		double kn = 0;
		double kf = 0;
		unsigned sector = 0;
		double deg = 0.0;
		double min = 360.0;
		double direction = 0.0;


		for(unsigned i = 0; i < wideValleys.size(); ++i){
			ktemp = wideValleys[i].first;
			deg = (ktemp*360.0)/(sectorSize*1.0);
			if(fabs(deg-robot_target_angle) < min){
				min = fabs(deg-robot_target_angle);
				sector = i;
			}
		}

		//ROS_INFO("min: %f, deg: %f, robot_target_angle: %f", min, deg, robot_target_angle);		

		std::pair<unsigned,unsigned> targetSector = wideValleys[sector];
		kn = (targetSector.first*360.0)/(sectorSize*1.0);
		kf = (targetSector.second*360.0)/(sectorSize*1.0);

		// ROS_INFO("kn: %f, targetSector.first: %d, kf: %f, targetSector.second: %d", kn, targetSector.first , kf, targetSector.second);

		direction = (kn+kf)/2.0;
		double angle = this->getRotation(globalPose);

		cmd_vel.angular.z = direction - angle;
		cmd_vel.linear.x = 0.5;
		// ROS_INFO("angle: %f, dir:%f, ang.z: %f, robot_target_angle: %f", angle, direction, cmd_vel.angular.z, robot_target_angle);
		return true;

	} else if (narrowValleys.size() != 0){
		/*If there are no wideValleys then resort
		 *to the narrowValleys*/

		/*return false for now*/
		return false;
	} else {
		/*If no candidate values exist then return false*/
		return false;
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