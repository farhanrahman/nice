#include <nice_global_planner/NiceGlobalPlannerROS.hpp>
#include <pluginlib/class_list_macros.h>
#include <utils/RosStamper.hpp>
#include <sstream>
#include <string>
#include <algorithm>

//Declare the NiceGlobalPlannerROS as a nav_core::BaseGlobalPlanner class
PLUGINLIB_DECLARE_CLASS(nice_global_planner, NiceGlobalPlannerROS, nice_global_planner::NiceGlobalPlannerROS, nav_core::BaseGlobalPlanner); 

namespace nice_global_planner{

NiceGlobalPlannerROS::NiceGlobalPlannerROS(void){
	initialised = false;
}

NiceGlobalPlannerROS::~NiceGlobalPlannerROS(void) {
	delete sampler;
	delete planner;
	delete stamper;
}

double NiceGlobalPlannerROS::footprintCost(double x_i, double y_i, double theta_i){
	if(!initialised){
	  ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
	  return -1.0;
	}
	//if we have no footprint... do nothing
	if(footprint_spec_.size() < 3)
	  return -1.0;

	//build the oriented footprint
	double cos_th = cos(theta_i);
	double sin_th = sin(theta_i);
	std::vector<geometry_msgs::Point> oriented_footprint;
	for(unsigned int i = 0; i < footprint_spec_.size(); ++i){
	  geometry_msgs::Point new_pt;
	  new_pt.x = x_i + (footprint_spec_[i].x * cos_th - footprint_spec_[i].y * sin_th);
	  new_pt.y = y_i + (footprint_spec_[i].x * sin_th + footprint_spec_[i].y * cos_th);
	  oriented_footprint.push_back(new_pt);
	}

	geometry_msgs::Point robot_position;
	robot_position.x = x_i;
	robot_position.y = y_i;

	//check if the footprint is legal
	double footprint_cost = world_model_->footprintCost(robot_position, oriented_footprint, inscribed_radius_, circumscribed_radius_);
	return footprint_cost;
}

bool NiceGlobalPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
	const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
	if(!initialised){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

	ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    (*this).sampler->update();

    if(goal.header.frame_id != (*this).sampler->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          (*this).sampler->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;

    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start,start_tf);

    double useless_pitch, useless_roll, goal_yaw, start_yaw;
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

    //we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;

    double diff_x = goal_x - start_x;
    double diff_y = goal_y - start_y;
    double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

    double target_x = goal_x;
    double target_y = goal_y;
    double target_yaw = goal_yaw;

    bool done = false;
    double scale = 1.0;
    double dScale = 0.01;

    while(!done)
    {
      if(scale < 0)
      {
        target_x = start_x;
        target_y = start_y;
        target_yaw = start_yaw;
        ROS_WARN("The carrot planner could not find a valid plan for this goal");
        break;
      }
      target_x = start_x + scale * diff_x;
      target_y = start_y + scale * diff_y;
      target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);
      
      double footprint_cost = footprintCost(target_x, target_y, target_yaw);
      if(footprint_cost >= 0)
      {
          done = true;
      }
      scale -=dScale;
    }

    geometry_msgs::PoseStamped new_goal = goal;
    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(target_yaw);

    new_goal.pose.position.x = target_x;
    new_goal.pose.position.y = target_y;

    new_goal.pose.orientation.x = goal_quat.x();
    new_goal.pose.orientation.y = goal_quat.y();
    new_goal.pose.orientation.z = goal_quat.z();
    new_goal.pose.orientation.w = goal_quat.w();

	(*this).planner->updateGoal(new_goal.pose);
	(*this).planner->updateStart(start.pose);
	(*this).planner->refresh();
	(*this).planner->makePlan(start.pose, goal.pose, plan);

	std::reverse(plan.begin(), plan.end());

	for(unsigned i = 0; i < plan.size(); ++i){
		std::cout << plan[i].pose.position.x << "," 
				  << plan[i].pose.position.y << 
		std::endl;
	}

	return true;

}

void NiceGlobalPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	if(initialised){
		ROS_WARN("NiceGlobalPlannerROS already initialised");
		return;
	}

	geometry_msgs::Pose start;
	geometry_msgs::Pose goal;

	/*Copy over the pointer to the costmap_ros*/
	this->costmap_ros = costmap_ros;

	this->nodeName = name;

	this->sampler = new Costmap2DSampler(costmap_ros);

	(*this).sampler->initialise(this);

	this->stamper = new utils::RosStamper();

	this->planner = new RRTPlanner(start, goal, stamper, sampler);

	costmap_ros->getCostmapCopy(costmap);
	world_model_ = new base_local_planner::CostmapModel(costmap); 

	inscribed_radius_ = costmap_ros->getInscribedRadius();
	circumscribed_radius_ = costmap_ros->getCircumscribedRadius();
	footprint_spec_ = costmap_ros->getRobotFootprint();

	/*Flag the initialised parameter to true*/
	this->initialised = true;
}

}