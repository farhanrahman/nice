#ifndef ROS_CONVERTER_HPP
#define ROS_CONVERTER_HPP 
#include "IStamper.hpp"
#include <ros/ros.h>

namespace utils{

class RosStamper : public IStamper
{
public:

	virtual void updateTime(void) {
		this->t = ros::Time::now();
	}

	virtual void setStamp (
		geometry_msgs::PoseStamped& poseStamped
	) 
	{
		poseStamped.header.stamp = t;	
	}

private:
	ros::Time t;
};

}

#endif