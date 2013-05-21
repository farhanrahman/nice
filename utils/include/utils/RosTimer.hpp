#ifndef ROS_TIMER
#define ROS_TIMER 

#include <ros/ros.h>

namespace utils{

class RosTimer
{
public:
	ros::Time getTime(void) {
		return ros::Time::now();
	}
};

}

#endif