##ifndef ROS_TIMER
#define ROS_TIMER 

#include "Timer.hpp"
#include <ros/ros.h>

namespace utils{

class RosTimer : public Timer
{
public:
	double getTime(void) {
		return ros::Time::now();
	}
};

}

#endif