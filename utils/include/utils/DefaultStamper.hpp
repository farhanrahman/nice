#ifndef DEFAULT_CONVERTER_HPP
#define DEFAULT_CONVERTER_HPP 
#include "IStamper.hpp"
#include <time.h>

namespace utils{

class DefaultStamper : public IStamper 
{
public:
	
	virtual void updateTime(void) {
		this->t = time(NULL);
	}

	virtual void setStamp (
		geometry_msgs::PoseStamped& poseStamped
	) 
	{
		poseStamped.header.stamp = (ros::Time) t;	
	}

private:
	double t;
};
	
}

#endif