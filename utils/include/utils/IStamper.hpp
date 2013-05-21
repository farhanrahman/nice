#ifndef I_STAMPER_HPP
#define I_STAMPER_HPP 
#include <vector>
#include <geometry_msgs/PoseStamped.h>

namespace utils {

class IStamper
{
public:
	virtual void updateTime(void) = 0;

	virtual void setStamp (
		geometry_msgs::PoseStamped& poseStamped
	) = 0;

	virtual ~IStamper() {}
};

}

#endif