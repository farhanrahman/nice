#ifndef COSTMAP_2D_SAMPLER
#define COSTMAP_2D_SAMPLER 

#include "Sampler.hpp"

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <geometry_msgs/Point.h>

#include <boost/thread/mutex.hpp>

namespace nice_global_planner{

class Costmap2DSampler : public Sampler
{
public:
	Costmap2DSampler(costmap_2d::Costmap2DROS *costmap_2d_ros);
	~Costmap2DSampler(void);

	geometry_msgs::Point samplePoint(const geometry_msgs::Pose& goal);

	void update(void);

	std::string getGlobalFrameID(void);

private:
	costmap_2d::Costmap2DROS *costmap_2d_ros;
	costmap_2d::Costmap2D costmap_2d;

	boost::mutex costmapLock;


};

}

#endif