#ifndef COSTMAP_2D_SAMPLER
#define COSTMAP_2D_SAMPLER 

#include "Sampler.hpp"

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <geometry_msgs/Point.h>

#include <boost/thread/mutex.hpp>

#include <utils/RandomNumberGenerator.hpp>

#define THRESHOLD 100.0

namespace nice_global_planner{

class Costmap2DSampler : public Sampler
{
public:
	Costmap2DSampler(costmap_2d::Costmap2DROS *costmap_2d_ros, double thresh = THRESHOLD);
	~Costmap2DSampler(void);

	geometry_msgs::Point samplePoint(const geometry_msgs::Pose& goal, const geometry_msgs::Pose& start);

	void update(void);

	std::string getGlobalFrameID(void);

	bool point2DInFreeConfig(const std::vector<double> &point, double yaw = 0);

	void initialise(CostCalculatorDelegate *costCalculatorDelegate){
		this->costCalculatorDelegate = costCalculatorDelegate;
	}

private:
	geometry_msgs::Point uniformSample(const geometry_msgs::Pose& goal, const geometry_msgs::Pose& start);
	geometry_msgs::Point gaussianSample(const geometry_msgs::Pose& goal, const geometry_msgs::Pose& start);

	costmap_2d::Costmap2DROS *costmap_2d_ros;
	costmap_2d::Costmap2D costmap;

	boost::mutex costmapLock;

	CostCalculatorDelegate *costCalculatorDelegate;

	utils::RandomNumberGenerator rng;

	double thresh;

};

}

#endif