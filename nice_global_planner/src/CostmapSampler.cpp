#include <nice_global_planner/CostmapSampler.hpp>

namespace nice_global_planner{

Costmap2DSampler::Costmap2DSampler(costmap_2d::Costmap2DROS *costmap_2d_ros)
	: costmap_2d_ros(costmap_2d_ros)
{
	(*this).costmap_2d_ros->getCostmapCopy(costmap_2d);
}

Costmap2DSampler::~Costmap2DSampler(void){

}

geometry_msgs::Point Costmap2DSampler::samplePoint(const geometry_msgs::Pose& goal){
	return goal.position;
}

void Costmap2DSampler::update(void){
	boost::mutex::scoped_lock(costmapLock);
	(*this).costmap_2d_ros->getCostmapCopy(costmap_2d);
}

std::string Costmap2DSampler::getGlobalFrameID(void){
	return costmap_2d_ros->getGlobalFrameID();
}

}