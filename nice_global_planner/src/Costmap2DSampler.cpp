#include <nice_global_planner/Costmap2DSampler.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <ctime>

#define THRESHOLD 50

namespace nice_global_planner{

Costmap2DSampler::Costmap2DSampler(costmap_2d::Costmap2DROS *costmap_2d_ros)
	: costmap_2d_ros(costmap_2d_ros)
{
	(*this).costmap_2d_ros->getCostmapCopy(costmap);
	srand(time(NULL));
}

Costmap2DSampler::~Costmap2DSampler(void){

}

geometry_msgs::Point Costmap2DSampler::samplePoint(const geometry_msgs::Pose& goal){
	unsigned sizeX = costmap.getSizeInCellsX();
	unsigned sizeY = costmap.getSizeInCellsY();
	unsigned randIndexX = (rand() / (float) RAND_MAX) * (float) sizeX;
	unsigned randIndexY = (rand() / (float) RAND_MAX) * (float) sizeY;
	double worldX, worldY;

	costmap.mapToWorld(randIndexX, randIndexY, worldX, worldY);

	geometry_msgs::Point ret;
	ret.x = worldX;
	ret.y = worldY;
	ret.z = 0.0;

	return ret;
}

void Costmap2DSampler::update(void){
	boost::mutex::scoped_lock(costmapLock);
	(*this).costmap_2d_ros->getCostmapCopy(costmap);
}

std::string Costmap2DSampler::getGlobalFrameID(void){
	return costmap_2d_ros->getGlobalFrameID();
}

bool Costmap2DSampler::point2DInFreeConfig(const std::vector<double> &point, double yaw){
	double cost = costCalculatorDelegate->footprintCost(point[0], point[1], yaw);
	if(cost < 0) {
		return false;
	} else if (cost <= THRESHOLD){
		return true;
	}
	return true;
}

}