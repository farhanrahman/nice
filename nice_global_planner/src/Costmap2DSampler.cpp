#include <nice_global_planner/Costmap2DSampler.hpp>

namespace nice_global_planner{

Costmap2DSampler::Costmap2DSampler(costmap_2d::Costmap2DROS *costmap_2d_ros, double thresh)
	: costmap_2d_ros(costmap_2d_ros)
{
	(*this).costmap_2d_ros->getCostmapCopy(costmap);
	(*this).thresh = thresh;
}

Costmap2DSampler::~Costmap2DSampler(void){

}

geometry_msgs::Point Costmap2DSampler::uniformSample(const geometry_msgs::Pose& goal, const geometry_msgs::Pose& start){
	unsigned sizeX = costmap.getSizeInCellsX();
	unsigned sizeY = costmap.getSizeInCellsY();
	unsigned randIndexX = rng.uniformReal(0, sizeX - 1);
	unsigned randIndexY = rng.uniformReal(0, sizeY - 1);
	double worldX, worldY;

	costmap.mapToWorld(randIndexX, randIndexY, worldX, worldY);

	geometry_msgs::Point ret;
	ret.x = worldX;
	ret.y = worldY;
	ret.z = 0.0;

	return ret;
}

geometry_msgs::Point Costmap2DSampler::gaussianSample(const geometry_msgs::Pose& goal, const geometry_msgs::Pose& start){
	unsigned mgX, mgY;
	unsigned msX, msY;

	unsigned sizeX = costmap.getSizeInCellsX();
	unsigned sizeY = costmap.getSizeInCellsY();

	costmap.worldToMap(goal.position.x, goal.position.y, mgX, mgY);
	costmap.worldToMap(start.position.x, start.position.y, msX, msY);

	double midPointX = ( mgX + msX ) / 2.0;
	double midPointY = ( mgY + msY ) / 2.0;

	double stdDevX = (double) midPointX - mgX;
	double stdDevY = (double) midPointY - mgY;

	unsigned gausX = (unsigned) rng.gaussian(midPointX, abs(stdDevX));
	unsigned gausY = (unsigned) rng.gaussian(midPointY, abs(stdDevY));

	if(gausX > sizeX)
		gausX = sizeX;

	if(gausY > sizeY)
		gausY = sizeY;

	double worldX, worldY;

	costmap.mapToWorld(gausX, gausY, worldX, worldY);

	geometry_msgs::Point ret;
	ret.x = worldX;
	ret.y = worldY;
	ret.z = 0.0;

	return ret;
}

geometry_msgs::Point Costmap2DSampler::samplePoint(const geometry_msgs::Pose& goal, const geometry_msgs::Pose& start){
	return this->gaussianSample(goal, start);
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
	unsigned char ucost = (unsigned char) cost;
	if(cost < 0) {
		return false;
	} else if(ucost == costmap_2d::LETHAL_OBSTACLE || ucost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || ucost == costmap_2d::NO_INFORMATION){
      return false;
    }		
	return true;
}

}