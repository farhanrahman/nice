#ifndef FAKE_SAMPLER_HPP
#define FAKE_SAMPLER_HPP 

#include "Sampler.hpp"
#include <stdlib.h>
#include <math.h>
#include <time.h>

namespace nice_global_planner{

class FakeSampler : public Sampler
{
public:
	FakeSampler(unsigned size = 1000)
	: size(size) {
		costmap = new double *[size];
		for(unsigned i = 0; i < size; ++i){
			costmap[i] = new double[size];
		}
		srand(time(NULL));
	}

	~FakeSampler(void){
		for(unsigned i = 0; i < size; ++i){
			delete[] costmap[i];
		}

		delete[] costmap;
	}

	geometry_msgs::Point samplePoint(const geometry_msgs::Pose& goal, const geometry_msgs::Pose& start){
		geometry_msgs::Point ret;
		ret.x = rand() % size;
		ret.y = rand() % size;
		return ret;
	}

	void update(void) {
	}

	std::string getGlobalFrameID(void) {
		std::string ret = "fakesampler";
		return ret;
	}

	bool point2DInFreeConfig(const std::vector<double> &point, double yaw = 0){
		return true;
	}

	void initialise(CostCalculatorDelegate *costCalculatorDelegate) {

	}

private:
	double **costmap;
	unsigned size;
};

}

#endif