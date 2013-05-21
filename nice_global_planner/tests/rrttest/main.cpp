#include <nice_global_planner/RRTPlanner.hpp>
#include <nice_global_planner/FakeSampler.hpp>
#include <utils/IStamper.hpp>
#include <utils/DefaultStamper.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <iostream>

int main(int argc, char const *argv[])
{

	unsigned size = 1000;

	std::vector<double> testVec;
	testVec.push_back(2.0);
	testVec.push_back(3.0);

	nice_global_planner::Sampler *sampler = new nice_global_planner::FakeSampler(size);

	// utils::Timer *timer = new utils::DefaultTimer();

	utils::IStamper *stamper = new utils::DefaultStamper();

	geometry_msgs::Pose start;
	geometry_msgs::Pose goal;

	start.position.x = rand()%size;
	start.position.y = rand()%size;

	goal.position.x = rand()%size;
	goal.position.y = rand()%size;

	nice_global_planner::RRTPlanner rrtPlanner(start, goal, stamper, sampler);

	std::vector<geometry_msgs::PoseStamped> plan;

	std::cout << "start.x: " << start.position.x << ",";
	std::cout << "start.y: " << start.position.y << std::endl;

	std::cout << "goal.x: " << goal.position.x << ",";
	std::cout << "goal.y: " << goal.position.y << std::endl;

	std::cout << std::endl;
	if(rrtPlanner.makePlan(start, goal, plan)){
		for(unsigned i = 0; i < plan.size(); ++i){
			std::cout << plan[i].pose.position.x << "," 
					  << plan[i].pose.position.y << 
			std::endl;
		}
	}



	delete sampler;
	delete stamper;

	return 0;
}
