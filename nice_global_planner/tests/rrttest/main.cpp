#include <nice_global_planner/RRTPlanner.hpp>
#include <nice_global_planner/FakeSampler.hpp>
#include <utils/Timer.hpp>
#include <utils/DefaultTimer.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

int main(int argc, char const *argv[])
{

	unsigned size = 1000;

	nice_global_planner::Sampler *sampler = new nice_global_planner::FakeSampler(size);

	utils::Timer *timer = new utils::DefaultTimer();

	geometry_msgs::Pose start;
	geometry_msgs::Pose goal;

	start.position.x = rand()%size;
	start.position.y = rand()%size;

	goal.position.x = rand()%size;
	goal.position.y = rand()%size;

	nice_global_planner::RRTPlanner rrtPlanner(start, goal, timer, sampler);

	std::vector<geometry_msgs::PoseStamped> plan;

	rrtPlanner.makePlan(start, goal, plan);

	delete sampler;
	delete timer;

	return 0;
}
