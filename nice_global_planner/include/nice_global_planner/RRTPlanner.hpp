#ifndef RRT_PLANNER_HPP
#define RRT_PLANNER_HPP
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include "KDTree.hpp"

#include <boost/thread/mutex.hpp>

#include <vector>

#include <utils/Timer.hpp>

#include <nice_global_planner/Sampler.hpp>

namespace nice_global_planner{

class RRTPlanner
{
public:
	/**
	 * @brief Default constructor that will initialise the tree
	 * @param start Starting position of the robot
	 * @param goal Goal position to be reached by the planner
	 */
	RRTPlanner(
		const geometry_msgs::Pose& start, 
		const geometry_msgs::Pose& goal,
		utils::Timer *timer,
		Sampler *sampler
	);

	~RRTPlanner(void);

	/**
	 * @brief Updates the goal position of the RRTPlanner
	 * @param goal New goal position
	 */
	 void updateGoal(const geometry_msgs::Pose& goal);

	 /*
	  * @brief Updates the start position of the robot
	  * @param start New start position of the robot*/
	 void updateStart(const geometry_msgs::Pose& start);

	 bool makePlan(
	 	const geometry_msgs::Pose& start, 
	 	const geometry_msgs::Pose& goal, 
	 	std::vector<geometry_msgs::PoseStamped> &plan
	 );

private:

	geometry_msgs::Point choosePoint(const geometry_msgs::Pose& goal);
	
	/**
	 * @brief Converts a geometry point to vector format
	 * @param data Reference to a vector that has been resized
	 *  to size of 2*/
	void pointToKDNodeVector2D(
		const geometry_msgs::Point& point,
		std::vector<double>& data
	);

	void vector2DToPoint(
		const std::vector<double> &data,
		geometry_msgs::Point& point
	);

	bool point2DInFreeConfig(const std::vector<double>& point);

	std::vector<double> extend(std::vector<double> q, std::vector<double> p);

	double distance2D(const std::vector<double> &p, const std::vector<double> &q);

	bool goalReached(const std::vector<double>& qnew, const std::vector<double>& goalPoint);

	void vector2DToPoseStamped(
		const std::vector<double>& data,
		geometry_msgs::PoseStamped poseStamped,
		ros::Time time
	);

	geometry_msgs::Pose start;
	geometry_msgs::Pose goal;

	boost::mutex goalLock;
	boost::mutex startLock; 

	KDTree<double> *kdtree;

	double goalTolerance;

	utils::Timer *timer;

	Sampler *sampler;

};
}

#endif