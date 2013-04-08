#include <nice_global_planner/RRTPlanner.hpp>
#include <math.h>

namespace nice_global_planner{

RRTPlanner::RRTPlanner(
	const geometry_msgs::Pose& start, 
	const geometry_msgs::Pose& goal,
	utils::Timer *timer,
	Sampler *sampler
	) : start(start), 
		goal(goal),
		timer(timer),
		sampler(sampler)
{
	this->kdtree = new KDTree<double>(2);
	this->goalTolerance = 0.5;
}

RRTPlanner::~RRTPlanner(void){
	delete kdtree;
}

void RRTPlanner::updateGoal(const geometry_msgs::Pose& goal){
	boost::mutex::scoped_lock(this->goalLock);
	this->goal = goal;
}

void RRTPlanner::updateStart(const geometry_msgs::Pose& start){
	boost::mutex::scoped_lock(this->startLock);
	this->start = start;
}

geometry_msgs::Point RRTPlanner::choosePoint(const geometry_msgs::Pose& goal){
	return sampler->samplePoint(goal);
}

double RRTPlanner::distance2D(const std::vector<double> &p, const std::vector<double> &q){
	double a = p[0] - q[0];
	double b = p[1] - q[1];
	return sqrt(a*a + b*b);
}

bool RRTPlanner::goalReached(const std::vector<double>& qnew, const std::vector<double>& goalPoint){
	return distance2D(qnew, goalPoint) < goalTolerance;
}

bool RRTPlanner::point2DInFreeConfig(const std::vector<double>& point){
	return true;
}

std::vector<double> RRTPlanner::extend(std::vector<double> q, std::vector<double> p){
	return p;
}

void RRTPlanner::pointToKDNodeVector2D(
	const geometry_msgs::Point& point,
	std::vector<double>& data
)
{
	data[0] = (double) point.x;
	data[1] = (double) point.y;
}

void RRTPlanner::vector2DToPoint(
	const std::vector<double> &data,
	geometry_msgs::Point& point
){
	point.x = data[0];
	point.y = data[1];
}

void RRTPlanner::vector2DToPoseStamped(
	const std::vector<double>& data,
	geometry_msgs::PoseStamped& poseStamped,
	ros::Time time
){
	poseStamped.header.stamp = time;
	poseStamped.header.frame_id = sampler->getGlobalFrameID();
	poseStamped.pose.position.x = data[0];
	poseStamped.pose.position.y = data[1];
	poseStamped.pose.position.z = 0.0;
	poseStamped.pose.orientation.x = 0.0;
	poseStamped.pose.orientation.y = 0.0;
	poseStamped.pose.orientation.z = 0.0;
	poseStamped.pose.orientation.w = 1.0;
}


bool RRTPlanner::makePlan(
 	const geometry_msgs::Pose& start, 
 	const geometry_msgs::Pose& goal, 
 	std::vector<geometry_msgs::PoseStamped> &plan
){
 	std::vector<double> pdata;
 	std::vector<double> qnew;
 	std::vector<double> gdata;
 	std::vector<double> sdata;

 	KDNode<double>* q = NULL;
 	KDNode<double>* root = NULL;
 	geometry_msgs::Point p;

 	double t = timer->getTime();

 	pdata.resize(2);
 	qnew.resize(2);
 	gdata.resize(2);
 	sdata.resize(2);

 	this->updateStart(start);
 	this->updateGoal(goal);
 	(*this).sampler->update();

	pointToKDNodeVector2D(goal.position,gdata);
	pointToKDNodeVector2D(start.position,sdata);

	kdtree->insert(sdata);
 	while(!goalReached(qnew,gdata)){
	 	p = this->choosePoint(this->goal);
	 	pointToKDNodeVector2D(p,pdata);
	 	q = kdtree->nearestNeighbour(pdata);
	 	qnew = extend(q->data, pdata);
	 	if(point2DInFreeConfig(qnew)){
	 		(*this).kdtree->insert(qnew);
	 	}
 	}

 	q = kdtree->findNode(qnew);
 	root = kdtree->findNode(sdata);

 	while (!q->equals(root)){
	 	geometry_msgs::PoseStamped pStamped;
	 	vector2DToPoseStamped(q->data, pStamped, (ros::Time) t);	 	
 		plan.push_back(pStamped);
 		q = q->parent;
 	}

 	geometry_msgs::PoseStamped pStamped;
 	vector2DToPoseStamped(root->data, pStamped, (ros::Time) t);	
 	plan.push_back(pStamped);

 	return plan.size() != 0;

}


}