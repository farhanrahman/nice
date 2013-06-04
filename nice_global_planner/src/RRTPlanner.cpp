#include <nice_global_planner/RRTPlanner.hpp>
#include <math.h>
#include <utils/Vector.hpp>
#include <boost/timer.hpp>

namespace nice_global_planner{

RRTPlanner::RRTPlanner(
	const geometry_msgs::Pose& start, 
	const geometry_msgs::Pose& goal,
	utils::IStamper *stamper,
	Sampler *sampler,
	double goalTolerance,
	double maxDistance,
	double planningTime
	) : start(start),
		goal(goal),
		stamper(stamper),
		sampler(sampler),
		planningTime(planningTime)
{
	this->kdtree = new KDTree<double>(2);
	this->goalTolerance = goalTolerance;
	this->maxDistance_ = maxDistance;
	this->maxDistanceSq_ = maxDistance_*maxDistance_;
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

geometry_msgs::Point RRTPlanner::choosePoint(const geometry_msgs::Pose& goal, const geometry_msgs::Pose& start){
	return sampler->samplePoint(goal,start);
}

double RRTPlanner::distance2D(const std::vector<double> &p, const std::vector<double> &q){
	double a = p[0] - q[0];
	double b = p[1] - q[1];
	return (a*a + b*b);
}

bool RRTPlanner::goalReached(const std::vector<double>& qnew, const std::vector<double>& goalPoint){
	return distance2D(qnew, goalPoint) < goalTolerance;
}

bool RRTPlanner::point2DInFreeConfig(const std::vector<double>& point, const std::vector<double>& goal){
	Vec2f end(goal[0], goal[1]);
	Vec2f start(point[0], point[1]);
	Vec2f endStart = end - start;

	double yaw = atan2(endStart.y(), endStart.x());

	return sampler->point2DInFreeConfig(point, yaw);
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
	geometry_msgs::PoseStamped& poseStamped
){
	// poseStamped.header.stamp = time;
	stamper->setStamp(poseStamped);
	poseStamped.header.frame_id = sampler->getGlobalFrameID();
	poseStamped.pose.position.x = data[0];
	poseStamped.pose.position.y = data[1];
	poseStamped.pose.position.z = 0.0;
	poseStamped.pose.orientation.x = 0.0;
	poseStamped.pose.orientation.y = 0.0;
	poseStamped.pose.orientation.z = 0.0;
	poseStamped.pose.orientation.w = 1.0;
}

void RRTPlanner::refresh(void){
	boost::mutex::scoped_lock(kdTreeLock); 
	delete kdtree;
	this->kdtree = new KDTree<double>(2);
}


std::vector<double> RRTPlanner::extend(const std::vector<double> &nearest, const std::vector<double> &target){
	double startX = nearest[0];
	double startY = nearest[1];

	double targetX = target[0];
	double targetY = target[1];

	Vec2f start(startX, startY);
	Vec2f end(targetX, targetY);

	double sqDistance = start.SqDistance(end);

	std::vector<double> ret;
	ret.resize(2);
	ret[0] = target[0];
	ret[1] = target[1];

	if(sqDistance > maxDistanceSq_){
		Vec2f direction = (end - start).Normalized();
		double alpha = maxDistance_ / sqrt(sqDistance);
		Vec2f point = start + direction*alpha;
		ret[0] = point.x();
		ret[1] = point.y();
	}

	return ret;
}

bool RRTPlanner::makePlan(
 	const geometry_msgs::Pose& start, 
 	const geometry_msgs::Pose& goal, 
 	std::vector<geometry_msgs::PoseStamped> &plan
){
 	std::vector<double> targetData;
 	std::vector<double> extended;
 	std::vector<double> gdata;
 	std::vector<double> sdata;

 	KDNode<double>* q = NULL;
 	KDNode<double>* nearestNode = NULL;
 	KDNode<double>* root = NULL;
 	geometry_msgs::Point target;

 	// double t = timer->getTime();
 	(*this).stamper->updateTime();

 	targetData.resize(2);
 	gdata.resize(2);
 	sdata.resize(2);
 	extended.resize(2);

 	this->updateStart(start);
 	this->updateGoal(goal);
 	(*this).sampler->update();

	pointToKDNodeVector2D(goal.position,gdata);
	pointToKDNodeVector2D(start.position,sdata);

	{
		boost::mutex::scoped_lock(kdTreeLock);
		kdtree->insert(sdata);

		boost::timer t;
	 	while( !goalReached(extended,gdata) ){
		 	target = this->choosePoint(this->goal, this->start);
		 	pointToKDNodeVector2D(target,targetData);
		 	nearestNode = kdtree->nearestNeighbour(targetData);
		 	extended = this->extend(nearestNode->data, targetData);
		 	if(point2DInFreeConfig(extended,gdata)){
		 		(*this).kdtree->insert(extended);
		 	}

		 	if(t.elapsed() > planningTime){
		 		return false;
		 	}
	 	}

	 	q = kdtree->findNode(extended);
	 	root = kdtree->findNode(sdata);

	 	while (!q->equals(root)){
		 	geometry_msgs::PoseStamped pStamped;
		 	vector2DToPoseStamped(q->data, pStamped);	 	
	 		plan.push_back(pStamped);
	 		q = q->parent;
	 	}

	 	geometry_msgs::PoseStamped pStamped;
	 	vector2DToPoseStamped(root->data, pStamped);	
	 	plan.push_back(pStamped);
	}

 	return true;
}


}