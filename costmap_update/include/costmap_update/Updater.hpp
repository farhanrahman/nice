#ifndef UPDATER_HPP
#define UPDATER_HPP
#include <ros/ros.h>
#include <string>
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>

#define BUFFER_SIZE 10

class Updater
{
public:
	/*@param[numUsers] the number of people
	 * to track by default*/
	Updater(
			ros::NodeHandle& n,
			unsigned trackLimit = 10
		) : trackLimit(trackLimit) 
	{
		n.subscribe("tf", BUFFER_SIZE, &Updater::updateCostMap, &(*this));
	}

	~Updater(void){}

	void setTrackLimit(unsigned trackLimit){
		this->trackLimit = trackLimit;
	}

	void updateCostMap(const tf::tfMessage::ConstPtr& msg){
		std::stringstream message;
		std::string str_msg;

		for (unsigned i = 0; i < (*msg).transforms.size(); ++i){
			message << (*msg).transforms.at(i).child_frame_id << std::endl;
		}

		message >> str_msg;
		ROS_INFO(str_msg.c_str());
	}

private:
	int trackLimit;
	//std::vector<tf::StampedTransform> trackedData;
	//tf::TransformListener tfl;
};

#endif