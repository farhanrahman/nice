#ifndef DETECTION_MSG_HPP
#define DETECTION_MSG_HPP
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>

namespace nice_core
{
class DetectionMessage
{
public:
	DetectionMessage(
			geometry_msgs::Vector3 centroid, 
			float height, 
			double confidence, 
			float distance, 
			bool occluded,
			std_msgs::Header header,
			geometry_msgs::PointStamped point
	) : 
		centroid(centroid), 
		height(height), 
		confidence(confidence), 
		distance(distance), 
		occluded(occluded),
		header(header),
		point(point)
	{};

	DetectionMessage(){}

	DetectionMessage& operator=(const DetectionMessage& dmsg){
		this->centroid = dmsg.centroid;
		this->height = dmsg.height;
		this->distance = dmsg.distance;
		this->occluded = dmsg.occluded;
		this->header = dmsg.header;
		this->point = dmsg.point;
		return *this;
	}

	~DetectionMessage(){}

	geometry_msgs::Vector3 centroid;
	float height;
	double confidence; 
	float distance; 
	bool occluded;
	std_msgs::Header header;
	geometry_msgs::PointStamped point;
};

}


#endif