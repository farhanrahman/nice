#ifndef UPDATER_HPP
#define UPDATER_HPP
#include <ros/ros.h>
#include <string>
#include <vector>
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>

#define BUFFER_SIZE 10
#define MAX_COST_VALUE 254

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
		sub = n.subscribe("tf", BUFFER_SIZE, &Updater::updateCostMap, &(*this));
		publisher = n.advertise<sensor_msgs::PointCloud>("point_cloud_topic",1000);
	}

	~Updater(void){}

	void setTrackLimit(unsigned trackLimit){
		this->trackLimit = trackLimit;
	}

	void updateCostMap(const tf::tfMessage::ConstPtr& msg){
		std::stringstream message;
		std::string str_msg;

		sensor_msgs::PointCloud pc;
		pc.header.stamp = ros::Time::now();
		pc.header.frame_id = "openni_depth_frame";


		/*Points for pc.points*/
		std::vector<geometry_msgs::Point32> points;
		/*Channel for pc.channel*/
		std::vector<sensor_msgs::ChannelFloat32> channel;
		
		/*Only one channel for the number of points to be stored*/
		sensor_msgs::ChannelFloat32 distance;
		/*The channel is going to contain distance data for each point*/
		distance.name = "distance";
	
		/*Store the points into the point cloud*/
		for (unsigned i = 0; i < (*msg).transforms.size(); ++i){
			geometry_msgs::Transform transform = (*msg).transforms.at(i).transform;
			geometry_msgs::Point32 point;

			/*For each point in the message. Copy the x, y and z
			 *components accross to the x,y and z co-ordinates
			 *of the points.*/
			point.x = transform.translation.x;
			point.y = transform.translation.y;
			point.z = transform.translation.z;

			/*Copy the depth value recorded from the sensor*/
			std_msgs::Float32 d;
			d.data = point.x;

			/*Update the distance array of the channel*/
			distance.values.push_back(d.data);

			/*Update the points array*/
			points.push_back(point);

		}

		/*Update the channel with the distance data*/
		channel.push_back(distance);

		/*Update the points for the point cloud to be published*/
		pc.points = points;

		/*Update the point cloud's channels data*/
		pc.channels = channel;

		/*Publish the point cloud over the topic for the cost map*/

		publisher.publish(pc);
	}

private:
	int trackLimit;
	ros::Publisher publisher;
	ros::Subscriber sub;
};

#endif