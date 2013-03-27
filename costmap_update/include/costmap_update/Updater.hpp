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
#include <math.h>
#include <costmap_update/tfMessageParser.hpp>

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
			try{
				tfparser::message m = tfMessageParser::parseFrame((std::string) (*msg).transforms.at(i).child_frame_id);
				if(m.part == "/torso"){
					generateBlob(points, (*msg).transforms.at(i).transform, distance);
					ROS_INFO("generating blob for torso");
				}
			} catch(const FrameFormatNotCorrect &e){
				ROS_WARN(e.what());
			}
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

	void generateBlob(
			std::vector<geometry_msgs::Point32> &points, 
			const geometry_msgs::Transform& transform,
			sensor_msgs::ChannelFloat32& distance,
			unsigned density = 1000 /*By Default there should be 1000 points in the point cloud*/
	){
			const float distanceFromCentre = 0.05; //80 cm
			/*Generate random seed*/
			srand((unsigned)time(0));

			for(unsigned i = 0; i < density; ++i){
				geometry_msgs::Point32 point;
				float r = (float)rand()/(float)RAND_MAX;
				/*For each point in the message. Copy the x, y and z
				 *components accross to the x,y and z co-ordinates
				 *of the points.*/

				int decision = rand() % 2;

				point.x = decision == 0 ? transform.translation.x + r*distanceFromCentre : transform.translation.x - r*distanceFromCentre;
				r = (float)rand()/(float)RAND_MAX;
				point.y = decision == 0 ? transform.translation.y + r*distanceFromCentre : transform.translation.y - r*distanceFromCentre;
				r = (float)rand()/(float)RAND_MAX;
				point.z = decision == 0 ? transform.translation.z + r*distanceFromCentre : transform.translation.z - r*distanceFromCentre;

				/*Copy the depth value recorded from the sensor*/
				std_msgs::Float32 d;
				d.data = point.x;

				/*Update the distance array of the channel*/
				distance.values.push_back(d.data);

				/*Update the points array*/
				points.push_back(point);
			}
	}

	int trackLimit;
	ros::Publisher publisher;
	ros::Subscriber sub;
};

#endif