#ifndef NICE_PREDICTOR_HPP
#define NICE_PREDICTOR_HPP

#include <string>
#include <vector>
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>

#include <costmap_update/tfMessageParser.hpp>
#include <costmap_update/ParserMessage.hpp>

#include <otl.h>
#include <otl_oesgp.h>

#include <boost/thread/mutex.hpp>
#include <queue>

namespace nice_predictor
{
class NicePredictor
{
public:
	NicePredictor(
		ros::NodeHandle& n,
		unsigned rate = 10
	);
	~NicePredictor();

	/**
	 * @brief Main ros node loop.
	 */
	void nodeLoop(void);

private:
	/**
	 * @brief Callback function when openni_tracker publishes a
	 *  transform or tf message.
	 * @param msg message published by openni_tracker
	 */
	void trackerCallback(const tf::tfMessage::ConstPtr& msg);

	ros::NodeHandle* nPtr;

	ros::Subscriber trackerListener;

	OTL::OESGP mt;
	OTL::OESGP mp;

	boost::mutex tfMutex;
	std::queue<geometry_msgs::TransformStamped> tfQueue;

	boost::mutex fileMutex;

	ros::Publisher predictionPublisher;

	unsigned rate;

	tf::TransformListener tfl;	

};

}

#endif