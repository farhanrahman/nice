#include <ros/ros.h>
#include <costmap_update/Updater.hpp>

int main(int argc, char *argv[])
{
	/* code */

	ros::init(argc, argv, "costmap_update");
	ros::NodeHandle nodeHandle;

	Updater costMapUpdater(nodeHandle);

	ros::spin();

	return 0;
}