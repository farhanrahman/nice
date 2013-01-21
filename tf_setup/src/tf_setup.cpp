#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_setup/WorldObject.h>

int main(int argc, char * argv[]){

	ros::init(argc, argv, "broadcaster");

	ros::NodeHandle n;
	
	tf::TransformBroadcaster broadCaster;

	ros::Rate r(10); //10 Hz

	while(ros::ok()){
		//Broadcast the transforms for the base_link and the base_link_laser

				


		r.spinOnce();
	}

	return 0;
}
