#include <ros/ros.h>
#include <tf_setup/TransformManager.h>
#include <tf_setup/Initialiser.h>

int main(int argc, char * argv[]){

	ros::init(argc, argv, "tf_setup");	

	/*Create a private NodeHandle to access parameters in the node's namespace*/
	ros::NodeHandle n("~"); 

	tf_setup::TransformManager tm;
	
	tf_setup::Initialiser::initTransformManager(tm, n);

	ros::Rate r(10); //10 Hz

	//tf_setup::WorldObject base_footprint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, "odom", "base_footprint");
	//tf_setup::WorldObject base_link(0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0, "base_footprint", "base_link");
	//tf_setup::WorldObject base_laser_link(0.2, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0, "base_link", "base_laser_link");

	//tm.addObject(base_footprint);
	//tm.addObject(base_link);
	//tm.addObject(base_laser_link);

	while(ros::ok()){
		//Broadcast the user objects stored in the world
		tm.broadcast();
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
