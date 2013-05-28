#include <nice_core/NiceCore.hpp>
#include <ros/ros.h>

int main(int argc, char * argv[]){
	ros::init(argc, argv, "nice_core");

	nice_core::NiceCore nc;

	nc.nodeLoop();

	return 0;
}