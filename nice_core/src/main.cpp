#include <nice_core/NiceCore.hpp>
#include <ros/ros.h>

int main(int argc, char * argv[]){
	ros::init(argc, argv, "nice_core");

	ros::NodeHandle n;

	nice_core::NiceCore nc(n);

	nc.nodeLoop();

	return 0;
}