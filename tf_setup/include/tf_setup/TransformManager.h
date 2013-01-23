#ifndef TRANSFORM_MANAGER_H
#define TRANSFORM_MANAGER_H

#include <vector>
#include <tf_setup/WorldObject.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <sstream>

namespace tf_setup{
class TransformManager{
public:
	TransformManager(){}

	TransformManager(tf_setup::WorldObject obj){
		this->objects.push_back(obj);
	}

	TransformManager(std::vector<tf_setup::WorldObject> objects) : objects(objects) {};

	~TransformManager(){};

	void addObject(tf_setup::WorldObject obj){
		objects.push_back(obj);
	}

	void broadcast(void){
		std::vector<tf_setup::WorldObject>::iterator it;
		ros::Time t = ros::Time::now();
		for(it = objects.begin(); it != objects.end(); ++it){
			std::cout << (*it) << std::endl;
			broadCaster.sendTransform(tf::StampedTransform(
							(*it).getPose(), 
							t, 
							(*it).getParentFrame(), 
							(*it).getFrame()
							)
						);
		}
	}

private:
	std::vector<tf_setup::WorldObject> objects;
	tf::TransformBroadcaster broadCaster;
};
}

#endif
