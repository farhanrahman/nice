#ifndef INITIALISER_H
#define INITIALISER_H

#include <ros/ros.h>
#include <tf_setup/WorldObject.h>
#include <tf_setup/TransformManager.h>
#include <string>
#include <sstream>

template<typename T>
float toFloat(T t){
	std::stringstream ss;
	float ret;
	ss << t;
	ss >> ret;
	return ret;
}

namespace tf_setup{



class Initialiser{
public:
	static void initTransformManager(tf_setup::TransformManager &tm, const ros::NodeHandle& n){
		if(n.hasParam("world_objects")){
			XmlRpc::XmlRpcValue list;
			n.getParam("world_objects", list);
			ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

			for (int32_t i = 0; i < list.size(); ++i) {
				ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
				std::string val = static_cast<std::string>(list[i]).c_str();
				if(n.hasParam(val)) {
					XmlRpc::XmlRpcValue input;
					std::string frame;
					std::string parentFrame;
		
					n.getParam(val+"/frame_id", frame);
					
					n.getParam(val+"/parent_frame_id", parentFrame);
					
					tf_setup::WorldObject obj = tf_setup::WorldObject::withFrames(parentFrame, frame);
					
					n.getParam(val+"/rw", input);
					obj.setRW(toFloat(input));

					n.getParam(val+"/rx", input);
					obj.setRX(toFloat(input));

					n.getParam(val+"/ry", input);
					obj.setRY(toFloat(input));

					n.getParam(val+"/rz", input);
					obj.setRZ(toFloat(input));

					n.getParam(val+"/x", input);
					obj.setX(toFloat(input));

					n.getParam(val+"/y", input);
					obj.setY(toFloat(input));

					n.getParam(val+"/z", input);
					obj.setZ(toFloat(input));
		
					tm.addObject(obj);	
				} else {
					ROS_WARN(std::string("config file not set properly, " + val + " is missing").c_str());
				}	
			}

		} else {
			ROS_WARN("world_objects parameter has not been set");
		}
	}

};
}
#endif
