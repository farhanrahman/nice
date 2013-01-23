#ifndef WORLD_OBJECT_H
#define WORLD_OBJECT_H

#include <tf/LinearMath/Transform.h>
#include <iostream>
#include <sstream>
#include <string>

namespace tf_setup{

class WorldObject{
public:
	static WorldObject withFrames(std::string parentFrame, std::string frame) {
		return WorldObject(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, parentFrame, frame);
	}

	/*Default constructor*/
	WorldObject(float x = 0.0, float y = 0.0, float z = 0.0,
			float rx = 0.0, float ry = 0.0, float rz = 0.0, float rw = 0.0,
			std::string parentFrame = "world", std::string frame = "base_link") :
		x(x), y(y), z(z), rx(rx), ry(ry), rz(rz), rw(rw), parentFrame(parentFrame), frame(frame) {
		if(this->rw == 0.0){
			this->rw = 1.0;
		}
	};

	~WorldObject(){};

	tf::Quaternion getRotation(void){
		return tf::Quaternion(this->rx, this->ry, this->rz, this->rw);
	}

	tf::Vector3 getPosition(void){
		return tf::Vector3(this->x, this->y, this->z);
	}

	tf::Transform getPose(void){
		return tf::Transform(this->getRotation(), this->getPosition());
	}

	/*Getter functions*/

	float getX(void) const{
		return this->x;
	}

	float getY(void) const{
		return this->y;
	}

	float getZ(void) const{
		return this->z;
	}
	
	float getRX(void) const{
		return this->rx;
	}

	float getRY(void) const{
		return this->ry;
	}

	float getRZ(void) const{
		return this->rz;
	}

	float getRW(void) const{
		return this->rw;
	}

	std::string getFrame(void) const{
		return this->frame;
	}

	std::string getParentFrame(void) const{
		return this->parentFrame;
	}

	/*Setter functions*/

	void setX(float x){
		this->x = x;
	}

	void setY(float y){
		this->y = y;
	}

	void setZ(float z){
		this->z = z;
	}

	void setRX(float rx){
		this->rx = rx;
	}

	void setRY(float ry){
		this->ry = ry;
	}

	void setRZ(float rz){
		this->rz = rz;
	}

	void setRW(float rw){
		this->rw = rw;
	}

private:
/*Three dimensional vector representation of object*/
	float x;
	float y;
	float z;

/*Rotation representation (rx,ry,rz) representing axis
 *and theta representing the amount of rotation about the axis*/
	float rx;
	float ry;
	float rz;
	float rw;

	std::string frame;
	std::string parentFrame;
};
}

std::ostream& operator<<(std::ostream& oss, const tf_setup::WorldObject& o){
	oss << "position: [" << o.getX() << ", " << o.getY() << ", " << o.getZ()
		<< "]" << " orientation: [" << o.getRX() << ", " << o.getRZ() << ", " 
		<< o.getRY() << ", " << o.getRW() << "] " << std::endl;
	return oss;
}

std::stringstream& operator<<(std::stringstream& ss, const tf_setup::WorldObject& o){
	ss << "position: [" << o.getX() << ", " << o.getY() << ", " << o.getZ()
		<< "]" << " orientation: [" << o.getRX() << ", " << o.getRZ() << ", " 
		<< o.getRY() << ", " << o.getRW() << "] " << std::endl;
	return ss;
}

#endif
