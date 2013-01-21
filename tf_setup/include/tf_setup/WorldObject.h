#ifndef WORLD_OBJECT_H
#define WORLD_OBJECT_H

#include <tf/Transform.h>
#include <string>

namespace tf_setup{

class WorldObject{
public:
	/*Default constructor*/
	WorldObject(float x = 0.0, float y = 0.0, float z = 0.0,
			float rx = 0.0, float ry = 0.0, float rz = 0.0, float theta = 0.0,
			std::string parentFrame = "world", std::string frame = "base_link") :
		x(x), y(y), z(z), rx(rx), ry(ry), rz(rz), theta(theta), parentFrame, frame {};

	~WorldObject(){};

	tf::Quaternion getRotation(void){
		return tf::Quaternion(this->rx, this->ry, this->rz, this->theta);
	}

	tf::Vector3 getPosition(void){
		return tf::Vector3(this->x, this->y, this->z);
	}

	tf::Transform getPose(void){
		return tf::Transform(this->getRotation(), this->getPosition);
	}

	/*Getter functions*/

	float getX(void){
		return this->x;
	}

	float getY(void){
		return this->y;
	}

	float getZ(void){
		return this->z;
	}
	
	float getRX(void){
		return this->rx;
	}

	float getRY(void){
		return this->ry;
	}

	float getRZ(void){
		return this->rz;
	}

	std::string getFrame(void){
		return this->frame;
	}

	std::string getParentFrame(void){
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

	void setTheta(float theta){
		this->theta = theta;
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
	float theta;

	std::string frame;
	std::string parentFrame;
};
}
#endif
