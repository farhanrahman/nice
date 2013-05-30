#include <ros/ros.h>
#include <otl.h>
#include <otl_oesgp.h>

#include <iostream>
#include <cmath>

#include <nice_predictor/NicePredictor.hpp>



int main(int argc, char * argv[]){

    ros::init(argc, argv, "nice_predictor");

    ros::NodeHandle n;

    nice_predictor::NicePredictor np(n);

    np.nodeLoop();

    return 0;

}