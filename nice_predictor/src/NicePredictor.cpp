#include <nice_predictor/NicePredictor.hpp>
#include <fstream>

namespace nice_predictor{

NicePredictor::NicePredictor(
	ros::NodeHandle& n,
	unsigned rate
) : rate(rate),
	nPtr(&n)
{

	trackerListener = n.subscribe(
			"tf",
			1000,
			&NicePredictor::trackerCallback,
			&(*this)
		);

    //Problem parameters
    //we want to predict a simple sine wave
    int input_dim = 3;
    int output_dim = 3;

    //Reservoir Parameters
    //you can change these to see how it affects the predictions
    int reservoir_size = 100;
    double input_weight = 1.0;
    double output_feedback_weight = 0.0;
    int activation_function = OTL::Reservoir::TANH;
    double leak_rate = 0.9;
    double connectivity = 0.1;
    double spectral_radius = 0.90;
    bool use_inputs_in_state = false;

    OTL::VectorXd kernel_parameters(2); //gaussian kernel parameters
    kernel_parameters << 1.0, 1.0; //l = 1.0, alpha = 1.0

    //SOGP parameters
    double noise = 0.01;
    double epsilon = 1e-3;
    int capacity = 200;

    int random_seed = 0;

    try{
        mp.init( input_dim, output_dim, reservoir_size,
                    input_weight, output_feedback_weight,
                    activation_function,
                    leak_rate,
                    connectivity, spectral_radius,
                    use_inputs_in_state,
                    kernel_parameters,
                    noise, epsilon, capacity, random_seed); 
        mt.init( input_dim, output_dim, reservoir_size,
                    input_weight, output_feedback_weight,
                    activation_function,
                    leak_rate,
                    connectivity, spectral_radius,
                    use_inputs_in_state,
                    kernel_parameters,
                    noise, epsilon, capacity, random_seed); 	
    } catch(OTL::OTLException &e){
    	e.showError();
    }

    predictionPublisher = n.advertise<geometry_msgs::TransformStamped>(
    				"nice_core/prediction",
    				1000
    			);

    mt.save("mt");

}

NicePredictor::~NicePredictor(){

}

void NicePredictor::nodeLoop(void){
	ros::Rate r((*this).rate);
    OTL::VectorXd prediction;
    OTL::VectorXd prediction_variance;

	while(ros::ok()){
		std::vector<geometry_msgs::TransformStamped> accum;
		{
			boost::mutex::scoped_lock(tfMutex);
			for(unsigned i = 0; i < tfQueue.size(); ++i){
				accum.push_back(tfQueue.front());
				tfQueue.pop();
			}
		}

		if(!accum.empty()){
			for(unsigned i = 0; i < accum.size(); ++i){
				geometry_msgs::TransformStamped msg = accum[i];
				OTL::VectorXd input(3);
				input(0) = msg.transform.translation.x;
				input(1) = msg.transform.translation.y;
				input(2) = msg.transform.translation.z;
				mp.update(input);
			}
		}

		OTL::VectorXd Rt(mp.getStateSize());
		mp.getState(Rt);

		try
		{
			boost::mutex::scoped_lock(fileMutex);
			mp.load("mt");
		} catch(OTL::OTLException &e){
			e.showError();
		}

		mp.setState(Rt);

		mp.predict(prediction, prediction_variance);

		geometry_msgs::TransformStamped pubMessage;

		pubMessage.header.frame_id = "openni_depth_frame";
		pubMessage.child_frame_id = "torso_1_prediction";
		pubMessage.header.stamp = ros::Time::now();
		pubMessage.transform.translation.x = prediction(0);
		pubMessage.transform.translation.y = prediction(1);
		pubMessage.transform.translation.z = prediction(2);	

		predictionPublisher.publish(pubMessage);

		ros::spinOnce();
		r.sleep();
	}
}

void NicePredictor::trackerCallback(const tf::tfMessage::ConstPtr& msg){

	for (unsigned i = 0; i < (*msg).transforms.size(); ++i){
		tfparser::message m = tfMessageParser::parseFrame((std::string) (*msg).transforms.at(i).child_frame_id);
		if ((m.part == "/torso") && (m.user == 1)){
			try{
				ros::Time past = ros::Time::now() - ros::Duration(2.0);
		        
				tf::StampedTransform transform;

				bool okay = tfl.waitForTransform(
								"torso_1", 
								"openni_depth_frame", 
								ros::Time::now() - ros::Duration(2.0), 
								ros::Duration(0.1)
							);
				if (okay){
					tfl.lookupTransform(
								"torso_1", 
								"openni_depth_frame", 
								past, 
								transform
							);

					{
						boost::mutex::scoped_lock(tfMutex);
						tfQueue.push((*msg).transforms.at(i));
					}

					OTL::VectorXd xtm2(3);
					xtm2(0) = transform.getOrigin().x();
					xtm2(1) = transform.getOrigin().y();
					xtm2(2) = transform.getOrigin().z();

					OTL::VectorXd xt(3);
					xt(0) = (*msg).transforms.at(i).transform.translation.x;
					xt(1) = (*msg).transforms.at(i).transform.translation.y;
					xt(2) = (*msg).transforms.at(i).transform.translation.z;
					
					mt.update(xtm2);
					mt.train(xt);
				}

			} catch(tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}
			
			{
				boost::mutex::scoped_lock(fileMutex);
				mt.save("mt");
			}

		}
	}
}

}
