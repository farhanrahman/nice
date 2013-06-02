#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nice_detector/Detection.h>
#include <nice_detector/DetectionList.h>
#include <nice_detector/prediction.h>
#include "munkres.cpp"
#include "track.h"
#include <boost/thread.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

/*ROS tracking node*/
namespace nice_tracker{

	class NiceTracker: public nodelet::Nodelet{
		private:
			volatile bool running;
      		boost::shared_ptr<boost::thread> kalmanThread;
			boost::shared_ptr<boost::thread> driverThread;
			ros::NodeHandlePtr nh;
			tf::TransformListener* mTransformListener;

		public: 
			ros::Publisher track;
			ros::Publisher published_marker;
			ros::ServiceServer predictions;
      		ros::Publisher cmdpub;
			visualization_msgs::Marker *track_mark;
			geometry_msgs::PointStampedPtr latest_prediction;
		  	double dt;
			float robo_x;
			float robo_y;
			float measure_voxel_error;
			ros::Time current_time, last_time;		
		  	std::list<Track>* tracks;

			typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
			move_base_msgs::MoveBaseGoal goal;
			MoveBaseClient *ac;
			

			typedef message_filters::sync_policies::ApproximateTime<nice_detector::DetectionList, nav_msgs::Odometry> MySyncPolicy;
			// typedef message_filters::sync_policies::ApproximateTime<nice_detector::DetectionList> MySyncPolicy;
			typedef message_filters::Synchronizer<MySyncPolicy> Synchronizer;

			ros::Subscriber detectionListener;

			ros::Publisher kalmanPointPublisher;

			message_filters::Subscriber<nice_detector::DetectionList> detect;
			message_filters::Subscriber<nav_msgs::Odometry> odom;			
			boost::shared_ptr<Synchronizer> sync;
			
			NiceTracker():running(false){}
			
			
		  	void start_tracking (const nice_detector::DetectionListConstPtr& detects/*, const nav_msgs::OdometryConstPtr& odom*/);
			void create_new_track(const nice_detector::Detection detection, geometry_msgs::PointStampedPtr& worldPoint, double z);
			void Mark_path(double x, double y, double z, int r, int g, int b);
			void KalmanLoop();	
			void DriverLoop();
			bool get_prediction(nice_detector::prediction::Request& request, nice_detector::prediction::Response& response);
			void make_command(geometry_msgs::PointStampedPtr& worldPoint);
			geometry_msgs::PointStamped getWorldPoint(geometry_msgs::PointStampedPtr& p, const char *frame);
			virtual void onInit();
			~NiceTracker();
			std::list<Track>* get_tracks(void){
				boost::mutex::scoped_lock l(m_mutex);
				return tracks;
			}
	};

}
