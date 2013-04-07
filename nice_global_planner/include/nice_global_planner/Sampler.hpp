#ifndef SAMPLER_HPP
#define SAMPLER_HPP
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <string>


namespace nice_global_planner
{

class Sampler
{
public:
	/**
	 * @brief Given a goal point to reach this method should
	 *  return a sampled point from the underlying data structure
	 *  representing the costmap or however the environment is
	 *  is being represented in any data structure. The derived classes
	 *  overriding this method will be responsible for defining the costmap
	 * @param goal Goal position to bias the sampling
	 * @return geometry_msgs::Point A sampled point from the environment
	 */

	virtual geometry_msgs::Point samplePoint(const geometry_msgs::Pose& goal) = 0;

	/**
	 * @brief Method to be called by delegate when the sampler needs
	 *  to update the underlying data structure
	 */
	virtual void update(void) = 0;

	 /**
	  * @brief This method returns the global frame id associated with the map
	  * @return std::string Returns the global frame id
	  */
	virtual std::string getGlobalFrameID(void) = 0;

	/**
	 * @brief Destructor
	 */
	virtual ~Sampler(void){}
};

}

#endif