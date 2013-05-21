#ifndef SAMPLER_HPP
#define SAMPLER_HPP
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <string>
#include "CostCalculatorDelegate.hpp"

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
	 * @brief Method should return whether the point being passed
	 *  lies in the free configuration space
	 * @param point The vector point to be checked
	 * @return True if given point lies in free configuration
	 */
	virtual bool point2DInFreeConfig(const std::vector<double> &point, double yaw = 0) = 0;

	/*@brief Initialises the sampler with the cost calculator delegate
	 *@param costCalculatorDelegate Delegate which calculates the
	 * legality of occupying a cell in the costmap
	 **/
	virtual void initialise(CostCalculatorDelegate *costCalculatorDelegate) = 0;

	/**
	 * @brief Destructor
	 */
	virtual ~Sampler(void){}
};

}

#endif