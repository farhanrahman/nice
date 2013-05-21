#ifndef COST_CALCULATOR_DELEGATE_HPP
#define COST_CALCULATOR_DELEGATE_HPP

namespace nice_global_planner{

class CostCalculatorDelegate
{
public:
	/*@brief An interface function for other classes that can
	 * use this function to calculate the cost of occupying a
	 * cell in the cosmap.
	 *@param x_i x co-ordinate of the test point
	 *@param y_i y co-ordinate of the test point
	 *@param theta_i yaw value of the test point
	 **/
	virtual double footprintCost(double x_i, double y_i, double theta_i) = 0;
};

}

#endif