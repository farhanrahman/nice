#ifndef DEFAULT_TIMER_HPP
#define DEFAULT_TIMER_HPP 
#include <time.h>
#include "Timer.hpp"

namespace utils{

class DefaultTimer : public Timer
{
public:
	double getTime(void){
		return (double) time(NULL);
	}
};

}

#endif