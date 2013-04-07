#ifndef DefaultTimer
#define DefaultTimer 
#include <time.h>
#include "Timer.hpps"

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