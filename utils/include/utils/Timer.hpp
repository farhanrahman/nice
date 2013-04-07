#ifndef TIMER_HPP
#define TIMER_HPP

namespace utils{

class Timer{
public:
	virtual double getTime(void) = 0;
	virtual ~Timer(void){}
};

}

#endif