#ifndef FRAME_FORMAT_NOT_CORRECT
#define FRAME_FORMAT_NOT_CORRECT
#include <iostream>
#include <string>


class FrameFormatNotCorrect : public std::exception{
public:
	FrameFormatNotCorrect(const std::string& input)
		: errorMessage("input format not correct, got: " + input){}

	~FrameFormatNotCorrect(void) throw(){}

    const char* what() const throw(){
            const std::string ret = this->errorMessage;
            return ret.c_str();
    }   

	std::string errorMessage;
};

#endif