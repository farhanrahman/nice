#ifndef PATH_NOT_FOUND_EXCEPTION
#define PATH_NOT_FOUND_EXCEPTION

#include <string>


class PathNotFoundException : public std::exception{
public:
	PathNotFoundException(std::string person) :
		message("path for '" + person + "' does not exist in the database") {}

	const char * what() {
		const std::string errMsg = this->getMessage();
		return errMsg.c_str();
	}

	std::string getMessage(void){
		return message;
	}

private:
	std::string message;
};

#endif
