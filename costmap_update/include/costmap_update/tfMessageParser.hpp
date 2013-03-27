#ifndef TF_MESSAGE_PARSER_HPP
#define TF_MESSAGE_PARSER_HPP
#include <vector>
#include <string>
#include <stdlib.h>
#include <sstream>
#include <costmap_update/ParserMessage.hpp>
#include <costmap_update/FrameFormatNotCorrect.hpp>

class tfMessageParser {
public:
	static tfparser::message parseFrame(const std::string &childFrameID)
		throw (FrameFormatNotCorrect)
	{
		std::vector<std::string> splitted = tfMessageParser::split(childFrameID, '_');
		if(splitted.size() < 2){
			throw FrameFormatNotCorrect(childFrameID);
		}

		tfparser::message msg;
		std::stringstream part;
		for(unsigned i = 0; i < splitted.size() - 1; ++i){
			part << splitted[i];
			if(i < splitted.size() - 2){
				part << "_";
			}
		}
		part >> msg.part;

		std::stringstream ss;
		ss << splitted[splitted.size() - 1];
		ss >> msg.user;
		return msg;
	}

private:
	static std::vector<std::string> split(const std::string &s, char delim) {
    	std::vector<std::string> elems;
    	return tfMessageParser::split(s, delim, elems);
	}

	static std::vector<std::string> &split(
				const std::string &s, 
				char delim, 
				std::vector<std::string> &elems) 
	{
	    std::stringstream ss(s);
	    std::string item;
	    while(std::getline(ss, item, delim)) {
	        elems.push_back(item);
	    }
	    return elems;
	}	
};

#endif