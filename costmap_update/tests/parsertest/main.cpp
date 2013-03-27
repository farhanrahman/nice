#include <costmap_update/tfMessageParser.hpp>
#include <iostream>
#include <string>

#define TEST_FAILED std::cerr << "TEST_FAILED" << std::endl; exit(1)
#define TEST_PASSED std::cout << "TEST_PASSSED" << std::endl


int main(int argc, char * argv[]){
	std::string testMessage = "head_1";

	tfparser::message msg = tfMessageParser::parseFrame(testMessage);
	if(msg.part != "head"){
		TEST_FAILED;
	}

	testMessage = "left_shoulder_1";
	msg = tfMessageParser::parseFrame(testMessage);
	if(msg.part != "left_shoulder"){
		TEST_FAILED;
	}

	testMessage = "errormessage";
	try{
		msg = tfMessageParser::parseFrame(testMessage);		
		TEST_FAILED;
	} catch(const FrameFormatNotCorrect &e){
		TEST_PASSED;
	}


	return 0;
}