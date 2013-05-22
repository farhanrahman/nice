#include <utils/RandomNumberGenerator.hpp>


int main(int argc, char * argv[]){
	utils::RandomNumberGenerator rng;

	for(int i = 0; i < 100; ++i){
		std::cout << rng.uniform01() << std::endl;
	}

	return 0;
}