#include <iostream>
#include <nice_global_planner/KDTree.hpp>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <utility>

int main(int argc, char * argv[]){

	KDTree<unsigned> kdTree(2);
	//unsigned store = 0;

	std::vector<std::vector<unsigned> > stored;

	for(unsigned i = 0; i < 5; ++i){
		std::vector<unsigned> data;
		data.push_back(rand()%10);
		data.push_back(rand()%10);
		stored.push_back(data);
		kdTree.insert(data);
	}

	kdTree.printTreeFromRoot();
	//std::cout << store-1 << std::endl;
	for(unsigned i = 0; i < stored.size(); ++i){
		std::cout << "stored[" << i << "][0]: " << stored[i][0];
		std::cout << " stored[" << i << "][1]: " << stored[i][1];
		std::cout << ": kdTree.findNode(stored[" << i << "]): ";
		std::cout << kdTree.findNode(stored[i]) << std::endl;
	}

	return 0;
}