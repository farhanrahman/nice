#include <iostream>
#include <nice_global_planner/KDTree.hpp>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <utility>
#include <assert.h>
#include <ctime>

int main(int argc, char * argv[]){

	srand (time(NULL));

	nice_global_planner::KDTree<unsigned> kdTree(2);

	std::vector<std::vector<unsigned> > stored;

	unsigned limit = 10000;
	unsigned range = 1000;

	for(unsigned i = 0; i < limit; ++i){
		std::vector<unsigned> data;
		data.resize(2);
		data[0] = rand()%range;
		data[1] = rand()%range;
		stored.push_back(data);
		kdTree.insert(data);
	}

	for(unsigned i = 0; i < stored.size(); ++i){
		assert(kdTree.findNode(stored[i]));
	}

	unsigned index = rand()%limit;

	nice_global_planner::KDNode<unsigned> *nearest = kdTree.nearestNeighbour(stored[index]);

	assert(stored[index][0] == (*nearest).data[0]);
	assert(stored[index][1] == (*nearest).data[1]);

	std::cout << "TEST PASSED" << std::endl;

	return 0;
}