#include <iostream>
#include <nice_global_planner/KDTree.hpp>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <utility>
#include <assert.h>

int main(int argc, char * argv[]){

	KDTree<unsigned> kdTree(2);
	//unsigned store = 0;

	std::vector<std::vector<unsigned> > stored;

	for(unsigned i = 0; i < 500; ++i){
		std::vector<unsigned> data;
		data.push_back(rand()%1000);
		data.push_back(rand()%1000);
		stored.push_back(data);
		kdTree.insert(data);
	}

	//kdTree.printTreeFromRoot();
	//std::cout << store-1 << std::endl;
	for(unsigned i = 0; i < stored.size(); ++i){
		/*std::cout << "stored[" << i << "][0]: " << stored[i][0];
		std::cout << " stored[" << i << "][1]: " << stored[i][1];
		std::cout << ": kdTree.findNode(stored[" << i << "]): ";
		std::cout << kdTree.findNode(stored[i]) << std::endl;*/
		assert(kdTree.findNode(stored[i]));
	}

	unsigned index = 2;

	KDNode<unsigned> *nearest = kdTree.nearestNeighbour(stored[index]);

	std::cout << "stored[" << index << "][0]: " << stored[index][0];
	std::cout << " nearestneighbour[0]: " << (*nearest).data[0] << std::endl;	
	std::cout << "stored[" << index << "][1]: " << stored[index][1];
	std::cout << " nearestneighbour[1]: " << (*nearest).data[1] << std::endl;

	return 0;
}