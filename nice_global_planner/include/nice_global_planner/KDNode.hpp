#ifndef KD_NODE_HPP
#define KD_NODE_HPP
#include <vector>
#include <assert.h>
#include <map>
#include <limits.h>
#include <iostream>
#include <math.h>
#include "BinaryDecision.hpp"

namespace nice_global_planner{

template<typename T>
class KDNode{
public:
	KDNode (const std::vector<T>& inputData, unsigned decisionDim)
	: decisionDim(decisionDim)
	{
		data.resize(inputData.size());
		for(unsigned i = 0; i < inputData.size(); ++i){
			this->data[i] = inputData[i];
		}
		this->commonInitialiser();
	}

	KDNode(unsigned dim, unsigned decisionDim)
	: decisionDim(decisionDim)
	{
		data.resize(dim);
		this->commonInitialiser();
	}

	BinaryDecision getNextDirection(const std::vector<T>& data){
		if(this->data[decisionDim] < data[decisionDim]){
			return RIGHT;
		} else {
			return LEFT;
		}
	}

	BinaryDecision getNextDirection(const KDNode<T> *node){
		if(this->data[decisionDim] < (*node).data[decisionDim]){
			return RIGHT;
		} else {
			return LEFT;
		}
	}

	KDNode * getNextStorageNode(const std::vector<T>& data){
		BinaryDecision binaryDecision = this->getNextDirection(data);
		if(binaryDecision == LEFT){
			return left;
		} else {
			return right;
		}
	}

	unsigned getNextDecisionDim(unsigned dim){
		return (this->decisionDim + 1)%dim;
	}

	T getSplittingValue(void){
		return (*this).data[decisionDim];
	}

	unsigned getSplittingAxis(void){
		return this->decisionDim;
	}


	bool equals(const KDNode<T> *node){
		bool equal = false;

		if((*node).data.size() != (*this).data.size()){
			return equal;
		}

		for(unsigned i = 0; i < (*this).data.size(); ++i){
			if((*this).data[i] != (*node).data[i]){
				return equal;
			}
		}
		
		equal = true;
		return equal;
	}

	double distance(const KDNode<T> *node){
		return sqrt(this->sqDistance(node));
	}

	double sqDistance(const KDNode<T> *node){
		double sqDistance = 0.0;
		for(unsigned i = 0; i < data.size(); ++i){
			double diff = this->data[i] - (*node).data[i];
			sqDistance += diff*diff;
		}
		return sqDistance;
	}

	void printNode(void){
		std::cout << "[";
		for(unsigned i = 0; i < data.size(); ++i){
			if(i == data.size() - 1){
				std::cout << "data["<< i << "]: " << data[i];
			} else {
				std::cout << "data[" << i << "]: " << data[i] << ",";
			}
		}
		std::cout << "]" << std::endl;
	}

	static KDNode<T> createTempNode(const std::vector<T>& data){
		return KDNode<T> (data, 0);
	}


	std::vector<T> data;
	KDNode *left, *right;
	KDNode *parent;

private:
	unsigned decisionDim;

	void commonInitialiser(void){
		left = NULL;
		right = NULL;
		parent = NULL;
	}
};

}

#endif