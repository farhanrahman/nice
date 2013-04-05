#ifndef KD_TREE_HPP
#define KD_TREE_HPP
#include <vector>
#include <assert.h>
#include <map>
#include <limits.h>

typedef enum BinaryDecision{
	LEFT = 0, RIGHT = 1
} BinaryDecision;

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


template<typename T>
class KDTree{
public:
	KDTree(unsigned dim)
	: dim(dim){
		root = NULL;
	}

	~KDTree(void){
		this->pruneTree(this->root);
	}

	void insert(const std::vector<T> &data){
		if(root == NULL){
			root = new KDNode<T>(data, 0);
		} else {
			this->insertHelper(root, data);
		}
	}

	void printTreeFromRoot(void){
		std::map<KDNode<T> *, bool> printed;
		printTree(this->root, printed);
	}

	KDNode<T> * findNode(const std::vector<T> &data){
		KDNode<T> searchNode = KDNode<T>::createTempNode(data);
		return findNodeHelper(root, const_cast<KDNode<T> *>(&searchNode));
	}


	KDNode<T> * nearestNeighbour(const std::vector<T> &data){
		KDNode<T> newNode = KDNode<T>::createTempNode(data);
		KDNode<T> *currentBest = root;
		double currentBestDistance = root->sqDistance(const_cast<KDNode<T> *> (&newNode));
		this->nearestNeighbour(root, &currentBest, currentBestDistance, const_cast<KDNode<T> *> (&newNode));
		return currentBest;
	}

private:

	void nearestNeighbour(KDNode<T> *node, KDNode<T> **currentBest, double &currentBestDistance, const KDNode<T> *newNode){
		if(node == NULL){
			return;
		} else {
			double sqDistance = node->sqDistance(newNode);
			if(sqDistance < currentBestDistance){
				(*currentBest) = node;
				currentBestDistance = sqDistance;
			}
						
			BinaryDecision binaryDecision = node->getNextDirection(newNode);
			//double diff = abs((*newNode).data[node->getSplittingAxis()] - node->getSplittingValue());

			// if(binaryDecision == LEFT){
			// 	if(diff <= currentBestDistance){
			// 		nearestNeighbour(node->left, currentBest, currentBestDistance, newNode);
			// 	} else {
			// 		nearestNeighbour(node->right, currentBest, currentBestDistance, newNode);
			// 	}
			// } else {
			// 	if(diff < currentBestDistance){
			// 		nearestNeighbour(node->right, currentBest, currentBestDistance, newNode);
			// 	} else {
			// 		nearestNeighbour(node->left, currentBest, currentBestDistance, newNode);
			// 	}
			// }
			T newNodeSplitData = (*newNode).data[node->getSplittingAxis()];
			T nodeSplitData = node->getSplittingValue();
			if(binaryDecision == LEFT){
				if(newNodeSplitData - currentBestDistance <= nodeSplitData) {
					nearestNeighbour(node->left, currentBest, currentBestDistance, newNode);
				}

				if(newNodeSplitData + currentBestDistance > nodeSplitData) {
					nearestNeighbour(node->right, currentBest, currentBestDistance, newNode);
				}

			} else {
				if(newNodeSplitData + currentBestDistance > nodeSplitData) {
					nearestNeighbour(node->right, currentBest, currentBestDistance, newNode);
				}

				if(newNodeSplitData - currentBestDistance <= nodeSplitData) {
					nearestNeighbour(node->left, currentBest, currentBestDistance, newNode);
				}
			}
		}
	}


	KDNode<T> *findNodeHelper(KDNode<T> *node, const KDNode<T> *searchNode){
		if(node == NULL){
			return NULL;
		} else if(node->equals(searchNode)){
			return node;
		} else {
			BinaryDecision binaryDecision = node->getNextDirection(searchNode);
			if(binaryDecision == LEFT){
				return findNodeHelper(node->left, searchNode);
			} else {
				return findNodeHelper(node->right, searchNode);
			}
		}
	}

	void printTree(KDNode<T> *node, std::map<KDNode<T> *, bool> &printed){
		if(node == NULL){
			return;
		} else {
			if(printed.find(node) == printed.end()){
				printed[node] = false;
			} 			

			printTree(node->left, printed);

			if(printed[node] == false){
				node->printNode();
			}	

			printTree(node->right, printed);		
		}
	}

	void insertHelper(KDNode<T> *node, const std::vector<T>& data){
		BinaryDecision binaryDecision;
		if(node->getNextStorageNode(data) == NULL){
			binaryDecision = node->getNextDirection(data);
			if(binaryDecision == LEFT){
				node->left = new KDNode<T>(data, node->getNextDecisionDim(this->dim));
				KDNode<T> *child = node->left;
				child->parent = node;
			} else {
				node->right = new KDNode<T>(data, node->getNextDecisionDim(this->dim));
				KDNode<T> *child = node->right;
				child->parent = node;
			}
			return;
		} else {
			binaryDecision = node->getNextDirection(data);
			if(binaryDecision == LEFT){
				this->insertHelper(node->left, data);
			} else {
				this->insertHelper(node->right, data);
			}
		}
	}

	void pruneTree(KDNode<T> *node){
		if(node == NULL){
			return;
		} else {
			pruneTree(node->left);
			pruneTree(node->right);
			delete node;
		}
	}

	KDNode<T> *root;

private:
	unsigned dim;
};

#endif