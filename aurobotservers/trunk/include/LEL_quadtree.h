/*
 * quadtree.h
 *
 *  Created on: 16/12/2009
 *      Author: eba
 */

#ifndef QUADTREE_H_
#define QUADTREE_H_

class QuadTreeLeaf;

class QuadTreePoint {

	public:

	float x;
	float y;
	float weight;
	bool compound;

	inline QuadTreePoint(float x, float y, float weight=1) {
		this->x=x;
		this->y=y;
		this->weight=weight;
		compound=false;
	}

	inline void copyExtra(QuadTreePoint * point) {
	}

	inline void blankHit(float weight) {
	this->weight+=weight;
	compound=true;
	}

};
class QuadTreeNode {

	public:

	QuadTreeLeaf * ul, *ur, *dl, *dr;
	int leafCount;

	QuadTreeNode(float cX,float cY,float w,float h,QuadTreeNode * parent, int quadrant, float minDist);
	QuadTreePoint * addPoint(float x, float y, float weight);
	inline void addPoint (QuadTreePoint * point) { // Note that, only a copy is added
		addPoint(point->x, point->y, point->weight)->copyExtra(point);
	}

	float centerX, centerY;
	float width, height;
	float minDist;

	QuadTreeNode * parent;
	int quadrant;

};

class QuadTreeLeaf {

	public:

	union {
		QuadTreeNode * node;
		QuadTreePoint * point;
	} pointers;

	bool isNode;

	inline QuadTreeLeaf(float x, float y, float weight) {
	isNode = false;
	pointers.point = new QuadTreePoint(x,y,weight);
	}

	inline QuadTreeLeaf(QuadTreeNode* node) {
	isNode = true;
	pointers.node=node;
	}

	inline ~QuadTreeLeaf() {
	if(isNode) delete pointers.node;
	else delete pointers.point;
	}

};

class QuadTree {

	public:

	QuadTreeNode * treeHead;
	float upLimit, downLimit, leftLimit, rightLimit;
	float minDist;

	inline QuadTree(float lL, float rL, float dL, float uL, float minDist) {

	this->upLimit=uL;
	this->downLimit=dL;
	this->leftLimit=lL;
	this->rightLimit=rL;

	this->minDist=minDist;

	float width = rL-lL, height = uL-dL;

	treeHead = new QuadTreeNode((lL+rL)/2,(uL+dL)/2, width , height, NULL, 0,minDist);

	}

	inline ~QuadTree() {
	delete treeHead;
	}

	QuadTreePoint * addPoint(float x, float y, float wt);
};

class PointDescriptor {
	public:
	QuadTreeNode * QTN;
	int quadrant;

	inline PointDescriptor(QuadTreeNode * QTN=NULL, int quadrant=0) {
		this->QTN=QTN;
		this->quadrant=quadrant;
	}

	QuadTreePoint * getPoint();

};

QuadTreePoint * checkNode(QuadTreeNode * QTN);
void removePoint(QuadTreeNode *QTN, int quadrant);

#endif /* QUADTREE_H_ */
