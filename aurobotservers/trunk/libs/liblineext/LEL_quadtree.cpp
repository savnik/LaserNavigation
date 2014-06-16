/***************************************************************************
 *   Copyright (C) 2008 by Project student,,,                              *
 *   ex20@nyquist.iau.dtu.dk                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "LEL_hough.h"
#include "LEL_quadtree.h"
#include <iostream>
using namespace std;

QuadTreeNode::QuadTreeNode (float cX,float cY,float w,float h, QuadTreeNode * parent, 
							int quadrant, float minDist) {
	ul=NULL;
	ur=NULL;
	dl=NULL;
	dr=NULL;
	leafCount=0;
	this->parent=parent;
	this->quadrant=quadrant;
	this->minDist = minDist;
	
	centerX =cX;
	centerY =cY;
	width = w;
	height = h;
}

QuadTreePoint * QuadTreeNode::addPoint (float x, float y, float weight) {

	switch(((x-centerX)>0)+2*(y-centerY>0)){
		case 0: // dl
			if (dl==NULL) {
				dl= new QuadTreeLeaf(x,y,weight);
				leafCount++;
				return dl->pointers.point;
			} else if(!dl->isNode) {
			if(fabs(x-dl->pointers.point->x)+fabs(y-dl->pointers.point->y)<minDist) 
				{dl->pointers.point->blankHit(weight);
				return dl->pointers.point;}
			QuadTreeNode * newNode = new QuadTreeNode(centerX-width/4,centerY-height/4,
									width/2,height/2,this,3, minDist);
			newNode->addPoint(dl->pointers.point);
			delete dl;
			dl = new QuadTreeLeaf(newNode);
			return dl->pointers.node->addPoint(x,y,weight);
			} else  return dl->pointers.node->addPoint(x,y,weight);
		
		case 1: // dr
			if (dr==NULL) {
				dr= new QuadTreeLeaf(x,y,weight);
				leafCount++;
				return dr->pointers.point;
			} else if(!dr->isNode) {
			if(fabs(x-dr->pointers.point->x)+fabs(y-dr->pointers.point->y)<minDist) 
				{dr->pointers.point->blankHit(weight);
				return dr->pointers.point;}
			QuadTreeNode * newNode = new QuadTreeNode(centerX+width/4,centerY-height/4,
									width/2,height/2,this,4, minDist);
			newNode->addPoint(dr->pointers.point);
			delete dr;
			dr = new QuadTreeLeaf(newNode);
			return dr->pointers.node->addPoint(x,y,weight);
			} else  return dr->pointers.node->addPoint(x,y,weight);
			
		case 2: // ul
			if (ul==NULL) {
				ul= new QuadTreeLeaf(x,y,weight);
				leafCount++;
				return ul->pointers.point;
			} else if(!ul->isNode) {
			if(fabs(x-ul->pointers.point->x)+fabs(y-ul->pointers.point->y)<minDist)
				{ul->pointers.point->blankHit(weight);
				return ul->pointers.point;}
			QuadTreeNode * newNode = new QuadTreeNode(centerX-width/4,centerY+height/4,
									width/2,height/2,this,2, minDist);
			newNode->addPoint(ul->pointers.point);
			delete ul;
			ul = new QuadTreeLeaf(newNode);
			return ul->pointers.node->addPoint(x,y,weight);
			} else  return ul->pointers.node->addPoint(x,y,weight);
			
		default: case 3: // ur
			if (ur==NULL) {
				ur= new QuadTreeLeaf(x,y,weight);
				leafCount++;
				return ur->pointers.point;
			} else if(!ur->isNode) {
			if(fabs(x-ur->pointers.point->x)+fabs(y-ur->pointers.point->y)<minDist) 
				{ur->pointers.point->blankHit(weight);
				return ur->pointers.point;}
			QuadTreeNode * newNode = new QuadTreeNode(centerX+width/4,centerY+height/4,
									width/2,height/2,this,1, minDist);
			newNode->addPoint(ur->pointers.point);
			delete ur;
			ur = new QuadTreeLeaf(newNode);
			return ur->pointers.node->addPoint(x,y,weight);
			} else  return ur->pointers.node->addPoint(x,y,weight);
	}

}

QuadTreePoint * checkNode(QuadTreeNode * QTN) {

	bool leafExists =false;
	int theLeaf=0;
	if(QTN->ur!=NULL) {
		if(QTN->ur->isNode) return NULL;
		leafExists =true;
		theLeaf=1;}

	if(QTN->ul!=NULL) {
		if(QTN->ul->isNode || leafExists) return NULL;
		leafExists =true;
		theLeaf=2;}

	if(QTN->dl!=NULL) {
		if(QTN->dl->isNode || leafExists) return NULL;
		leafExists =true;
		theLeaf=3;}
		
	if(QTN->dr!=NULL) {
		if(QTN->dr->isNode || leafExists) return NULL;
		leafExists =true;
		theLeaf=4;}
		
	switch (theLeaf) {
	case 1: return QTN->ur->pointers.point;
	case 2: return QTN->ul->pointers.point;
	case 3: return QTN->dl->pointers.point;
	case 4: return QTN->dr->pointers.point;
	default: return NULL;
	}
		
}

void removePoint(QuadTreeNode *QTN, int quadrant) {

	QuadTreeNode * parent;
	QuadTreeLeaf * leaf;
	switch(quadrant) {
	case 1: delete QTN->ur;
		QTN->ur = NULL;
		break;
	case 2: delete QTN->ul;
		QTN->ul = NULL;
		break;
	case 3: delete QTN->dl;
		QTN->dl = NULL;
		break;
	case 4: delete QTN->dr;
		QTN->dr = NULL;
		break;
	}
	QTN->leafCount--;
	while(1) {
	QuadTreePoint * remainingPoint=checkNode(QTN);
	if(remainingPoint==NULL) return;
	if((parent = QTN->parent)==NULL) return;
	quadrant = QTN->quadrant;
	switch(quadrant) {
	case 1: leaf = parent->ur;
		parent->ur = NULL;
		parent->addPoint(remainingPoint);
		break;
	case 2: leaf = parent->ul;
		parent->ul = NULL;
		parent->addPoint(remainingPoint);
		break;
	case 3: leaf = parent->dl;
		parent->dl = NULL;
		parent->addPoint(remainingPoint);
		break;
	case 4: leaf = parent->dr;
		parent->dr = NULL;
		parent->addPoint(remainingPoint);
		break;
	}
	delete leaf;
	QTN = parent;
	}
}
