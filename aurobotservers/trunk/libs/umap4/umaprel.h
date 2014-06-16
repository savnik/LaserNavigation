/***************************************************************************
 *   Copyright (C) 2004 by Christian Andersen                              *
 *   jca@pc89                                                              *
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
#ifndef UMAPREL_H
#define UMAPREL_H

class UMapObj;
class UPose;

/**
This class holds information on relations between two map objects. 
The relation is 2d positional and may contain the same information as a pose, i.e. x,y, heading, and covariance matrix related to this relation. The heading constraint will normally be unknown.

@author Christian Andersen
*/
class UMapRel
{
public:
  /**
  Default constructor */
  UMapRel(UMapSegment * parentSegment);
  /**
  destructor.
  Decouples the object from related objects. */
  ~UMapRel();
  /**
  REturn pointer to this object */
  UMapRel * get() { return this;};
  /**
  Remove this object from the constraint list
  of both related objects. */
  void unlink();
  /**
  Save relation in xml-like format */
  bool save(Uxml3D * fxmap);
  /**
  Load relation in xml-like format */
  bool load(Uxml3D * fxmap);
  /**
  Get parent segment */
  inline UMapSegment * getParent() { return parent;};
  /**  
  Set parent segment */
  inline void setParent(UMapSegment * parentSegment) 
           { parent = parentSegment;};  
  /**
  Get reference (pointer) to relative pose structure */
  inline UPoseQ * getRelPose(bool notNull) 
  { 
    if ((relPose == NULL) and notNull)
      relPose = new UPoseQ();
    return relPose;
  };            
  /**
  Get pointer to other object.
  That is the object that is different from this. 
  Returns pointer to other object. */
  UMapObj * getOther(UMapObj * notThis); 
  /**
  Set links to these two objects.
  If the relation is linked already - i.e.
  obj1 and/or obj2 is not NULL - then
  these relation are broken first. 
  Returns true if references are both != NULL. */
  bool setLinks(UMapObj * ref1, UMapObj * ref2);
           
protected:
  /**
  Parent segment pointer */
  UMapSegment * parent;
  /**
  Relation in position, using x,y,h defined
  by segment (local) coordinate system.
  Position is from first object to second object.
  i.e. according to this relation the position
  of object 2 should be at object 1 position 
  plus this relation.
  The covariance (or spring stiffness) is
  defined by the Q matrix. */
  UPoseQ * relPose;
  /**
  Is heading constraint valid? */
  bool headingValid;
  /**
  Reference to object 1.
  This reference is NULL if no object is defined. */
  UMapObj * obj1;
  /**
  Reference to object 2.
  This reference is NULL if no object is defined. */
  UMapObj * obj2;
  
};

#endif
