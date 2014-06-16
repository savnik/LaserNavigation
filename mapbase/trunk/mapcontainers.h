/* 
 * File:   mapcontainers.h
 * Author: andersbeck
 *
 * Created on January 13, 2009, 11:31 AM
 */

#include <string>
#include <vector>
#include <map>

#include <umap4/upose.h>

#ifndef _MAPCONTAINERS_H
#define	_MAPCONTAINERS_H 1

using namespace std;

#define NAMELEN 128

struct pose {
	double  th;  /**< Pose yaw in degrees */
	double  thr; /**< Pose yaw in rad  */
   double  x;   /**< Pose x */
   double  y;   /**< Pose y */
};

class connector {
public:
    connector();
    virtual ~connector();
    void copy(connector source);
    int     id;
    char    name[NAMELEN];
    double  th;  /**< Pose yaw in degrees in parent coordinate system */
	 double  thr; /**< Pose yaw in rad in parent coordinate system */
    double  x;   /**< Pose x in parent coordinate system */
    double  y;   /**< Pose y in parent coordinate system */
    pose poseMap;
    pose poseLocal;
    UPose pMap;
    UPose pLocal;
    int autoconnect;
    map<string, double> features; /**< Mapped list of features */
private:

};

class node {
public:
    node();
    virtual ~node();
    void copy(node);
    UPose getOrigin()
    {
      UPose result(x,y,th);
      return result;
    };
	 UPose getMapOrigin()
	 {
		 UPose result(poseMap.x,poseMap.y,poseMap.thr);
		 return result;

	 };
    char            name[NAMELEN];
    char            typeName[NAMELEN];
    char            shape[NAMELEN];
    int             depth;
    int             parentId;
    double          width;
    double          heigth;
    double          diameter;
    double          x;
    double          y;
    double          th;
    double          thr;
    pose            poseMap;
    pose            poseLocal;
    UPose           pMap;
    UPose           pLocal;
    vector<connector> connectors; /**< Connectors for this node */
    map<string, double> features; /**< Mapped list of features */
private:

};


class edge {
public:
    edge();
    virtual ~edge();
    void copy(edge);
    
    char        name[NAMELEN];
    char        typeName[NAMELEN];
    double      length;
    bool        directional;
    bool        blocked;
    bool        disabled;
    node        *startnode;
    node        *endnode;
    connector   *startconnector;
    connector   *endconnector;
    char        start[NAMELEN];
    char        end[NAMELEN];
    char        startNodeName[NAMELEN];
    char        endNodeName[NAMELEN];
    char        startConnName[NAMELEN];
    char        endConnName[NAMELEN];
    int         startNodeIndex;
    int         endNodeIndex;
    int         startConnIndex;
    int         endConnIndex;
    int         startConnId;
    int         endConnId;
    
    map<string, double> features; /**< Mapped list of features */
private:

};

class route {
public:
    string      startName;
    string      endName;
    int         edgeLength;
    int         totalDist;
    int         id;
    edge        routeEdge;
    int         edgeId;
    node        startNode;
    node        endNode;
    connector   startConn;
    connector   endConn;
    int         startNodeId;
    int         endNodeId;
    int         startConnId;
    int         endConnId;
private:
};


#endif	/* _MAPCONTAINERS_H */

