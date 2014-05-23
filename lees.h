//#############################################################################
//
//		ECE 556
//		Team OOPW
//
//		Header for implemeting Lee's Routing Algorithm
//
//#############################################################################

#ifndef __LEES__
#define __LEES__

#include "ece556.h"
#include <type_traits>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <set>
#include <map>
#include <limits.h>
#include <unordered_map>
#include <utility>

using std::vector;
using std::set;
using std::unordered_map;
//#############################################################################
//
//		Defines
//
//#############################################################################

//This constant is used to set the size of the bounding box around
//the box formed by the start and end pins of a specific route.
//a value of 0 means that it is hte same as the Start-End box, with
//every value after that increasing both x and y offsets by 1 (on all sides,
//so value of 1 means the box inc by 2 in width and heighth)
const int BBoffset = 2;

//same as rst->gy, must be set before using maps
static int gy;

struct  LeesNode
{
	point loc;     		// x,y of this node
	LeesNode* parent;  	// previous node 
	int cost;			// cost of this node
	int edgeUtil;  		// util of edge from parent to loc
	int edgeCap;   		// cap of edge from parent to loc

	bool operator==(const LeesNode& n) const
	{
		return (loc == n.loc);
	}

	bool operator!=(const LeesNode& n) const
	{
		return !(loc == n.loc);
	}
};

struct Hash {
   size_t operator()(const point &pt) const {
     return pt.y + pt.x * gy;
   }
};

//#############################################################################
//
//		Function Prototypes
//
//#############################################################################

set<LeesNode*, CompareNodeCost>::iterator findNodeAtLocation(set<LeesNode*, CompareNodeCost>& group,
  unordered_map<point, LeesNode*, Hash> m_group2, LeesNode* searchNode);

void deleteMap( unordered_map<point, LeesNode*, Hash>& group );

void deleteGroup( vector<LeesNode*>& group );

int retrace(routingInst *rst, LeesNode& nS, LeesNode* current, int netInd, int segInd);

int solveRoutingLees(routingInst *rst);

int routeLees(routingInst* rst, int netInd, int SpinInd, int TpinInd );

void getNeighbors(routingInst *rst, vector<LeesNode*>& neighbors, LeesNode* current,
                    unordered_map<point, LeesNode*, Hash>& group1,
                    vector<LeesNode*>& group2,
                    unordered_map<point, LeesNode*, Hash>& group3);

vector<LeesNode*>::iterator findNodeAtLocation(vector<LeesNode*>& group, LeesNode* searchNode);
#endif
