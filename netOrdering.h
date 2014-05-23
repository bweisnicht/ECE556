/*
	ECE 556 Project
	Calculate Net Ordering
	Team OOPW
*/

#ifndef __NET_ORDERING__
#define __NET_ORDERING__

#include <cstdlib>
#include <vector>
#include <algorithm>
#include "ece556.h"

using std::sort;
using std::vector;

//#############################################################################
//	Defines
//#############################################################################

//#############################################################################
//	Function Prototypes
//#############################################################################

// Going to use qsort from the c library for now.  Might change this later to see if there is an improvement.
int orderNets(routingInst *rst);

int vecCompareNets(net netA, net netB);

// used in calling the qsort() library function
// takes care of the comparison between two nets
// return negative if a is before b
// return positive if a is after b
// return 0 otherwise
int compareNets(const void *a, const void *b);

#endif
