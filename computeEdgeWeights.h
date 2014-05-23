#ifndef __EDGE_WEIGHT_COMP__
#define __EDGE_WEIGHT_COMP__

#include "ece556.h"
#include <vector>

using std::vector;

/* Computes the weight of each edge in the grid.
 *
 * Returns:
 * total weight of all the edges.
 */

void computeEdgeWeights(routingInst* rst, vector<int>& edgeOverflow, vector<int>& edgeWeight, vector<int>& edgeHistory);
#endif
