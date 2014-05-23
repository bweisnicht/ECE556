#include "ece556.h"
#include <cstdio>
#include <cstdlib>
#include <vector>

using std::vector;

/*Function for computing edge weights*/

//may need to make changes depending on how we implement rip reroute stuff

void computeEdgeWeights(routingInst* rst, vector<int>& edgeOverflow, vector<int>& edgeWeight, vector<int>& edgeHistory){

	int temp = 0;

	for(int i = 0; i <= ((rst->numNets)-1); i++){
		for(int j=0; j <= ((rst->nets[i].nroute.numSegs)-1); j++){ //for each segment in the net
			for(int k=0; k <= ((rst->nets[i].nroute.segments[j].numEdges)-1); k++){ //for each edge in the segment
				//parst3: compute overflow for each edge in route
				temp = rst->nets[i].nroute.segments[j].edges[k]; //get edge number
				//edgeOverflow[temp-1] = rst->edgeUtils[temp-1] - rst->edgeCaps[temp-1];
				edgeOverflow.at(temp-1) = rst->edgeUtils[temp-1] - rst->edgeCaps[temp-1];
				if(edgeOverflow.at(temp-1) < 0){
					edgeOverflow.at(temp-1) = 0;
				}
				//part4: compute weight for each edge in route
				if(edgeOverflow.at(temp-1) > 0){ //compute edge history
					edgeHistory.at(temp-1) +=1;
				}
				edgeWeight.at(temp-1) = edgeOverflow.at(temp-1) * edgeHistory.at(temp-1);
				//part 5: total edge weights in route
				rst->nets[i].nroute.cost += edgeWeight.at(temp-1);
			}
		}
	}
}

//FIGURE BELOW OUT...
/*part 1 and part 2 might be done outside of the function, before we call the function
because we need previous iterations values. if so, simple fix*/
