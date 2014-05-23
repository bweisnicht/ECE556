#include "ece556.h"
// #include "aStar.h"
#include "netOrdering.h"
#include "computeEdgeWeights.h"
// #include "quicksort.h"
#include <sys/time.h>
#include <cstdio>
#include <cstdlib>
#include <vector>

using std::vector;

int main(){

	routingInst *rst = new routingInst;
	rst->numNets = 5;
	rst->nets = (net*)malloc( (rst->numNets) * sizeof(net) );
	
	rst->nets[0].id = 0;
	rst->nets[1].id = 1;
	rst->nets[2].id = 2;
	rst->nets[3].id = 3;
	rst->nets[4].id = 4;
	rst->nets[0].nroute.cost = 1;
	rst->nets[1].nroute.cost = 6;
	rst->nets[2].nroute.cost = 10;
	rst->nets[3].nroute.cost = 20;
	rst->nets[4].nroute.cost = 3;
	
	for(int i = 0; i < rst->numNets; i++){
		printf("Net %d Cost: %d\n", rst->nets[i].id, rst->nets[i].nroute.cost);
	}

	orderNets(rst);

	printf("Net ordering after orderNets...\n");
	
	for(int i = 0; i < rst->numNets; i++){
		printf("Net %d Cost: %d\n", rst->nets[i].id, rst->nets[i].nroute.cost);
	}
	
	return 1;
}
