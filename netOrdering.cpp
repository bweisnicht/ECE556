/*
	ECE 556 Project
	Calculate Net Ordering
	Team OOPW
*/

#include "netOrdering.h"

// used in calling the qsort() library function
// takes care of the comparison between two nets
// return negative if a is before b
// return positive if a is after b
// return 0 otherwise
int compareNets(const void *a, const void *b)
{
	net netA = *(net*) a;
	net netB = *(net*) b;

	if (netA.nroute.cost > netB.nroute.cost)
	{
		return -1;
	}
	else if (netA.nroute.cost < netB.nroute.cost)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int vecCompareNets(net netA, net netB)
{	
	if (netA.nroute.cost > netB.nroute.cost)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

// Going to use qsort from the c library for now.  Might change this later to see if there is an improvement.
int orderNets(routingInst *rst)
{
	sort(rst->nets, rst->nets + rst->numNets, vecCompareNets);
	return EXIT_SUCCESS;
}

