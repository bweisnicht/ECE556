// ECE556 - Copyright 2014 University of Wisconsin-Madison.  All Rights Reserved.

#include "ece556.h"
//#include "lees.h"
#include "aStar.h"
#include "netOrdering.h"
#include "computeEdgeWeights.h"
#include <sys/time.h>
#include <cstdio>
#include <cstdlib>
#include <vector>

using std::vector;

int shouldContinue(timeval startTime)
{
  timeval currTime;
  gettimeofday(&currTime, NULL);

  if (currTime.tv_sec - startTime.tv_sec > MAX_TIME)
  {
    return false;
  }
  else
  {
	return true;
  }

  // TODO more complex termination evaluation
}

int main(int argc, char **argv)
{
  // Store the time at start of execution
  timeval startTime;
  gettimeofday(&startTime, NULL);

 	if(argc!=5)
  {
 		printf("Usage : ./ROUTE.exe <input_benchmark_name> <output_file_name> \n");
 		return 1;
 	}

  int status;
  int netDecompEnabled;
  int netOrderingEnabled;
  int bestCost = 0;
  int firstRun = 1;
  int noChange = 0;
  char *inputFileName = argv[3];
  char *outputFileName = argv[4];
  vector<int> edgeOverflow;
  vector<int> edgeWeight;
  vector<int> edgeHistory;
 	
 	/// create a new routing instance
 	routingInst *rst = new routingInst;
  
  if(sscanf(argv[1], "-d=%d", &netDecompEnabled) != 1)
  {
    printf("WARNING: Incorrect argument for net decomposition.\n");
    return 1; 
  }

  if(sscanf(argv[2], "-n=%d", &netOrderingEnabled) != 1)
  {
    printf("WARNING: Incorrect argument for net ordering.\n");
    return 1;
  }

/*
  int *edgeOverflow = (int*)malloc(rst->numEdges * sizeof(int));
  if (edgeOverflow==NULL)
  {
      printf("Failed to allocate memory for edgeOverflow.");
  }

  int *edgeWeight = (int*)malloc(rst->numEdges * sizeof(int));
  if (edgeWeight==NULL)
  {
      printf("Failed to allocate memory for edgeWeight.");
  }

  int *edgeHistory = (int*)malloc(rst->numEdges * sizeof(int));
  if (edgeHistory==NULL)
  {
      printf("Failed to allocate memory for edgeHistory.");
  }
*/
  

 	

  /* initializes routingInst pointers to null */
  rst->edgeCaps = NULL;
  rst->edgeUtils = NULL;
  rst->nets = NULL;

    
	
 	/// read benchmark
 	status = readBenchmark(inputFileName, rst);
 	if(status==EXIT_FAILURE){
 		printf("ERROR: reading input file \n");
 		return 1;
 	}

  //status = solveRoutingLees(rst);
  //if( status == EXIT_FAILURE )
  //{
    //printf( "ERROR: lees routing\n");
    //return 1;
  //}
  //return 0;

		status = solveRouting(rst);
   //status = solveRoutingAstar(rst);
   	if(status==EXIT_FAILURE){
   		printf("ERROR: running routing \n");
   		release(rst);
   		return 1;
   	}

  if (netDecompEnabled)
  {
    /* decompose nets */
    netDecompose(rst);
  }

  int vecSize = rst->numEdges;

  edgeOverflow.resize(vecSize);
  edgeWeight.resize(vecSize);
  edgeHistory.resize(vecSize);;

  for (int i = 0; i < rst->numEdges; i++)
  {
    edgeOverflow.at(i) = 0;
    edgeWeight.at(i) = 0;
    edgeHistory.at(i) = 0;
  }
 
  int temp = 0;
  // Calculate edge utilization after inital solution
  for(int i=0; i < ((rst->numNets)-1); i++){ //for each net in routing instance
    for(int j=0; j < ((rst->nets[i].nroute.numSegs)-1); j++){ //for each segment in the net
      for(int k=0; k < ((rst->nets[i].nroute.segments[j].numEdges)-1); k++){ //for each edge in the segment
        temp = rst->nets[i].nroute.segments[j].edges[k]; //get edge number
        rst->edgeUtils[temp-1] += 1; //update edge utilization
      }
    }
  }
int iterationNum = 0;

  if((netOrderingEnabled == 1) || (netDecompEnabled == 1)){ //if d=0 and n=0 from cmd line, skip RRR
	  while (shouldContinue(startTime) || (noChange < 1))
	  {
		printf("RRR iteration %d\n", iterationNum);
		int totalCost = 0;
	    // compute total edge cost for each net
	    computeEdgeWeights(rst, edgeOverflow, edgeWeight, edgeHistory);
	    
	    releaseSegsAndEdges(rst); // TODO possibly need to update this depending on how a* allocates memory 

	    if (netOrderingEnabled)
	    {
	      orderNets(rst);
	  	}

	    /*printRoutingInst(*rst); */
	   	/// run actual routing
	   	//status = solveRoutingLees(rst);
			status = solveRoutingAstar(rst);
	   	if(status==EXIT_FAILURE){
	   		printf("ERROR: running routing \n");
	   		release(rst);
	   		return 1;
	   	}

		//calculate total cost of the routing instance
		for(int i = 0; i < rst->numNets; i++){
			totalCost += rst->nets[i].nroute.cost;
		}

		if(firstRun == 1){
			bestCost = totalCost;
			firstRun = 0;
		}

printf("totalCost = %d\nbestCost = %d\n", totalCost, bestCost);

		if(bestCost <= totalCost){
			noChange += 1;
		}

		//test if routing instance cost is better than our best instances cost.
		//if it is better, write it out. also write it out if it is the first
		//run of the while loop
		if((totalCost < bestCost) || (firstRun == 1)){
			/// write the result
			bestCost = totalCost;
			noChange = 0;
			status = writeOutput(outputFileName, rst);
			if(status==EXIT_FAILURE){
				printf("ERROR: writing the result \n");
				release(rst);
				return 1;
			}
		}
iterationNum += 1;
	  }
  }	

	release(rst);
	delete rst;

  //free(edgeOverflow);
  //free(edgeWeight);
  //free(edgeHistory);
  
 	printf("\nDONE!\n");	

 	return 0;
}
