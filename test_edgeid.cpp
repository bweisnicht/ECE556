#include "ece556.h"

int main(int argc, char **argv)
{

 	if(argc!=3){
 		printf("Usage : ./ROUTE.exe <input_benchmark_name> <output_file_name> \n");
 		return 1;
 	}

  int i, j, k;
 	int status;
	char *inputFileName = argv[1];
 	char *outputFileName = argv[2];

 	/// create a new routing instance
 	routingInst *rst = new routingInst;

  /* initializes routingInst pointers to null */
  rst->edgeCaps = NULL;
  rst->edgeUtils = NULL;
  rst->nets = NULL;
  
  
	
 	/// read benchmark
 	status = readBenchmark(inputFileName, rst);
 	if(status==0){
 		printf("ERROR: reading input file \n");
 		return 1;
 	}

  point pt1, pt2;
  int edgeID;
  /* testing pts -> edgeid conversion */
  for(i = 0; i < rst->gx ; i++)
  {
    for(j = 0; j < rst->gy; j++)
    {
      /* at each point, test edge to right, and edge above */
      pt1 = (point) { i, j };
      pt2 = (point) { i+1, j};

      printf("Pts of edge:\n(%d,%d)-(%d,%d)\n",pt1.x, pt1.y, pt2.x, pt2.y);
      edgeID=  getEdgeID(rst, pt1.x, pt1.y, pt2.x, pt2.y);
      printf("EdgeID: %d\n", edgeID);
      /* at each point, test edge to right, and edge above */
      pt2 = (point) { i, j+1};

      printf("Pts of edge:\n(%d,%d)-(%d,%d)\n",pt1.x, pt1.y, pt2.x, pt2.y);
      edgeID=  getEdgeID(rst, pt1.x, pt1.y, pt2.x, pt2.y);
      printf("EdgeID: %d\n", edgeID);
    }
  }


  /* testing edgeid -> pts conversion */
  printf("Grid size: %d x %d\n", rst->gx, rst->gy);
  for(i = 1; i <= ( (rst->gx - 1)*rst->gy + (rst->gy - 1)*rst->gx ); i++)
  {
    printf( "EdgeID: %d\n", i);
    getEdgePts(rst, i, &pt1, &pt2);
    printf( "-(%d, %d) to (%d, %d)\n", pt1.x, pt1.y, pt2.x, pt2.y);
  }


  
 	release(rst);
  delete rst;
  
 	printf("\nDONE!\n");	

 	return 0;
}
  

