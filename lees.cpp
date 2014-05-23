//#############################################################################
//
//		ECE 556
//		Team OOPW
//
//		Function implementing Lee's Routing Algorithm
//
//#############################################################################
#include "lees.h"

//needed in both initialization and getNeighbors functions and 
//don't feel like passing them around. these describe the bounding box
static int BBleftX;
static int BBrightX;
static int BBtopY;
static int BBbottomY;
/* used to compare elements in the open set */

bool compareNodeCost( const LeesNode* n1, const LeesNode* n2 ) 
{
  //returns true if e2 has less cost than e1
  return n1->cost < n2->cost;
}

int solveRoutingLees(routingInst *rst)
{
  gy = rst->gy;
	// Iterate through all nets
	for (int i = 0; i < rst->numNets; i++)
	{
    // printf( "Routing net %d\n", i);
		rst->nets[i].nroute.numSegs = rst->nets[i].numPins - 1;
		rst->nets[i].nroute.segments = (segment*)malloc( (rst->nets[i].numPins-1) * sizeof(segment) );

		// Route each segment
		for (int j = 0; j < rst->nets[i].numPins - 1; j++)
		{
			int retVal = routeLees(rst, i, j, j+1);
			if (retVal == EXIT_FAILURE)
			{
				fprintf(stderr, "Failed to route net at index %d\n", i);
				return EXIT_FAILURE;
			}
		}
	}
	return EXIT_SUCCESS;
}


int routeLees(routingInst* rst, int netInd, int SpinInd, int TpinInd )
{
  //set based on 
	set<LeesNode*, CompareNodeCost> group2; // TODO set up comparisons
  unordered_map<point, LeesNode*, Hash> m_group2;

  //attempting to use unordered maps for groups1 and 3 
  //since we don't need to sort them
  unordered_map<point, LeesNode* , Hash>group1;
  unordered_map<point, LeesNode* , Hash>group3;
  
  
  //vector<LeesNode*> group2;
	vector<LeesNode*> neighbors;

	// need to keep the current node
	LeesNode*  nS;
  LeesNode* nT;
  LeesNode *curr_node;

  //dynamically allocate so free functions are easy
  nS = new LeesNode;
  nT = new LeesNode;
	// Initializa starting node
	nS->loc = rst->nets[netInd].pins[SpinInd];
	nS->parent = NULL;
	nS->cost = 0;			       // cost is 0 for the starting node
	nS->edgeCap = rst->cap; 	 // no predecessor, but use default edge capacity 
	nS->edgeUtil = 0;  		   // no pred, but 0 will make it the lowest cost node

	// Put the starting node in group3
	//group3.push_back(nS);
  group3.insert(std::make_pair( (point)nS->loc, (LeesNode*)nS));

  // Initialize ending node
  nT->loc = rst->nets[netInd].pins[TpinInd];
  nT->cost = INT_MAX;
  nT->parent = NULL;
  nT->edgeCap = rst->cap;
  nT->edgeUtil = 0;

  //reserve memory now to hopefully avoid having to reallocate and 
  //copy elements later on
  //assuming we use a bounding box relatively close to the 
  //box made by S and T, this is pretty much the area of that box
  int dx = nT->loc.x - nS->loc.x;
  int dy = nT->loc.y - nS->loc.y;
  int approxElems = abs( dx ) + abs( dy);
  group1.reserve( approxElems );
  group2.reserve( approxElems );
  m_group2.reserve(approxElems);
  group3.reserve( approxElems );

  
  // Insert the final node into the unvisited nodes set
  group1.insert(std::make_pair((point)nT->loc, (LeesNode*)nT));

	// Set the current node to the starting node
	curr_node = nS;

  // create a LeesNode in group1 for all other points
  // now that we are using a bounding box, only allocate for nodes
  // in the box. first need to determine the relative positions of nS and nT

  if( dx > 0 )
  {
    //nS is left of nT
    BBleftX = nS->loc.x - BBoffset;
    BBrightX = nT->loc.x + BBoffset;
  }else
  {
    //nT is left of nS, or are in a vertical line
    BBleftX = nT->loc.x - BBoffset;
    BBrightX = nS->loc.x + BBoffset;
  }

  if( dy > 0 )
  {
    //nS is below nT
    BBbottomY= nS->loc.y - BBoffset;
    BBtopY = nT->loc.y + BBoffset;
  }else
  {
    BBbottomY = nT->loc.y - BBoffset;
    BBtopY = nS->loc.y + BBoffset;
  }

  //need to make sure BBleftX, BBrightX, BBbottomY, BBtopY are all inside total area
  if( BBleftX < 0 )
    BBleftX = 0;
  if( BBrightX >= rst->gx )
    BBrightX = rst->gx - 1;
  if( BBbottomY < 0 )
    BBbottomY = 0;
  if( BBtopY >= rst->gy )
    BBtopY = rst->gy - 1;

  for (int i = BBleftX; i <= BBrightX; i++)
  {
    for( int j = BBbottomY; j <= BBtopY; j++)
    {
      point pt;
      pt.x = i;
      pt.y = j;
      if (nS->loc != pt && nT->loc != pt )
      {
        LeesNode* n = new LeesNode;
        n->loc = pt;
        n->cost = INT_MAX;
        n->parent = NULL;
        n->edgeCap = rst->cap;
        n->edgeUtil = 0;
        group1.insert(std::make_pair((point)n->loc,(LeesNode*)n));    // Insert the new node into the unvisited nodes set
      }
    }
  }

	while (*curr_node != *nT)
	{
    // remove the previous neighbors
		neighbors.clear();

    // find the neighbors for the current node
		getNeighbors(rst, neighbors, curr_node,group1, group2, m_group2, group3 );  //TODO update call

    
    // visit each of the neighbors
		for (int i = 0; i < neighbors.size(); i++)
		{
      
      auto it = group3.find( neighbors[i]->loc );
			// shortest path is already known for this node
			if (it != group3.end())
			{
				continue;
			}

			// calculate the cost for following nodes
      int edgeID = getEdgeID(rst, curr_node->loc.x, curr_node->loc.y, 
                        neighbors[i]->loc.x, neighbors[i]->loc.y );
      
			int trial_cost = curr_node->cost + 1 + ((rst->edgeUtils[edgeID-1] - rst->edgeCaps[edgeID-1] > 0) ? rst->edgeUtils[edgeID-1] - rst->edgeCaps[edgeID-1] : 0);

      it = group1.find(neighbors[i]->loc);
      // if the node has not been visited add it to the visited group
			if (it != group1.end())
			{
				(it->second)->cost = trial_cost;
				(it->second)->parent = curr_node;
				group2.push_back(it->second);
				group1.erase(it);
			}else
      {
      // if the node has already been visited, check if the a better path has been found
        auto it2 = findNodeAtLocation(group2,m_group2, neighbors[i]);
        if (trial_cost < neighbors[i]->cost)
        {
          (*it2)->cost = trial_cost;
          (*it2)->parent = curr_node;
        }
      }
		}


    //sort group2 by cost to get lowest cost at front
    std::sort( group2.begin(), group2.end(), compareNodeCost);
    vector<LeesNode*>::iterator currIt = group2.begin();
    if(currIt == group2.end() )
    {
      printf("no more nodes in group2 to pull, couldn't find route.\n");
      return EXIT_FAILURE;
    }
		curr_node = (*currIt);

    // lowest cost route now found for curr_node so move to group3
		group3.insert(std::make_pair((point)curr_node->loc, (LeesNode*)curr_node));
		group2.erase(currIt);
	}

  int retVal = retrace(rst, *nS, curr_node, netInd, SpinInd  );
  //need to free memory originally allocated to group1, will
  //now be spread across the three groups.  must go through all groups
  //and delete all nodes remaining
  deleteMap( group1 );
  deleteGroup( group2 );
  deleteMap( group3 );

  return retVal;

}

// TODO check if all stuff is initialized as needed
void getNeighbors(routingInst *rst, vector<LeesNode*>& neighbors, LeesNode* current,
                    unordered_map<point, LeesNode*, Hash>& group1,
                    set<LeesNode*, CompareNodeCost>& group2,
                    unordered_map<point, LeesNode*>& m_group2,
                    unordered_map<point, LeesNode*, Hash>& group3)
{
  LeesNode temp;
  std::set<LeesNode*,CompareNodeCost>::iterator foundNode;
  int leftX, rightX, topY, botY;
  leftX = current->loc.x - 1;
  rightX = current->loc.x + 1;
  topY = current->loc.y + 1;
  botY = current->loc.y - 1;

  //the calculated leftX, rightX, topY, botY are now 
  //checked against the bounding box locations
  if( leftX < BBleftX )
    leftX = BBleftX;
  if( rightX > BBrightX )
    rightX = BBrightX;
  if( topY > BBtopY )
    topY = BBtopY;
  if( botY < BBbottomY )
    botY = BBbottomY;

  if( leftX >= 0 )
  {
    temp.loc = current->loc;
    temp.loc.x = leftX;
    auto fnode = group3.find(temp.loc);
    if (fnode!= group3.end())
    {
      neighbors.push_back(fnode->second);
    }
    else
    {
      foundNode = findNodeAtLocation(group2, m_group2, &temp);
      if (foundNode!= group2.end())
      {
        neighbors.push_back(*foundNode);
      }
      else
      {
        fnode = group1.find(temp.loc);
        if (fnode!=group1.end())
        {
          neighbors.push_back(fnode->second);
        }
      }
    }    
  } 

  if( rightX < rst->gx )
  {
    temp.loc = current->loc;
    temp.loc.x = rightX;
    auto fnode = group3.find(temp.loc);
    if (fnode!= group3.end())
    {
      neighbors.push_back(fnode->second);
    }
    else
    {
      foundNode = findNodeAtLocation(group2,m_group2,&temp);
      if (foundNode!= group2.end())
      {
        neighbors.push_back(*foundNode);
      }
      else
      {
        fnode = group1.find(temp.loc);
        if (fnode!=group1.end())
        {
          neighbors.push_back(fnode->second);
        }
      }
    }    
  }

  if( topY < rst->gy )
  {
    temp.loc = current->loc;
    temp.loc.y = topY;
    auto fnode = group3.find(temp.loc);
    if (fnode!= group3.end())
    {
      neighbors.push_back(fnode->second);
    }
    else
    {
      foundNode = findNodeAtLocation(group2, m_group2, &temp);
      if (foundNode!= group2.end())
      {
        neighbors.push_back(*foundNode);
      }
      else
      {
        fnode = group1.find(temp.loc);
        if (fnode!=group1.end())
        {
          neighbors.push_back(fnode->second);
        }
      }
    }    
  }

  if( botY >= 0 )
  {
    temp.loc = current->loc;
    temp.loc.y = botY;
    auto fnode = group3.find(temp.loc);
    if (fnode!= group3.end())
    {
      neighbors.push_back(fnode->second);
    }
    else
    {
      foundNode = findNodeAtLocation(group2, m_group2, &temp);
      if (foundNode!= group2.end())
      {
        neighbors.push_back(*foundNode);
      }
      else
      {
        fnode = group1.find(temp.loc);
        if (fnode!=group1.end())
        {
          neighbors.push_back(fnode->second);
        }
      }
    }    
  }
}

vector<LeesNode*>::iterator findNodeAtLocation(vector<LeesNode*>& group, LeesNode* searchNode)
{
  vector<LeesNode*>::iterator it = group.begin();
  while( it != group.end() )
  {
    if( (*it)->loc == searchNode->loc)
      return it;
    it++;
  }
  return it;
}


set<LeesNode*, CompareNodeCost>::iterator findNodeAtLocation(set<LeesNode*, CompareNodeCost>& group,
  unordered_map<point, LeesNode*, Hash> m_group2, LeesNode* searchNode)
{
  set<LeesNode*, CompareNodeCost>::iterator it = group.begin();
  while( it != group.end() )
  {
    if( (*it)->loc == searchNode->loc)
      return it;
    it++;
  }
  return it;
}
// TODO rework retrace to work with my data
int retrace(routingInst *rst, LeesNode& nS, LeesNode* current, int netInd, int segInd) 
{
  LeesNode *tmp = current;
  

  /* number of edges is simply the length from the starting node */
  rst->nets[netInd].nroute.segments[segInd].p2 = current->loc;
  rst->nets[netInd].nroute.segments[segInd].p1 = nS.loc;

  //printf("\nstart loc: (%d, %d)\tend loc: (%d, %d)\n",
  //          nS.loc.x, nS.loc.y, current->loc.x, current->loc.y );

  //path distance not stored, so must retrace route once first
  int numEdges = 0;
  while( tmp->parent != NULL )
  {
    tmp = tmp->parent;
    numEdges++;
  }
  tmp = current;

  
  rst->nets[netInd].nroute.segments[segInd].numEdges = numEdges;
  rst->nets[netInd].nroute.segments[segInd].edges = 
    (int*)malloc(sizeof(int) * numEdges );

  if(rst->nets[netInd].nroute.segments[segInd].edges == NULL)
  {
    fprintf(stderr, "Couldn't malloc for segments[segInd].edges pointer.\n");
    return EXIT_FAILURE;
  }

  int i  = 0;
  while( tmp->parent != NULL )
  {
    //printf("loc: (%d,%d)\tparent addr: %x\n", 
     //       tmp->loc.x, tmp->loc.y,  tmp->parent);

    int edgeID = getEdgeID( rst, tmp->loc.x, tmp->loc.y, 
        (*tmp->parent).loc.x, (*tmp->parent).loc.y );

    rst->nets[netInd].nroute.segments[segInd].edges[i] = edgeID;

    /* update edge utilization.. assumes array has been
     * initialized with all zeros */
    rst->edgeUtils[edgeID-1]++;

    /* get previous node */
    tmp =  tmp->parent;
    i++;
  }

  return EXIT_SUCCESS;
}

void deleteGroup( vector<LeesNode*>& group )
{
  for( auto it = group.begin(); it != group.end(); it++ )
  {
    //free element
    delete (*it);
  }
}

void deleteMap( unordered_map<point, LeesNode*, Hash>& group )
{
  for( auto it = group.begin(); it != group.end(); it++ )
  {
    //free element
    delete (it->second);
  }
}

