# Solution to the test assignment

## General comments
I have not worked with the Vehicle Routing Problem (VRP) before. It’s a narrow area within a much broader field of optimization. The quality of the offered solution was limited by the time available. Given more time to examine VRP algorithms I believe I can improve this solution.

The proposed idea is based on the following assumptions. We have a hierarchical objective function:
1. Visit as many deliveries as possible (with 1 pickup)
2. Do the best to minimize the route\
The 2nd objective is a bit vague - find "reasonably short route”. I interpreted it as “try to minimize the route, but no need to get the optimal solution”.

The problem is further complicated by adding a capacity constraint: capacity of the car = *Q*.

## Input
Two input data files were generated from Gehring & Homberger benchmark link. There’s a *‘delivery.dat’* and *‘pickup.dat’*. Each contains 100 events. These files have the following schema: there are 4 columns: *index, x-coord, y-coord, demand (capacity)*.
Both data files have to be in the same folder with *‘Routing.py’*.

## Algorithm outline
For analysis purposes let *M = number of delivery events*.
1. Clearly the maximum number of deliveries is limited and defined by the capacity of the car *Q*.
Sort all the deliveries by capacity in ascending order. Consider the first *N* deliveries whose total capacity is <= *Q*. Let the first *N* such delivery events that fit in the car define a set *S*. Very likely *N << M*.\
This will guarantee that the primary objective function (maximize the number of deliveries) is fulfilled. Note, there can be many unique combinations / sets of delivery events such that their sum <= *Q* (such that they fit in the car). In this solution we are not searching for the optimal set of delivery events yielding the shortest route.\
Complexity: *O(M log M)* due to sorting.

2. Approximately solve the TSP for set *S* of delivery events to find the “reasonably short route”. Solving TSP exactly is not feasible. Therefore an approximate method has to be applied. The 2x accurate approach was chosen.
- Compute the Minimum Spanning Tree (MST) using Prim's algorithm
- Perform a preorder DFS on the MST and add the starting vertex (depot) at the end\
This approach gives a route which in the worst case is 2x the optimal shortest path.\
Complexity: *O(N^2)* due to Prim's algorithm working with the adjacency matrix.

3) Deal with the single pickup. Let *K = number of pickup events* \
Traverse the route produced in step (2). As we go along the route keep track of the capacity of the car. Deliveries are unloaded and the car becomes emptier, which gives more choices of pickup events that would fit in the car.\
At each delivery point we look at all *K* pickup events. 
We search for such a pickup event that would result in a minimum increase of the route length (route computed in step #2).\
At each delivery event loop through all *K* pickup events:
-	If capacity of the pickup event does NOT allow to pick it up, then continue to the next one
-	Else compute the increment in route length delta – by how much longer will the route become if we detour and do this pickup\
Find a pickup event with the minimum delta. It will be added to the output tour.\
Complexity: _O (N*K)_. Similar to step #2 it is also quadratic, so it can be done in reasonable time.
