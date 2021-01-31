#!/usr/bin/env python

from math import hypot
import unittest

class Route:
    def __init__(self, car_capacity=200):
        self.Q = car_capacity   # maximum capacity of the car
        self.D = []             # list of the delivery events
        self.P = []             # list of the pickup event
        # each event consists of [index, x-coord, y-coord, capacity]

    # loads the input data contained in the 'full_path' text file
    # data file schema - 4 columns: index, x-coord, y-coord, demand(capacity)
    # the first event (row) denotes the depot
    def load_data(self, full_path):
        result = []
        with open(full_path, 'r') as token:
            for line in token:
                res = line.strip().split('\t')
                index, x, y, demand = int(res[0]), int(res[1]), int(res[2]), int(res[3])
                result.append([index, x, y, demand])

        return result

    # method to construct MST of a graph represented by an adjacency matrix
    # Prim's algorithm is used
    # input: adjacency matrix
    def prim_MST(self, adj):
        # an auxiliary function to find the vertex with minimum distance value,
        # from the set of vertices not yet included in MST
        def get_min_key(key, MST_set):
            N = len(key)
            min_value = float('inf')
            min_index = -1

            for v in range(N):
                if key[v] < min_value and MST_set[v] == False:
                    min_value = key[v]
                    min_index = v

            return min_index

        # debug method to print the edges of the MST
        def print_MST_edges(parent):
            print ("Edge \tWeight")
            for i in range(1, N):
                print (parent[i], "-", i, "\t", adj[i][parent[i]])


        N = len(adj)
        keys = [float('inf')]*N
        parent = [None]*N  # parent of each vertex in the constructed MST
        parent[0] = -1
        keys[0] = 0   # mark as 0 the starting vertex
        MST_set = [False]*N
        graph = {}    # this is the output graph
        self.MST_dist = 0  # the total length of the minimum spanning tree for debugging purposes

        for _ in range(N):
            # find the minimum distance vertex in the unprocessed set
            u = get_min_key(keys, MST_set)
            p = parent[u]
            if p != -1:
                # add a vertex to the output graph
                self.MST_dist += adj[u][p]
                if p in graph:
                    graph[p].append(u)
                else:
                    graph[p] = [u]

            MST_set[u] = True
            # update dist value of the adjacent vertices only if the current distance
            # is greater than new distance and the vertex is not already in the MST
            for v in range(N):
                # 1) adj[u][v] is non zero only for adjacent vertices of u
                # 2) mstSet[v] is false for vertices not yet included in the MST
                # 3) update the key only if adj[u][v] is smaller than key[v]
                if adj[u][v] > 0 and MST_set[v] == False and keys[v] > adj[u][v]:
                    keys[v] = adj[u][v]
                    parent[v] = u


        #print('DEBUG: MST dist = ' + str(MST_dist))
        #print_MST_edges(parent)   # print all MST edges for debug
        return graph


    # helper function needed to compute Euclidean distances between events
    def dist(self, event1, event2):
        #return ((event1[1]-event2[1])**2 + (event1[2]-event2[2])**2)**0.5
        return hypot(event1[1]-event2[1], event1[2]-event2[2])

    # calculates the adjacency matrix; may be optimized since the matrix is symmetric
    def compute_adjacency(self, events, N):
        adjacency = [[0] * N for _ in range(N)]
        for i in range(N):
            for j in range(N):
                if i == j:
                    adjacency[i][j] = 0
                else:
                    adjacency[i][j] = self.dist(events[i], events[j])

        return adjacency

    # traverses the MST graph with preorder DFS to obtain a good route
    def generate_route(self, MST_graph):
        route = []

        def dfs(node):
            route.append(node)
            if node not in MST_graph:
                return
            for child in MST_graph[node]:
                dfs(child)

        dfs(0)
        route.append(0)  # return back to the depot to complete the route
        return route

    def solve(self):
        # step 1: determine the N delivery events that we will visit
        self.D.sort(key=lambda x:x[3])  # sort in-place!

        car_capacity = 0
        index = 0
        while car_capacity <= self.Q and index < len(self.D):
            car_capacity += self.D[index][3]
            index += 1

        car_capacity -= self.D[index-1][3]
        N = index - 1

        # step 2: for the N delivery events find the reasonably short route
        # calculate the adjacency matrix
        # the adjacency matrix is symmetric, which hints at potential future optimization
        adjacency = self.compute_adjacency(self.D, N)

        # generate a MST
        graph = self.prim_MST(adjacency)
        # traverse the MST graph with preorder DFS to obtain a good route
        route = self.generate_route(graph)


        # step #3: deal with the single pickup
        # find such a pickup event that would result in a minimum increase of route length!
        K = len(self.P)
        min_delta, delivery_index, pickup_index = float('inf'), -1, -1
        route_dist = 0
        # travel along the route
        for i in range(N-1):
            delivery_i = route[i]
            delivery_next = route[i + 1]
            d1 = adjacency[delivery_i][delivery_next]  # we already know this distance
            route_dist += d1

            for j in range(K):
                if self.P[j][3] > self.Q-car_capacity:
                    continue    # pickup is too big, skip

                d2 = self.dist(self.D[delivery_i], self.P[j])
                d3 = self.dist(self.D[delivery_next], self.P[j])
                delta = d2+d3-d1  # how much longer will this pickup make the route
                if delta < min_delta:
                    min_delta, delivery_index, pickup_index = delta, i, j

            # as we go along the route we unload the deliveries. the car becomes emptier
            car_capacity -= self.D[route[i]][3]


        # finally print the results
        print('Route generation complete!')
        print('Number of delivery events visited: ' + str(N-1))
        print('Route length: ' + str(route_dist))
        print('MST length: ' + str(self.MST_dist))
        print('--------------------------------')
        for index, delivery in enumerate(route):
            print(f'Delivery event {self.D[delivery][0]:d}. Coords: ({self.D[delivery][1]:d}, {self.D[delivery][2]:d})')
            if index == delivery_index:
                print(f'Pickup event {self.P[pickup_index][0]:d}. Coords: ({self.P[pickup_index][1]:d}, {self.P[pickup_index][2]:d})')

# a set of basic unit tests that have been used in debugging
# later coverage may be further improved
class TestRouteMethods(unittest.TestCase):

    def setUp(self):
        self.solver = Route()

        # a set of events that was used for development / debugging
        self.events1 = [[0, 2, 3, 0], [1, 0, 0, 0], [2, 10, 0, 0], [3, 4, 2, 0]]
        self.adj1 = self.solver.compute_adjacency(self.events1, 4)

        # a non-Euclidean graph
        self.adj2 = [[0, 28, 0, 0, 0, 10, 0],
                     [28, 0, 16, 0, 0, 0, 14],
                     [0, 16, 0, 12, 0, 0, 0],
                     [0, 0, 12, 0, 22, 0, 18],
                     [0, 0, 0, 22, 0, 25, 24],
                     [10, 0, 0, 0, 25, 0, 0],
                     [0, 14, 0, 18, 24, 0, 0]]

    def test_adjacency_matrix(self):
        # a rectangular graph with 3-4-5 right triangles gives integer edges
        events = [[0, 0, 0, 0], [1, 0, 3, 0], [2, 4, 3, 0], [3, 4, 0, 0]]
        adj = self.solver.compute_adjacency(events, 4)
        self.assertEqual(adj, [[0, 3, 5, 4], [3, 0, 4, 5], [5, 4, 0, 3], [4, 5, 3, 0]])

    def test_prim_MST(self):
        graph1 = self.solver.prim_MST(self.adj1)
        self.assertEqual(graph1, {0: [3, 1], 3: [2]})
        route1 = self.solver.generate_route(graph1)
        self.assertEqual(route1, [0, 3, 2, 1, 0])

        graph2 = self.solver.prim_MST(self.adj2)
        self.assertEqual(graph2, {0: [5], 5: [4], 4: [3], 3: [2], 2: [1], 1: [6]})
        route2 = self.solver.generate_route(graph2)
        self.assertEqual(route2, [0, 5, 4, 3, 2, 1, 6, 0])


if __name__ == "__main__":
    #unittest.main()    # uncomment to run tests

    solver = Route()
    solver.D = solver.load_data('delivery.dat')
    solver.P = solver.load_data('pickup.dat')
    solver.solve()