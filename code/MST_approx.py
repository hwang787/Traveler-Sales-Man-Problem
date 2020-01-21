#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 25 16:05:19 2019

@author: hanchen
"""
import random
import sys
from collections import defaultdict
import networkx as nx
import numpy as np
from scipy.spatial.distance import squareform
from scipy.spatial.distance import pdist
from dataParser import *

#MST approximation basically find a TSP path using DFS from MST in the graph

# first find a MST
#reference : https://www.geeksforgeeks.org/prims-minimum-spanning-tree-mst-greedy-algo-5/
#reference : https://www.geeksforgeeks.org/depth-first-search-or-dfs-for-a-graph/
def get_cost(path,dist):
    if len(path)<len(dist) :
        return sys.maxsize
    summ=0
    for i in range(len(path)):
        summ+=dist[path[i-1]][path[i]]
    return summ

def load_graph(filename):
    coordinates=[]
    G = None    # empty graph

    file = open(filename, 'r')

    for i in range(0, 5):
        line = file.readline()
        if i == 2:

            n = int(line.strip().split(' ')[-1])

    G = nx.complete_graph(n)
    for line in file.readlines()[:n]:
        node_id, x, y = line.strip().split(' ')
        node_id = int(node_id)-1
        G.node[node_id]['x'] = float(x)
        G.node[node_id]['y'] = float(y)
        coordinates.append([float(x),float(y)])
        coordinates_array = np.array(coordinates)
        dist_array = pdist(coordinates_array)
    for u, v, d in G.edges(data=True):
        d['weight'] = int(np.sqrt((G.node[v]['x'] - G.node[u]['x'])**2 + (G.node[v]['y'] - G.node[u]['y'])**2)+0.5)
    return G, squareform(dist_array),len(coordinates)

class MST(object):

    # this is the constructor
    def __init__(self):

        # default dictionary to  graph
        self.graph = defaultdict(list)
        self.path = []

    # add an edge to graph
    def addEdge(self, u, v):
        self.graph[u].append(v)

    # A helper function
    def DFSUtil(self, v, visited):

        # Mark the current node as visited
        visited[v] = True
        self.path.append(v)
        for i in self.graph[v]:
            if visited[i] == False:
                self.DFSUtil(i, visited)

    # The function to do DFS traversal.
    def DFS(self, v):

        # Mark all the vertices as not visited
        visited = [False] * (len(self.graph))

        # Call the recursive helper function
        self.DFSUtil(v, visited)

class Frontier(object):
    def __init__(self):
        self.queue = []

    def __str__(self):
        return ' '.join([str(i) for i in self.queue])

    # for checking if the queue is empty
    def isEmpty(self):
        return len(self.queue) == []

    # for inserting an element in the queue
    def insert(self, problem):
        self.queue.append(problem)

    # for popping an element based on Priority
    def delete(self):
        try:
            min = 0
            for i in range(len(self.queue)):
                if self.queue[i].lowerBound > self.queue[min].lowerBound:
                    min = i
            Subproblem = self.queue[min]
            del self.queue[min]
            return Subproblem
        except IndexError:
            print()
            exit()
#reference : https://www.geeksforgeeks.org/prims-minimum-spanning-tree-mst-greedy-algo-5/
#reference : https://www.geeksforgeeks.org/depth-first-search-or-dfs-for-a-graph/
# construct a graph
class Graph():

    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for column in range(vertices)]
                    for row in range(vertices)]
        self.m = MST()

    # A utility function to print the constructed MST stored in parent[]
    def printMST(self, parent):

        for i in range(1, self.V):
            #print (parent[i], "-", i, "\t", self.graph[i][ parent[i] ])
            self.m.addEdge(parent[i],i)
            self.m.addEdge(i,parent[i])

    def minKey(self, key, mstSet):

        # Initilaize min value
        min = sys.maxsize
        min_index =0

        for v in range(self.V):
            if key[v] < min and mstSet[v] == False:
                min = key[v]
                min_index = v

        return min_index

    # Function to construct and print MST for a graph represented using adjacency matrix representation
    def primMST(self):

        # Key values used to pick minimum weight edge in cut
        key = [sys.maxsize] * self.V
        parent = [None] * self.V # Array to store constructed MST
        # Make key 0 so that this vertex is picked as first vertex
        key[0] = 0
        mstSet = [False] * self.V

        parent[0] = -1 # First node is always the root of

        for cout in range(self.V):

            # Pick the minimum distance vertex from  the set of vertices not yet processed.
            u = self.minKey(key, mstSet)

            # Put the minimum distance vertex in the shortest path tree
            mstSet[u] = True

            # Update dist value of the adjacent vertices
            for v in range(self.V):
                # graph[u][v] is non zero only for adjacent vertices of m

                if self.graph[u][v] > 0 and mstSet[v] == False and key[v] > self.graph[u][v]:
                    # mstSet[v] is false for vertices not yet included in MST

                        key[v] = self.graph[u][v]
                        parent[v] = u
                        # Update the key only if graph[u][v] is smaller than key[v]
        self.printMST(parent)
        #self.printMST(parent)

class problem(object):
    def __init__(self,dist,subsolution,cost):
        self.lowerBound = 9999
        self.subsolution = subsolution
        self.dist=dist
        self.cost=cost
    def updateLowerBound(self,newBound):
        self.lowerBound = int(newBound)

    def exapnd(self):
        #do not contain certain edge
        #set certain part of the dist to be infinity
        return [problem(dist,subsolution,cost),problem(dist,subsolution,cost)]


def mst_app(dist):
    g = Graph(len(dist.tolist()))
    g.graph = dist.tolist()
#print(dist.tolist())
    g.primMST()
    #g.m.DFS(len(dist)-1)
    g.m.DFS(random.randint(0,len(dist)-1))
    return [int(get_cost(g.m.path,dist)),g.m.path]

def mst_any_app(dist,x):
    g = Graph(len(dist))
    g.graph = dist
#print(dist.tolist())
    g.primMST()
    g.m.DFS(x)
    return [int(get_cost(g.m.path,dist)),g.m.path]

def mst_opt_app(dist):
    cost = sys.maxsize
    solution = []
    for i in range(len(dist)):
        new_cost = mst_any_app(dist,i)[0]
        new_solution = mst_any_app(dist,i)[1]
        if cost > new_cost:
            cost = new_cost
            solution = new_solution

    return [cost,solution]
