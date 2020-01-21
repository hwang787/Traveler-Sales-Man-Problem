#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 25 17:47:24 2019

@author: hanchen
"""
import networkx as nx
import numpy as np
from scipy.spatial.distance import squareform
from scipy.spatial.distance import pdist

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
    # fill the edges with EUC2D weights

def get_cost(path,dist):
    summ=0
    for i in range(len(path)):
        summ+=dist[path[i-1]][path[i]]
    return summ