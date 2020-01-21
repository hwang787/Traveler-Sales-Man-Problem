import sys
import numpy as np
from scipy.spatial.distance import squareform
from scipy.spatial.distance import pdist
import networkx as nx
import time
import random
import MST_approx as MST


def load_graph(filename):
    coordinates = []
    G = None  # empty graph

    file = open(filename, 'r')

    for i in range(0, 5):
        line = file.readline()
        if i == 2:
            n = int(line.strip().split(' ')[-1])

    G = nx.complete_graph(n)
    for line in file.readlines()[:n]:
        node_id, x, y = line.strip().split(' ')
        node_id = int(node_id) - 1
        G.node[node_id]['x'] = float(x)
        G.node[node_id]['y'] = float(y)
        coordinates.append([float(x), float(y)])
        coordinates_array = np.array(coordinates)
        dist_array = pdist(coordinates_array)
    for u, v, d in G.edges(data=True):
        d['weight'] = int(
            np.sqrt((G.node[v]['x'] - G.node[u]['x']) ** 2 + (G.node[v]['y'] - G.node[u]['y']) ** 2) + 0.5)
    return G, squareform(dist_array), len(coordinates)

class BNB:

    def __init__(self,distance,cutoff):
        self.res_path = [0] * (len(distance) + 1)
        self.cities_number = len(distance)
        self.visited = np.zeros(len(distance) + 1)
        self.final_dis = sys.maxsize
        self.dis = distance
        self.cutoff=cutoff

    def getFinal(self, cur_path):
        for i in range(self.cities_number):
            self.res_path[i] = cur_path[i]
        self.res_path[self.cities_number] = cur_path[0]

    def find_Min(self, city_k):
        temp_min = sys.maxsize
        for i in range(self.cities_number):
            if self.dis[city_k][i] < temp_min and city_k != i:
                temp_min = self.dis[city_k][i]
        return temp_min

    def find_secMin(self, city_k):
        temp_mini = sys.maxsize
        temp_sec = sys.maxsize
        for i in range(self.cities_number):
            if city_k == i:
                continue
            if self.dis[city_k][i] <= temp_mini:
                temp_sec = temp_mini
                temp_mini = self.dis[city_k][i]
            elif self.dis[city_k][i] <= temp_sec and self.dis[city_k][i] != temp_mini:
                temp_sec = self.dis[city_k][i]
        return temp_sec

    def TSP_Value(self, cur_bound, cur_weight, handing_number, cur_path,start,trace):
        if time.clock()-start>=self.cutoff:
            return
        if handing_number == self.cities_number:
            if self.dis[cur_path[handing_number - 1]][cur_path[0]] != 0:
                cur_res = cur_weight + self.dis[cur_path[handing_number - 1]][cur_path[0]]
                if cur_res < self.final_dis:
                    self.getFinal(cur_path)
                    self.final_dis = cur_res
                    trace.append([time.clock() - start, self.final_dis])
                    #print(self.final_dis)
            return
        l=[]
        for i in range(self.cities_number):
            l.append(i)
        random.shuffle(l)
        for i in l:
            if self.dis[cur_path[handing_number - 1]][i] != 0 and self.visited[i] == 0:
                temp_bound = cur_bound
                cur_weight += self.dis[cur_path[handing_number - 1]][i]
                if handing_number == 1:
                    cur_bound -= (self.find_Min(cur_path[handing_number - 1]) + self.find_Min(i)) / 2
                else:
                    cur_bound -= (self.find_secMin(cur_path[handing_number - 1]) + self.find_Min(i)) / 2
                if cur_bound + cur_weight < self.final_dis:
                    cur_path[handing_number] = i
                    self.visited[i] = 1
                    self.TSP_Value(cur_bound, cur_weight, handing_number + 1, cur_path,start,trace)
                cur_weight -= self.dis[cur_path[handing_number - 1]][i]
                cur_bound = temp_bound
                self.visited[self.visited > 0] = 0
                for j in range(handing_number):
                    self.visited[cur_path[j]] = 1

    def TSP(self):
        start=time.clock()
        trace=[]
        cur_path = [-1] * (self.cities_number + 1)
        cur_bound = 0
        for i in range(self.cities_number):
            cur_bound += self.find_Min(i) + self.find_secMin(i)
        if cur_bound == 1:
            cur_bound = cur_bound / 2 + 1
        else:
            cur_bound = cur_bound / 2
        self.visited[0] = 1
        cur_path[0] = 0
        mstuppert=MST.mst_opt_app(self.dis)[0]
        t=time.clock()-start
        #mstlow=MST.mst_app(self.dis)[0]/2
        temp=MST.mst_opt_app(self.dis)[1]
        temp.append(0)
        self.final_dis=mstuppert
        trace.append([t,mstuppert])
        self.TSP_Value(cur_bound, 0, 1, cur_path,start,trace)
        if len(self.res_path)==self.cities_number:
            return [self.final_dis, self.res_path],trace
        return [self.final_dis,temp],trace


