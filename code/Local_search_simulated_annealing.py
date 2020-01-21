import sys
import numpy as np
import random
import math
from scipy.spatial.distance import squareform
from scipy.spatial.distance import pdist
import networkx as nx
import MST_approx as MST
import time

# args = sys.argv
# if len(args) != 2:
#     sys.exit("Usage: {} [input]".format(args[0]))
# in_file_path = sys.argv[1]
# file = open(in_file_path, "r")
# data = [line for line in file]
# size = len(data)
# data = data[5:size - 1]
# data = [list(map(float, i.split(' ')))[1:] for i in data]
# print(data)
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
class LS:
    def __init__(self, distance,cutoff,seed):
        self.res_path = [0] * (len(distance) + 1)
        self.cities_number = len(distance)
        self.visited = np.zeros(len(distance) + 1)
        self.final_dis = sys.maxsize
        self.dis = distance
        random.seed(seed);
        self.cutoff=cutoff
    def twonodes(self):
        random_node1 = random.randint(0,self.cities_number-1)
        random_node2 = random.randint(0,self.cities_number-1)
        while random_node1 == 0:
            random_node1 = random.randint(0,self.cities_number-1)
        while random_node2 == 0 or random_node1 == random_node2:
            random_node2 = random.randint(0,self.cities_number-1)
        if random_node1>random_node2:
            temp=random_node1
            random_node1=random_node2
            random_node2=temp
        return [random_node1, random_node2]

    def prob(self, number):
        return math.pow(1.1, -1 * number)

    def initial_nodes(self):
        city_list = []
        for i in range(self.cities_number):
            city_list.append(i)
        city_list = random.sample(city_list, self.cities_number)
        city_list.append(city_list[0])

        # city_list =MST.mst_app(self.dis)[1]
        # city_list.append(0)
        # print(city_list)
        return city_list

    def initial_value(self, city_list):
        temp_value = 0
        for i in range(self.cities_number):
            temp_value += self.dis[city_list[i]][city_list[i + 1]]
       # temp_value=MST.mst_app(dis)[0]
        return temp_value

    def localsearch(self):
        start=time.clock()
        it = 0
        exchange_number = 0
        cur_path = self.initial_nodes()
        cur_value = self.initial_value(cur_path)
        #print(self.dis)
        trace, best_value = [], sys.maxsize
        while time.clock()-start<self.cutoff:
            # print(it)
            it += 1
            nodes=self.twonodes()
            node1=nodes[0]
            node2=nodes[1]
            pos1=cur_path[node1]
            pos2=cur_path[node2]
            if node1-node2==-1:
                value_a=self.dis[pos1][cur_path[node2+1]]+self.dis[pos2][cur_path[node1-1]]
                value_b=self.dis[pos1][cur_path[node1-1]]+self.dis[pos2][cur_path[node2+1]]
            else:
                value_a = self.dis[pos1][cur_path[node2-1]] + self.dis[pos1][cur_path[node2+1]] + self.dis[pos2][cur_path[node1-1]] + self.dis[pos2][cur_path[node1+1]]
                value_b = self.dis[pos1][cur_path[node1-1]] + self.dis[pos1][cur_path[node1+1]] + self.dis[pos2][cur_path[node2-1]] + self.dis[pos2][cur_path[node2+1]]
            if value_a <= value_b:
                cur_path[node1] = pos2
                cur_path[node2] = pos1
                cur_value += value_a - value_b
                it=0
                if cur_value < best_value:
                    trace.append([time.clock()-start,cur_value])
                    best_value = cur_value
            elif self.prob(exchange_number) > random.random():
                exchange_number+=1
                #print(exchange_number)
                cur_path[node1] = pos2
                cur_path[node2] = pos1
                cur_value += value_a - value_b
                it=0

        self.res_path=cur_path
        self.final_dis=cur_value
        return [self.final_dis,self.res_path],trace

# dis = load_graph('F:\\Georgia Tech\\Courses\\CSE6140\\project\\DATA\\DATA\\UKansasState.tsp')[1];
# # print(MST.mst_app(dis)[1])
# # print(MST.mst_app(dis)[0])
# test = LS(dis.tolist(),20,4)
# print(test.dis)
# test.localsearch()
# # print(test.final_dis)
# # print(test.res_path)
# print(test.localsearch()[0])
