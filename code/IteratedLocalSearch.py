import random
import math
import argparse
import time


class LS2:
    def __init__(self, distance_matrix, cutoff, seed):
        self.alg = "LS2"
        self.start_time = 0  # I should initialize start time when I run the local search method
        self.trace = []
        self.distance_matrix = distance_matrix
        self.cities_cnt = len(self.distance_matrix)
        self.final_path = None
        self.final_cost = 0
        self.cutoff = cutoff
        random.seed(seed)

    @staticmethod
    def roundNumber(number):
        return math.floor(number + 0.5)  # https://realpython.com/python-rounding/#rounding-half-up

    def computeTotalDistance(self, path):
        total_distance = 0.
        for i in range(1, len(path)):
            total_distance += self.distance_matrix[path[i-1]][path[i]]
        return int(total_distance)

    @staticmethod
    def constructInitialPath(N):
        """
        construct an initial path through array shuffle
        :param N: the length of the array
        :return: an initial path
        """
        path = list(range(1, N))
        # permute the array
        random.shuffle(path)
        return [0] + path + [0]

    @staticmethod
    def twoOptSwap(path, i, j):
        """
        Return a new path after 2-opt-swap: https://en.wikipedia.org/wiki/2-opt
        :param j: one location to swap
        :param i: another location to swap
        :param path: the travel order
        :return: a new path
        """
        if i == 0 or i >= len(path) or j >= len(path):
            return
        return path[:i] + path[j:i-1:-1] + path[j+1:]

    # Random choose i and j
    def localSearch(self, path):
        """
        returns the best path find using 2-opt-swap
        """
        iteration = 0
        dup_count = 0
        max_dup_count = self.cities_cnt ** 2
        final_distance = self.computeTotalDistance(path)
        prev_distance = final_distance * 100

        while dup_count < max_dup_count and time.time() - self.start_time < self.cutoff:
            if final_distance == prev_distance:
                dup_count += 1
            else:
                dup_count = 0
            prev_distance = final_distance
            iteration += 1

            while True:
                i, j = sorted(random.choices(list(range(1, len(path) - 1)), k=2))
                if j - i > 1:
                    break

            cur_path = LS2.twoOptSwap(path, i, j)
            cur_distance = self.computeTotalDistance(cur_path)

            # add the new improved version to the trace
            if cur_distance < self.final_cost:
                self.final_path = cur_path
                self.final_cost = cur_distance
                t = round(time.time() - self.start_time, 2)
                self.trace.append([t, cur_distance])

            # find a better solution than given path
            if cur_distance < final_distance:
                final_distance = cur_distance
                path = cur_path

        return path, final_distance

    @staticmethod
    def doubleBridge(path, i, j, k):
        """
        perturbation: double-bridge move, select 3 locations and break the path into 4 parts
        :return: the new path after perturbation
        """
        A = path[0:i+1]
        B = path[i+1:j+1]
        C = path[j+1:k+1]
        D = path[k+1:-1]
        return A + D + C + B + [0]

    # Random move
    @staticmethod
    def perturbation(path):
        while True:
            i, j, k = sorted(random.choices(list(range(1, len(path)-1)), k=3))
            if j - i > 1 and k - j > 1:
                break
        return LS2.doubleBridge(path, i, j, k)

    def iteratedLocalSearch(self):
        """
        The main function to do iterated local search
        :return: the best found path and cost
        """
        self.start_time = time.time()

        # - Construct an initial solution
        self.final_path = LS2.constructInitialPath(self.cities_cnt)
        self.final_cost = self.computeTotalDistance(self.final_path)
        t = round(time.time() - self.start_time, 2)
        self.trace.append((t, self.final_cost))

        # do local search on initial solution
        self.localSearch(self.final_path)

        # Do iterations
        iteration = 0
        prev_final_cost = self.final_cost * 100
        dup_count = 0
        max_dup_count = self.cities_cnt ** 2

        while time.time() - self.start_time < self.cutoff and dup_count < max_dup_count:
            iteration += 1
            if prev_final_cost == self.final_cost:
                dup_count += 1
            else:
                dup_count = 0
            prev_final_cost = self.final_cost
            final_path_prime = LS2.perturbation(self.final_path)
            self.localSearch(final_path_prime)

        return [self.final_cost, self.final_path], self.trace

    def writeSolution(self, out_file_path):
        with open(out_file_path, "w") as file:
            file.write(str(self.final_cost))
            file.write("\n")
            final_path = list(map(str, self.final_path))
            file.write(",".join(final_path))

    def writeSolutionTrace(self, out_file_path):
        trace = []
        for record in self.trace:
            trace.append(str(record[0]) + "," + str(record[1]) + "\n")
        with open(out_file_path, "w") as file:
            file.writelines(trace)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-inst", help="input file name")
    parser.add_argument("-alg", help="Algorithm to use: BnB | Approx | LS1 | LS2")
    parser.add_argument("-time", help="cut off in seconds")
    parser.add_argument("-seed", help="cut off in seconds")
    args = parser.parse_args()
    in_file_path = args.inst
    # hardcode for test
    alg = "LS2"
    time_cutoff = 60
    # alg = args.alg
    # time_cutoff = float(args.time)
    # if alg == "LS1" or alg == "LS2":
    #     seed = float(args.seed)

    # Read data from file
    file = open(in_file_path, "r")
    data = [line.strip() for line in file]
    start = 0
    end = len(data)
    for i in range(len(data)):
        if "NODE_COORD_SECTION" in data[i]:
            start = i + 1
        if "EOF" in data[i]:
            end = i
            break
    data = data[start:end]
    #print("Number of cities in this file: ", len(data))
    data = [list(map(float, i.split(' ')))[1:] for i in data]
    # print(data)
    final_cost_sum = 0
    run_count = 1
    for i in range(run_count):
        seed = random.randint(0, 1000)
        # Run the iterated local search algorithm
        test = LS2(data, time_cutoff, seed)
        final_path, final_cost = test.iteratedLocalSearch()
        final_cost_sum += final_cost
    #print(">>>>>>Average distance: ", final_cost_sum / run_count)
        # Write output file
        # if alg == "LS1" or alg == "LS2":
        #     out_file_path = "_".join([in_file_path.split('/')[-1][:-4], alg, str(time_cutoff), str(seed)])
        # else:
        #     out_file_path = "_".join([in_file_path.split('/')[-1][:-4], alg, str(time_cutoff)])
        # print(out_file_path)

        # out_solution_path = out_file_path + ".sol"
        # test.writeSolution(out_solution_path)
        # out_solution_trace_path = out_file_path + ".trace"
        # test.writeSolutionTrace(out_solution_trace_path)
