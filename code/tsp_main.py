import os
# import networkx as nx
import numpy as np
from scipy.spatial.distance import squareform
from scipy.spatial.distance import pdist
import time
import argparse
import math
from itertools import islice
from decimal import Decimal, ROUND_HALF_UP
from itertools import islice, combinations

from IteratedLocalSearch import LS2
from MST_approx import mst_opt_app
from branch_and_bound import BNB
from Local_search_simulated_annealing import LS

_OUTPUT_DIR = os.path.join(os.getcwd(), 'output')

class NoSeedException(Exception):
    pass


def load_graph(filename):
    with open(filename, 'r') as f:
        data = []
        for line in islice(f, 5, None):
            if line.strip() == 'EOF': break
            data.append(np.array(list(map(float, line.split()[1:]))))

    distance = [int(np.sqrt(((u - v) ** 2).sum()) + 0.5) for u, v in combinations(data, 2)]
    return squareform(distance)

def run(filename, alg, cutoff=600, seed=None):
    if not os.path.exists(_OUTPUT_DIR):
        os.makedirs(_OUTPUT_DIR)

    dist = load_graph(filename)
    filename = os.path.split(filename)[1]
    start_time = time.time()

    if alg == 'BnB':
        bnb = BNB(dist,cutoff = cutoff)
        sol, trace = bnb.TSP()

        #  sol = [1234, [1,4,2,3,5]]
        # trace = [[3.45,102],[7.94,95]]

    elif alg == 'Approx':
        sol = mst_opt_app(dist)

    elif alg == 'LS1':
        local_search = LS(dist.tolist(), cutoff=cutoff, seed=seed)
        sol, trace = local_search.localsearch()

    elif alg == 'LS2':
        localsearch = LS2(dist.tolist(), cutoff=cutoff, seed=seed)
        sol, trace = localsearch.iteratedLocalSearch()

    total_time = time.time() - start_time
    print('Best solution found: {}, complete in {:.2f}s'.format(sol[0], total_time))

    solution_file = os.path.join(_OUTPUT_DIR,
        ('_'.join([filename.split('.')[0], alg, str(cutoff)])) +
        ('_{}'.format(seed) if seed is not None else '') + '.sol'
    )

    trace_file = _OUTPUT_DIR + '\\' + (
        '_'.join([filename.split('.')[0], alg, str(cutoff)]) +
        ('_{}'.format(seed) if seed is not None else '') + '.trace'
    )

    with open(solution_file, 'w') as f:
        f.write(str(sol[0]) + '\n')
        f.write(','.join(map(str, sol[1])) + '\n')

    if alg != 'Approx':
        with open(trace_file, 'w') as f:
            for timestamp, value in trace:
                f.write('{:.2f}, {}\n'.format(timestamp, value))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-inst', required=True, help='Name of datafile'
    )
    parser.add_argument(
        '-alg', required=True, choices=['BnB', 'Approx', 'LS1', 'LS2'],
        help='Algorithm options: [BnB|Approx|LS1|LS2]'
    )
    parser.add_argument(
        '-time', required=True, type=int, help='Cutoff time in seconds'
    )
    parser.add_argument(
        '-seed', required=False, type=int, help='Seed for random number generator'
    )
    args = parser.parse_args()

    if args.alg in ('LS1', 'LS2') and args.seed is None:
        raise NoSeedException('Random seed is required for local search')

    run(args.inst, args.alg, args.time, args.seed)


if __name__ == '__main__':
    main()
