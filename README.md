# TSP 

Tiantian Fu, Hanchen Wang, Xingjian Wang, Jing Shi

### Main modules within code
The main components of the code are as follows:

`tsp_main.py`:            main executable code for the following algorithms.

`branch_and_bound.py`:    implements Branch-and-Bound algorithm;

`MST_approx.py`:          implements MST Approximation algorithm;

`Local_search_simulated_annealing.py`: implements Local Search algorithm with simulated annealing;

`IteratedLocalSearch.py`: implements Iterated Local Search algorithm;

### How to run?

Navigate to the main folder and run the following command to run the algorithms:

`python tsp_main.py -inst <filename> -alg [BnB|Approx|LS1|LS2] -time <cutoff in seconds> -seed <random seed>`

in which `<filename>` refers to one of the datafiles found in the `Data` folder, `[BnB|Approx|LS1|LS2]` is one of the following algorithms: `BnB | Approx | LS1 | LS2`, `<cutoff in seconds>` is the cutoff time (in seconds), and `<random seed>` is the random seed used for the local search algorithms.


An example:

`python tsp_main.py -inst Atlanta.tsp -alg LS1 -time 600 -seed 3`


Output files will be written to a folder called `output` with the following format:

`<instance>_<algorithm>_<Cutoff>_<randomSeed>.sol`

and

`<instance>_<algorithm>_<Cutoff>_<randomSeed>.trace`

When the algorithm is deterministic (e.g., BnB), the random seed is set to None and the output filename formats are:

`<instance>_<algorithm>_<Cutoff>.sol`

and

`<instance>_<algorithm>_<Cutoff>.trace`
