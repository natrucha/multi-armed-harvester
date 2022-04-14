import numpy as np
# import math
import sys

######## IMPORT MY OWN MODULES ########
# from trajectory import *           # import the trajectory time calc (bang-bang) 
# from plotStates_updated import *   # import module to plot % time each arm is in each state
# from fruit_distribution import *   # import module to create the various desired fruit distributions
# from IG_scheduling import *        # import module to perform interval graph scheduling similar to melon paper
# from IG_melon_scheduling import *  # import module to perform melon paper's exact interval graph scheduling
# from IG_data_analysis import *     # import module to analyze the data from the snapshots
from IG_single_run import *        # performs a single run of the desired scheduling algorithm
from IG_multi_run import *         # uses IG_single_run to perform multiple runs to perform data analysis  


def getRNGSeedList(n_runs):
        '''
           Open the random seed list rngseed_list_20200901.csv with 200 seeds for each of the 3 real fruit coordinate axis
           and 3 fake fruit coordinate axis.
        '''
        # keeps track of the row number of the csv being read (each row contains the seeds for one run)
        csv_i     = 0

        seed_list = list()

        with open('./rngseed_list_20200901.csv') as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
            for row in reader:
                seed_list.append(row)
                if csv_i == n_runs:
                    break

                csv_i += 1

        # print(seed_list)
        return(seed_list)
    

def main():
    print_out = 1 # determine if it should print out the results
    plot_out  = 1 # detemrine if it should plot the results

    n_runs = 1

    seed_list = getRNGSeedList(n_runs)
    print(seed_list)

    for run in range(n_runs):
        # get seeds for x, y, and z RNG (probably unneccessary right now, especially for x)
        seed = [seed_list[run][0], seed_list[run][1], seed_list[run][2]]

    # for the fruit distribution, want to keep it the same for these tests
    # x_seed = 37428395352013185889194479428694397783
    # y_seed = 13250124924871709375127216220749555998
    # z_seed = 165440185943501291848242755689690423219
    # seed             = [x_seed, y_seed, z_seed]

    ## set fruit distribution flag
    # 0     == Raj's digitized fruits
    # 1     == uniform random  (if algorithm == 1, use melon version)
    # 2     == uniform random, equal cell density
    # 3     == multiple densities separated by some space (only melon for now)
    # 4     == fruit in vertical columns
    set_distribution = 1  

    density          = 3
    Td               = 2
    v_vy             = .36
    n_arm            = 6
    n_row            = 1
    cell_l           = .3
    cell_h           = 2.
    horizon_l        = 0.
    travel_l         = 5 + (n_arm * cell_l)

    run_once = IG_single_run(print_out, seed, set_distribution, density, Td, v_vy, n_arm, n_row, cell_l, cell_h, horizon_l, travel_l)
    run_once.singleRun(plot_out)

if __name__ == '__main__':
    main()