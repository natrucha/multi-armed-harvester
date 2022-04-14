import csv                      # read and write CSV file-type
import numpy as np
from scipy.stats import poisson
import math
import sys

######## IMPORT MY OWN MODULES ########
# from trajectory import *           # import the trajectory time calc (bang-bang) 
# from plotStates_updated import *   # import module to plot % time each arm is in each state
# from fruit_distribution import *   # import module to create the various desired fruit distributions
# from IG_scheduling import *        # import module to perform interval graph scheduling similar to melon paper
# from IG_melon_scheduling import *  # import module to perform melon paper's exact interval graph scheduling
# from IG_data_analysis import *     # import module to analyze the data from the snapshots
from IG_single_run import *        # import module that runs the scheduling algorithm once

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


def singleRun(seed, set_distribution, density, Td, v_vy, n_arm, n_row, cell_l, cell_h, horizon_l, travel_l):
    '''Run the simulation once with given variables and parameters'''
    print_out = 0  # don't want to print out multiple of the 
    plot_out  = 0

    run_once = IG_single_run(print_out, seed, set_distribution, density, Td, v_vy, n_arm, n_row, cell_l, cell_h, horizon_l, travel_l)
    [real_FPE, real_FPT] = run_once.singleRun(plot_out)

    return([real_FPE, real_FPT])


def calcAvgHarvestRatio(density, cell_l, cell_h, v_vy, Td, n_arm):
    '''
        Calculate the expected harvest ratio of a multi-armed, 3DoF robotic fruit harvester
        given the number of arms, fruit density, row width, frame length, forward velocity, and
        handling time (melon combinatorial paper Eq.13 modified for multi-arm)
    '''
    alpha = 0.85                          # correction factor, empirically chosen
    # the average number of fruits that lie within an interval for multi-armed harvesters
    l     = density*cell_h * (v_vy*Td - cell_l/2) 
    K     = n_arm                         # number of arms
    
    # for poisson ditribution, https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.poisson.html
    # F(K-1, lambda), Poisson cumulative distribution function 
    F     = poisson.cdf(K-1, l)  
    # f(K-1, lambda), Poisson density function 
    f     = poisson.pmf(K-1, l) 

    # calculate the expected harvest ratio of a multi-armed, 3DoF robotic fruit harvester
    p = (1-alpha*K/l)*F + alpha*K/l - alpha*f

    return(p*100)


def printRunSettings(n_row, n_arm, Td, vehicle_l, cell_l, horizon_l, travel_l, density):
    print('Number of rows:', n_row, 'number of arms:', n_arm)
    print('Fruit handling time:', Td, 'sec')
        
    print()
    print('Vehicle length:  {0:.2f}'.format(vehicle_l), 'm, with cell length:', cell_l, 'm')
    print('Horizon length:', horizon_l, 'm')
    print('Step length:', travel_l , 'm')
    print()
    print('Fruit density:', density, 'fruit/m^2')
    print()


def monteCarlo_numRun(print_out, n_runs, seed_list, set_distribution, density, Td, v_vy, n_arm, n_row, cell_l, cell_h, vehicle_l, horizon_l, travel_l):
    '''Calculate the average FPE and FPT of a set of constant settings with n_run number of seeds equal to number of runs'''
    run_FPE = np.zeros(n_runs)
    run_FPT = np.zeros(n_runs)

    plot_out = 0

    if print_out == 1:
        print('Running Monte Carlo with parameters:')
        print('-------------------------------------')  
        print('Vehicle velocity set at', v_vy, 'm/s')
        print()
        printRunSettings(n_row, n_arm, Td, vehicle_l, cell_l, horizon_l, travel_l, density)

    # use current seed
    for run in range(n_runs):
        # get seeds for x, y, and z RNG (probably unneccessary right now, especially for x)
        seed = [seed_list[run][0], seed_list[run][1], seed_list[run][2]]
        run_results = singleRun(seed, set_distribution, density, Td, v_vy, n_arm, n_row, cell_l, cell_h, horizon_l, travel_l)
        run_FPE[run] =  run_results[0]
        run_FPT[run] =  run_results[1]

    # obtain the expected harvest ratio (one value for a set of settings)
    p = calcAvgHarvestRatio(density, cell_l, cell_h, v_vy, Td, n_arm)

    if print_out == 1:
        print('-------------------------------------')
        print('For', n_runs, 'runs:')
        print('The mean FPE is {0:.2f}'.format(np.mean(run_FPE)), '% +/- {0:.2f}'.format(np.std(run_FPE)), '%')
        print('The mean FPT is {0:.2f}'.format(np.mean(run_FPT)), 'fruit/s +/- {0:.2f}'.format(np.std(run_FPT)), 'fruit/s')
        print('The mean expected harvest ratio is {0:.2f}'.format(p))
        ('------------------------------------------')
        print()

    return([run_FPE, run_FPT, p])


def multiV_vy(print_out, plot_out, v_vy_list, n_runs, seed_list, set_distribution, density, Td, n_row, n_arm, cell_l, cell_h, vehicle_l, horizon_l, travel_l):
    '''View effects of vehicle velocity on FPE and FPT'''

    print('Testing the effects of vehicle velocity on FPE and FPT')
    ('-------------------------------------')
    print('Testing', len(v_vy_list), 'velocities')
    print('ranging from', v_vy_list[0], 'to', v_vy_list[-1])
    print()
    printRunSettings(n_row, n_arm, Td, vehicle_l, cell_l, horizon_l, travel_l, density)

    run_FPE = np.zeros(len(v_vy_list))
    run_FPT = np.zeros(len(v_vy_list))
    run_p   = np.zeros(len(v_vy_list))

    for index, v_vy in enumerate(v_vy_list):
        [multi_fpe, multi_fpt, multi_p] = monteCarlo_numRun(print_out, n_runs, seed_list, set_distribution, density, Td, v_vy, n_arm, n_row, cell_l, cell_h, vehicle_l, horizon_l, travel_l)
        # run_results = singleRun(seed, distribution, density, Td, v_vy, n_arm, n_row, cell_l, cell_h, horizon_l, travel_l)

        # take the average of the monte carlo run results
        run_FPE[index] =  np.mean(multi_fpe)
        run_FPT[index] =  np.mean(multi_fpt)
        run_p[index]   =  multi_p

    fig, ax = plt.subplots(2, 1)
    # want subplots to stack in columns so they can be compared, see
    # see https://matplotlib.org/stable/gallery/lines_bars_and_markers/cohere.html#sphx-glr-gallery-lines-bars-and-markers-cohere-py
    ax[0].plot(v_vy_list, run_FPE, 'xr', label='simulation')
    # ax[0].plot(v_vy_list, run_p, color='k', label='expected')
    # axs[0].set_xlabel('time')
    ax[0].set_ylabel('FPE [%]')
    ax[0].grid(True)
    legend = ax[0].legend(bbox_to_anchor=(1.1, 1),loc='upper right')

    # title_string = 'Harvest ratio, density = ' + str(density) + 'fruit/m^2, T_d = ' + str(Td) + 's, and K = ' + str(n_arm) 
    # axs[0].title(title_string)

    ax[1].plot(v_vy_list, run_FPT, color='c')
    ax[1].set_ylabel('FPT [fruit/s]')
    ax[1].grid(True)

    ax[1].set_xlabel('Vehicle velocity [m/s]')

    fig.tight_layout()
    
    file_name = './plots/range_velocities.png'
    print('Saving plot of FPT and FPE vs vehicle velocity', file_name)
    plt.savefig(file_name,dpi=300)


def findBest(print_out, plot_out, v_vy_list, n_runs, seed_list, set_distribution, density, Td, n_row, n_arm, cell_l, cell_h, vehicle_l, horizon_l, travel_l, min_FPE):
    '''Loop thorugh v_vy values to find the best FPE and FPT results'''

    print('Testing the effects of vehicle velocity on FPE and FPT')
    ('-------------------------------------')
    print('Testing', len(v_vy_list), 'velocities')
    print('ranging from', v_vy_list[0], 'to', v_vy_list[-1])
    print()
    printRunSettings(n_row, n_arm, Td, vehicle_l, cell_l, horizon_l, travel_l, density)

    best_FPE  = 0
    best_FPT  = 0
    best_v_vy = 0

    for index, v_vy in enumerate(v_vy_list):
        [multi_fpe, multi_fpt, multi_p] = monteCarlo_numRun(print_out, n_runs, seed_list, set_distribution, density, Td, v_vy, n_arm, n_row, cell_l, cell_h, vehicle_l, horizon_l, travel_l)
        # run_results = singleRun(seed, distribution, density, Td, v_vy, n_arm, n_row, cell_l, cell_h, horizon_l, travel_l)

        # take the average of the monte carlo run results
        curr_FPE =  np.mean(multi_fpe)
        curr_FPT =  np.mean(multi_fpt)

        if curr_FPE >= min_FPE:
            if curr_FPT > best_FPT:
                # the v_vy has resulted in better overall FPE and FPT so choose it as the current 'best'
                best_FPE = curr_FPE
                best_FPT = curr_FPT
                best_v_vy = v_vy


    print('chosen velocity {:.3f} m/s'.format(best_v_vy)) 
    print('FPE, {:.3f}%'.format(best_FPE), ', and FPT {:.3f}'.format(best_FPT))

    

def main():
    args = sys.argv[1:] # use the command line arguments to set values
    ## command line arguments
    # args[0] == n_arm
    # args[1] == n_fruit
    # args[2] == n_run

    ## set the run's objective  
    # 0     == multiple runs of the same variables with varying RNG seeds (Monte Carlo)
    # 1     == effect of vehicle velocity on FPE and FPT
    # 2     == loop through v_vy values to get best FPE and FPT combo
    set_objective = 2

    ## set fruit distribution flag
    # 0     == Raj's digitized fruits
    # 1     == uniform random  (if algorithm == 1, use melon version)
    # 2     == uniform random, equal cell density
    # 3     == multiple densities separated by some space (only melon for now)
    # 4     == fruit in vertical columns
    # 5     == fruit in a line with uniform distance between fruit
    # 6     == fruit randomly placed in a line along the y-axis
    set_distribution = 6

    n_fruit = int(args[1])  # total number of fruits in the orchard row (assuming total knowledge of fruits in row)

    density   = 3   # in fruit/m^2
    Td        = 2   # in s

    FPE_min = 95.

    n_arm     = int(args[0])  # number of arms in a horizontal row
    n_row     = 1        # number of arms in each vertical column

    cell_l    = 0.3 # in m
    cell_h    = 2 / n_row  # in m
    horizon_l = 0.0 # in m

    single_arm_vvy = 17  # in cm/s, the chosen velocity if there is only one arm (to help set upper bound)

    v_vy_lb_cmps = math.ceil(cell_l / Td * 100)   # in cm/s
    v_vy_ub_cmps = (single_arm_vvy * n_arm) + 10  # in cm/s, platform max velocity: 90

    vehicle_l = cell_l * n_arm  # in m, hard coded number of arms for now, based on melon paper

    n_runs = int(args[2])  # number of times simulation is run 
    # obtain a the seed list 
    seed_list = getRNGSeedList(n_runs)

    if set_distribution == 0:
        travel_l  = 12 + vehicle_l # in m
        density   = 48.167         # in fruit/m^2 (on avg.), constant
        n_runs    = 1

    elif set_distribution == 1:
        travel_l  = 5 + vehicle_l # in m, was 20
    
    elif set_distribution == 3:
        travel_l  = 30 + vehicle_l # in m

    elif set_distribution == 5 or set_distribution == 6:
        dy = math.floor(n_fruit / density) # in m, the distance needed to have the right density and right number of fruit
        travel_l  = dy + vehicle_l # in m

    else: 
        travel_l  = 10 + vehicle_l # in m  


    if set_objective == 0:
        v_vy      = 0.36 # in m/s, set velocity 

        print_out = 1 # used to decide to print settings and results
        plot_out  = 1 # used to decide to plot results or not

        ## get avg fpe and fpt for one set of settings over n_runs number of runs
        monteCarlo_numRun(print_out, n_runs, seed_list, set_distribution, density, Td, v_vy, n_arm, n_row, cell_l, cell_h, vehicle_l, horizon_l, travel_l)

    elif set_objective == 1:
        # create array of desired velocities to test
        # v_vy_list = np.linspace(v_vy_lb_cmps, v_vy_ub_cmps, 100, endpoint=True)
        v_vy_list = np.arange(v_vy_lb_cmps, v_vy_ub_cmps +1) /100 # in m/s

        print_out = 0 # used to decide to print settings and results
        plot_out  = 1 # used to decide to plot results or not

        # research effects of v_vy on FPE and FPT
        multiV_vy(print_out, plot_out, v_vy_list, n_runs, seed_list, set_distribution, density, Td, n_row, n_arm, cell_l, cell_h, vehicle_l, horizon_l, travel_l)

    elif set_objective == 2:
        # create array of desired velocities to test
        # v_vy_list = np.linspace(v_vy_lb_cmps, v_vy_ub_cmps, 100, endpoint=True)
        v_vy_list = np.arange(v_vy_lb_cmps, v_vy_ub_cmps +1) /100 # in m/s

        print_out = 1 # used to decide to print settings and results
        plot_out  = 1 # used to decide to plot results or not

        # research effects of v_vy on FPE and FPT
        findBest(print_out, plot_out, v_vy_list, n_runs, seed_list, set_distribution, density, Td, n_row, n_arm, cell_l, cell_h, vehicle_l, horizon_l, travel_l, FPE_min)



if __name__ == '__main__':
    main()