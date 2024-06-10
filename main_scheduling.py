# Main code to run multi-arm havester scheduling code 
# Copyright (C) 2024  Natalie C. Pueyo Svoboda

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>. 
# 

import csv
import math
import numpy as np
from datetime import datetime
from datetime import date
# from datetime import timedelta
from pathlib import Path
import sys

# from fruit_distribution import *   # import module to create the various desired fruit distributions 
from data_analysis import *       # import module to analyze the data from the snapshots
from MIP_algorithm import *          # import module that solves the extended MIP in melon algorith *add citation*
from MIP_queu_manager import *       # import module that adds queue management to the scheduler to add dynamisism
from fruit_handler import *          # import module that handles the data creation and calculations (distribution, calcPCT, etc.)
from FCFS_algorithm import *


# tested with Python 3.7.0 & Gurobi 9.0

### Run MIP solver on muliple datasets to get average and variance on the results. Runs once on the data set (no horizon) and containes cluster finding functions. 


def printScen(scenStr):
    sLen = len(scenStr)
    print("\n" + "*"*sLen + "\n" + scenStr + "\n" + "*"*sLen + "\n")

# function to display floats in numpy arrays to 3 decimals of precision (does not affect computation)
float_formatter = "{:.3f}".format

def analysisMultiple(set_algorithm, set_distribution, set_edges, n_runs, n_snapshots_array, run_list, time_list, print_out, log_out):
    '''Obtain average and variance of results and time-to-run for all the runs'''

    n_snapshots = np.max(n_snapshots_array) # use the max value of snapshots to build the data analysis arrays and avoid index errors
    print('each run has this many possible snapshots (max of all of them)', n_snapshots_array)

    # checks if any of the runs had zero fruits which were skipped and should not be used
    use_indexes = np.where(n_snapshots_array > 0)
    print('indexes that should be used', use_indexes[0])
    n_runs = len(use_indexes[0]) # real number of runs if we're skipping empty runs
    print(f'the number of usable runs is %d' %n_runs)

    # arrays to save the final FPE and FPT values of each run to get avgs and std
    FPE_global_array = np.zeros(n_runs)
    FPT_global_array = np.zeros(n_runs)

    C_array           = np.zeros([n_runs, n_snapshots])
    R_array           = np.zeros([n_runs, n_snapshots])
    N_array           = np.zeros([n_runs, n_snapshots])
    N_deadspace_array = np.zeros([n_runs, n_snapshots])
    time_array        = np.zeros([n_runs, n_snapshots])
    FPE_array         = np.zeros([n_runs, n_snapshots])
    FPT_array         = np.zeros([n_runs, n_snapshots])
    FPEavg_array      = np.zeros([n_runs, n_snapshots])
    FPTavg_array      = np.zeros([n_runs, n_snapshots])
    Td_array          = np.zeros([n_runs, n_snapshots])      
    # snapshot_density = np.zeros([n_runs, n_snapshots]) # in fruits/m^2
    v_vy_cmps_array   = np.zeros([n_runs, n_snapshots]) # save the velocity in cm/s used in each snapshot
   
    # v_vy_cmps = int(run_list[0][0].v_vy * 100)
    # print('the run\'s v_vy', v_vy_cmps)

    print()
    print('----------------------------------------------')
    print('  Analyze results for all runs and snapshots  ')
    print('----------------------------------------------')
    print()

    for i_run in range(n_runs):
        run_nonempty_index = use_indexes[0][i_run]
        # print(f'index assuming only using nonempty runs %d' %run_nonempty_index)
        # run up to the number of snapshots in the run
        for i_snap in range(n_snapshots_array[run_nonempty_index]):
            # print(f'i_snap value for i_run %d is %d' %(run_nonempty_index,i_snap))
            # print('Currently analyzing i_run %d and i_snap %d' % (i_run, i_snap))
            time_array[i_run,i_snap]        = time_list[run_nonempty_index][i_snap].total_seconds()
            # run list is made up of snapshot lists
            C_array[i_run,i_snap]           = run_list[run_nonempty_index][i_snap].n_col
            R_array[i_run,i_snap]           = run_list[run_nonempty_index][i_snap].n_row
            N_array[i_run,i_snap]           = run_list[run_nonempty_index][i_snap].N_snap
            N_deadspace_array[i_run,i_snap] = run_list[run_nonempty_index][i_snap].N_deadspace
            FPE_array[i_run,i_snap]         = run_list[run_nonempty_index][i_snap].FPE_global * 100
            FPEavg_array[i_run,i_snap]      = run_list[run_nonempty_index][i_snap].FPEavg * 100
            FPT_array[i_run,i_snap]         = run_list[run_nonempty_index][i_snap].FPT_global
            FPTavg_array[i_run,i_snap]      = run_list[run_nonempty_index][i_snap].FPTavg
            v_vy_cmps_array[i_run,i_snap]   = run_list[run_nonempty_index][i_snap].v_vy * 100 # in cm/s
            Td_array[i_run,i_snap]          = run_list[run_nonempty_index][i_snap].Td # mean handling time

            # density_rows = np.average(snapshot_cell[i_snap][0], axis=1)
            # snapshot_density[i_run,i_snap] = np.average(snapshot_cell[i_snap][0])

            # print('average snapshot density:', snapshot_density[i_run,i_snap])
            # print('row densities:\n', density_rows)
            # print('max row density:', max_row_density[i_run,i_snap])
            # print('min row density:', min_row_density[i_run,i_snap], '\n')

        n_snap_array_index = n_snapshots_array[run_nonempty_index]-1
        print('this run\'s n_snap_array_index', n_snap_array_index)

        avg_time_run = np.average(time_array[i_run, :n_snap_array_index])
        std_time_run = np.std(time_array[i_run, :n_snap_array_index])
        
        avg_FPE_run = np.average(FPEavg_array[i_run, :n_snap_array_index])
        std_FPE_run = np.std(FPEavg_array[i_run, :n_snap_array_index])
        
        avg_FPT_run = np.average(FPTavg_array[i_run, :n_snap_array_index])
        std_FPT_run = np.std(FPTavg_array[i_run, :n_snap_array_index])
          
        FPE_global_array[i_run] = FPE_array[i_run, n_snap_array_index]
        FPT_global_array[i_run] = FPT_array[i_run, n_snap_array_index]

        if print_out > 0:
            # print out this run's snapshot average results
            print('Run', run_nonempty_index, 'results over the run\'s snapshots:')
            print('------------------------------------------------')
            print(f'number of columns: %d \nnumber of row: %d' %(C_array[i_run,0], R_array[i_run,0]))
            print('average time to solve per snapshot, {:.3f}'.format(avg_time_run), '+/-{:.3f} s'.format(std_time_run)) 
            print('\n                   Global value                  Average snapshot value')
            print(f' FPE (percent):       %3.3f                        %3.3f +/- %3.3f' % (FPE_array[i_run, n_snap_array_index], avg_FPE_run, std_FPE_run))
            print(f' FPT (fruit/s):        %3.3f                         %3.3f +/- %3.3f\n\n' % (FPT_array[i_run, n_snap_array_index], avg_FPT_run, std_FPT_run)) 

    if n_runs > 1:
        print()
        print('----------------------------------------------')
        print('  Final mean FPE and FPT values for all runs  ')
        print('----------------------------------------------')
        print()
        # print out the overall average results assuming there was more than one run
        avg_time = np.average(time_array)
        std_time = np.std(time_array)
        print('average time to solve a snapshot over every run, {:.3f}'.format(avg_time), ' {:.3f} s'.format(std_time)) 

        avg_global_FPE = np.average(FPE_global_array)
        std_global_FPE = np.std(FPE_global_array)
        print('average global FPE for every run, {:.3f}'.format(avg_global_FPE), ' {:.3f} %'.format(std_global_FPE))  

        avg_global_FPT = np.average(FPT_global_array)
        std_global_FPT = np.std(FPT_global_array)
        print('average global FPT for every run, {:.3f}'.format(avg_global_FPT), ' {:.3f} fruits/s'.format(std_global_FPT))  
        print() 

    #######################################################################################
    # save the velocity, fpe, fpt, and density of the runs (maybe add horizon later) in cvs
    # to do regression and find out if it can be used to set correct velocity
    if log_out == 1:
        # date object of today's date
        today = date.today() 
        # print("Current year:", today.year)
        # print("Current month:", today.month)
        # print("Current day:", today.day)
        # print()

        if today.month < 10:
            month = str(0) + str(today.month)
        else:
            month = str(today.month)

        date_today = str(today.year) + month + str(today.day)
        # date_today = '20220815' # if hardcoded is necessary

        algo = 'goal'

        if set_algorithm == 3:
            algo = 'fcfs'
        elif set_algorithm == 1 and set_algorithm == 4:
            algo = 'goal'
        elif set_algorithm == 2 and set_algorithm == 5:
            algo = 'epsilon'

        if set_distribution < 2:
            # using the 2022 orchard data
            file_name = './csv_files/' + date_today + '_' + algo + '_2022_data'
        elif set_distribution == 2 or set_distribution == 4:
            # use raj's original dataset
            file_name = './csv_files/' + date_today + '_' + algo + '_apple_data_raj'
        elif set_distribution == 3 or set_distribution == 5:
            # use juan's original dataset
            file_name = './csv_files/' + date_today + '_' + algo + '_2021_data_juan'    
        else:
            # using RNG to create artificial distributions
            file_name = './csv_files/' + date_today + '_' + algo + '_RNG_test'

        num_checks = 20        # how many times it'll check if the file name exists 
        file_base = file_name  # save the base of the file name

        ## set file save type distribution flag
            # 0     == new csv file
            # 1     == append to csv file
        if set_distribution == 1:
            # when breaking up a dataset into smaller sets, append each result to an existing file
            set_file_save = 1
        else:
            # if using full datasets, each one should be saved in it's onwn file
            set_file_save = 0


        if set_file_save == 0:
            file_mode = 'w' # we are writing to a file
            # add check to see if the file already exists, if so, add a value at the end
            for i_check in range(num_checks):
                # do the check around 10 times, after which just put up an error message that the last file was overwritten
                check_file = Path(file_name + '.csv')
                # run 10 checks after which just send up an error message 
                if check_file.is_file():
                    # file exists so add an extension to the name
                    file_name = file_base + '[' + str(i_check) + ']'
                    if i_check == num_checks - 1:
                        # no more checks, the first file will be overwritten
                        print('WARNING: Files will be overwritten after this run.')
                        file_name = file_base + 'OVERWRITE'
                else:
                    # file doesn't exist so can use this name
                    # file_full = file_name + '.csv'
                    break

        elif set_file_save == 1:
            if set_edges == 0:
                # z-bounds set so all row heights are equal
                bounds_by = '_height_CR'
            elif set_edges == 1:
                #z-bounds set so all rows have equal number of fruits
                bounds_by = '_fruit_CR'

            v_file_max = np.amax(v_vy_cmps_array)
            v_file_min = np.amin(v_vy_cmps_array)

            if v_file_max != v_file_min:
                v_file = '_Vbest'
            else:
                v_file = '_V' + str(int(v_file_max))

            file_name = file_base + bounds_by + str(int(C_array[0,0])) + str(int(R_array[0,0])) + v_file # save new name so it's seperate from the new file/run set of CSVs
            # append the newest data to the csv (make sure you know what file you are working with!!!)
            check_file = Path(file_name + '.csv')
            # run 10 checks after which just send up an error message 
            if check_file.is_file():
                # already exists, so append to the file
                file_mode = 'a' # we are appending to a file
            else: 
                # doesn't exist, so create and write into the file
                file_mode = 'w' # we are writing to a new file

        file_full = file_name + '.csv'

        # print('file name:', file_full)
        # print()

        with open(file_full, mode=file_mode) as wr_file:
            wr = csv.writer(wr_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            
            for i_run in range(n_runs):
                run_nonempty_index = use_indexes[0][i_run]
                for i_snap in range(n_snapshots_array[run_nonempty_index]):
                    row_data = [run_nonempty_index, i_snap, n_snapshots_array[run_nonempty_index], N_array[i_run,i_snap], N_deadspace_array[i_run,i_snap], C_array[i_run,0], R_array[i_run,0], v_vy_cmps_array[i_run,i_snap], time_array[i_run,i_snap], FPE_array[i_run,i_snap], FPEavg_array[i_run,i_snap], FPT_array[i_run,i_snap], FPTavg_array[i_run,i_snap], Td_array[i_run,i_snap]]    

                    wr.writerow(row_data)
                time_array[i_run,:]
            print('CSV written into', file_full)


def main():
    args = sys.argv[1:] # use the command line arguments to set values

    # print all numpy array floats to 3 decimal precision
    np.set_printoptions(formatter={'float_kind':float_formatter})
     
    ## command line arguments
    # args[0] == n_col
    # args[1] == n_row
    # args[2] == n_runs
    # args[3] == v_vy_lb or the single speed being tested in cmps
    # args[4] == v_vy_ub (set equal to v_vy_lb if only wanting to test one velocity)
    # args[5] == algorithm
        # 0 == extended TOPTW MIP model with objective to maximize the number of harvested fruit, takes one velocity or a range of velocities to determine best FPE vs. FPT
        # 1 == TOPTW MIP model with the objective to maximize FPE*FPT, includes slack variables, minFPE, and minFPT
        # 2 == makespan TOPTW MIP model with the objective to minimize makespan, includes slack variable and minFPE
        # 3 == FCFS
        # 4 == FCFS to find speed lower bound, FPE*FPT to find schedule
        # 5 == FCFS to find speed lower bound, makespan to find schedule
        # 6 == SPT
    # args[6] == distribution
        # 0 == Raj's digitized fruits (data for the right side)
        # 1 == uniform random  (if algorithm == 1, use melon version)
        # 2 == uniform random, equal cell density
        # 3 == multiple densities separated by some space (only melon for now)
        # 4 == fruit in vertical columns
        # 5 == "melon" version of columns (user inputs desired no. fruits, z height, and distance between fruit in y-coord)
        # 6 == reduced Raj's digitized fruits; can reduce the density to a desired value 
        # 7 == Juan's digitized fruits
        # 8 == reduced Juan's digitized fruits; can reduce the density to a desired value 
    # args[7] == set_MPC
        # 0 == not MPC (for now, FPEmin = 0.35)
        # 1 == yes MPC (for now, FPEmin = 0.95)

    ##################### VARIABLES #####################
    # number of runs per variable change 
    n_runs    = int(args[2])

    # Base model
    n_col = int(args[0])  # number of columns of arms, previously n_arm 
    n_row = int(args[1])  # number of rows of arms

    # maximum velocity and acceleration, as well as the constant amount of time it takes to harvest a fruit once reached for the arm motors
    v_max            = 0.5   # these values are currently incorrect. The real values are in MIP_algorithm.py and they are different for each axis
    a_max            = 1.    # these values are currently incorrect. The real values are in MIP_algorithm.py and they are different for each axis
    t_grab           = 0.5 

    d_cell           = 1.0#0.7             # in m, length of the cell along the orchard row (y-axis), parallel to vehicle travel. Matches the prototype
    d_o              = 0.15            # in m, space between the columns
    d_hrzn           = 0.5             # in m, the extra length (horizon) in front of the robot that the robot can see
    if int(args[9]) != 2:
        # if set_view_field does not call for a horizon
        d_hrzn = 0
    h_cell           = 3.5 / n_row      # in m, width/height of the horizontal row of arms (z-axis) perpendicular to vehicle travel
    h_g              = 0.05            # in m, height of the gripper (dead space inducing)
    w_arm            = 1               # how far the arm can reach into the canopy
    # will end up removing this because d_o makes much more sense and easier to deal with
    l_real_y_travel  = d_cell          # in m, actual arm horizontal (y-coordinate) travel distance within a cell (measured on the prototype to be 0.46 m)

    # for MIP v_vy loop or makespan, requires an upper and lower bound for v_vy_cmps
    v_vy_lb_cmps     = int(args[3])    # in cm/s, the single velocity being tested or the lower bound for makespan or v_vy loop
    v_vy_ub_cmps     = int(args[4])    # in cm/s, when testing many velocities, this determines the top velocity tested

    v_vy_fruit_cmps  = v_vy_lb_cmps    # in cm/s, the single velocity being used also used for setup of MIP module (can this be changed and cleaned up?)
    v_vy             = v_vy_fruit_cmps / 100 # change to m/s

    d_vehicle        = n_col * d_cell + (n_col - 1) * d_o # in m, the length of the vehicle (along the y-axis), takes into account space between cells, if any
    h_vehicle        = n_row * h_cell  # in m, the height of the vehicle (along the z-axis)

    q_vy             = 0 - d_vehicle   # in m, the start of the back of the vehicle. Negative start so the system starts before the fruits start appearing (as it moves into the row)
    q_vy_start       = q_vy            # in m, saves the start location (q_vy will change as the system 'moves' along the orchard row)

    # orchard row limis along all three axes (fruits above or below limits cannot be picked). Used especially when creating fake fruit districutions to test against
    x_lim            = [0.2, 3.0]               # in m, also determines the distance the arm can move into the canopy
    y_lim            = [q_vy_start, d_vehicle+d_hrzn]  # in m, sets upper and lower bounds of travel for the vehicle. Can be used to stop the vehicle early with the real fruit distribution data
    z_lim            = [0., h_vehicle]          # in m, how tall the columns of the robots are 

    # initialize, will change within the loop to measure global values
    FPT = 0
    FPE = 0
    total_picked = 0

    ## set fruit distribution flag
    # 0     == 2022 Digitized fruit data (Pink Ladies, Jeff Colombini orchard)
    # 1     == segments of 2022 digitized fruit data between a given ymin and ymax determined in fruit_handler.py createFruitDistribution(), runs through all 2022 Colombini files 
    # 2     == Raj's digitized fruits (right side)
    # 3     == Juan's digitized fruits (Stavros phone video)
    # 4     == reduced Raj's digitized fruits; can reduce the density to a desired value (density hardcoded currently to 20f/m^3)
    # 5     == reduced Juan's digitized fruits; can reduce the density to a desired value (density hardcoded currently to 20f/m^3)
    # 6     == uniform random  (if algorithm == 1, use melon version)
    # 7   * not available *  == uniform random, equal cell density
    # 8   * not available *  == multiple densities separated by some space (only melon for now)
    # 9   * not available *  == fruit in vertical columns
    # 10  * not available *  == "melon" version of columns (user inputs desired no. fruits, z height, and distance between fruit in y-coord)
    set_distribution = int(args[6])

    ## set algorithm being used 
    # 0     == extended TOPTW MIP model with objective to maximize the number of harvested fruit, takes one velocity or a range of velocities to determine best FPE vs. FPT
    # 1     == TOPTW MIP model with the objective to maximize FPE*FPT, includes slack variables, minFPE, and minFPT
    # 2     == makespan TOPTW MIP model with the objective to minimize makespan, includes slack variable and minFPE
    # 3     == FCFS
    # 4     == FCFS to find speed lower bound, FPE*FPT to find schedule
    # 5     == FCFS to find speed lower bound, makespan to find schedule
    # 6     == SPT ************ DOES NOT CURRENTLY EXIST ************
    set_algorithm = int(args[5])

    ## set MPC on or off
    # 0     == non-MPC, solve using the travel length
    # 1     == MPC, solve using the whole view window (vehicle + horizon), but only use the results up to the travel length
    set_MPC = int(args[7])

    ## set how z-coord edges are calculated
    # 0     == z-edges are divided equally along orchard height
    # 1     == z-edges are divided so each row has equal number of fruit (or close to equal)
    set_edges = int(args[8])

    ## set if the vehicle can see the whole dataset or just what's in front
    # 0     == robot sees the whole dataset
    # 1     == robot only sees what's in front of it
    # 2     == robot only sees what's in front of it plus a horizon
    set_view_field = int(args[9])
    # if set_distribution == 1:
    #     # made the mistake not to hard-set this flag for the distribution that requires it
    #     set_view_field = 0 

    ## set if solving per row or the whole view at once 
    # 0     == solve all rows at once 
    # 1     == solve per row 
    set_solve_row = 0

    if set_distribution == 1:
        segment_n = int(args[10])  # sets which start and end coordinates will be used for a segment (only when set_distribution= 1):
        y_lim[1]  = 3.5        # make the segment longer than d_vehicle+d_hrzn by some amount (test segment length effect)
    else:
        segment_n = 1
    # 0 == y-coordinates between 0    - 2.7 m
    # 1 == y-coordinates between 2.7  - 5.4 m
    # 2 == y-coordinates between 5.4  - 8.1 m
    # 3 == y-coordinates between 8.1  - 10.8 m
    # 4 == y-coordinates between 10.8 - 13.5 m

    ## set print, logging, and plot settings on or off
    # 0     == print/plot is off
    # 1     == print/plot is on
    print_out = 0  # turns printing states, indexes, and settings on/off (all or nothing)
    plot_out  = 0  # turns plotting on/off
    log_out   = 1  # turns logging off gurobi files and csv creation files such as ilp, lp, mlp file creation on/off

    ## set Td to be constant or variable? -> see if needed as a flag (not currently)
    # 0     == Td constant
    # 1     == Td variable based on vacuum

    # set density if specific to the set_distibution setting
    if set_distribution >= 4:
        # reduced Raj/Juan or RNG distributions, set density as argument from user
        density = 40       # in fruit/m^2, makespan is being limited to rho = 2 with random placement
    else:
        # using real localization data so we don't care about this value
        density = 16 

    # if set_algorithm == 0 or set_algorithm >= 3:
    #     # have a list of vehicle velocities to loop through, if ub and lb are equal, this means a single speed will be tested
    #     v_vy_cmps_try = np.arange(v_vy_lb_cmps, v_vy_ub_cmps+1) # +1 to include largest value in arange
    #     print('velocities being attempted:', v_vy_cmps_try)
    # elif set_algorithm == 1 or set_algorithm == 2:
    #     # makespan already takes care of the multiple velocities, set this as *one* velocity to loop through only (we don't particularily care which (we think)
    #     v_vy_cmps_try = np.arange(v_vy_lb_cmps, v_vy_lb_cmps+1) # +1 to include largest value in arange
        # print('velocities being attempted:', v_vy_cmps_try)

    fruit_data = fruit_handler(n_col, n_row, t_grab, x_lim, y_lim, z_lim)
    # fruit_data.calcYlimMax(set_distribution)


    ##################### GLOBAL LISTS and ARRAYS #####################
    run_list       = list()       # saves the results of each run for analysis
    time_list      = list()       # saves how long each run took

    snap_num_array = np.zeros(n_runs, dtype=np.int8)
    seed_list      = fruit_data.getRNGSeedList(n_runs, './rngseed_list_20200901.csv') # csv file name to determine the distribution

    ##################### RUN MIP PYTHON SCRIPT #####################
    for i_run in range(n_runs):
        # create or clear the snapshot lists or they'll keep growing every run, breaking data analysis (dunno if data analysis needed for each of the multiple runs?)
        snapshot_list   = list()
        snapshot_cell   = list()
        time_snap_list  = list()
        this_seed       = list()

        # since we're changing the algorithm based on number of fruits, make sure the main algorithm is always reset for a run
        # set_algorithm = int(args[5])

        # because we're restarting the object at every run, need to only provide this run's seed list. Needs a list of lists since it assumes it could be multiple runs
        this_seed.append(seed_list[i_run]) 

        # create the simulated environment
        # print('seeds being passed to buildOrchard:', seed_list[i_run][:3])
        if set_distribution == 0:
            fruit_data.buildOrchard(set_distribution, seed_list[i_run][:3], run_n=i_run) 
        elif set_distribution == 1:
            fruit_data.buildOrchard(set_distribution, seed_list[i_run][:3], run_n=i_run, seg_n=segment_n)
        elif set_distribution >= 4:
            fruit_data.buildOrchard(set_distribution, seed_list[i_run][:3], density=density, run_n=i_run) 
        else:
            fruit_data.buildOrchard(set_distribution, seed_list[i_run][:3])
        
        
        # save the complete dataset of fruits
        # total_sortedFruit = np.copy(mip_melon.sortedFruit)

        # # find number of problem clusters (extend later to know where the clusters are)
        # # findClustersTotal(mip_melon.sortedFruit, v_vy, mip_melon.Td, n_col)
        # fruits2remove = findClustersByRow(mip_melon.sortedFruit, v_vy, mip_melon.Td, n_col, n_row, mip_melon.z_row_bot_edges, mip_melon.z_row_top_edges)
        # for fruit_i in fruits2remove: 
        #     mip_melon.sortedFruit[4,fruit_i] = 2  ############ CAREFUL, flag=2 IS USED AS SCHED+PICK ############

        # set the number of snapshots if the vehicle cannot see the whole dataset and change the step length accordingly
        if set_view_field == 0:
            # global view
            n_snapshots = 1 
            d_plan      = fruit_data.y_lim[1] - fruit_data.y_lim[0] # in m, harvester can see the whole dataset
            D_          = fruit_data.y_lim[1] + d_vehicle  # in m, the travel length
            FPE_min     = 0.95 # in %, what is the minimum FPE desired (not exclusive) for the velocity loop setting

        elif set_view_field == 1:
            # limited view, no horizon
            d_plan      = d_vehicle # in m, the length along the orchard row that the robot can see, moves with the vehicle
            D_          = 0.3  # in m, the travel length before resolving MIP 
            n_snapshots = math.ceil((fruit_data.y_lim[1] - fruit_data.y_lim[0]) / D_)  # set just to do the area in front of the vehicle
            FPE_min     = D_ / (d_vehicle)  # the ratio of traveled length to observed length (how many fruits could be harvested vs. viewed)

        else: 
            # limited view with horizon
            d_plan      = d_vehicle + d_hrzn  # in m, the length along the orchard row that the robot can see, moves with the vehicle
            D_          = d_vehicle / int(args[11]) # in m, the travel length before resolving MIP
            n_snapshots = math.ceil((fruit_data.y_lim[1] - fruit_data.y_lim[0]) / D_)  # set just to do the area in front of the vehicle

            if set_MPC == 0:
                FPE_min = 0.95  # 0.90# # D_ / (d_plan) * 2  # the ratio of traveled length to observed length (how many fruits could be harvested vs. viewed)
            else:
                FPE_min = 0.95 # D_ / (d_plan) * 2  # the ratio of traveled length to observed length (how many fruits could be harvested vs. viewed)
            
        snap_num_array[i_run] = n_snapshots

        # if fruit_data.N < 10:
        if fruit_data.N == 0:
            # run has no fruits, should be skipped completely
            # print('WARNING: Run has less than 10 fruits and will be skipped')
            print('WARNING: Run has no fruits and will be skipped')
            print('----------------------------------------------')
            print('----------------------------------------------\n\n')

            snap_num_array[i_run] = 0 # to indicate that this run is not counted

            snapshot_list.append(list()) # append empty lists as a place holders (otherwise data analysis is a pain)
            time_snap_list.append(list())

            run_list.append(list())
            time_list.append(list())

            continue 
            
        # arrays saving end of snapshot result values
        chosen_v_vy_mps_array = np.zeros(n_snapshots) # in m/s, array to save each snapshot's chosen velocity 
        snapshot_time_array   = np.zeros(n_snapshots) # in s, the time a snapshot takes based on chosen velocity
        global_FPE_array      = np.zeros(n_snapshots) # in %, array that saves global FPE at end of each snapshot
        global_FPT_array      = np.zeros(n_snapshots) # in fruits/s, array that saves global FPT at end of each snapshot
        mean_Td_array         = np.zeros(n_snapshots) # in s, saves the mean Td value of each snapshot

        if print_out == 1:
            print()
            print('length of the full dataset:', (fruit_data.y_lim[1] - fruit_data.y_lim[0]), 'm')
            print('vehicle planning window length: {:.1f} m'.format(d_plan)) 
            print('vehicle travel per snapshot: {:.1f} m'.format(D_)) 
            print('number of snapshots:', n_snapshots)
            print('FPE minimum set at {:.1f}%'.format(FPE_min*100))
            if v_vy_lb_cmps != v_vy_ub_cmps:
                # there is a range of velocities being tested
                print('\nMin velocity: %d cm/s \nMax velocity: %d cm/s' % (v_vy_lb_cmps, v_vy_ub_cmps))
            else:
                print('\nVelocity being tested: %d cm/s' % (v_vy_lb_cmps))

            # print()
            # print('this seed\n', seed_list[i_run])
            # print()
            print('-----------------------------------------------------------------')
            print('-----------------------------------------------------------------')

        # create the arm object lists, should only be done once per full dataset if not solving by row
        if set_solve_row == 0:
            mip_arm = fruit_data.createArms()

        # init the MIP melon object 
        if set_solve_row == 0 and set_algorithm < 3: # the first 3 algorithms are MIP-based
            n_row_loop = 1 # because the whole view is being solved at once, no need to loop
            mip_melon = MIP_algorithm(q_vy, n_col, n_row, 0, d_cell, d_o, d_hrzn, x_lim, fruit_data.y_lim, z_lim, density)
            # set the y-coordinate movement limits within the cell if not equal to cell length. Only affects the Jobs() object if it's not equal to the d_cell 
            if set_distribution == 0:
                # since we are using FCFS for N < 20, we also need to init FCFS 
                # create a flag that determines when FCFS is done and MIP has to be used to determine the schedule with the new velocity range
                task_alloc_done_flag = 0
                # init the first come first serve solver
                fcfs = FCFS()

        elif set_solve_row == 1 and set_algorithm < 3: # the first 3 algorithms are MIP-based
            n_row_loop = n_row # because we are solving per row, we need to loop through each row in each view/snapshot
            # init the MIP melon object, n_row set to one for each since we're running it per row
            # start the row at 0th row and then change the row as needed after this and before create arms
            mip_melon = MIP_algorithm(q_vy, n_col, 1, 0, d_cell, d_o, d_hrzn, x_lim, fruit_data.y_lim, z_lim, density)

        elif set_algorithm == 3:
            n_row_loop = 1 # because the whole view is being solved at once, no need to loop
            # create a flag that determines when FCFS is done and MIP has to be used to determine the schedule with the new velocity range
            task_alloc_done_flag = 0
            # init the first come first serve solver
            fcfs = FCFS()

        elif set_algorithm == 4 or set_algorithm == 5:
            n_row_loop = 1 # because the whole view is being solved at once, no need to loop
            # init the first come first serve solver for speed
            fcfs = FCFS()
            # create a flag that determines when FCFS is done and MIP has to be used to determine the schedule with the new velocity range
            task_alloc_done_flag = 0
            # init MIP to then solve for more optimal scheduling
            mip_melon = MIP_algorithm(q_vy, n_col, n_row, 0, d_cell, d_o, d_hrzn, x_lim, fruit_data.y_lim, z_lim, density)

        elif set_algorithm == 6:
            n_row_loop = 1 # because the whole view is being solved at once, no need to loop
            # create a flag that determines when FCFS is done and MIP has to be used to determine the schedule with the new velocity range
            task_alloc_done_flag = 0
            # init the first come first serve solver
            spt = SPT()

        if set_algorithm != 3 and set_algorithm != 6:
            mip_melon.setTravelLength(D_)           # needed an easy and clear way to set travel length in mip_melon since it changes depending on the settings

        for i_snap in range(n_snapshots):
            fruit_picked_by = list()
            # Figure out which fruits go into this snapshot and transform their index to start with zero (save the index start)
            i_snap_sortedFruit_index  = fruit_data.fruitsInView(q_vy, d_plan, fruit_data.sortedFruit, d_cell, l_real_y_travel)
            # make a copy of sortedFruit that works for the snapshot which will also be used to create the necessary Arm() Objects for the snapshot
            i_snap_sortedFruit        = np.copy(fruit_data.sortedFruit[:,i_snap_sortedFruit_index])
            # calculate the number of fruits in the snapshot
            i_snap_numFruit           = len(i_snap_sortedFruit[0,:])
            # update number of fruits to account for scheduled + harvested flag
            index_unavailable         = np.where(i_snap_sortedFruit[4,:] == 2)
            i_snap_available_numFruit = i_snap_numFruit - len(index_unavailable[0])

            if i_snap_available_numFruit <= 20:
                # run FCFS since it does better for smaller amounts of fruits
                # allow at least 50% loss with the speed
                V_lb_cmps = 1
                
                # set_algorithm = 3
                # # we're going to use FCFS so set a range of velocities
                # v_vy_cmps_try = np.arange(v_vy_lb_cmps, v_vy_ub_cmps+1)
            else:
                # V_lb_cmps = math.floor((n_row * n_col)*(d_plan*100) / (i_snap_available_numFruit * 4.5 * 0.50))
                V_lb_cmps = 1
                # run the base formulation
                # set_algorithm = int(args[11])
                # # MIP takes care of multiple velocities, set this as *one* velocity to loop through only (we don't particularily care which
                # v_vy_cmps_try = np.arange(v_vy_lb_cmps, v_vy_lb_cmps+1) # +1 to include largest value in arange

            if set_algorithm == 0 or set_algorithm >= 3:
                # have a list of vehicle velocities to loop through, if ub and lb are equal, this means a single speed will be tested
                v_vy_cmps_try = np.arange(V_lb_cmps, v_vy_ub_cmps+1) # +1 to include largest value in arange
                print('velocities being attempted:', v_vy_cmps_try)
            elif set_algorithm == 1 or set_algorithm == 2:
                # makespan already takes care of the multiple velocities, set this as *one* velocity to loop through only (we don't particularily care which (we think)
                v_vy_cmps_try = np.arange(V_lb_cmps, v_vy_lb_cmps+1) # +1 to include largest value in arange
                
            # determine the z_edges for this snapshot, add RNG seeds to give the z-bounds a random offset between columns to decrease dead-space 
            [bot_edge, top_edge] = fruit_data.calcRowZBounds(set_edges, z_lim, h_cell, h_g, i_snap_sortedFruit, seed_list[i_run][3])
            N_deadspace = fruit_data.N_deadspace
            # print('bottom z-bounds')
            # print(bot_edge)
            # print('top z-bounds')
            # print(top_edge)

            if set_algorithm != 3 and set_algorithm != 6:
                mip_melon.setZlim(bot_edge, top_edge)
            
            # need to calculate the density, R, etc. so as to determine the best v_vy value
            horizon_indexes = fruit_data.getHorizonIndex(i_snap_sortedFruit, q_vy, d_vehicle, d_hrzn)

            ## calculate multiple R and v_vy values based on multiple slices of the current view
            # return a list of fruit densities in each cell 
            fruit_density = fruit_data.calcDensity(q_vy, n_col, n_row, d_cell, w_arm, i_snap_sortedFruit)
            # calculate the row densities
            # d_row = np.average(d, axis=1)
            # d_tot = np.average(d)

            ## using the fruit densities, determine the vehicle speed to set a specific R value?
            # currently, the R value would be 
            R_melon = fruit_data.calcR(v_vy, len(horizon_indexes), d_hrzn, h_vehicle, w_arm)  # calculated based on columns and the horizon length

            snapshot_cell.append([fruit_density, R_melon])

            if print_out == 1:
                print('number of fruits in this snaphsot (also counts sched+pick):', i_snap_numFruit)
                print('actual number of fruits in this snaphsot (removed sched+pick) :', i_snap_available_numFruit)
                # print()
                # print('the density of fruits in each cell in this snapshot is:\n{:.2}'.format(d_tot))
                # print()
                # print('the density of fruits in each cell in this snapshot is:\n', d)
                # print()
                # print('the density of fruits in each row in this snapshot is:\n', d_row)
                # if set_view_field == 2:
                #     # horizon indexes only make sense here
                #     print()
                #     print('the indexes of the fruits in the horizon are:\n', horizon_indexes)
                #     print()
                #     print('the R value of this snapshot is: {:.2} \n'.format(R))
                # print()
                print('Vehicle location at start of this snapshot {:.1f} m on y-axis \n'.format(q_vy))

            printScen("Solving base scenario model")

            FPT = 0  # start FPT at 0 because when doing vehicle speed loop, FPT needs to be initilized already
            # solution_found = 0 # changes to 1 if at least one solution fits desired min values
            start_timer = datetime.now() # to figure out how looping through v_vy compares to pure MIP

            # needed to know if this is the first attempted v_vy value or not
            v_vy_loop_i = 0

            # loop throught the list of velocity values 
            for v_vy_curr_cmps in v_vy_cmps_try:
                # how many times will we loop through a snapshot and velocity attempt also depends on if we're solving once or by row
                if i_snap_available_numFruit < 1:
                    # there are no fruits, choose the highest velocity
                    v_vy_curr_cmps  = v_vy_ub_cmps
                    v_vy            = v_vy_ub_cmps / 100 # in m
                    # local FPE and FPT can't reaaaally be trusted, no fruit's available so the values are distorted
                    FPE             = 1.
                    FPT             = 0.
                    total_picked    = 0
                    chosen_j        = np.zeros([n_row, n_col])      # save the v_curr_j variable for the chosen run
                    # lists of the desired results
                    fruit_picked_by = list()                      # list that saves which arm picks which fruit
                    # build out the lists to be the right size (tried replication *, but the lists point at each other)
                    for r in range(n_row):
                        if n_row  > 1:
                            fruit_picked_by.append([])

                        for c in range(n_col+1):
                            if n_row > 1:
                                fruit_picked_by[r].append([])
                            else:
                                fruit_picked_by.append([])

                    q_vy += D_
                    print('########################### NO FRUIT, SET V = Vmax ###########################')
                    print()
                    # don't need to run the mip solver
                    break

                # if solving by row, loop thorugh the number of rows, otherwise this only runs once
                for i_loop in range(n_row_loop):
                    # need to update the 'starting row number' in the MIP_algorithm object
                    if set_algorithm != 3 and set_algorithm != 6:
                        mip_melon.starting_row_n = i_loop

                    if set_solve_row == 1:
                        # now create the correct arms objects for only one row
                        mip_arm = fruit_data.createArms()

                    # create the fruit object lists with updated starting_row_n
                    mip_fruit = fruit_data.createFruits(i_snap_numFruit, i_snap_sortedFruit)
                    mip_job   = fruit_data.createJobs(mip_arm, mip_fruit, v_vy_curr_cmps, q_vy, d_cell)

                    # solve for the optimal schedule for this row/loop, for this snapshot
                    if set_algorithm == 0:
                        [i_loop_fruit_picked_by, i_loop_fruit_picked_at] = mip_melon.solve_mip(mip_fruit, mip_job, i_snap_available_numFruit, v_vy_curr_cmps, set_distribution, set_algorithm, set_MPC, fruit_data.fruit_travel_matrix, fruit_data.sortedFruit)
                        make_v_vy = v_vy_curr_cmps
                        curr_count = np.copy(mip_melon.curr_j)

                    elif set_algorithm == 1 or set_algorithm == 2:
                        # goal programing and epsilon constraint require velocity upper and lower bounds, as well as a min FPE to find (there are default values: v_ub = 5, v_lb = 1, FPE_min = .5)
                        [i_loop_fruit_picked_by, i_loop_fruit_picked_at, make_v_vy] = mip_melon.solve_mip(mip_fruit, mip_job, i_snap_available_numFruit, v_vy_curr_cmps, set_distribution, set_algorithm, set_MPC, fruit_data.fruit_travel_matrix, fruit_data.sortedFruit, FPE_min=FPE_min, v_vy_lb_cmps=v_vy_lb_cmps, v_vy_ub_cmps=v_vy_ub_cmps)
                        curr_count = np.copy(mip_melon.curr_j)
                        if make_v_vy == 0:
                            # no solution was found, use the max speed 
                            make_v_vy = v_vy_ub_cmps

                    elif (set_algorithm == 4 or set_algorithm == 5) and task_alloc_done_flag == 1:
                        if v_combo_lb+1 > v_vy_ub_cmps:
                            v_combo_lb = v_vy_ub_cmps-7# avoid having the top speed go above the top speed
                        # in the combo solving FCFS has finished obtaining a best low bound V, so use that to solve the schedule using MIP
                        [i_loop_fruit_picked_by, i_loop_fruit_picked_at, make_v_vy] = mip_melon.solve_mip(mip_fruit, mip_job, i_snap_available_numFruit, v_combo_lb, set_distribution, set_algorithm, set_MPC, fruit_data.fruit_travel_matrix, fruit_data.sortedFruit, FPE_min=FPE_min, v_vy_lb_cmps=(v_combo_lb-0), v_vy_ub_cmps=(v_combo_lb+7))
                        curr_count = np.copy(mip_melon.curr_j)
                        if make_v_vy == 0:
                            # no solution was found, use the max speed 
                            make_v_vy = v_vy_ub_cmps

                    elif (set_algorithm == 4 or set_algorithm == 5) and task_alloc_done_flag == 0 and v_vy_curr_cmps == v_vy_ub_cmps:
                        # at the last possible velocity value. Since FCFS has not reached a 'best' vehicle speed, use the top speed as the upper bound. 
                        # otherwise, only uses FCFS and the results might not be correct
                        task_alloc_done_flag == 1
                        [i_loop_fruit_picked_by, i_loop_fruit_picked_at, make_v_vy] = mip_melon.solve_mip(mip_fruit, mip_job, i_snap_available_numFruit, v_vy_curr_cmps, set_distribution, set_algorithm, set_MPC, fruit_data.fruit_travel_matrix, fruit_data.sortedFruit, FPE_min=FPE_min, v_vy_lb_cmps=(v_vy_ub_cmps-1), v_vy_ub_cmps=v_vy_ub_cmps)
                        curr_count = np.copy(mip_melon.curr_j)
                        if make_v_vy == 0:
                            # no solution was found, use the max speed 
                            make_v_vy = v_vy_ub_cmps

                    elif set_algorithm >= 3 and set_algorithm <= 5 and task_alloc_done_flag == 0:
                        # run FCFS algorithm
                        [i_loop_fruit_picked_by, i_loop_fruit_picked_at] = fcfs.main(n_col, n_row, mip_fruit, mip_job, v_vy_curr_cmps, q_vy, d_cell, d_o, fruit_data.fruit_travel_matrix, fruit_data.sortedFruit, bot_edge, top_edge)
                        make_v_vy = v_vy_curr_cmps
                        curr_count = np.copy(fcfs.curr_j)

                    elif set_algorithm == 6 and task_alloc_done_flag == 0:
                        # run SPT algorithm
                        [i_loop_fruit_picked_by, i_loop_fruit_picked_at] = spt.main(n_col, n_row, mip_fruit, mip_job, v_vy_curr_cmps, q_vy, d_cell, fruit_data.fruit_travel_matrix, fruit_data.sortedFruit, bot_edge, top_edge)
                        make_v_vy = v_vy_curr_cmps
                        curr_count = np.copy(spt.curr_j)


                    # print('EXIT TO CHECK CURRENT RESULTS')
                    # sys.exit(0)

                    # set the current velocity as the velocity chosen by the makespan
                    v_vy_mps = make_v_vy / 100 # change to m/s to because it starts getting used from here on out

                    # print('the fruits scheduled to be picked in this snapshot are:', i_loop_fruit_picked_by)
                    # print('the times at which the fruits were picked are:', i_loop_fruit_picked_at)
                    # print()

                    # check what fruits have been harvested, scheduled, or not picked
                    where_no = np.where(fruit_data.sortedFruit[4,:] == 0)
                    where_sh = np.where(fruit_data.sortedFruit[4,:] == 1)
                    where_pi = np.where(fruit_data.sortedFruit[4,:] == 2)
                    # print('Before queue management sortedFruit')
                    # print('Global number unpicked:', len(where_no[0]), '\nGlobal number scheduled:', len(where_sh[0]), '\nGlobal number picked', len(where_pi[0]))

                    ################################################################################################################
                    # pre_queu_FPE calculated here will be used as the check_FPE lower down 
                    # saves the results from solving the whole snapshot/workspace (like MPC) and then compares to the higher minFPE  
                    if set_MPC == 1:
                        for row in range(n_row):
                            for column in range(n_col):
                                # calculate how many fruits each arm picked having cleaned out unpickable fruit above
                                if i_loop > 0:
                                    # stack the row's number of fruits picked by each arm
                                    pre_queue_chosen_j = np.vstack((pre_queue_chosen_j, np.copy(curr_count))) 
                                    # now figure out what fruit were not picked
                                    # see https://stackoverflow.com/questions/16163546/checking-to-see-if-same-value-in-two-lists
                                    pre_queue_unpicked = set(pre_queue_unpicked).intersection(i_loop_fruit_picked_by[-1])
                                else: 
                                    # initialization for stack the row's number of fruits picked by each arm
                                    pre_queue_chosen_j = np.copy(curr_count)
                                    # initialization to figure out what fruit were not picked
                                    pre_queue_unpicked = i_loop_fruit_picked_by[-1]
                                # print('unpicked,', unpicked)

                                # put fruit_picked_by list of lists together to reflect the columns and rows
                                if set_solve_row == 0:
                                    pre_queue_fruit_picked_by = i_loop_fruit_picked_by.copy()
                                    # fruit_picked_at = i_loop_fruit_picked_at
                                elif set_solve_row == 1:
                                    # when solving by row, to get the right 'geometry,' need to append each row's results
                                    pre_queue_fruit_picked_by.append(i_loop_fruit_picked_by.copy())

                        # save the pre-queue FPE and FPT values to compare against
                        # determine total picked in this run for this velocity for this snapshot after going through queue manager
                        pre_queue_total_picked = np.sum(pre_queue_chosen_j)

                        # add the pre_queue_unpicked list to current velocity fruit_picked_by list in the correct location for each row
                        if n_row > 1 and set_solve_row == 1:
                            # print('pre_queue_ unpicked', list(pre_queue_unpicked))
                            for i_row in range(0, n_row):
                                pre_queue_fruit_picked_by[i_row][-1].clear()
                            
                            pre_queue_fruit_picked_by[0][-1] = list(pre_queue_unpicked)

                        pre_queue_FPT = pre_queue_total_picked / (d_plan / v_vy_mps)

                        # # calculate the velocity's FPT
                        # if set_distribution == 1:
                        #     # if we are working with single segments, all arms have a vehicle's length of time where they aren't working (entering and exiting the segment)
                        #     pre_queue_FPT = pre_queue_total_picked / ((D_ - d_vehicle) / v_vy_mps)

                        # else:
                        #     pre_queue_FPT = pre_queue_total_picked / (D_ / v_vy_mps)                       

                        # calculate the velocity's FPE 
                        if i_snap_available_numFruit > 0:
                            pre_queue_FPE = (pre_queue_total_picked / i_snap_available_numFruit)
                        else:
                            # if there are zero fruit, set FPE = 100% since it got everything. 
                            # FPT will be 0, showing/balancing out what happened
                            pre_queue_FPE = 1.

                        print('MPC FPE and FPT before queue: {:.2f} percent and {:.2f} f/s'.format(pre_queue_FPE*100, pre_queue_FPT))

                    ################################################################################################################

                    # Use a queue to manage which fruits can actually be harvested during the snapshot traveling time 
                    for row in range(n_row):
                        for column in range(n_col):
                            if n_row > 1:
                                # print(f'for arm in row %d and column %d:' %(n_row, n_col))
                                fruit_pick_arm = i_loop_fruit_picked_by[row][column]
                                fruit_when     = i_loop_fruit_picked_at[row][column]  # if fruit_pick_arm doesn't work, neither will this
                                # print('this arm is picking', fruit_pick_arm, 'at', fruit_when)

                            else: 
                                # print(f'for arm in row %d and column %d:' %(n_row, n_col))
                                fruit_pick_arm = i_loop_fruit_picked_by[column]
                                fruit_when     = i_loop_fruit_picked_at[column]  # if fruit_pick_arm doesn't work, neither will this
                                # print('this arm is picking', fruit_pick_arm, 'at', fruit_when)

                            queue_manager = MIP_queu_manager(v_vy_mps, D_, fruit_pick_arm, fruit_when)
                            not_picked = queue_manager.unpicked_queue
                            yes_picked = queue_manager.picked_queue
                            
                            if n_row > 1:
                                i_loop_fruit_picked_by[row][column] = list(yes_picked)

                                # update curr_j for this arm, to then be able to update chosen_j
                                curr_count[row, column] = queue_manager.updateSumPicked(yes_picked)
                                # print('changes to curr_j in row', row, 'and column', column, 'results in\n', curr_count)
                                    
                                # update sortedFruit so that fruit that was not picked in time gets reset from picked to not picked
                                if len(not_picked) > 0:
                                    i_loop_fruit_picked_by[0][n_col] = queue_manager.updateUnpicked(i_loop_fruit_picked_by[0][n_col])
                                    fruit_data.sortedFruit[4,not_picked] = 0  # not_picked should not be used later, so safe to do this here?
                                    # print('not picked list', not_picked)
                            else:
                                i_loop_fruit_picked_by[column] = list(yes_picked)

                                # update curr_j for this arm, to then be able to update chosen_j
                                curr_count[0,column] = queue_manager.updateSumPicked(yes_picked)
                                # print('changes to curr_j in row', row, 'and column', column, 'results in\n', curr_count)
                                    
                                # update sortedFruit so that fruit that was not picked in time gets reset from picked to not picked
                                if len(not_picked) > 0:
                                    i_loop_fruit_picked_by[n_col] = queue_manager.updateUnpicked(i_loop_fruit_picked_by[n_col])
                                    fruit_data.sortedFruit[4,not_picked] = 0  # not_picked should not be used later, so safe to do this here?
                                    # print('not picked list', not_picked)

                    # calculate how many fruits each arm picked having cleaned out unpickable fruit above
                    if i_loop > 0:
                        # stack the row's number of fruits picked by each arm
                        v_curr_chosen_j = np.vstack((v_curr_chosen_j, np.copy(curr_count))) 
                        # now figure out what fruit were not picked
                        # see https://stackoverflow.com/questions/16163546/checking-to-see-if-same-value-in-two-lists
                        unpicked = set(unpicked).intersection(i_loop_fruit_picked_by[-1])
                    else: 
                        # initialization for stack the row's number of fruits picked by each arm
                        v_curr_chosen_j = np.copy(curr_count)
                        # initialization to figure out what fruit were not picked
                        unpicked = i_loop_fruit_picked_by[-1]
                    # print('unpicked,', unpicked)

                    # put fruit_picked_by list of lists together to reflect the columns and rows
                    if set_solve_row == 0:
                        v_curr_fruit_picked_by = i_loop_fruit_picked_by.copy()
                        # fruit_picked_at = i_loop_fruit_picked_at
                    elif set_solve_row == 1:
                        # when solving by row, to get the right 'geometry,' need to append each row's results
                        v_curr_fruit_picked_by.append(i_loop_fruit_picked_by.copy())

                # check what fruits have been harvested, scheduled, or not picked
                where_no = np.where(fruit_data.sortedFruit[4,:] == 0)
                where_sh = np.where(fruit_data.sortedFruit[4,:] == 1)
                where_pi = np.where(fruit_data.sortedFruit[4,:] == 2)
                # print('After queue management sortedFruit')
                # print('Global number unpicked:', len(where_no[0]), '\nGlobal number scheduled:', len(where_sh[0]), '\nGlobal number picked', len(where_pi[0]))

                # determine total picked in this run for this velocity for this snapshot after going through queue manager
                v_curr_total_picked = np.sum(v_curr_chosen_j)

                # add the unpicked list to current velocity fruit_picked_by list in the correct location for each row
                if n_row > 1 and set_solve_row == 1:
                    # print('final unpicked', list(unpicked))
                    for i_row in range(0, n_row):
                        # reset all v_curr_fruit_picked_by individually
                        v_curr_fruit_picked_by[i_row][-1].clear()
                    
                    v_curr_fruit_picked_by[0][-1] = list(unpicked)

                v_curr_FPT = v_curr_total_picked / (D_ / v_vy_mps)

                # calculate the velocity's FPT
                # if set_distribution == 1:
                #     # if we are working with single segments, all arms have a vehicle's length of time where they aren't working (entering and exiting the segment)
                #     v_curr_FPT = v_curr_total_picked / ((D_ - d_vehicle) / v_vy_mps)
                # else:
                #     v_curr_FPT = v_curr_total_picked / (D_ / v_vy_mps)

                # calculate the velocity's FPE 
                if i_snap_available_numFruit > 0:
                    v_curr_FPE = (v_curr_total_picked / i_snap_available_numFruit)
                else:
                    # if there are zero fruit, set FPE = 100% since it got everything. 
                    # FPT will be 0, showing/balancing out what happened
                    v_curr_FPE = 1.

                if set_MPC == 1:
                    print('MPC FPE and FPT after queue: {:.2f} percent and {:.2f} f/s'.format(v_curr_FPE*100, v_curr_FPT))
                    # since we're using MPC, we want to compare against the pre_queue values since we solved for the whole workspace with a higher minFPE that is reduced after the queue 
                    check_FPE = pre_queue_FPE
                elif set_MPC == 0:
                    check_FPE = v_curr_FPE

                print()
                print('\nVelocity being tested: %0.2f m/s\n' %v_vy_mps)

                if check_FPE >= FPE_min or v_vy_loop_i == 0 or v_curr_FPE >= FPE:
                    
                    # if the correct FPE and FPT is found, or it's the first run but there are still too many fruits to pick even at the lowest speed
                    # print('FPE and FPT after conditional but before queue: {:.1f} percent and {:.1f} f/s'.format(v_curr_FPE*100, v_curr_FPT))
                    # print('FPE and FPT after conditional and queue: {:.1f} percent and {:.1f} f/s \n'.format(v_curr_FPE*100, v_curr_FPT))

                    if v_curr_FPT >= FPT or v_vy_loop_i == 0:
                        # a new solution can be used, though check if there are any fruits available to see if velocity should beset to upper bound and break out of loop
                        FPE          = v_curr_FPE
                        FPT          = v_curr_FPT
                        total_picked = v_curr_total_picked
                        chosen_j     = np.copy(v_curr_chosen_j)   # save the v_curr_j variable for the chosen run
                        fruit_picked_by = v_curr_fruit_picked_by.copy() # copy the chosen run's fruit picked by list
                        
                        if i_snap_available_numFruit > 1:
                            # if there are fruits, the solution works and should be saved as is
                            # I hav eit check for more than one fruit because it can get stuck not picking the last fruit and limiting the speed to 1 cm/s
                            print('***** NEW SOLUTION SAVED *****\n\n')
                            # solution_found = 1
                            v_vy           = v_vy_mps # in m
                            
                        else:
                            print('***** NO FRUITS AVAILABLE, BREAKING OUT *****')
                            # there are probably no fruits in this run, need to break out with speed at highest value
                            # solution_found = 1    # technically, a solution was found, it's just not great
                            v_vy           = v_vy_ub_cmps / 100 # in mv_vy_mps

                            # nothing picked, so zero out chosen_j -> check first if necessary
                            # zero_j = np.zeros([n_row, n_col])
                            # print('chosen_j should be zeroed out:')
                            print(chosen_j)
                            if not chosen_j.any():
                                # if there are any non-zero values, then it comes up as false and this is skipped
                                print('chosen_j is zeroed out')
                            else:
                                # chosen_j should be zeroed out again
                                print('WARNING: chosen_j should be zeroed out but it is not')

                            # clear the sortedFruit changes made in this run or the scheduled but not harvested results bleed over to the following runs
                            # fruit_picked_by has been saved and will be used to mark fruits as harvested if this ends up being the final solution
                            for row in range(n_row):
                                for column in range(n_col):
                                    if n_row > 1:
                                        fruit_data.sortedFruit[4,v_curr_fruit_picked_by[row][column]] = 0
                                    else:
                                        fruit_data.sortedFruit[4,v_curr_fruit_picked_by[column]] = 0

                            print('########################### FORCED END RUN ###########################')
                            print()
                            break    

                        if (set_algorithm == 4 or set_algorithm == 5 or set_algorithm == 6) and task_alloc_done_flag == 1:
                            # has already probably run once and if it runs again as is, will just result in the same run over and over until all the known velocities are tested (results don't change)
                            # also, the results have been saved, move on :)
                            print('***** FINISHED BOTH FCFS/SPT LOOP AND MIP, BREAKING OUT *****')
                            print('Found %0.2f cm/s to be the best lower bound velocity\n' %make_v_vy)
                            print('########################### FORCED END RUN ###########################')
                            print()
                            break
                   
                    else:
                        # when doing FCFS+MIP something is missing here because the first condition can be met, but not the second, leading to unneccessary loops 
                        # and if it gets to the last V, MIP will never run. 
                        print('v_curr_FPT < FPT, moving to check if MIP or early exit')

                        # previous velocity is probably going to be the correct one since FPE only drops. FPT supposedly keeps rising, but I don't think we can
                        # depend on it. -> treat it like if elif v_curr_FPE < FPE_min: was reached?

                        # clear the sortedFruit changes made in this run or the scheduled but not harvested results bleed over to the following runs
                        # fruit_picked_by has been saved and will be used to mark fruits as harvested if this ends up being the final solution
                        for row in range(n_row):
                            for column in range(n_col):
                                if n_row > 1:
                                    fruit_data.sortedFruit[4,v_curr_fruit_picked_by[row][column]] = 0 # try removing all v_curr_fruit_picked_by since fruit_picked_by may not be updated
                                else:
                                    fruit_data.sortedFruit[4,v_curr_fruit_picked_by[column]] = 0 

                        if (set_algorithm == 4 or set_algorithm == 5 or set_algorithm == 6) and task_alloc_done_flag == 0:
                            print('***** DONE WITH FCFS/SPT LOOP, MOVING ON TO MIP SOLVER RESULTS *****\n\n')
                            print('FCFS/SPT found %0.2f cm/s to be the best lower bound velocity\n' %make_v_vy)
                            # a "best" velocity has been found for the snapshot using FCFS, can this be rerun but with MIP and a different range of velocities?
                            # if set_algorithm == 4 or set_algorithm == 5:
                            task_alloc_done_flag = 1
                            # need loop to keep running, but there has to be a new range of velocities and switch to MIP 
                            v_combo_lb = make_v_vy

                            if v_vy_curr_cmps == v_vy_ub_cmps:
                                # might be kicked out of the loop anyway, see if avoiding that is possible by changing the value of v_vy_curr_cmps
                                v_vy_curr_cmps -= 1

                        elif (set_algorithm == 4 or set_algorithm == 5 or set_algorithm == 6) and task_alloc_done_flag == 1:
                            # the run, FCFS/SPT and MIP is done, so break out of the current loop
                            print('***** FINISHED BOTH FCFS/SPT LOOP AND MIP, BREAKING OUT *****')
                            print('Found %0.2f cm/s to be the best lower bound velocity\n' %make_v_vy)
                            v_vy         = make_v_vy / 100 # in m
                            # the MIP's new solution should be used
                            FPE          = v_curr_FPE
                            FPT          = v_curr_FPT
                            total_picked = v_curr_total_picked
                            chosen_j     = np.copy(v_curr_chosen_j)   # save the v_curr_j variable for the chosen run
                            fruit_picked_by = v_curr_fruit_picked_by.copy() # copy the chosen run's fruit picked by list
                            print('########################### FORCED END RUN ###########################')
                            print()
                            break 

                        else:
                            # the run is actually done, so break out of the current loop
                            print('***** NO IMPROVEMENT IN FPE POSSIBLE, BREAKING OUT *****')
                            print('Current NOT usable FPE and FPT are', v_curr_FPE, v_curr_FPT)
                            print('########################### FORCED END RUN ###########################')
                            print()
                            break

                # compare the FPE with FPE_min to see if this velocity works
                elif check_FPE < FPE_min: 
                    # there is no way FPE rises as velocity rises, so either a solution was found, or it wasn't 

                    # clear the sortedFruit changes made in this run or the scheduled but not harvested results bleed over to the following runs
                    # fruit_picked_by has been saved and will be used to mark fruits as harvested if this ends up being the final solution
                    for row in range(n_row):
                        for column in range(n_col):
                            if n_row > 1:
                                fruit_data.sortedFruit[4,v_curr_fruit_picked_by[row][column]] = 0 # try removing all v_curr_fruit_picked_by since fruit_picked_by may not be updated
                            else:
                                fruit_data.sortedFruit[4,v_curr_fruit_picked_by[column]] = 0 

                    if (set_algorithm == 4 or set_algorithm == 5 or set_algorithm == 6) and task_alloc_done_flag == 0:
                        print('***** DONE WITH FCFS/SPT LOOP, MOVING ON TO MIP SOLVER RESULTS *****\n\n')
                        print('FCFS/SPT found %0.2f cm/s to be the best lower bound velocity\n' %make_v_vy)
                        # a "best" velocity has been found for the snapshot using FCFS, can this be rerun but with MIP and a different range of velocities?
                        # if set_algorithm == 4 or set_algorithm == 5:
                        task_alloc_done_flag = 1
                        # need loop to keep running, but there has to be a new range of velocities and switch to MIP 
                        v_combo_lb = make_v_vy

                        if v_vy_curr_cmps == v_vy_ub_cmps:
                            # might be kicked out of the loop anyway, see if avoiding that is possible by changing the value of v_vy_curr_cmps
                            v_vy_curr_cmps -= 1
                        # else:
                        #     v_vy           = make_v_vy / 100 # in m
                        #     # the MIP's new solution should be used
                        #     FPE          = v_curr_FPE
                        #     FPT          = v_curr_FPT
                        #     total_picked = v_curr_total_picked
                        #     chosen_j     = np.copy(v_curr_chosen_j)   # save the v_curr_j variable for the chosen run
                        #     fruit_picked_by = v_curr_fruit_picked_by.copy() # copy the chosen run's fruit picked by list
                        #     print('########################### FORCED END RUN ###########################')
                        #     print()
                        #     # don't need to run the mip solver
                        #     break

                    elif (set_algorithm == 4 or set_algorithm == 5 or set_algorithm == 6) and task_alloc_done_flag == 1:
                        # the run, FCFS/SPT and MIP is done, so break out of the current loop
                        print('***** FINISHED BOTH FCFS/SPT LOOP AND MIP, BREAKING OUT *****')
                        print('Found %0.2f cm/s to be the best lower bound velocity\n' %make_v_vy)
                        v_vy         = make_v_vy / 100 # in m
                        # the MIP's new solution should be used
                        FPE          = v_curr_FPE
                        FPT          = v_curr_FPT
                        total_picked = v_curr_total_picked
                        chosen_j     = np.copy(v_curr_chosen_j)   # save the v_curr_j variable for the chosen run
                        fruit_picked_by = v_curr_fruit_picked_by.copy() # copy the chosen run's fruit picked by list
                        print('########################### FORCED END RUN ###########################')
                        print()
                        break 

                    else:
                        # the run is actually done, so break out of the current loop
                        print('***** NO IMPROVEMENT IN FPE POSSIBLE, BREAKING OUT *****')
                        print('Current NOT usable FPE and FPT are', v_curr_FPE, v_curr_FPT)
                        print('########################### FORCED END RUN ###########################')
                        print()
                        break 

                else:
                    print('***** POSSIBLY AN ERROR, FIGURE OUT HOW IT GOT HERE *****')
                    # no clue when this would be possible
                    # solution_found = 0

                # clear the sortedFruit changes made in this run or the scheduled but not harvested results bleed over to the following runs
                # fruit_picked_by has been saved and will be used to mark fruits as harvested if this ends up being the final solution
                for row in range(n_row):
                    for column in range(n_col):
                        if n_row > 1:
                            fruit_data.sortedFruit[4,v_curr_fruit_picked_by[row][column]] = 0 # try removing all v_curr_fruit_picked_by since it may not update 
                        else:
                            fruit_data.sortedFruit[4,v_curr_fruit_picked_by[column]] = 0 # try removing all v_curr_fruit_picked_by since it may not update 

                v_vy_loop_i += 1

            # if solution_found == 1:
            # use fruit_picked_by to manage sortedFruit and switch new scheduled only fruit to scheduled and picked
            # possible because sceduled but not picked were already 'removed' earlier during the queue managemet phase
            for row in range(n_row):
                for column in range(n_col):
                    if n_row > 1:
                        # print('getting list index out of range, so check row =', row, 'column =',column)
                        fruit_data.sortedFruit[4,fruit_picked_by[row][column]] = 2
                    else:
                        fruit_data.sortedFruit[4,fruit_picked_by[column]] = 2

            # # check what fruits have been harvested, scheduled, or not picked
            where_no = np.where(fruit_data.sortedFruit[4,:] == 0)
            where_sh = np.where(fruit_data.sortedFruit[4,:] == 1)
            where_pi = np.where(fruit_data.sortedFruit[4,:] == 2)
            # print('Before reseting sortedFruit')
            # print('Global number unpicked:', len(where_no[0]), '\nGlobal number scheduled:', len(where_sh[0]), '\nGlobal number picked', len(where_pi[0]))

            # calculate how long each arm was working vs idle
            state_time = fruit_data.calcStateTime(fruit_picked_by, mip_fruit, D_, v_vy)
            # print(state_time)            

            # save chosen velocity
            chosen_v_vy_mps_array[i_snap] = v_vy
            # save the time this snapshot takes based on chosen v_vy
            snapshot_time_array[i_snap] = D_ / v_vy

            # calculate the mean Td for the snapshot, not working yet
            this_mean_Td = fruit_data.calcMeanTd(total_picked, state_time)
            # this_mean_Td = fruit_data.calcMeanTd(mip_fruit, total_picked, state_time)
            mean_Td_array[i_snap] = this_mean_Td

            # calculate the global FPE for this segment
            this_global_FPE = len(where_pi[0]) / len(fruit_data.sortedFruit[4,:])
            global_FPE_array[i_snap] = this_global_FPE

            # calculate the global FPT for this segment, total picked / total time up to end of this snapshot
            this_global_FPT = len(where_pi[0]) / np.sum(snapshot_time_array)
            global_FPT_array[i_snap] = this_global_FPT

            ## continue filling if needed: PCT, state_time, fruit_picked_by, fruit_list (all lists?)
            # fill in snapshot object and list with current results, object definition in fruit_handler.py
            snapshot = Snapshot(n_col, n_row, d_hrzn, d_vehicle, d_cell, D_, d_plan, bot_edge, top_edge, this_mean_Td, v_vy, this_global_FPE, FPE, this_global_FPT, FPT, fruit_data.y_lim, i_snap_available_numFruit, N_deadspace, chosen_j, fruit_data.sortedFruit, fruit_picked_by, state_time)
            snapshot_list.append(snapshot)

            if print_out == 1:
                # print('the fruits scheduled and harvested in this snapshot are:', i_loop_fruit_picked_by)
                # print('Number unpicked fruits at this time:', len(i_loop_fruit_picked_by[0][-1]))
                # print()
                # following print statement is based on the fruit states before harvestable scheduled fruits were set as harvested for this run
                print('Snapshot number %d finished solving' % i_snap)
                print('--------------------------------------')
                print('Global number unpicked:', len(where_no[0]), '\nGlobal number scheduled:', len(where_sh[0]), '\nGlobal number picked:', len(where_pi[0]))
                # print('harvested', where_pi[0])
                # print('scheduled left over', where_sh[0])
                # print('updates to the sortedFruit list\n', mip_melon.sortedFruit[4,:])
                print()
                print('Velocity chosen for this run {:.3f} m/s'.format(v_vy)) 
                print('Mean snapshot Td for all arms %.3f s' % this_mean_Td)
                print('Number of fruits picked by each arm: *bottom* \n *back* ', chosen_j, ' *front* \n *top*')
                print('Global FPE , {:.3f}%'.format(this_global_FPE*100))  
                print('Global FPT , {:.3f} fruits/s'.format(this_global_FPT))
                print('Local FPE, {:.3f}%'.format(FPE*100), ', and FPT {:.3f}'.format(FPT))  
                print()
                print('Locally, a total of %d fruits were harvested out of %d available fruits' % (total_picked, i_snap_available_numFruit))     
                print()

            time_run = datetime.now()-start_timer
            print('########################### END RUN ###########################')
            print()

            # update the vehicle's location
            q_vy += D_

            # reset the algorithm
            # set_algorithm = int(args[5])

            # reset FCFS_done flag to 0
            task_alloc_done_flag = 0

            if set_algorithm != 3 and set_algorithm != 6:
                # MIP algorithm needs q_vy to be updated at this point as well
                mip_melon.q_vy = q_vy

            time_run = datetime.now()-start_timer
            time_snap_list.append(time_run)

            if set_solve_row == 0:
                print('########################## END LOOP ###########################')
                print()
                print('Solving this snapshot took:', time_run, 'h:m:s')
                print()
                print('###############################################################')
                print()

            elif set_solve_row == 1:
                print('########################## END LOOP ###########################')
                print()
                print('Solving the rows independently took:', time_run, 'h:m:s')
                print('               ', n_row, 'loops')
                print()
                print('###############################################################')
                print()

            # sys.exit(0)

            # it's just printing out the last value right now
        # print('########################## END LOOP ###########################')
        # print()
        # print('Looping through v_vy values took:', time_run, 'h:m:s')
        # print()
        # print('###############################################################')
        # print()


        ################ RESULS FOR A RUN OF WHOLE ROW ################
        # if solution_found == 0:
        #     # for single runs only
        #     print('NO SOLUTION was found')
        #     print()

        # elif solution_found == 1:
        # combine the results based on the various snapshots taken
        results = data_analysis(snapshot_list, snapshot_cell, print_out)
        if print_out == 1:
            results.printSettings()
            print('\nThe velocities for each of the %d snapshots in m/s:' % n_snapshots)
            print(chosen_v_vy_mps_array)
            print('The mean Td for each snapshot in s:')
            print(mean_Td_array)
            print()

        # results.avgFPTandFPE() # works with snapshots but not multiple runs, run once at the end of a run
        # results.runFPEandFPT()
        # results.avgPCT()
        # print()
        # results.plotValuesOverDistance()
        if plot_out == 1:
            # set up the plot path and naming convention for the run
            results.filePath(i_run)
            # currently results.plotTotalStatePercent() takes the average time in each state per snapshot and returns the plot of the average time each arm spent in each state over the whole run
            results.plotTotalStatePercent()
            # currently plot does not save each run attempt's plot. Just the last one (they have the same name so they will get overwritten)
            snapshot_schedules_2_plot = range(n_snapshots)  
            # create a plot only of the snapshots that you want (by their index)
            print('\nWhat is being sent into plot2D', snapshot_schedules_2_plot)
            results.plot2DSchedule(snapshot_schedules_2_plot)

        run_list.append(snapshot_list)
        time_list.append(time_snap_list)

        # reset the location of the vehicle for the next run 
        q_vy = 0 - d_vehicle

        if set_algorithm != 3 and set_algorithm != 6:
            # MIP algorithm needs q_vy to be updated at this point as well
            mip_melon.q_vy = q_vy

    analysisMultiple(set_algorithm, set_distribution, set_edges, n_runs, snap_num_array, run_list, time_list, print_out, log_out)
    
    
if __name__ == '__main__':
    main()