import csv
import math
import numpy as np
from datetime import datetime
from datetime import date
import sys

# from fruit_distribution import *   # import module to create the various desired fruit distributions 
from IG_data_analysis import *       # import module to analyze the data from the snapshots
from MIP_melon import *              # import module that solves the extended MIP in melon algorith *add citation*
from MIP_queu_manager import *       # import module that adds queue management to the scheduler to add dynamisism
from fruit_handler import *          # import module that handles the data creation and calculations (distribution, calcPCT, etc.)
from FCFS_wall import *
from SPT_wall import *


# tested with Python 3.7.0 & Gurobi 9.0

### Run MIP solver on muliple datasets to get average and variance on the results. Runs once on the data set (no horizon) and containes cluster finding functions. 


def printScen(scenStr):
    sLen = len(scenStr)
    print("\n" + "*"*sLen + "\n" + scenStr + "\n" + "*"*sLen + "\n")


def analysisMultiple(set_distribution, n_runs, n_snapshots, run_list, time_list, snapshot_cell, print_out):
    '''Obtain average and variance of results and time-to-run for all the runs'''
    time_array = np.zeros([n_runs, n_snapshots])
    FPE_array  = np.zeros([n_runs, n_snapshots])
    FPT_array  = np.zeros([n_runs, n_snapshots])
    max_row_density  = np.zeros([n_runs, n_snapshots]) # in fruits/m^2 
    min_row_density  = np.zeros([n_runs, n_snapshots]) # in fruits/m^2
    snapshot_density = np.zeros([n_runs, n_snapshots]) # in fruits/m^2
    
    v_vy_cmps = int(run_list[0][0].v_vy * 100)
    print('the run\'s v_vy', v_vy_cmps)

    print()
    print('----------------------------------------------')
    print('  Analyze results for all runs and snapshots  ')
    print('----------------------------------------------')
    print()

    for i_run in range(n_runs):
        for i_snap in range(n_snapshots):
            # print('Currently analyzing i_run %d and i_snap %d' % (i_run, i_snap))
            time_array[i_run,i_snap] = time_list[i_run][i_snap].total_seconds()
            # run list is made up of snapshot lists
            # try:
            # some snapshot lists are empty when there is nothing to harvest so end up with an IndexError 
            FPE_array[i_run,i_snap]  = run_list[i_run][i_snap].FPE * 100
            FPT_array[i_run,i_snap]  = run_list[i_run][i_snap].FPT
            # except IndexError:
            #     # set the values at 100 (no harvestable fruits, no )
            #     FPE_array[i_run,i_snap]  = 100
            #     FPT_array[i_run,i_snap]  = 0

            density_rows = np.average(snapshot_cell[i_snap][0], axis=1)

            max_row_density[i_run,i_snap]  = density_rows.max()
            min_row_density[i_run,i_snap]  = density_rows.min()
            snapshot_density[i_run,i_snap] = np.average(snapshot_cell[i_snap][0])

            # print('average snapshot density:', snapshot_density[i_run,i_snap])
            # print('row densities:\n', density_rows)
            # print('max row density:', max_row_density[i_run,i_snap])
            # print('min row density:', min_row_density[i_run,i_snap], '\n')

        # print out this run's snapshot results
        print('Run', i_run, 'results over the run\'s snapshots:')

        avg_time_run = np.average(time_array[i_run,:])
        std_time_run = np.std(time_array[i_run,:])
        print('    average time, {:.3f}'.format(avg_time_run), '+/-{:.3f} s'.format(std_time_run)) 

        avg_FPE_run = np.average(FPE_array[i_run,:])
        std_FPE_run = np.std(FPE_array[i_run,:])
        print('    average FPE, {:.3f}'.format(avg_FPE_run), '+/-{:.3f}%'.format(std_FPE_run)) 

        avg_FPT_run = np.average(FPT_array[i_run,:])
        std_FPT_run = np.std(FPT_array[i_run,:])
        print('    average FPT, {:.3f}'.format(avg_FPT_run), '+/-{:.3f}%'.format(std_FPT_run)) 
        print() 

    avg_time = np.average(time_array)
    std_time = np.std(time_array)
    print('average time for every run, {:.3f} s'.format(avg_time), '+/-{:.3f} s'.format(std_time)) 

    avg_FPE = np.average(FPE_array)
    std_FPE = np.std(FPE_array)
    print('average FPE for every run, {:.3f}'.format(avg_FPE), '+/-{:.3f}%'.format(std_FPE))  

    avg_FPT = np.average(FPT_array)
    std_FPT = np.std(FPT_array)
    print('average FPT for every run, {:.3f}'.format(avg_FPT), '+/-{:.3f} fruits/s'.format(std_FPT))  
    print() 

    #######################################################################################
    # save the velocity, fpe, fpt, and density of the runs (maybe add horizon later) in cvs
    # to do regression and find out if it can be used to set correct velocity
    # date object of today's date
    today = date.today() 
    # print("Current year:", today.year)
    # print("Current month:", today.month)
    # print("Current day:", today.day)
    # print()

    # if today.month < 10:
    #     month = str(0) + str(today.month)
    # else:
    #     month = str(today.month)

    # date_today = str(today.year) + month + str(today.day)
    date_today = '20220815' # if hardcoded is necessary

    if set_distribution == 0 or set_distribution == 6:
        # use raj's original dataset
        file_name = './csv_files/' + date_today + '_apples_' + str(v_vy_cmps) + 'v_vy_raj.csv'
    elif set_distribution == 7 or set_distribution == 8:
        # use juan's original dataset
        file_name = './csv_files/' + date_today + '_apples_' + str(v_vy_cmps) + 'v_vy.csv'      

    print('file name:', file_name)
    print()

    with open(file_name, mode='w') as wr_file:
        wr = csv.writer(wr_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        
        for i_run in range(n_runs):
            for i_snap in range(n_snapshots):
                row_data = [v_vy_cmps, FPE_array[i_run,i_snap], FPT_array[i_run,i_snap], snapshot_density[i_run,i_snap], min_row_density[i_run,i_snap], max_row_density[i_run,i_snap]]    

                wr.writerow(row_data)
            
        print('CSV written into', file_name)



def main():
    args = sys.argv[1:] # use the command line arguments to set values
    ## command line arguments
    # args[0] == n_col
    # args[1] == n_row
    # args[2] == density
    # args[3] == v_vy in cmps

    ##################### VARIABLES #####################
    # Base model
    n_col = int(args[0])  # number of columns of arms, previously n_arm 
    n_row = int(args[1])  # number of rows of arms
 
    # number of runs per variable change (for Monte Carlo)
    n_runs = 1

    # maximum velocity and acceleration, as well as the constant amount of time it takes to harvest a fruit once reached for the arm motors
    v_max              = 0.5   # these values are currently incorrect. The real values are in MIP_melon.py and they are different for each axis
    a_max              = 1.    # these values are currently incorrect. The real values are in MIP_melon.py and they are different for each axis
    t_grab             = 0.5 

    cell_l             = 0.7             # in m, length of the cell along the orchard row (y-axis), parallel to vehicle travel. Matches the prototype
    cell_h             = 2. / n_row      # in m, width/height of the horizontal row of arms (z-axis) perpendicular to vehicle travel
    arm_reach          = 1 
    l_real_y_travel    = cell_l          # in m, actual arm horizontal (y-coordinate) travel distance within a cell (measured on the prototype to be 0.46 m)

    # for MIP v_vy loop or makespan, requires an upper and lower bound for v_vy_cmps
    v_vy_lb_cmps       = int(args[3])    # in cm/s, the single velocity being tested or the lower bound for makespan or v_vy loop
    v_vy_ub_cmps       = int(args[4])    # in cm/s, when testing many velocities, this determines the top velocity tested

    v_vy_fruit_cmps    = v_vy_lb_cmps    # in cm/s, the single velocity being used also used for setup of MIP module (can this be changed and cleaned up?)
    v_vy               = v_vy_fruit_cmps / 100 # change to m/s

    vehicle_l          = n_col * cell_l  # in m, the length of the vehicle (along the y-axis)
    vehicle_h          = n_row * cell_h  # in m, the height of the vehicle (along the z-axis)

    q_vy               = 0 - vehicle_l   # in m, the start of the back of the vehicle. Negative start so the system starts before the fruits start appearing (as it moves into the row)
    q_vy_start         = q_vy            # in m, saves the start location (q_vy will change as the system 'moves' along the orchard row)

    # orchard row limis along all three axes (fruits above or below limits cannot be picked). Used especially when creating fake fruit districutions to test against
    x_lim              = [0.2, 1.2]               # in m, also determines the distance the arm can move into the canopy
    y_lim              = [q_vy_start, vehicle_l]  # in m, sets upper and lower bounds of travel for the vehicle. Can be used to stop the vehicle early with the real fruit distribution data
    z_lim              = [0., vehicle_h]          # in m, how tall the columns of the robots are 

    total_arms         = n_col * n_row

    # initialize, will change within the loop to measure global values
    FPT = 0
    FPE = 0
    total_picked = 0

    l_hor_m  = 0.3                        # in m, the extra length (horizon) in front of the robot that the robot can see

    ## set fruit distribution flag
    # 0     == Raj's digitized fruits (right side)
    # 1     == uniform random  (if algorithm == 1, use melon version)
    # 2     == uniform random, equal cell density
    # 3     == multiple densities separated by some space (only melon for now)
    # 4     == fruit in vertical columns
    # 5     == "melon" version of columns (user inputs desired no. fruits, z height, and distance between fruit in y-coord)
    # 6     == reduced Raj's digitized fruits; can reduce the density to a desired value 
    # 7     == Juan's digitized fruits
    # 8     == reduced Juan's digitized fruits; can reduce the density to a desired value 
    set_distribution = int(args[6])

    ## set algorithm being used 
    # 0     == extended TOPTW MIP model with objective to maximize the number of harvested fruit, takes one velocity or a range of velocities to determine best FPE vs. FPT
    # 1     == TOPTW MIP model with the objective to maximize FPE*FPT, includes slack variables, minFPE, and minFPT
    # 2     == makespan TOPTW MIP model with the objective to minimize makespan, includes slack variable and minFPE
    # 3     == FCFS
    # 4     == FCFS to find speed lower bound, FPE*FPT to find schedule
    # 5     == FCFS to find speed lower bound, makespan to find schedule
    # 6     == SPT
    set_algorithm = int(args[5])

    ## set MPC on or off
    # 0     == non-MPC, solve using the travel length
    # 1     == MPC, solve using the whole view window (vehicle + horizon), but only use the results up to the travel length
    set_MPC = 1

    ## set how z-coord edges are calculated
    # 0     == z-edges are divided equally along orchard height
    # 1     == z-edges are divided so each row has equal number of fruit (or close to equal)
    set_edges = 1

    ## set if the vehicle can see the whole dataset or just what's in front
    # 0     == robot sees the whole dataset
    # 1     == robot only sees what's in front of it
    # 2     == robot only sees what's in front of it plus a horizon
    set_view_field = 2

    ## set if solving per row or the whole view at once 
    # 0     == solve all rows at once 
    # 1     == solve per row 
    set_solve_row = 0

    ## set print, logging, and plot settings on or off
    # 0     == print/plot is off
    # 1     == print/plot is on
    print_out = 1  # turns printing states, indexes, and settings on/off (all or nothing)
    plot_out  = 1  # turns plotting on/off
    log_out   = 1  # turns logging of gurobi files such as ilp, lp, mlp file creation on/off (not implemented yet)

    ## set Td to be constant or variable? -> see if needed as a flag (not currently)
    # 0     == Td constant
    # 1     == Td variable based on vacuum

    # set density if specific to the set_distibution setting
    if set_distribution == 1:
        density    = float(args[2])       # in fruit/m^2, makespan is being limited to rho = 2 with random placement
    elif set_distribution == 6:
        density    = 16
    else: 
        density    = 15          # figure this out later

    if set_view_field != 2:
        # there should be no horizon if the view field doesn't call for it
        l_hor_m = 0 

    l_view_m = vehicle_l + l_hor_m  # in m, the length along the orchard row that the robot can see, moves with the vehicle

    if set_algorithm == 0 or set_algorithm >= 3:
        # have a list of vehicle velocities to loop through, if ub and lb are equal, this means a single speed will be tested
        v_vy_cmps_try = np.arange(v_vy_lb_cmps, v_vy_ub_cmps+1) # +1 to include largest value in arange
        print('velocities being attempted:', v_vy_cmps_try)
    elif set_algorithm == 1 or set_algorithm == 2:
        # makespan already takes care of the multiple velocities, set this as *one* velocity to loop through only (we don't particularily care which (we think)
        v_vy_cmps_try = np.arange(v_vy_lb_cmps, v_vy_lb_cmps+1) # +1 to include largest value in arange
        # print('velocities being attempted:', v_vy_cmps_try)

    fruit_data = fruit_handler(n_col, n_row, t_grab, x_lim, y_lim, z_lim)
    fruit_data.calcYlimMax(set_distribution)


    ##################### LISTS #####################
    run_list      = list()   # saves the results of each run for analysis
    time_list     = list()   # saves how long each run took

    seed_list = fruit_data.getRNGSeedList(n_runs)

    ##################### RUN MIP PYTHON SCRIPT #####################
    for i_run in range(n_runs):
        # create or clear the snapshot lists or they'll keep growing every run, breaking data analysis (dunno if data analysis needed for each of the multiple runs?)
        snapshot_list   = list()
        snapshot_cell   = list()
        time_snap_list  = list()
        this_seed       = list()

        # init the MIP melon object 
        if set_solve_row == 0 and set_algorithm < 3: # the first 3 algorithms are MIP-based
            n_row_loop = 1 # because the whole view is being solved at once, no need to loop
            mip_melon = MIP_melon(q_vy, n_col, n_row, 0, set_distribution, v_vy_fruit_cmps, cell_l, cell_h, vehicle_h, l_hor_m, x_lim, fruit_data.y_lim, z_lim, density)
            # set the y-coordinate movement limits within the cell if not equal to cell length. Only affects the Jobs() object if it's not equal to the cell_l 

        elif set_solve_row == 1 and set_algorithm < 3: # the first 3 algorithms are MIP-based
            n_row_loop = n_row # because we are solving per row, we need to loop through each row in each view/snapshot
            # init the MIP melon object, n_row set to one for each since we're running it per row
            # start the row at 0th row and then change the row as needed after this and before create arms
            mip_melon = MIP_melon(q_vy, n_col, 1, 0, set_distribution, v_vy_fruit_cmps, cell_l, cell_h, vehicle_h, l_hor_m, x_lim, fruit_data.y_lim, z_lim, density)
            mip_melon.addArmTravelLimits(l_real_y_travel)

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
            mip_melon = MIP_melon(q_vy, n_col, n_row, 0, set_distribution, v_vy_fruit_cmps, cell_l, cell_h, vehicle_h, l_hor_m, x_lim, fruit_data.y_lim, z_lim, density)

        elif set_algorithm == 6:
            n_row_loop = 1 # because the whole view is being solved at once, no need to loop
            # create a flag that determines when FCFS is done and MIP has to be used to determine the schedule with the new velocity range
            task_alloc_done_flag = 0
            # init the first come first serve solver
            spt = SPT()

        # set the number of snapshots if the vehicle cannot see the whole dataset and change the step length accordingly
        if set_view_field == 0:
            # global view
            n_snapshots = 1 
            l_view_m = fruit_data.y_lim[1] - fruit_data.y_lim[0]
            l_step_m = fruit_data.y_lim[1] + vehicle_l  # in m, the travel length
            FPE_min = 0.95 # in %, what is the minimum FPE desired (not exclusive) for the velocity loop setting

        elif set_view_field == 1:
            # limited view, no horizon
            l_step_m = 0.3  # in m, the travel length before resolving MIP, now the step length taken before for a snapshot 
            n_snapshots = math.ceil((fruit_data.y_lim[1] - fruit_data.y_lim[0]) / l_step_m)  # set just to do the area in front of the vehicle
            FPE_min = l_step_m / (vehicle_l)  # the ratio of traveled length to observed length (how many fruits could be harvested vs. viewed)

        else: 
            # limited view with horizon
            l_step_m = 0.6 #vehicle_l + l_hor_m  # in m, the travel length before resolving MIP, now the step length taken before for a snapshot 
            if set_MPC == 0:
                FPE_min = .35 # l_step_m / (vehicle_l + l_hor_m) * 2  # the ratio of traveled length to observed length (how many fruits could be harvested vs. viewed)
            else:
                FPE_min = .95 # l_step_m / (vehicle_l + l_hor_m) * 2  # the ratio of traveled length to observed length (how many fruits could be harvested vs. viewed)
            n_snapshots = math.ceil((fruit_data.y_lim[1] - fruit_data.y_lim[0]) / l_step_m)  # set just to do the area in front of the vehicle
            

        # arrays saving end of snapshot result values
        chosen_v_vy_mps_array = np.zeros(n_snapshots) # in m/s, array to save each snapshot's chosen velocity 
        snapshot_time_array   = np.zeros(n_snapshots) # in s, the time a snapshot takes based on chosen velocity
        global_FPE_array      = np.zeros(n_snapshots) # in %, array that saves global FPE at end of each snapshot
        global_FPT_array      = np.zeros(n_snapshots) # in fruits/s, array that saves global FPT at end of each snapshot
        mean_Td_array         = np.zeros(n_snapshots) # in s, saves the mean Td value of each snapshot

        if set_algorithm != 3 and set_algorithm != 6:
            mip_melon.setTravelLength(l_step_m)           # needed an easy and clear way to set this in mip_melon since it changes depending on the settings

        # because we're restarting the object at every run, need to only provide this run's seed list. Needs a list of lists since it assumes it could be multiple runs
        this_seed.append(seed_list[i_run])    ## might want to clean this up so it only runs when creating fake fruit distributions

        if print_out == 1:
            print()
            print('length of the full dataset:', (fruit_data.y_lim[1] - fruit_data.y_lim[0]), 'm')
            print('vehicle view window length: {:.1f} m'.format(l_view_m)) 
            print('vehicle travel per snapshot: {:.1f} m'.format(l_step_m)) 
            print('number of snapshots:', n_snapshots)
            print('FPE minimum set at {:.1f}%'.format(FPE_min*100))
            if v_vy_lb_cmps != v_vy_ub_cmps:
                # there is a range of velocities being tested
                print('\nMin velocity: %d cm/s \nMax velocity: %d cm/s' % (v_vy_lb_cmps, v_vy_ub_cmps))
            else:
                print('\nVelocity being tested: %d cm/s' % (v_vy_lb_cmps))

            # print()
            # print('this seed\n', this_seed)
            # print()
            print('-----------------------------------------------------------------')
            print('-----------------------------------------------------------------')
        
        # create the simulated environment
        fruit_data.buildOrchard(1, set_algorithm, set_distribution, this_seed)

        # save the complete dataset of fruits
        # total_sortedFruit = np.copy(mip_melon.sortedFruit)

        # # find number of problem clusters (extend later to know where the clusters are)
        # # findClustersTotal(mip_melon.sortedFruit, v_vy, mip_melon.Td, n_col)
        # fruits2remove = findClustersByRow(mip_melon.sortedFruit, v_vy, mip_melon.Td, n_col, n_row, mip_melon.z_row_bot_edges, mip_melon.z_row_top_edges)
        # for fruit_i in fruits2remove: 
        #     mip_melon.sortedFruit[4,fruit_i] = 2  ############ CAREFUL, flag=2 IS USED AS SCHED+PICK ############

        # create the arm object lists, should only be done once per full dataset if not solving by row
        if set_solve_row == 0:
            mip_arm = fruit_data.createArms()

        for i_snap in range(n_snapshots):
            fruit_picked_by = list()
            # Figure out which fruits go into this snapshot and transform their index to start with zero (save the index start)
            i_snap_sortedFruit_index  = fruit_data.fruitsInView(q_vy, l_view_m, fruit_data.sortedFruit, cell_l, l_real_y_travel)
            # make a copy of sortedFruit that works for the snapshot which will also be used to create the necessary Arm() Objects for the snapshot
            i_snap_sortedFruit        = np.copy(fruit_data.sortedFruit[:,i_snap_sortedFruit_index])
            # calculate the number of fruits in the snapshot
            i_snap_numFruit           = len(i_snap_sortedFruit[0,:])
            # update number of fruits to account for scheduled + harvested flag
            index_unavailable         = np.where(i_snap_sortedFruit[4,:] == 2)
            i_snap_available_numFruit = i_snap_numFruit - len(index_unavailable[0])

            # determine the z_edges for this snapshot
            [bot_edge, top_edge] = fruit_data.set_zEdges(set_edges, z_lim, cell_h, i_snap_sortedFruit)
            if set_algorithm != 3 and set_algorithm != 6:
                mip_melon.setZlim(bot_edge, top_edge)
            
            # need to calculate the density, R, etc. so as to determine the best v_vy value
            horizon_indexes = fruit_data.getHorizonIndex(i_snap_sortedFruit, q_vy, vehicle_l, l_hor_m)

            ## calculate multiple R and v_vy values based on multiple slices of the current view
            # return a list of fruit densities in each cell 
            d = fruit_data.calcDensity(q_vy, n_col, n_row, cell_l, arm_reach, i_snap_sortedFruit)
            # calculate the row densities
            # d_row = np.average(d, axis=1)
            # d_tot = np.average(d)

            ## using the fruit densities, determine the vehicle speed to set a specific R value?
            # currently, the R value would be 
            R = fruit_data.calcR(v_vy, len(horizon_indexes), l_hor_m, vehicle_h, arm_reach)  # calculated based on columns and the horizon length

            snapshot_cell.append([d, R])

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

            FPT = 0  # start FPT at 0 so because when doing vehicle velocity loop, FPT needs to be initilized already
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
                    print('########################### NO FRUIT, SET V = Vmax ###########################')
                    print()
                    # don't need to run the mip solver
                    break

                # if solving by row, loop thorugh the number of rows, otherwise this only runs once
                for i_loop in range(n_row_loop):
                    # need to update the 'starting row number' in the MIP_melon object
                    if set_algorithm != 3 and set_algorithm != 6:
                        mip_melon.starting_row_n = i_loop

                    if set_solve_row == 1:
                        # now create the correct arms objects for only one row
                        mip_arm = fruit_data.createArms()

                    # create the fruit object lists with updated starting_row_n
                    mip_fruit = fruit_data.createFruits(i_snap_numFruit, i_snap_sortedFruit)
                    mip_job   = fruit_data.createJobs(mip_arm, mip_fruit, v_vy_curr_cmps, q_vy, cell_l)

                    # solve for the optimal schedule for this row/loop, for this snapshot
                    if set_algorithm == 0:
                        [i_loop_fruit_picked_by, i_loop_fruit_picked_at] = mip_melon.solve_mip(mip_fruit, mip_job, i_snap_available_numFruit, v_vy_curr_cmps, set_algorithm, set_MPC, fruit_data.fruit_travel_matrix, fruit_data.sortedFruit)
                        make_v_vy = v_vy_curr_cmps
                        curr_count = np.copy(mip_melon.curr_j)

                    elif set_algorithm == 1 or set_algorithm == 2:
                        # makespan requires velocity upper and lower bounds, as well as a min FPE to find (there are default values: v_ub = 5, v_lb = 1, FPE_min = .5)
                        [i_loop_fruit_picked_by, i_loop_fruit_picked_at, make_v_vy] = mip_melon.solve_mip(mip_fruit, mip_job, i_snap_available_numFruit, v_vy_curr_cmps, set_algorithm, set_MPC, fruit_data.fruit_travel_matrix, fruit_data.sortedFruit, FPE_min=FPE_min, v_vy_lb_cmps=v_vy_lb_cmps, v_vy_ub_cmps=v_vy_ub_cmps)
                        curr_count = np.copy(mip_melon.curr_j)

                    elif (set_algorithm == 4 or set_algorithm == 5) and task_alloc_done_flag == 1:
                        # in the combo solving FCFS has finished obtaining a best low bound V, so use that to solve the schedule using MIP
                        [i_loop_fruit_picked_by, i_loop_fruit_picked_at, make_v_vy] = mip_melon.solve_mip(mip_fruit, mip_job, i_snap_available_numFruit, v_combo_lb, set_algorithm, set_MPC, fruit_data.fruit_travel_matrix, fruit_data.sortedFruit, FPE_min=FPE_min, v_vy_lb_cmps=(v_combo_lb-1), v_vy_ub_cmps=(v_combo_lb+2))
                        curr_count = np.copy(mip_melon.curr_j)

                    elif set_algorithm >= 3 and set_algorithm <= 5 and task_alloc_done_flag == 0:
                        # run FCFS algorithm
                        [i_loop_fruit_picked_by, i_loop_fruit_picked_at] = fcfs.main(n_col, n_row, mip_fruit, mip_job, v_vy_curr_cmps, q_vy, cell_l, fruit_data.fruit_travel_matrix, fruit_data.sortedFruit, fruit_data.z_row_bot_edges, fruit_data.z_row_top_edges)
                        make_v_vy = v_vy_curr_cmps
                        curr_count = np.copy(fcfs.curr_j)

                    elif set_algorithm == 6 and task_alloc_done_flag == 0:
                        # run SPT algorithm
                        [i_loop_fruit_picked_by, i_loop_fruit_picked_at] = spt.main(n_col, n_row, mip_fruit, mip_job, v_vy_curr_cmps, q_vy, cell_l, fruit_data.fruit_travel_matrix, fruit_data.sortedFruit, fruit_data.z_row_bot_edges, fruit_data.z_row_top_edges)
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

                    # go through queu manager to clean up unpickable fruit due to difference between recalculating travel distance and observed distance
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

                            queue_manager = MIP_queu_manager(q_vy, q_vy_start, v_vy_mps, l_step_m, fruit_pick_arm, fruit_when)
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
                        v_curr_fruit_picked_by[i_row][-1].clear()
                    
                    v_curr_fruit_picked_by[0][-1] = list(unpicked)

                # calculate the velocity's FPT
                v_curr_FPT = v_curr_total_picked / (l_step_m / v_vy_mps)

                # calculate the velocity's FPE 
                if i_snap_available_numFruit > 0:
                    v_curr_FPE = (v_curr_total_picked / i_snap_available_numFruit)
                else:
                    # if there are zero fruit, set FPE = 100% since it got everything. 
                    # FPT will be 0, showing/balancing out what happened
                    v_curr_FPE = 1.

                print()
                print('\nVelocity being tested: %0.2f m/s\n' %v_vy_mps)

                if v_curr_FPE >= FPE_min or v_vy_loop_i == 0 or v_curr_FPE >= FPE:
                    # if the correct FPE and FPT is found, or it's the first run but there are still too many fruits to pick even at the lowest speed
                    if v_curr_FPT > FPT or v_vy_loop_i == 0:
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
                            print('chosen_j should be zeroed out:')
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
                            break

                # compare the FPE with FPE_min to see if this velocity works
                elif v_curr_FPE < FPE_min: 
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
                        if set_algorithm == 4 or set_algorithm == 5:
                            task_alloc_done_flag = 1
                            # need loop to keep running, but there has to be a new range of velocities and switch to MIP 
                            v_combo_lb = make_v_vy

                            if v_vy_curr_cmps == v_vy_ub_cmps:
                                # might be kicked out of the loop anyway, see if avoiding that is possible by changing the value of v_vy_curr_cmps
                                v_vy_curr_cmps -= 1
                        else:
                            v_vy           = make_v_vy / 100 # in m
                            # the MIP's new solution should be used
                            FPE          = v_curr_FPE
                            FPT          = v_curr_FPT
                            total_picked = v_curr_total_picked
                            chosen_j     = np.copy(v_curr_chosen_j)   # save the v_curr_j variable for the chosen run
                            fruit_picked_by = v_curr_fruit_picked_by.copy() # copy the chosen run's fruit picked by list
                            print('########################### FORCED END RUN ###########################')
                            print()
                            # don't need to run the mip solver
                            break

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
                        print('Current unusable FPE and FPT are', v_curr_FPE, v_curr_FPT)
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
            print('Before reseting sortedFruit')
            print('Global number unpicked:', len(where_no[0]), '\nGlobal number scheduled:', len(where_sh[0]), '\nGlobal number picked', len(where_pi[0]))

            # calculate how long each arm was working vs idle
            state_time = fruit_data.calcStateTime(fruit_picked_by, mip_fruit, l_step_m, v_vy)
            # print(state_time)            

            # save chosen velocity
            chosen_v_vy_mps_array[i_snap] = v_vy
            # save the time this snapshot takes based on chosen v_vy
            snapshot_time_array[i_snap] = l_step_m / v_vy  # in s, l_step in m, v_vy in m/s

            # calculate the mean Td for the snapshot, not working yet
            this_mean_Td = fruit_data.calcMeanTd(total_picked, state_time)
            # this_mean_Td = fruit_data.calcMeanTd(mip_fruit, total_picked, state_time)
            mean_Td_array[i_snap] = this_mean_Td

            # fill in snapshot object and list with current results, object definition in MIP_melon.py
            snapshot = Snapshot(n_col, n_row, l_hor_m, vehicle_l, cell_l, v_max, a_max, set_algorithm, this_mean_Td, v_vy, FPE, FPT, fruit_data.y_lim, i_snap_numFruit, chosen_j, fruit_data.sortedFruit, fruit_picked_by, state_time)
            snapshot_list.append(snapshot)

            # calculate the global FPE for this snapshot
            this_global_FPE = len(where_pi[0]) / len(fruit_data.sortedFruit[4,:])
            global_FPE_array[i_snap] = this_global_FPE

            # calculate the global FPT for this snapshot, total picked / total time up to end of this snapshot
            this_global_FPT = len(where_pi[0]) / np.sum(snapshot_time_array)
            global_FPT_array[i_snap] = this_global_FPT

            ## continue filling if needed: PCT, state_time, fruit_picked_by, fruit_list (all lists?)

            if print_out == 1:
                # print('the fruits scheduled and harvested in this snapshot are:', i_loop_fruit_picked_by)
                # print('Number unpicked fruits at this time:', len(i_loop_fruit_picked_by[0][-1]))
                # print()
                # following print statement is based on the fruit states before harvestable scheduled fruits were set as harvested for this run
                print('Snapshot number %d finished solving' % i_snap)
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
            q_vy += l_step_m

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
        results = IG_data_analysis(snapshot_list, snapshot_cell, l_step_m, fruit_data.y_lim, set_algorithm, print_out)
        if print_out == 1:
            results.printSettings()
            print('\nThe velocities for each of the %d snapshots in m/s:' % n_snapshots)
            print(chosen_v_vy_mps_array)
            print('The mean Td for each snapshot in s:')
            print(mean_Td_array)

    #     [realFPE, realFPT] = results.realFPEandFPT(sortedFruit, y_lim, v_vy)
        results.avgFPTandFPE()
        # results.avgPCT()
        # print()
        # results.plotValuesOverDistance()
        if plot_out == 1:
            # currently results.plotTotalStatePercent() does not work with multiple snapshots
            results.plotTotalStatePercent()

            # currently plot does not save each run attempt's plot. Just the last one (they have the same name so they will get overwritten)
            snapshot_schedules_2_plot = range(n_snapshots)  
            # create a plot only of the snapshots that you want (by their index)
            print('\nWhat is being sent into plot2D', snapshot_schedules_2_plot)
            results.plot2DSchedule(snapshot_schedules_2_plot)

        run_list.append(snapshot_list)
        time_list.append(time_snap_list)

    # if n_runs > 1:
    analysisMultiple(set_distribution, n_runs, n_snapshots, run_list, time_list, snapshot_cell, print_out)
    
    
if __name__ == '__main__':
    main()