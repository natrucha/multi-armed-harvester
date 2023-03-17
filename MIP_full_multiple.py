import csv
import math
import numpy as np
from datetime import datetime
from datetime import date
from scipy.spatial import KDTree
import sys

# from fruit_distribution import *   # import module to create the various desired fruit distributions 
from IG_data_analysis import *     # import module to analyze the data from the snapshots
from MIP_melon import *            # import module that solves the extended MIP in melon algorith *add citation*
from MIP_queu_manager import *     # import module that adds queue management to the scheduler to add dynamisism

# tested with Python 3.7.0 & Gurobi 9.0

### Run MIP solver on muliple datasets to get average and variance on the results. Runs once on the data set (no horizon) and containes cluster finding functions. 


def printScen(scenStr):
    sLen = len(scenStr)
    print("\n" + "*"*sLen + "\n" + scenStr + "\n" + "*"*sLen + "\n")


# def distCenterline(n_row, z_row_bot_edges, z_row_top_edges, sortedFruit):
#     '''Calculate mean and variance of fruits to centerline of their respective row'''
#     centerline   = np.zeros(n_row)
#     sum_distance = np.zeros(n_row)
#     # where_array_list = list()

#     fruit_z = np.copy(sortedFruit[2,:])

#     for row in range(n_row):
#         # calculate centerline of row
#         centerline[row] = (z_row_top_edges[0,row] - z_row_bot_edges[0,row])/2 + z_row_bot_edges[0,row]
#         print('row', row, 'centerline z-coordinate', centerline[row])

#         # check which fruits are in this rows
#         this_row = np.where((sortedFruit[2,:] > z_row_bot_edges[0,row]) & (sortedFruit[2,:] < z_row_top_edges[0,row]))
#         # where_array_list.append(this_row[0])

#         for fruit_i in this_row[0]:
#             fruit_z[fruit_i] = np.absolute(fruit_z[fruit_i] - centerline[row])

#         # calculate row's mean and variance for distance of fruit from centerline 
#         row_mean = np.mean(fruit_z[this_row[0]]) 
#         row_var  = np.var(fruit_z[this_row[0]])
#         print('row', row, 'mean', row_mean)
#         print('row', row, 'variance', row_var)

#     # print('sortedFruit', sortedFruit[2,:])
#     # print()
#     # print('distance from the centerline', fruit_z)
#     # print()


# def findClustersTotal(sortedFruit, v_vy, Td, n_col):
#     '''
#        Use k-d tree (scipy) to find clusters of fruits made up of n_col fruits at a v_vy*Td distance from each other
#        in the y and z axis (add x when it actually matters)

#        see https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.html#scipy.spatial.KDTree
#        and https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.query_ball_tree.html#scipy.spatial.KDTree.query_ball_tree
#     '''

#     d_cluster = (v_vy * Td) /2 # find all the neighbors that can be picked, half cause radius(?)
#     print()
#     print('Distance used to find clusters:', d_cluster)
#     print()

#     problem_cluster_num = 0

#     coordinates1 = np.copy(sortedFruit[0:2,:]).T
#     coordinates2 = np.copy(sortedFruit[0:2,:]).T

#     kd_tree1 = KDTree(coordinates1)   
#     kd_tree2 = KDTree(coordinates2)

#     indexes = kd_tree1.query_ball_tree(kd_tree2, r=d_cluster)

#     print()
#     # print('Number of pairs within the problem distance', len(indexes)) # this will always be = all because we're comparing
#     # the fruit against itself? => yup, this is what happens
#     print('Fruit index lists showing all neighbors within d distance from fruit index i', indexes)

#     plt.figure(figsize=(6, 6))
#     plt.plot(sortedFruit[1,:], sortedFruit[2,:], "ok", markersize=5)
#     plt.xlabel('Distance along orchard row (m)')
#     plt.ylabel('Height from ground (m)')
#     color = ['blue', 'red', 'purple', 'chartreuse', 'black', 'aqua', 'pink', 'sienna', 'deepskyblue', 'teal', 'tomato', 'slategrey']
#     color_index = 0

#     for i in range(len(indexes)):
#         if len(indexes[i]) > n_col+1: # because the list includes the fruit's index (tree compared to itself), so one extra
#             problem_cluster_num += 1

#             for j in indexes[i]:
#                 # plot only the problem clusters
#                 line_color = str(color[color_index])
#                 plt.plot([sortedFruit[1,i], sortedFruit[1,j]], [sortedFruit[2,i], sortedFruit[2,j]], linestyle='-', color='r')

#             color_index +=1 
#             if color_index == 12:
#                 color_index = 0

#     print('Number of problem clusters', problem_cluster_num)

#     plt.show()


# def findClustersByRow(sortedFruit, v_vy, Td, n_col, n_row, z_row_bot_edges, z_row_top_edges):
#     '''
#        Use k-d tree (scipy) to find clusters of fruits made up of n_col fruits at a v_vy*Td distance from each other
#        in the y and z axis (add x when it actually matters). Done individually for each row however the row was separated

#        see https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.html#scipy.spatial.KDTree
#        and https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.query_ball_tree.html#scipy.spatial.KDTree.query_ball_tree
#     '''
#     d_cluster = (v_vy * Td)  # find all the neighbors that can be picked, half cause radius(?)

#     all_cluster_list     = list()
#     fruits2remove_list   = list()
#     # fruits2remove_list = [134,135,136,137,140,143]

#     problem_cluster_num = 0

#     coordinates1 = np.copy(sortedFruit[1:3,:]).T
#     coordinates2 = np.copy(sortedFruit[1:3,:]).T

#     for row in range(n_row):
#         # check which fruits are in this rows
#         this_row = np.where((sortedFruit[2,:] > z_row_bot_edges[0,row]) & (sortedFruit[2,:] < z_row_top_edges[0,row]))
#         # print('this row indexes', this_row[0])

#         # the indexes in the cut down coordinates array do not match the ones in coordinates, need to 'transform' them back
#         kd_tree1 = KDTree(coordinates1[this_row[0],:])   
#         kd_tree2 = KDTree(coordinates2[this_row[0],:])
#         indexes = kd_tree1.query_ball_tree(kd_tree2, r=d_cluster)

#         # transform the indexes back
#         # print()
#         for i in range(len(indexes)):
#             all_cluster_list.append([this_row[0][i],this_row[0][indexes[i]]])   

#     # print(all_cluster_list)

#     plt.figure(figsize=(6, 6))
#     plt.plot(sortedFruit[1,:], sortedFruit[2,:], "ok", markersize=5)
#     plt.xlabel('Distance along orchard row (m)')
#     plt.ylabel('Height from ground (m)')
#     color = ['blue', 'red', 'purple', 'chartreuse', 'black', 'aqua', 'pink', 'sienna', 'deepskyblue', 'teal', 'tomato', 'slategrey']
#     color_index = 0

#     # print()
#     # print('sorted fruits list')
#     # print(sortedFruit)

#     print()  # helps make the following prints cleaner 
#     for cluster in all_cluster_list:
#         if len(cluster[1]) > n_col:
#             # if there are more fruits in the cluster than the number of arms+1
#             problem_cluster_num += 1
#             # get the correct index connected to the list of neighbors
#             i = cluster[0]
#             print('Fruits within d distance from fruit index',i, 'are', cluster[1])

#             # tag these fruits as removed, going to remove the index fruit which is the one close to all the other ones
#             # fruits2remove_list.append(i)

#             for j in cluster[1]:
#                 # plot only the problem clusters
#                 line_color = str(color[color_index])
#                 plt.plot([sortedFruit[1,i], sortedFruit[1,j]], [sortedFruit[2,i], sortedFruit[2,j]], linestyle='-', color=line_color)

#             color_index +=1 
#             if color_index == 12:
#                 color_index = 0
                
#     plt.show()

#     return(fruits2remove_list)



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



def analysisMultiple(n_runs, n_snapshots, run_list, time_list, snapshot_cell, print_out):
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

    if today.month < 10:
        month = str(0) + str(today.month)
    else:
        month = str(today.month)

    # date_today = str(today.year) + month + str(today.day)
    date_today = '20220815' # if hardcoded is necessary

    data_name = 'raj'

    if data_name == 'juan':
        file_name = './csv_files/' + date_today + '_apples_' + str(v_vy_cmps) + 'v_vy.csv'
    elif data_name == 'raj':
        file_name = './csv_files/' + date_today + '_apples_' + str(v_vy_cmps) + 'v_vy_raj.csv'

    print('file name:', file_name)
    print()

    with open(file_name, mode='w') as wr_file:
        wr = csv.writer(wr_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        
        for i_run in range(n_runs):
            for i_snap in range(n_snapshots):
                row_data = [v_vy_cmps, FPE_array[i_run,i_snap], FPT_array[i_run,i_snap], snapshot_density[i_run,i_snap], min_row_density[i_run,i_snap], max_row_density[i_run,i_snap]]    

                wr.writerow(row_data)
            
        print('CSV written into', file_name)




def fruitsInView(q_vy, l_view_m, total_sortedFruit, l_cell, l_arm_travel):
# def fruitsInView(q_vy, n_snapshots, l_view_m, total_sortedFruit, cell_l, pick_travel_l):
    '''Determine which fruits are unpicked and in front of the vehicle for each snapshot.'''
    
    # offset = (l_cell - l_arm_travel) / 2 # assume centered in cell
    start_y = q_vy                       # the y-coordinate start of the start of the view window for the snapshot ((0,0) for the vehicle at start of snapshot)

    index_this_sortedFruit = np.where((total_sortedFruit[1,:] > start_y) & (total_sortedFruit[1,:] < start_y + l_view_m))
    
    # print()
    # print('index of fruits in the view window', index_this_sortedFruit[0])
    # print()

    return(index_this_sortedFruit[0])


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
    # t_grab     = 0.1 

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
    # 0     == Raj's and Juan's digitized fruits -> the specific one has to be set manually here and in MIP_melon.py
    # 1     == uniform random  (if algorithm == 1, use melon version)
    # 2     == uniform random, equal cell density
    # 3     == multiple densities separated by some space (only melon for now)
    # 4     == fruit in vertical columns
    # 5     == "melon" version of columns (user inputs desired no. fruits, z height, and distance between fruit in y-coord)
    # 6     == Raj's and Juan's digitized fruits, but can reduce the density to a desired value 
    set_distribution = 0

    ## set algorithm being used 
    # 1     == MIP
    # not 1 == not MIP? say FCFS or SPT
    set_algorithm    = 1

    ## set MIP model settings
    # 0     == extended TOPTW MIP model with objective to maximize the number of harvested fruit, takes one velocity or a range of velocities to determine best FPE vs. FPT
    # 1     == makespan TOPTW MIP model with multiple objectives to minimize makespan and maximize number of harvested fruits
    set_MIPsettings = 1

    ## set how z-coord edges are calculated
    # 0     == z-edges are divided equally along orchard height
    # 1     == z-edges are divided so each row has equal number of fruit (or close to equal)
    set_edges = 0

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
    plot_out  = 0  # turns plotting on/off
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

    if set_MIPsettings == 0:
        # have a list of vehicle velocities to loop through, if ub and lb are equal, this means a single speed will be tested
        v_vy_cmps_try = np.arange(v_vy_lb_cmps, v_vy_ub_cmps+1) # +1 to include largest value in arange
        print('velocities being attempted:', v_vy_cmps_try)
    elif set_MIPsettings == 1:
        # makespan already takes care of the multiple velocities, set this as *one* velocity to loop through only (we don't particularily care which (we think)
        v_vy_cmps_try = np.arange(v_vy_lb_cmps, v_vy_lb_cmps+1) # +1 to include largest value in arange
        # print('velocities being attempted:', v_vy_cmps_try)


    ##################### LISTS #####################
    run_list      = list()   # saves the results of each run for analysis
    time_list     = list()   # saves how long each run took

    seed_list = getRNGSeedList(n_runs)

    ##################### RUN MIP PYTHON SCRIPT #####################
    for i_run in range(n_runs):
        # create or clear the snapshot lists or they'll keep growing every run, breaking data analysis (dunno if data analysis needed for each of the multiple runs?)
        snapshot_list   = list()
        snapshot_cell   = list()
        time_snap_list  = list()
        this_seed       = list()

        # init the MIP melon object 
        if set_solve_row == 0:
            n_row_loop = 1 # because the whole view is being solved at once, no need to loop
            mip_melon = MIP_melon(q_vy, n_col, n_row, 0, set_distribution, v_vy_fruit_cmps, cell_l, cell_h, vehicle_h, l_hor_m, x_lim, y_lim, z_lim, density)

        elif set_solve_row == 1:
            n_row_loop = n_row # because we are solving per row, we need to loop through each row in each view/snapshot
            # init the MIP melon object, n_row set to one for each since we're running it per row
            # start the row at 0th row and then change the row as needed after this and before create arms
            mip_melon = MIP_melon(q_vy, n_col, 1, 0, set_distribution, v_vy_fruit_cmps, cell_l, cell_h, vehicle_h, l_hor_m, x_lim, y_lim, z_lim, density)

        # set the y-coordinate movement limits within the cell if not equal to cell length. Only affects the Jobs() object if it's not equal to the cell_l 
        mip_melon.addArmTravelLimits(l_real_y_travel) 

        # set the number of snapshots if the vehicle cannot see the whole dataset and change the step length accordingly
        if set_view_field == 0:
            # global view
            n_snapshots = 1 
            l_view_m = mip_melon.y_lim[1] - mip_melon.y_lim[0]
            l_step_m = mip_melon.y_lim[1] + vehicle_l  # in m, the travel length
            FPE_min = 0.95 # in %, what is the minimum FPE desired (not exclusive) for the velocity loop setting

        elif set_view_field == 1:
            # limited view, no horizon
            l_step_m = 0.3  # in m, the travel length before resolving MIP, now the step length taken before for a snapshot 
            n_snapshots = math.ceil((mip_melon.y_lim[1] - mip_melon.y_lim[0]) / l_step_m)  # set just to do the area in front of the vehicle
            FPE_min = l_step_m / (vehicle_l)  # the ratio of traveled length to observed length (how many fruits could be harvested vs. viewed)

        else: 
            # limited view with horizon
            l_step_m = 0.6  # in m, the travel length before resolving MIP, now the step length taken before for a snapshot 
            n_snapshots = math.ceil((mip_melon.y_lim[1] - mip_melon.y_lim[0]) / l_step_m)  # set just to do the area in front of the vehicle
            FPE_min = .35 # l_step_m / (vehicle_l + l_hor_m) * 2  # the ratio of traveled length to observed length (how many fruits could be harvested vs. viewed)

        # arrays saving end of snapshot result values
        chosen_v_vy_mps_array = np.zeros(n_snapshots) # in m/s, array to save each snapshot's chosen velocity 
        snapshot_time_array   = np.zeros(n_snapshots) # in s, the time a snapshot takes based on chosen velocity
        global_FPE_array      = np.zeros(n_snapshots) # in %, array that saves global FPE at end of each snapshot
        global_FPT_array      = np.zeros(n_snapshots) # in fruits/s, array that saves global FPT at end of each snapshot
        mean_Td_array         = np.zeros(n_snapshots) # in s, saves the mean Td value of each snapshot

        mip_melon.setTravelLength(l_step_m)           # needed an easy and clear way to set this in mip_melon since it chnages depending on the settings

        # because we're restarting the object at every run, need to only provide this run's seed list. Needs a list of lists since it assumes it could be multiple runs
        this_seed.append(seed_list[i_run])    ## might want to clean this up so it only runs when creating fake fruit distributions

        if print_out == 1:
            print()
            print('length of the full dataset:', (mip_melon.y_lim[1] - mip_melon.y_lim[0]), 'm')
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
        mip_melon.buildOrchard(1, set_algorithm, set_distribution, this_seed)

        # save the complete dataset of fruits
        total_sortedFruit = np.copy(mip_melon.sortedFruit)

        # # find number of problem clusters (extend later to know where the clusters are)
        # # findClustersTotal(mip_melon.sortedFruit, v_vy, mip_melon.Td, n_col)
        # fruits2remove = findClustersByRow(mip_melon.sortedFruit, v_vy, mip_melon.Td, n_col, n_row, mip_melon.z_row_bot_edges, mip_melon.z_row_top_edges)
        # for fruit_i in fruits2remove: 
        #     mip_melon.sortedFruit[4,fruit_i] = 2  ############ CAREFUL, flag=2 IS USED AS SCHED+PICK ############

        # create the arm object lists, should only be done once per full dataset if not solving by row
        if set_solve_row == 0:
            mip_arm = mip_melon.createArms()

        for i_snap in range(n_snapshots):
            fruit_picked_by = list()
            # Figure out which fruits go into this snapshot and transform their index to start with zero (save the index start)
            i_snap_sortedFruit_index  = fruitsInView(mip_melon.q_vy, l_view_m, mip_melon.sortedFruit, cell_l, l_real_y_travel)
            # make a copy of sortedFruit that works for the snapshot which will also be used to create the necessary Arm() Objects for the snapshot
            i_snap_sortedFruit        = np.copy(mip_melon.sortedFruit[:,i_snap_sortedFruit_index])
            # calculate the number of fruits in the snapshot
            i_snap_numFruit           = len(i_snap_sortedFruit[0,:])
            # update number of fruits to account for scheduled + harvested flag
            index_unavailable         = np.where(i_snap_sortedFruit[4,:] == 2)
            i_snap_available_numFruit = i_snap_numFruit - len(index_unavailable[0])

            # determine the z_edges for this snapshot
            mip_melon.set_zEdges(set_edges, z_lim, n_row, i_snap_numFruit, i_snap_sortedFruit)
            
            # need to calculate the density, R, etc. so as to determine the best v_vy value
            horizon_indexes = mip_melon.getHorizonIndex(mip_melon.sortedFruit, mip_melon.q_vy, vehicle_l)

            ## calculate multiple R and v_vy values based on multiple slices of the current view
            # return a list of fruit densities in each cell 
            d = mip_melon.calcDensity(mip_melon.q_vy, v_vy, n_row, n_col, cell_l, arm_reach, i_snap_sortedFruit)
            # calculate the row densities
            d_row = np.average(d, axis=1)
            d_tot = np.average(d)

            ## using the fruit densities, determine the vehicle speed to set a specific R value?
            # currently, the R value would be 
            R = mip_melon.calcR(v_vy, len(horizon_indexes), vehicle_h, arm_reach)  # calculated based on columns and the horizon length

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
                print('Vehicle location at start of this snapshot {:.1f} m on y-axis \n'.format(mip_melon.q_vy))

            printScen("Solving base scenario model")

            FPT = 0  # start FPT at 0 so because when doing vehicle velocity loop, FPT needs to be initilized already
            # solution_found = 0 # changes to 1 if at least one solution fits desired min values
            start_timer = datetime.now() # to figure out how looping through v_vy compares to pure MIP

            # needed to know if this is the first attempted v_vy value or not
            v_vy_loop_i = 0

            # loop throught the list of velocity values 
            for v_vy_curr_cmps in v_vy_cmps_try:
                # how many times will we loop through a snapshot and velocity attempt also depends on if we're solving once or by row
                # if solving by row, loop thorugh the number of rows, otherwise this only runs once
                for i_loop in range(n_row_loop):
                    # need to update the 'starting row number' in the MIP_melon object
                    mip_melon.starting_row_n = i_loop

                    if set_solve_row == 1:
                        # now create the correct arms objects for only one row
                        mip_arm = mip_melon.createArms()

                    # create the fruit object lists with updated starting_row_n
                    mip_fruit = mip_melon.createFruits(i_snap_numFruit, i_snap_sortedFruit)

                    # solve for the optimal schedule for this row/loop, for this snapshot
                    if set_MIPsettings == 0:
                        [i_loop_fruit_picked_by, i_loop_fruit_picked_at] = mip_melon.solve_melon_mip(mip_arm, mip_fruit, i_snap_available_numFruit, v_vy_curr_cmps, set_MIPsettings)
                        make_v_vy = v_vy_curr_cmps
                    elif set_MIPsettings == 1:
                        # makespan requires velocity upper and lower bounds, as well as a min FPE to find (there are default values: v_ub = 5, v_lb = 1, FPE_min = .5)
                        [i_loop_fruit_picked_by, i_loop_fruit_picked_at, make_v_vy] = mip_melon.solve_melon_mip(mip_arm, mip_fruit, i_snap_available_numFruit, v_vy_curr_cmps, set_MIPsettings, FPE_min=FPE_min, v_vy_lb_cmps=v_vy_lb_cmps, v_vy_ub_cmps=v_vy_ub_cmps)
                        
                    # set the current velocity as the makespan's chosen velocity
                    v_vy_mps = make_v_vy / 100 # change to m/s to because it starts getting used from here on out

                    # print('the fruits scheduled to be picked in this snapshot are:', i_loop_fruit_picked_by)
                    # print('the times at which the fruits were picked are:', i_loop_fruit_picked_at)
                    # print()

                    # print('original curr_j\n', mip_melon.curr_j)

                    # check what fruits have been harvested, scheduled, or not picked
                    where_no = np.where(mip_melon.sortedFruit[4,:] == 0)
                    where_sh = np.where(mip_melon.sortedFruit[4,:] == 1)
                    where_pi = np.where(mip_melon.sortedFruit[4,:] == 2)
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

                            queue_manager = MIP_queu_manager(mip_melon.q_vy, q_vy_start, v_vy_mps, l_step_m, fruit_pick_arm, fruit_when)
                            not_picked = queue_manager.unpicked_queue
                            yes_picked = queue_manager.picked_queue
                            
                            if n_row > 1:
                                i_loop_fruit_picked_by[row][column] = list(yes_picked)

                                # update curr_j for this arm, to then be able to update chosen_j
                                mip_melon.curr_j[row, column] = queue_manager.updateSumPicked(yes_picked)
                                # print('changes to curr_j in row', row, 'and column', column, 'results in\n', mip_melon.curr_j)
                                    
                                # update sortedFruit so that fruit that was not picked in time gets reset from picked to not picked
                                if len(not_picked) > 0:
                                    i_loop_fruit_picked_by[0][n_col] = queue_manager.updateUnpicked(i_loop_fruit_picked_by[0][n_col])
                                    mip_melon.sortedFruit[4,not_picked] = 0  # not_picked should not be used later, so safe to do this here?
                                    # print('not picked list', not_picked)
                            else:
                                i_loop_fruit_picked_by[column] = list(yes_picked)

                                # update curr_j for this arm, to then be able to update chosen_j
                                mip_melon.curr_j[0,column] = queue_manager.updateSumPicked(yes_picked)
                                # print('changes to curr_j in row', row, 'and column', column, 'results in\n', mip_melon.curr_j)
                                    
                                # update sortedFruit so that fruit that was not picked in time gets reset from picked to not picked
                                if len(not_picked) > 0:
                                    i_loop_fruit_picked_by[n_col] = queue_manager.updateUnpicked(i_loop_fruit_picked_by[n_col])
                                    mip_melon.sortedFruit[4,not_picked] = 0  # not_picked should not be used later, so safe to do this here?
                                    # print('not picked list', not_picked)

                    # calculate how many fruits each arm picked having cleaned out unpickable fruit above
                    if i_loop > 0:
                        # stack the row's number of fruits picked by each arm
                        v_curr_chosen_j = np.vstack((v_curr_chosen_j, np.copy(mip_melon.curr_j))) 
                        # now figure out what fruit were not picked
                        # see https://stackoverflow.com/questions/16163546/checking-to-see-if-same-value-in-two-lists
                        unpicked = set(unpicked).intersection(i_loop_fruit_picked_by[-1])
                    else: 
                        # initialization for stack the row's number of fruits picked by each arm
                        v_curr_chosen_j = np.copy(mip_melon.curr_j)
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
                where_no = np.where(mip_melon.sortedFruit[4,:] == 0)
                where_sh = np.where(mip_melon.sortedFruit[4,:] == 1)
                where_pi = np.where(mip_melon.sortedFruit[4,:] == 2)
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

                if v_curr_FPE >= FPE_min or v_vy_loop_i == 0:
                    # if the correct FPE and FPT is found, or it's the first run but there are still too many fruits to pick even at the lowest speed
                    if v_curr_FPT > FPT or v_vy_loop_i == 0:
                        # a new solution can be used, though check if there are any fruits available to see if velocity should beset to upper bound and break out of loop
                        FPE          = v_curr_FPE
                        FPT          = v_curr_FPT
                        total_picked = v_curr_total_picked
                        chosen_j     = np.copy(v_curr_chosen_j)   # save the v_curr_j variable for the chosen run
                        fruit_picked_by = v_curr_fruit_picked_by.copy() # copy the chosen run's fruit picked by list

                        if i_snap_available_numFruit > 0:
                            # if there are fruits, the solution works and should be saved as is
                            print('***** NEW SOLUTION SAVED *****')
                            # solution_found = 1
                            v_vy           = v_vy_mps # in m
                            
                        else:
                            print('***** NO FRUITS AVAILABLE, BREAKING OUT *****')
                            # there are probably no fruits in this run, need to break out with speed at highest value
                            # solution_found = 1    # technically, a solution was found, it's just not great
                            v_vy           = v_vy_ub_cmps / 100 # in mv_vy_mps

                            # nothing picked, so zero out chosen_j -> check first if necessary
                            zero_j = np.zeros([n_row, n_col])
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
                                        mip_melon.sortedFruit[4,v_curr_fruit_picked_by[row][column]] = 0
                                    else:
                                        mip_melon.sortedFruit[4,v_curr_fruit_picked_by[column]] = 0

                            print('########################### FORCED END RUN ###########################')
                            print()
                            break

                            # move some stuff over here. Have fun!!!!!  
                    
                    # else:
                    #     # no improvement in FPT means that this is not a viable solution, keep looping
                    #     solution_found = 0

                # compare the FPE with FPE_min to see if this velocity works
                elif v_curr_FPE < FPE_min: 
                    print('***** NO IMPROVEMENT IN FPE POSSIBLE, BREAKING OUT *****')
                    # there is no way FPE rises as velocity rises, so either a solution was found, or it wasn't 

                    # clear the sortedFruit changes made in this run or the scheduled but not harvested results bleed over to the following runs
                    # fruit_picked_by has been saved and will be used to mark fruits as harvested if this ends up being the final solution
                    for row in range(n_row):
                        for column in range(n_col):
                            if n_row > 1:
                                mip_melon.sortedFruit[4,v_curr_fruit_picked_by[row][column]] = 0 # try removing all v_curr_fruit_picked_by since fruit_picked_by may not be updated
                            else:
                                mip_melon.sortedFruit[4,v_curr_fruit_picked_by[column]] = 0 

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
                            mip_melon.sortedFruit[4,v_curr_fruit_picked_by[row][column]] = 0 # try removing all v_curr_fruit_picked_by since it may not update 
                        else:
                            mip_melon.sortedFruit[4,v_curr_fruit_picked_by[column]] = 0 # try removing all v_curr_fruit_picked_by since it may not update 

                v_vy_loop_i += 1

            # if solution_found == 1:
            # use fruit_picked_by to manage sortedFruit and switch new scheduled only fruit to scheduled and picked
            # possible because sceduled but not picked were already 'removed' earlier during the queue managemet phase
            for row in range(n_row):
                for column in range(n_col):
                    if n_row > 1:
                        # print('getting list index out of range, so check row =', row, 'column =',column)
                        mip_melon.sortedFruit[4,fruit_picked_by[row][column]] = 2
                    else:
                        mip_melon.sortedFruit[4,fruit_picked_by[column]] = 2

            # # check what fruits have been harvested, scheduled, or not picked
            where_no = np.where(mip_melon.sortedFruit[4,:] == 0)
            where_sh = np.where(mip_melon.sortedFruit[4,:] == 1)
            where_pi = np.where(mip_melon.sortedFruit[4,:] == 2)
            # print('Before reseting sortedFruit')
            # print('Global number unpicked:', len(where_no[0]), '\nGlobal number scheduled:', len(where_sh[0]), '\nGlobal number picked', len(where_pi[0]))

            # calculate how long each arm was working vs idle
            state_time = mip_melon.calcStateTime(fruit_picked_by, l_view_m, v_vy, total_arms, n_row, n_col, mip_melon.Td)

            # fill in snapshot object and list with current results, object definition in MIP_melon.py
            snapshot = Snapshot(n_col, n_row, l_hor_m, vehicle_l, mip_melon.cell_l, v_max, a_max, set_algorithm, mip_melon.Td, v_vy, FPE, FPT, mip_melon.y_lim, i_snap_numFruit, chosen_j, mip_melon.sortedFruit, fruit_picked_by, state_time)
            snapshot_list.append(snapshot)

            # save chosen velocity
            chosen_v_vy_mps_array[i_snap] = v_vy
            # save the time this snapshot takes based on chosen v_vy
            snapshot_time_array[i_snap] = l_step_m / v_vy  # in s, l_step in m, v_vy in m/s

            # calculate the mean Td for the snapshot, not working yet
            this_mean_Td = mip_melon.calcMeanTd(fruit_picked_by, mip_fruit, total_picked)
            mean_Td_array[i_snap] = this_mean_Td

            # calculate the global FPE for this snapshot
            this_global_FPE = len(where_pi[0]) / len(mip_melon.sortedFruit[4,:])
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
                print('\Snapshot number %d finished solving' % i_snap)
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
            mip_melon.q_vy += l_step_m

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
        results = IG_data_analysis(snapshot_list, snapshot_cell, mip_melon.travel_l, mip_melon.y_lim, set_algorithm, print_out)
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
    analysisMultiple(n_runs, n_snapshots, run_list, time_list, snapshot_cell, print_out)
    
    
if __name__ == '__main__':
    main()