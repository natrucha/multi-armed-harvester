import math
import numpy as np
from datetime import datetime
from scipy.spatial import KDTree
import sys

# from fruit_distribution import *   # import module to create the various desired fruit distributions 
from IG_data_analysis import *     # import module to analyze the data from the snapshots
from MIP_melon import *            # import module that solves the extended MIP in melon algorith *add citation*

# tested with Python 3.7.0 & Gurobi 9.0

### Run MIP solver on muliple datasets to get average and variance on the results. Runs once on the data set (no horizon) and containes cluster finding functions. 


def printScen(scenStr):
    sLen = len(scenStr)
    print("\n" + "*"*sLen + "\n" + scenStr + "\n" + "*"*sLen + "\n")


def distCenterline(n_row, z_row_bot_edges, z_row_top_edges, sortedFruit):
    '''Calculate mean and variance of fruits to centerline of their respective row'''
    centerline   = np.zeros(n_row)
    sum_distance = np.zeros(n_row)
    # where_array_list = list()

    fruit_z = np.copy(sortedFruit[2,:])

    for row in range(n_row):
        # calculate centerline of row
        centerline[row] = (z_row_top_edges[0,row] - z_row_bot_edges[0,row])/2 + z_row_bot_edges[0,row]
        print('row', row, 'centerline z-coordinate', centerline[row])

        # check which fruits are in this rows
        this_row = np.where((sortedFruit[2,:] > z_row_bot_edges[0,row]) & (sortedFruit[2,:] < z_row_top_edges[0,row]))
        # where_array_list.append(this_row[0])

        for fruit_i in this_row[0]:
            fruit_z[fruit_i] = np.absolute(fruit_z[fruit_i] - centerline[row])

        # calculate row's mean and variance for distance of fruit from centerline 
        row_mean = np.mean(fruit_z[this_row[0]]) 
        row_var  = np.var(fruit_z[this_row[0]])
        print('row', row, 'mean', row_mean)
        print('row', row, 'variance', row_var)

    # print('sortedFruit', sortedFruit[2,:])
    # print()
    # print('distance from the centerline', fruit_z)
    # print()


def findClustersTotal(sortedFruit, v_vy, Td, n_arm):
    '''
       Use k-d tree (scipy) to find clusters of fruits made up of n_arm fruits at a v_vy*Td distance from each other
       in the y and z axis (add x when it actually matters)

       see https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.html#scipy.spatial.KDTree
       and https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.query_ball_tree.html#scipy.spatial.KDTree.query_ball_tree
    '''

    d_cluster = (v_vy * Td) /2 # find all the neighbors that can be picked, half cause radius(?)
    print()
    print('Distance used to find clusters:', d_cluster)
    print()

    problem_cluster_num = 0

    coordinates1 = np.copy(sortedFruit[0:2,:]).T
    coordinates2 = np.copy(sortedFruit[0:2,:]).T

    kd_tree1 = KDTree(coordinates1)   
    kd_tree2 = KDTree(coordinates2)

    indexes = kd_tree1.query_ball_tree(kd_tree2, r=d_cluster)

    print()
    # print('Number of pairs within the problem distance', len(indexes)) # this will always be = all because we're comparing
    # the fruit against itself? => yup, this is what happens
    print('Fruit index lists showing all neighbors within d distance from fruit index i', indexes)

    plt.figure(figsize=(6, 6))
    plt.plot(sortedFruit[1,:], sortedFruit[2,:], "ok", markersize=5)
    plt.xlabel('Distance along orchard row (m)')
    plt.ylabel('Height from ground (m)')
    color = ['blue', 'red', 'purple', 'chartreuse', 'black', 'aqua', 'pink', 'sienna', 'deepskyblue', 'teal', 'tomato', 'slategrey']
    color_index = 0

    for i in range(len(indexes)):
        if len(indexes[i]) > n_arm+1: # because the list includes the fruit's index (tree compared to itself), so one extra
            problem_cluster_num += 1

            for j in indexes[i]:
                # plot only the problem clusters
                line_color = str(color[color_index])
                plt.plot([sortedFruit[1,i], sortedFruit[1,j]], [sortedFruit[2,i], sortedFruit[2,j]], linestyle='-', color='r')

            color_index +=1 
            if color_index == 12:
                color_index = 0

    print('Number of problem clusters', problem_cluster_num)

    plt.show()


def findClustersByRow(sortedFruit, v_vy, Td, n_arm, n_row, z_row_bot_edges, z_row_top_edges):
    '''
       Use k-d tree (scipy) to find clusters of fruits made up of n_arm fruits at a v_vy*Td distance from each other
       in the y and z axis (add x when it actually matters). Done individually for each row however the row was separated

       see https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.html#scipy.spatial.KDTree
       and https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.query_ball_tree.html#scipy.spatial.KDTree.query_ball_tree
    '''
    d_cluster = (v_vy * Td)  # find all the neighbors that can be picked, half cause radius(?)

    all_cluster_list     = list()
    fruits2remove_list   = list()
    # fruits2remove_list = [134,135,136,137,140,143]

    problem_cluster_num = 0

    coordinates1 = np.copy(sortedFruit[1:3,:]).T
    coordinates2 = np.copy(sortedFruit[1:3,:]).T

    for row in range(n_row):
        # check which fruits are in this rows
        this_row = np.where((sortedFruit[2,:] > z_row_bot_edges[0,row]) & (sortedFruit[2,:] < z_row_top_edges[0,row]))
        # print('this row indexes', this_row[0])

        # the indexes in the cut down coordinates array do not match the ones in coordinates, need to 'transform' them back
        kd_tree1 = KDTree(coordinates1[this_row[0],:])   
        kd_tree2 = KDTree(coordinates2[this_row[0],:])
        indexes = kd_tree1.query_ball_tree(kd_tree2, r=d_cluster)

        # transform the indexes back
        # print()
        for i in range(len(indexes)):
            all_cluster_list.append([this_row[0][i],this_row[0][indexes[i]]])   

    # print(all_cluster_list)

    plt.figure(figsize=(6, 6))
    plt.plot(sortedFruit[1,:], sortedFruit[2,:], "ok", markersize=5)
    plt.xlabel('Distance along orchard row (m)')
    plt.ylabel('Height from ground (m)')
    color = ['blue', 'red', 'purple', 'chartreuse', 'black', 'aqua', 'pink', 'sienna', 'deepskyblue', 'teal', 'tomato', 'slategrey']
    color_index = 0

    # print()
    # print('sorted fruits list')
    # print(sortedFruit)

    print()  # helps make the following prints cleaner 
    for cluster in all_cluster_list:
        if len(cluster[1]) > n_arm:
            # if there are more fruits in the cluster than the number of arms+1
            problem_cluster_num += 1
            # get the correct index connected to the list of neighbors
            i = cluster[0]
            print('Fruits within d distance from fruit index',i, 'are', cluster[1])

            # tag these fruits as removed, going to remove the index fruit which is the one close to all the other ones
            # fruits2remove_list.append(i)

            for j in cluster[1]:
                # plot only the problem clusters
                line_color = str(color[color_index])
                plt.plot([sortedFruit[1,i], sortedFruit[1,j]], [sortedFruit[2,i], sortedFruit[2,j]], linestyle='-', color=line_color)

            color_index +=1 
            if color_index == 12:
                color_index = 0
                
    plt.show()

    return(fruits2remove_list)



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



def analysisMultiple(n_runs, n_snapshots, run_list, time_list, print_out):
    '''Obtain average and variance of results and time-to-run for all the runs'''
    time_array = np.zeros([n_runs, n_snapshots])
    FPE_array  = np.zeros([n_runs, n_snapshots])
    FPT_array  = np.zeros([n_runs, n_snapshots])

    print()
    print('----------------------------------------------')
    print('  Analyze results for all runs and snapshots  ')
    print('----------------------------------------------')
    print()

    for i_run in range(n_runs):
        for i_snap in range(n_snapshots):
            time_array[i_run,i_snap] = time_list[i_run][i_snap].total_seconds()
            # run list is made up of snapshot lists which ay come into play when there are multiple snapshots. For now, only need the 0th object in the list
            FPE_array[i_run,i_snap]  = run_list[i_run][i_snap].FPE * 100
            FPT_array[i_run,i_snap]  = run_list[i_run][i_snap].FPT

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
        print('    average FPE, {:.3f}'.format(avg_FPT_run), '+/-{:.3f}%'.format(std_FPT_run)) 
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



def fruitsInView(n_snapshots, view_distance_m, total_sortedFruit):
    '''Determine which fruits are in front of the vehicle for each snapshot'''
    this_sortedFruit = list()

    start_y = 0

    for i_snap in range(n_snapshots):
        # find all the fruits in the snapshot's view
        current = np.where((total_sortedFruit[1,:] > start_y) & (total_sortedFruit[1,:] < start_y + view_distance_m))
        # save the list with the indexes of the fruits that go in each snapshot
        this_sortedFruit.append(current[0])

        start_y += view_distance_m

    return(this_sortedFruit)


def main():
    args = sys.argv[1:] # use the command line arguments to set values
    ## command line arguments
    # args[0] == n_arm
    # args[1] == n_row
    # args[2] == density

    ##################### VARIABLES #####################
    # Base model
    n_arm = int(args[0])
    n_row = int(args[1])

    # number of runs per variable change
    n_runs = 1

    v_max      = 0.5
    a_max      = 1.
    # t_grab     = 0.1 

    cell_l     = 0.3        # in m, length of the cell along the orchard row (y-axis), parallel to vehicle travel
    cell_h     = 2. / n_row # in m, width/height of the horizontal row of arms (z-axis) perpendicular to vehicle travel
    arm_reach  = 1  

    v_vy_fruit_cmps = 8  # in cm/s
    v_vy = v_vy_fruit_cmps / 100 # change to m/s

    vehicle_l  = n_arm * cell_l
    vehicle_h  = n_row * cell_h

    x_lim     = [0.2, 1.2]
    y_lim     = [0., vehicle_l]
    z_lim     = [0., vehicle_h] 

    total_arms = n_arm * n_row

    # initialize, will be added into in a loop
    FPT = 0
    FPE = 0
    total_picked = 0

    horizon_m = 0                            # in m, the extra length (horizon) in front of the robot that the robot can see
    view_distance_m = vehicle_l + horizon_m  # in m, the length along the orchard row that the robot can see, moves with the vehicle

    ## set fruit distribution flag
    # 0     == Raj's digitized fruits
    # 1     == uniform random  (if algorithm == 1, use melon version)
    # 2     == uniform random, equal cell density
    # 3     == multiple densities separated by some space (only melon for now)
    # 4     == fruit in vertical columns
    # 5     == "melon" version of columns (user inputs desired no. fruits, z height, and distance between fruit in y-coord)
    # 6     == Raj's digitized fruits, but which can reduce the density to a desired density
    set_distribution = 6

    ## set algorithm being used 
    # 1     == melon
    # not 1 == not melon?
    set_algorithm    = 1

    ## set MIP model settings
    # 0     == basic MIP model from the melon paper with arms not sharing space
    # 1     == basic MIP model with velocity as a variable
    # 2     == makespan MIP (have seperate code, don't use this until proven the same as the other)
    set_MIPsettings = 0

    ## set how z-coord edges are calculated
    # 0     == edges are divided equally along orchard height
    # 1     == edges are divided so each row has equal number of fruit (or close to equal)
    set_edges = 1

    ## set if the vehicle can see the whole dataset or just what's in front
    # 0     == robot sees the whole dataset
    # 1     == robot only sees what's in front of it plus a horizon
    set_view_field = 1

    ## set print and plot settings on or off
    # 0     == print/plot is off
    # 1     == print/plot is on
    print_out = 1
    plot_out  = 1

    # set density if specific to the set_distibution setting
    if set_distribution == 1:
        density    = float(args[2])       # in fruit/m^2, makespan is being limited to rho = 2 with random placement
    elif set_distribution == 6:
        density    = 16
    else: 
        density    = 15          # figure this out later


    ##################### LISTS #####################
    run_list      = list()   # saves the results of each run for analysis
    time_list     = list()   # saves how long each run took

    seed_list = getRNGSeedList(n_runs)

    ##################### RUN MIP PYTHON SCRIPT #####################
    for i_run in range(n_runs):
        # create or clear the snapshot lists or they'll keep growing every run, breaking data analysis (dunno if data analysis needed?)
        snapshot_list  = list()
        snapshot_cell  = list()
        time_snap_list = list()
        this_seed      = list()

        # init the MIP melon object 
        mip_melon = MIP_melon(n_arm, n_row, 0, set_distribution, set_algorithm, set_MIPsettings, set_edges, v_vy_fruit_cmps, cell_l, cell_h, vehicle_h, horizon_m, x_lim, y_lim, z_lim, density)

        # set the number of snapshots if the vehicle cannot see the whole dataset. For now, just a constant
        if set_view_field == 1:
            n_snapshots = math.ceil((mip_melon.y_lim[1] - mip_melon.y_lim[0]) / view_distance_m)  # set just to do the
        else: 
            n_snapshots = 1 
            view_distance_m = mip_melon.y_lim[1] - mip_melon.y_lim[0]

        print()
        print('length of the full dataset:', (mip_melon.y_lim[1] - mip_melon.y_lim[0]), 'm')
        print('vehicle view distance:', view_distance_m, 'm')
        print('number of snapshots:', n_snapshots)
        print()

        # because we're restarting the object at every run, need to only provide this run's seed list. Needs a list of lists since it assumes it could be multiple runs
        this_seed.append(seed_list[i_run])
        print('this seed', this_seed)
        print('')
        # create the simulated environment
        mip_melon.buildOrchard(1, set_algorithm, set_distribution, this_seed)
        # create the up/down edges just once
        mip_melon.set_zEdges(set_edges, z_lim, n_row) # later this should be abl eto change per snapshot

        # save the complete dataset of fruits
        total_sortedFruit = np.copy(mip_melon.sortedFruit)

        # # find number of problem clusters (extend later to know where the clusters are)
        # # findClustersTotal(mip_melon.sortedFruit, v_vy, mip_melon.Td, n_arm)
        # fruits2remove = findClustersByRow(mip_melon.sortedFruit, v_vy, mip_melon.Td, n_arm, n_row, mip_melon.z_row_bot_edges, mip_melon.z_row_top_edges)
        # for fruit_i in fruits2remove: 
        #     mip_melon.sortedFruit[4,fruit_i] = 2

        # create the arm object lists, should only be done once per full dataset
        mip_arm = mip_melon.createArms()

        # separate the fruit indexes according to the snapshot they'll be in
        this_sortedFruit_index = fruitsInView(n_snapshots, view_distance_m, total_sortedFruit)

        for i_snap in range(n_snapshots):
            # Figure out which fruits go into this snapshot and transform their index to start with zero (save the index start)
            this_sortedFruit = np.copy(total_sortedFruit[:,this_sortedFruit_index[i_snap]])
            this_numFruit    = len(this_sortedFruit[0,:])
            print()
            print('------------------------------------')
            print('number of fruits in this snaphsot:', this_numFruit)
            # print('fruit in this snapshot', this_sortedFruit)
            print('------------------------------------')
            print()

            # create the fruit object lists
            mip_fruit = mip_melon.createFruits(this_numFruit, this_sortedFruit)

            printScen("Solving base scenario model")

            FPT = 0
            solution_found = 0 # changes to 1 if at least one solution fits desired min values

            start_timer = datetime.now() # to figure out how looping through v_vy compares to pure MIP

            if set_MIPsettings == 0:
                solution_found = 1 # set as done since there's only one run

                fruit_picked_by = mip_melon.solve_melon_mip(mip_arm, mip_fruit, v_vy_fruit_cmps, set_MIPsettings)
                print('the fruits picked in this snapshot are:', fruit_picked_by)
                
                print()
                print('------------------------------------')
                print('where is the vehicle right now?', mip_melon.q_vy)
                print('------------------------------------')
                print()

                chosen_j = np.copy(mip_melon.curr_j)

                total_picked = np.sum(chosen_j)
                FPE = (total_picked / this_numFruit)
                FPT = total_picked / (view_distance_m / v_vy)
                # total_picked = np.sum(mip_melon.curr_j)
                # FPE = (total_picked / mip_melon.numFruit)
                # FPT = total_picked / (mip_melon.travel_l / v_vy)

                # ## solve for the mean and variance of fruits in each row 
                # distCenterline(n_row, mip_melon.z_row_bot_edges, mip_melon.z_row_top_edges, mip_melon.sortedFruit)

                ##### brought this up #####
                # calculate how long each arm was working vs idle
                state_time = mip_melon.calcStateTime(fruit_picked_by, view_distance_m, v_vy, total_arms, n_row, n_arm, mip_melon.Td)

                # fill in snapshot object and list with current results, object definition in MIP_melon.py
                snapshot = Snapshot(n_arm, n_row, horizon_m, vehicle_l, mip_melon.cell_l, v_max, a_max, set_algorithm, mip_melon.Td, v_vy, FPE, FPT, mip_melon.y_lim, this_numFruit, chosen_j, mip_melon.sortedFruit, fruit_picked_by, state_time)
                snapshot_list.append(snapshot)
                ## continue filling if needed: PCT, state_time, fruit_picked_by, fruit_list (all lists?)

                horizon_indexes = mip_melon.getHorizonIndex(mip_melon.sortedFruit, mip_melon.q_vy, vehicle_l, horizon_m)

                ## calculate multiple R and v_vy values based on multiple slices of the current view
                # return a list of fruit densities in each cell 
                d = mip_melon.calcDensity(mip_melon.q_vy, v_vy, n_row, n_arm, cell_l, arm_reach, this_sortedFruit)
                # d = mip_melon.calcDensity(mip_melon.q_vy, v_vy, n_row, n_arm, cell_l, arm_reach, mip_melon.sortedFruit)
                ## I wonder if calculating the max number of fruit in a bunch would help... -> clusters! all the clusters...

                ## using the fruit densities, determine the vehicle speed to set a specific R value?
                # currently, the R value would be 
                R = mip_melon.calcR(v_vy, len(horizon_indexes), horizon_m, vehicle_h, arm_reach)  # calculated based on columns and the horizon length

                snapshot_cell.append([d, R])

                # update the vehicle's location (figure out how)
                mip_melon.q_vy += view_distance_m

                if print_out == 1:
                    print()
                    print()
                    print('velocity set at {:.3f} m/s'.format(v_vy)) 
                    print('Number of fruits picked by each arm: *bottom* \n *back* ', chosen_j, ' *front* \n *top*')
                    print('FPE, {:.3f}%'.format(FPE*100), ', and FPT {:.3f}'.format(FPT))   
                    print()
                    print('A total of', total_picked, 'fruits were harvested out of', this_numFruit)         
                    print()
                    print()
                    print('the indexes of the fruits in the horizon are:', horizon_indexes)
                    print('the density of fruit in this snapshot is:', d)
                    print('the R value of this snapshot is:', R)
                    print()
                    print()

                time_run = datetime.now()-start_timer
                time_snap_list.append(time_run)

                print('########################## END LOOP ###########################')
                print()
                print('Running once took:', time_run, 'h:m:s')
                print()
                print('###############################################################')
                print()


            elif set_MIPsettings == 1:
                # loop throught the list of velocity values 
                for v_vy_curr in v_vy_cmps_try:
                    fruit_picked_by_curr = mip_melon.solve_melon_mip(mip_arm, mip_fruit, v_vy_curr, set_MIPsettings)

                    v_vy_mps = v_vy_curr / 100 # change to m/s

                    curr_total_picked = np.sum(mip_melon.curr_j)
                    curr_FPE = (curr_total_picked / mip_melon.numFruit)
                    curr_FPT = curr_total_picked / (mip_melon.travel_l / v_vy_mps)

                    if print_out == 1:
                        print()
                        print('FPE:', curr_FPE*100, '%, and FPT:', curr_FPT, 'fruit/s') 
                        print('when v_vy set at', v_vy_mps, 'm/s')
                        print()            
                        print('A total of', curr_total_picked, 'fruits were harvested out of', mip_melon.numFruit)
                        print('How many fruit were picked by each arm: *front* ', mip_melon.curr_j, ' *back*, fruit')
                        print()

                    if curr_FPE < FPE_min - 0.03: 
                        # there is no way FPE rises, so either a solution was found, or it wasn't 
                        print('########################### FORCED END RUN ###########################')
                        print()
                        break

                    elif curr_FPE >= FPE_min:
                        if curr_FPT > FPT:

                            solution_found = 1

                            FPE          = curr_FPE
                            FPT          = curr_FPT
                            v_vy         = v_vy_mps
                            total_picked = curr_total_picked
                            chosen_j     = np.copy(mip_melon.curr_j)   # save the curr_j variable for the chosen run
                            fruit_picked_by = fruit_picked_by_curr.copy() # copy the chosen run's fruit picked by list

                    curr_j = np.zeros([n_row, n_arm]) # reset so that the next run does not keep adding into the variable

                    print('########################### END RUN ###########################')
                    print()

                time_run = datetime.now()-start_timer

        print('########################## END LOOP ###########################')
        print()
        print('Looping through v_vy values took:', time_run, 'h:m:s')
        print()
        print('###############################################################')
        print()


        #     v_vy = v_vy_cmps / 100 # change to m/s
        #     solve_melon_mip_read()

        if solution_found == 0:
            print('NO SOLUTION was found')
            print()

        elif solution_found == 1:
            if set_MIPsettings == 1:
                print()
                print('chosen velocity {:.3f} m/s'.format(v_vy)) 
                print('Number of fruits picked by each arm: *bottom* \n *back* ', chosen_j, ' *front* \n *top*')
                print('FPE, {:.3f}%'.format(FPE*100), ', and FPT {:.3f}'.format(FPT))   
                print()
                print('A total of', total_picked, 'fruits were harvested out of', mip_melon.numFruit)
                print()

                print('Sorted fruit list')
                print(sortedFruit)

                # calculate how long each arm was working vs idle
                state_time = mip_melon.calcStateTime(fruit_picked_by, mip_melon.travel_l, v_vy, total_arms, n_row, n_arm, mip_melon.Td)

                # fill in snapshot object and list with current results, object definition in MIP_melon.py
                snapshot = Snapshot(n_arm, n_row, horizon_m, vehicle_l, mip_melon.cell_l, v_max, a_max, set_algorithm, mip_melon.Td, v_vy, FPE, FPT, mip_melon.y_lim, mip_melon.numFruit, chosen_j, mip_melon.sortedFruit, fruit_picked_by, state_time)
                snapshot_list.append(snapshot)
                ## continue filling if needed: PCT, state_time, fruit_picked_by, fruit_list (all lists?)

                horizon_indexes = mip_melon.getHorizonIndex(mip_melon.sortedFruit, mip_melon.q_vy, vehicle_l, horizon_m)
                print(horizon_indexes)

                ## calculate multiple R and v_vy values based on multiple slices of the current view
                # return a list of fruit densities in each cell
                d = mip_melon.calcDensity(mip_melon.q_vy, v_vy, n_row, n_arm, cell_l, cell_h, arm_reach, mip_melon.sortedFruit)
                # print()
                ## I wonder if calculating the max number of fruit in a bunch would help...

                ## using the fruit densities, determine the vehicle speed to set a specific R value?
                # currently, the R value would be 
                R = mip_melon.calcR(v_vy, len(horizon_indexes), horizon_m, vehicle_h, arm_reach)  # calculated based on columns and the horizon length

                snapshot_cell.append([d, R])

            # # combine the results based on the various snapshots taken
            results = IG_data_analysis(snapshot_list, snapshot_cell, mip_melon.travel_l, mip_melon.y_lim, set_algorithm, print_out)
            if print_out == 1:
                results.printSettings()

        #     [realFPE, realFPT] = results.realFPEandFPT(sortedFruit, y_lim, v_vy)
            results.avgFPTandFPE()
            # results.avgPCT()
            # print()
            # results.plotValuesOverDistance()
            if plot_out == 1:
                ## currently does not work with multiple snapshots
                # results.plotTotalStatePercent()

                snapshot_schedules_2_plot = range(n_snapshots)  
                # create a plot only of the snapshots that you want (by their index)
                print('what is being sent into plot2D', snapshot_schedules_2_plot)
                results.plot2DSchedule(snapshot_schedules_2_plot)

            run_list.append(snapshot_list)
            time_list.append(time_snap_list)

    # if n_runs > 1:
    analysisMultiple(n_runs, n_snapshots, run_list, time_list, print_out)

if __name__ == '__main__':
    main()