import math
import numpy as np
from datetime import datetime
from scipy.spatial import KDTree
import sys

# from fruit_distribution import *   # import module to create the various desired fruit distributions 
from IG_data_analysis import *     # import module to analyze the data from the snapshots
from MIP_melon import *            # import module that solves the extended MIP in melon algorith *add citation*

# tested with Python 3.7.0 & Gurobi 9.0


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

    color = ['blue', 'red', 'purple', 'chartreuse', 'black', 'aqua', 'pink', 'sienna', 'deepskyblue', 'teal', 'tomato', 'slategrey']
    color_index = 0

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
    plt.plot(sortedFruit[1,:], sortedFruit[2,:], "xk", markersize=14)

    for i in range(len(indexes)):
        if len(indexes[i]) > n_arm: # because the list includes the fruit's index (tree compared to itself), so one extra
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
    d = (v_vy * Td) # find all the neighbors that can be picked, half cause radius(?)

    problem_cluster_num = 0

    coordinates1 = np.copy(sortedFruit[1:3,:]).T
    coordinates2 = np.copy(sortedFruit[1:3,:]).T

    for row in range(n_row):
        # check which fruits are in this rows
        this_row = np.where((sortedFruit[2,:] > z_row_bot_edges[0,row]) & (sortedFruit[2,:] < z_row_top_edges[0,row]))

        for i in range(len(indexes)):
            if len(indexes[i]) > n_arm: # because the list includes the fruit's index (tree compared to itself), so one extra
                problem_cluster_num += 1

                for j in indexes[i]:
                    # plot only the problem clusters
                    line_color = str(color[color_index])
                    plt.plot([coordinates1[i,0], coordinates2[j,0]], [coordinates1[i,1], coordinates2[j,1]], linestyle='-', color=line_color)

                color_index +=1 
                if color_index == 12:
                    color_index = 0








def main():
    ##################### VARIABLES #####################
    # Base model
    n_arm = 4
    n_row = 3

    v_max      = 0.5
    a_max      = 1.
    # t_grab     = 0.1 

    cell_l     = 0.3        # in m, length of the cell along the orchard row (y-axis), parallel to vehicle travel
    cell_h     = 2. / n_row # in m, width/height of the horizontal row of arms (z-axis) perpendicular to vehicle travel
    arm_reach  = 1  

    v_vy_fruit_cmps = 8  # in cm/s

    vehicle_l  = n_arm * cell_l
    vehicle_h  = n_row * cell_h

    horizon_l = 0

    x_lim     = [0.2, 1.2]
    y_lim     = [0. , vehicle_l]
    z_lim     = [0., vehicle_h] 

    total_arms = n_arm * n_row

    # initialize, will be added into in a loop
    FPT = 0
    FPE = 0
    total_picked = 0

    print_out = 1
    plot_out  = 1

    n_snapshots = 1 # for now a constant

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
    # not 1 == not melon
    set_algorithm    = 1

    ## set MIP model settings
    # 0     == basic MIP model from the melon paper with arms not sharing space
    # 1     == basic MIP model with velocity as a variable
    # 2     == makespan MIP (have seperate code, don't use this until proven the same as the other)
    set_MIPsettings = 0

    ## set how z-coord edges are calculated
    # 0     == edges are divided equally along orchard height
    # 1     == edges are divided so each row has equal number of fruit (or close to equal)
    set_edges = 0

    ##################### LISTS #####################
    # list for when there are multiple snapshots over the length of travel
    snapshot_list = list()
    snapshot_cell = list()

    ##################### RUN MIP PYTHON SCRIPT #####################
    # init the MIP melon object 
    mip_melon = MIP_melon(n_arm, n_row, 0, set_distribution, set_algorithm, set_MIPsettings, set_edges, v_vy_fruit_cmps, cell_l, cell_h, vehicle_h, horizon_l, x_lim, y_lim, z_lim)

    # create the simulated environment just once
    mip_melon.buildOrchard(1, set_algorithm, set_distribution)
    # create the up/down edges just once
    mip_melon.set_zEdges(set_edges, z_lim, n_row)

    # create the arm and fruit object lists
    mip_arm = mip_melon.createArms()
    mip_fruit = mip_melon.createFruits()

    printScen("Solving base scenario model")

    total_arms = n_arm * n_row

    FPT = 0
    solution_found = 0 # changes to 1 if at least one solution fits desired min values

    start_timer = datetime.now() # to figure out how looping through v_vy compares to pure MIP

    if set_MIPsettings == 0:
        solution_found = 1 # set as done since there's only one run

        fruit_picked_by = mip_melon.solve_melon_mip(mip_arm, mip_fruit, v_vy_fruit_cmps, set_MIPsettings)

        v_vy = v_vy_fruit_cmps / 100 # change to m/s

        chosen_j = np.copy(mip_melon.curr_j)

        total_picked = np.sum(mip_melon.curr_j)
        FPE = (total_picked / mip_melon.numFruit)
        FPT = total_picked / (mip_melon.travel_l / v_vy)

        ## solve for the mean and variance of fruits in each row 
        distCenterline(n_row, mip_melon.z_row_bot_edges, mip_melon.z_row_top_edges, mip_melon.sortedFruit)

        ## find number of problem clusters (extend later to know where the clusters are)
        findClustersTotal(mip_melon.sortedFruit, v_vy, mip_melon.Td, n_arm)

        print()
        print('FPE:', FPE*100, '%, and FPT:', FPT, 'fruit/s') 
        print('when v_vy set at', v_vy, 'm/s')
        print()            
        print('A total of', total_picked, 'fruits were harvested out of', mip_melon.numFruit)
        print('How many fruit were picked by each arm: *front* ', mip_melon.curr_j, ' *back*, fruit')
        print()

        print('########################## END LOOP ###########################')
        print()
        print('Running once took:', datetime.now()-start_timer, 'h:m:s')
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

        print('########################## END LOOP ###########################')
        print()
        print('Looping through v_vy values took:', datetime.now()-start_timer, 'h:m:s')
        print()
        print('###############################################################')
        print()


    #     v_vy = v_vy_cmps / 100 # change to m/s
    #     solve_melon_mip_read()

    if solution_found == 0:
        print('NO SOLUTION was found')
        print()

    elif solution_found == 1:
        print()
        print('chosen velocity {:.3f} m/s'.format(v_vy)) 
        print('Number of fruits picked by each arm: *bottom* \n *back* ', chosen_j, ' *front* \n *top*')
        print('FPE, {:.3f}%'.format(FPE*100), ', and FPT {:.3f}'.format(FPT))   
        print()
        print('A total of', total_picked, 'fruits were harvested out of', mip_melon.numFruit)
        print()

    #     print('Sorted fruit list')
    #     print(sortedFruit)

        # calculate how long each arm was working vs idle
        state_time = mip_melon.calcStateTime(fruit_picked_by, mip_melon.travel_l, v_vy, total_arms, n_row, n_arm, mip_melon.Td)

        # fill in snapshot object and list with current results
        snapshot = Snapshot(n_arm, n_row, horizon_l, vehicle_l, mip_melon.cell_l, v_max, a_max, set_algorithm, mip_melon.Td, v_vy, FPE, FPT, mip_melon.y_lim, mip_melon.numFruit, chosen_j, mip_melon.sortedFruit, fruit_picked_by, state_time)
        snapshot_list.append(snapshot)
        ## continue filling if needed: PCT, state_time, fruit_picked_by, fruit_list (all lists?)

        horizon_indexes = mip_melon.getHorizonIndex(mip_melon.sortedFruit, mip_melon.q_vy, vehicle_l, horizon_l)
        print(horizon_indexes)

        ## calculate multiple R and v_vy values based on multiple slices of the current view
        # return a list of fruit densities in each cell
        d = mip_melon.calcDensity(mip_melon.q_vy, v_vy, n_row, n_arm, cell_l, cell_h, arm_reach, mip_melon.sortedFruit)
        # print()
        ## I wonder if calculating the max number of fruit in a bunch would help...

        ## using the fruit densities, determine the vehicle speed to set a specific R value?
        # currently, the R value would be 
        R = mip_melon.calcR(v_vy, len(horizon_indexes), horizon_l, vehicle_h, arm_reach)  # calculated based on columns and the horizon length

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
            results.plotTotalStatePercent()

            snapshot_schedules_2_plot = range(n_snapshots)  
            results.plot2DSchedule(snapshot_schedules_2_plot)

if __name__ == '__main__':
    main()