import math
import numpy as np
from datetime import datetime
import sys

# from fruit_distribution import *   # import module to create the various desired fruit distributions 
from IG_data_analysis import *     # import module to analyze the data from the snapshots
from MIP_melon import *            # import module that solves the extended MIP in melon algorith *add citation*

# tested with Python 3.7.0 & Gurobi 9.0


def printScen(scenStr):
    sLen = len(scenStr)
    print("\n" + "*"*sLen + "\n" + scenStr + "\n" + "*"*sLen + "\n")
    

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
    set_edges = 1

    ##################### LISTS #####################
    row_mip_list    = list()
    mip_arm_list    = list()
    mip_fruit_list  = list()
    fruit_picked_by = list()
    chosen_j_list   = list()
    # list for when there are multiple snapshots over the length of travel
    snapshot_list = list()
    snapshot_cell = list()

    ##################### RUN SEPERATE MIP PER ROW #####################
    for row in range(n_row):
        # init the MIP melon object, n_row set to one for each since we're running it per row
        this_mip = MIP_melon(n_arm, 1, row, set_distribution, set_algorithm, set_MIPsettings, set_edges, v_vy_fruit_cmps, cell_l, cell_h, vehicle_h, horizon_l, x_lim, y_lim, z_lim)
        
        if row == 0:
            # create the simulated environment just once
            this_mip.buildOrchard(1, set_algorithm, set_distribution)
            # create the up/down edges just once
            this_mip.set_zEdges(set_edges, z_lim, n_row)
            
            # only use the first created sortedFruit and numFruit
            sortedFruit_0 = this_mip.sortedFruit
            numFruit_0    = this_mip.numFruit
            # only use the first created bottom and top z-edges
            bot_0  = this_mip.z_row_bot_edges
            top_0  = this_mip.z_row_top_edges
        else: 
            # set the calculated simulated environment for all the other rows
            this_mip.sortedFruit = sortedFruit_0
            this_mip.numFruit    = numFruit_0
            # set the bottom and top z-edges for all the other rows
            this_mip.z_row_bot_edges = bot_0
            this_mip.z_row_top_edges = top_0
            

        # create the arm and fruit object lists
        this_mip_arm = this_mip.createArms()
    #     print('row', row, 'arms in list:', this_mip_arm)
    #     print()
        this_mip_fruit = this_mip.createFruits()
    #     print('row', row, 'fruits in list:', this_mip_fruit)
    #     print()

        # rows are numbered from bottom 0 to top n_row
        row_mip_list.append(this_mip)
        mip_arm_list.append(this_mip_arm)
        mip_fruit_list.append(this_mip_fruit)
        

    printScen("Solving base scenario model")

    solution_found = 0 # changes to 1 if at least one solution fits desired min values

    start_timer = datetime.now() # to figure out how looping through v_vy compares to pure MIP

    if set_MIPsettings == 0:
        solution_found = 1 # set as done since there's only one run
        
        v_vy = v_vy_fruit_cmps / 100 # change to m/s
        
        for row in range(n_row):
            this_fruit_picked_by = row_mip_list[row].solve_melon_mip(mip_arm_list[row], mip_fruit_list[row], v_vy_fruit_cmps, set_MIPsettings)
            
            # need to fix fruit_picked_by since data analysis expects a list n_row x n_arm+1 with the +1 for missed fruit
            # except the missed fruit should only be in row[0] x arm[n_arm]. Currently it's in every row and the missed fruits 
            # fruits are incorrect (one row may have it as missed, but it got picked in a seperate row) 
            #              and
            # stack curr_j arrays to fit data analysis' requirements (bottom/0th row on top)
            if row > 0:
                # stack the row's number of fruits picked by each arm
                chosen_j = np.vstack((chosen_j, np.copy(row_mip_list[row].curr_j)[0])) 
                # now figure out what fruit were not picked
                # see https://stackoverflow.com/questions/16163546/checking-to-see-if-same-value-in-two-lists
                unpicked = set(unpicked).intersection(this_fruit_picked_by[-1])
            else: 
                # initialization for stack the row's number of fruits picked by each arm
                chosen_j = np.copy(row_mip_list[row].curr_j)[0]
                # initialization to figure out what fruit were not picked
                unpicked = this_fruit_picked_by[-1]
    #         print('unpicked,', unpicked)
            
            fruit_picked_by.append(this_fruit_picked_by)
            
            total_picked += np.sum(row_mip_list[row].curr_j)
            
        # add the unpicked list to fruit_picked_by to the correct location
        if n_row > 1:
            print('final unpicked', list(unpicked))
            for row in range(0, n_row):
                fruit_picked_by[row][-1].clear()
            
            fruit_picked_by[0][-1] = list(unpicked)
            
        
        print(fruit_picked_by)
            
        FPE = total_picked / row_mip_list[0].numFruit
        FPT = total_picked / (row_mip_list[0].travel_l / v_vy)
        
        total_fruit = row_mip_list[0].numFruit
        
            
        print()
        print('FPE:', FPE*100, '%, and FPT:', FPT, 'fruit/s') 
        print('when v_vy set at', v_vy, 'm/s')
        print()            
        print('A total of', total_picked, 'fruits were harvested out of', total_fruit)
        print('How many fruit were picked by each arm: *front* ', chosen_j, ' *back*, fruit')
        print()

        print('########################## END LOOP ###########################')
        print()
        print('Running once took:', datetime.now()-start_timer, 'h:m:s')
        print()
        print('###############################################################')
        print()
            
    # elif set_MIPsettings == 1:
    #     # loop throught the list of velocity values 
    #     for v_vy_curr in v_vy_cmps_try:
    #         fruit_picked_by_curr = mip_melon.solve_melon_mip(mip_arm, mip_fruit, v_vy_curr, set_MIPsettings)

    #         v_vy_mps = v_vy_curr / 100 # change to m/s

    #         curr_total_picked = np.sum(mip_melon.curr_j)
    #         curr_FPE = (curr_total_picked / mip_melon.numFruit)
    #         curr_FPT = curr_total_picked / (mip_melon.travel_l / v_vy_mps)

    #         print()
    #         print('FPE:', curr_FPE*100, '%, and FPT:', curr_FPT, 'fruit/s') 
    #         print('when v_vy set at', v_vy_mps, 'm/s')
    #         print()            
    #         print('A total of', curr_total_picked, 'fruits were harvested out of', mip_melon.numFruit)
    #         print('How many fruit were picked by each arm: *front* ', mip_melon.curr_j, ' *back*, fruit')
    #         print()

    #         if curr_FPE < FPE_min - 0.03: 
    #             # there is no way FPE rises, so either a solution was found, or it wasn't 
    #             print('########################### FORCED END RUN ###########################')
    #             print()
    #             break

    #         elif curr_FPE >= FPE_min:
    #             if curr_FPT > FPT:

    #                 solution_found = 1

    #                 FPE          = curr_FPE
    #                 FPT          = curr_FPT
    #                 v_vy         = v_vy_mps
    #                 total_picked = curr_total_picked
    #                 chosen_j     = np.copy(mip_melon.curr_j)   # save the curr_j variable for the chosen run
    #                 fruit_picked_by = fruit_picked_by_curr.copy() # copy the chosen run's fruit picked by list

    #         curr_j = np.zeros([n_row, n_arm]) # reset so that the next run does not keep adding into the variable

    #         print('########################### END RUN ###########################')
    #         print()

    #     print('########################## END LOOP ###########################')
    #     print()
    #     print('Looping through v_vy values took:', datetime.now()-start_timer, 'h:m:s')
    #     print()
    #     print('###############################################################')
    #     print()


    # #     v_vy = v_vy_cmps / 100 # change to m/s
    # #     solve_melon_mip_read()

    if solution_found == 0:
        print('NO SOLUTION was found')
        print()

    elif solution_found == 1:
        print()
        print('chosen velocity {:.3f} m/s'.format(v_vy)) 
        print('Number of fruits picked by each arm: *bottom* \n *back* ', chosen_j, ' *front* \n *top*')
        print('FPE, {:.3f}%'.format(FPE*100), ', and FPT {:.3f}'.format(FPT))   
        print()
        print('A total of', total_picked, 'fruits were harvested out of', total_fruit)
        print()

    #     print('Sorted fruit list')
    #     print(sortedFruit)

    print('fruit picked by:')
    print(fruit_picked_by)

    # calculate how long each arm was working vs idle
    state_time = row_mip_list[0].calcStateTime(fruit_picked_by, row_mip_list[0].travel_l, v_vy, total_arms, n_row, n_arm, row_mip_list[0].Td)

    # fill in snapshot object and list with current results
    snapshot = Snapshot(n_arm, n_row, horizon_l, vehicle_l, cell_l, v_max, a_max, set_algorithm, row_mip_list[0].Td, v_vy, FPE, FPT, row_mip_list[0].y_lim, row_mip_list[0].numFruit, chosen_j, row_mip_list[0].sortedFruit, fruit_picked_by, state_time)
    snapshot_list.append(snapshot)
    ## continue filling if needed: PCT, state_time, fruit_picked_by, fruit_list (all lists?)

    horizon_indexes = row_mip_list[0].getHorizonIndex(row_mip_list[0].sortedFruit, row_mip_list[0].q_vy, vehicle_l, horizon_l)
    # print(horizon_indexes)

    ## calculate multiple R and v_vy values based on multiple slices of the current view
    # return a list of fruit densities in each cell
    d = row_mip_list[0].calcDensity(row_mip_list[0].q_vy, v_vy, n_row, n_arm, cell_l, cell_h, arm_reach, row_mip_list[0].sortedFruit)
    # print()
    ## I wonder if calculating the max number of fruit in a bunch would help...

    ## using the fruit densities, determine the vehicle speed to set a specific R value?
    # currently, the R value would be 
    R = row_mip_list[0].calcR(v_vy, len(horizon_indexes), horizon_l, vehicle_h, arm_reach)  # calculated based on columns and the horizon length

    snapshot_cell.append([d, R])

    # # combine the results based on the various snapshots taken
    results = IG_data_analysis(snapshot_list, snapshot_cell, row_mip_list[0].travel_l, row_mip_list[0].y_lim, set_algorithm, print_out)
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