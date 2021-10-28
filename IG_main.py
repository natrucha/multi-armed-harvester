import numpy as np
# import math

######## IMPORT MY OWN MODULES ########
from trajectory import *          # import the trajectory time calc (bang-bang) 
from plotStates_updated import *  # import module to plot % time each arm is in each state
from fruit_distribution import *  # import module to create the various desired fruit distributions
from IG_scheduling import *       # import module to perform interval graph scheduling similar to melon paper
from IG_data_analysis import *    # import module to analyze the data from the snapshots

def vehicleStep(q_vy, distance):
    '''Moves the vehicle a given distance'''
    new_q_vy = q_vy + distance
    return(new_q_vy)


def whatIsInFront(sortedFruit, q_vy, vehicle_l):
    '''Get the number of fruit in front of vehicle and their coordinates for scheduling'''
    vehicle_front   = q_vy + vehicle_l

    # does not check if fruit is set as picked -> doing that in scheduling to preserve indexes
    # fruit_index     = np.where((sortedFruit[1,:] >= q_vy) & (sortedFruit[1,:] < vehicle_front))
    fruit_index     = np.where((sortedFruit[1,:] >= q_vy) & (sortedFruit[1,:] < vehicle_front) & 
                                (sortedFruit[4,:] < 1))
    new_numFruit    = len(fruit_index[0])

    # print('to pick fruit indexes:', fruit_index[0])
    # print('world frame index to pick (should be same as above):', sortedFruit[3,fruit_index[0]])
    # print()

    # WIIF_index_start = fruit_index[0][0]
    # WIIF_index_end   = fruit_index[0][-1]
    # print('START INDEX:', WIIF_index_start)

    new_sortedFruit = np.copy(sortedFruit[:,fruit_index[0]])  # this does not match indexes with sortedFruit

    # due to indexing issues, this will remain until only unpicked apples appear before the vehicle.
    # already_picked = np.where(new_sortedFruit[4,:] < 1)

    print('Num fruit in front of vehicle:', new_numFruit, 'at coordinates')
    # print('Num fruit in front of vehicle:', new_numFruit - len(already_picked[0]), 'at coordinates')
    # print(new_sortedFruit)
    return([new_numFruit, new_sortedFruit])
    # return([new_numFruit, new_sortedFruit, WIIF_index_start])


def getHorizonIndex(sortedFruit, q_vy, vehicle_l, horizon_l):
    '''
       Saves this snapshot's horizon fruit indexes based on the sortedFruit indexes to 
       compare and remove picked fruit.
    '''
    # edges of the horizon based on vehicle location and length
    horizon_back  = q_vy + vehicle_l
    horizon_front = horizon_back + horizon_l

    H_fruit_index = np.where((sortedFruit[1,:] >= horizon_back) & (sortedFruit[1,:] < horizon_front))

    return(H_fruit_index[0])


def calcDensity(q_vy, v_vy, n_row, n_arm, cell_l, cell_h, arm_reach, sortedFruit):
    '''Get the fruit density, d, of each cell'''
    ## should the columns be based on cell length? number of arms? 
    #  should the columns be the same width? increase/decrease the closer to the front of vehicle?
    #  should I calculate R per horizontal row of arms?

    d = np.zeros([n_row, n_arm])  # total number of cells
    # starting position on the z-axis (up-down on robot)
    row_z = 0.

    for n in range(n_row):
        # starting position in the y_axis (front-back on robot)
        col_y = q_vy
        
        for k in range(n_arm):
            # print('col', n, 'row', k)
            # print('back', col_y, 'front', col_y + cell_l)
            # print('bottom', row_z, 'top', row_z + cell_h)
            index = np.where((sortedFruit[1,:] >= col_y) & (sortedFruit[1,:] < col_y + cell_l) & 
                        (sortedFruit[2,:] >= row_z) & (sortedFruit[2,:] < row_z + cell_h) & 
                        (sortedFruit[4,:] < 1))
            # save the number of fruit in this cell
            d[n,k] = len(index[0])
            # print(d)
            # move to the next column of cells
            col_y += cell_l
            
        # move up to the next cell on this column
        row_z += cell_h

    # before calculating the true density, check total number of fruit
    # print('which sums to', np.sum(d))   # has to be equal to numer of fruit
    # divide all the values by the volume of space in front of each cell 
    d = d / (arm_reach * cell_l * cell_h)

    print('fruit density in each cell [fruit/m^3]:')
    print(d)

    return(d)


def calcR(v_vy, fruit_in_horizon, horizon_l, vehicle_h, arm_reach):
    '''Calculate the R value given a speed and horizon volume and density'''
    try:
        density_H = fruit_in_horizon / (horizon_l * vehicle_h * arm_reach)
        time_H    = horizon_l / v_vy

        R         = density_H / time_H # in fruit / (m^3 * s)

    except ZeroDivisionError:
        R         = 0 
    
    print('Fruit incoming rate based on the horizon [fruit/(m^3 s)]:')
    print(R)
    return(R)   


def setAsPicked(sortedFruit, slice_sortedFruit, n_arm, n_cell, picked_fruit):
    '''Set fruit picked by scheduler as picked so that they aren't repicked'''
    # get the real sortedFruit (not sliced) index by adding the index start offset 
    index_list = list()
    for n in range(n_cell):
        for k in range(n_arm):
            index_list += picked_fruit[n][k]

    # check it works  
    # before flattening (rememeber the last list in each one is for unpicked)
    # print('picked_fruit:', picked_fruit) 
    # print()
    # print('before offset index list:', index_list)
    slice_picked_index = np.array(index_list) #, dtype=np.uint)
    sorted_slice       = np.sort(slice_picked_index)
    # print('slice picked fruit sorted index', sorted_slice)

    try:
        real_picked_index = slice_sortedFruit[3,sorted_slice]
        # print('world frame picked sorted index:', np.uint(np.sort(real_picked_index)))
        # print()
        sortedFruit[4, np.uint(np.sort(real_picked_index))] = 1.
        ## Doesn't really help. Choose a value you know should change to 1. and check that the value changes
        # print('sortedFruit after fruits set as picked')
        # print(sortedFruit[4,:])
        return(sortedFruit)

    except IndexError:
        return(sortedFruit)

    

def main(args=None):
    # create fruit distribution(s)
    ### environment constants
    x_lim = [0.2, 1.2]  # -> now know physically, arm can extend 1 m in length
    y_lim = [0., 12.]
    z_lim = [0., 3.]

    # for the fruit distribution, want to keep it the same for these tests
    x_seed = PCG64(37428395352013185889194479428694397783)
    y_seed = PCG64(13250124924871709375127216220749555998)
    z_seed = PCG64(165440185943501291848242755689690423219)

    ### robot constants and variables
    n_cell = 4       # total number of horizontal rows s
    n_arm  = 5       # total number of arms in a row
    
    cell_l = 0.3       # in m, length of individual arm cell (was 0.3)
    cell_h = (z_lim[1] - z_lim[0]) / n_cell # in m, height of each hor. row

    vehicle_l = n_arm * cell_l 
    vehicle_h = n_cell * cell_h  # will probably need to switch to no. horizontal row, rather than n_cell

    # horizon_l = 0. # for now
    horizon_l = cell_l *2 # for now

    arm_reach = x_lim[1] - x_lim[0]

    v_vy    = 0.05      # in m/s vehicle velocity -> max 0.9 m/s
    q_vy   = y_lim[0]  # in m, vehicle's current position (backmost part) 

    # arm settings, also in calcTd function
    v_max = 0.5     # in m/s, Oriental Motor Co. value
    a_max = 1.      # in m/s^2, Oriental Motor Co. value
    d_max = a_max

    t_grab = 0.1  # in sec

    ### Create Fruit Distribution ###
    fruitD = fruitDistribution(x_lim, y_lim, z_lim)

    # settings/variables for the various allowed distributions
    density = 13.333      # fruit/m^3, average density of whole orchard
    fruit_in_cell = math.ceil(density * (cell_h*cell_l*arm_reach)) #* math.floor(1/0.3) # num of fruit in front of cell if using (equalCellDensity())
    print('Number of fruit in each cell:', fruit_in_cell)
    print()
    csv_file = './TREE_FRUIT_DATA/apple_tree_data_2015/Applestotheright.csv'

    # [numFruit, sortedFruit] = fruitD.csvFile(csv_file, 0)
    # [numFruit, sortedFruit] = fruitD.column(v_vy, v_max, a_max, t_grab, n_cell, n_arm, cell_h, z_seed)
    # [numFruit, sortedFruit] = fruitD.uniformRandom(density, x_seed, y_seed, z_seed)
    [numFruit, sortedFruit] = fruitD.equalCellDensity(n_cell, n_arm, cell_h, cell_l, arm_reach, fruit_in_cell, x_seed, y_seed, z_seed)

    # ### init IG scheduler module ###
    # sched = IG_scheduling(v_vy, n_arm, n_cell, cell_l, y_lim, z_lim)
    ## create list to save each snapshot's schedule and data
    snapshot_list = list()
    snapshot_cell = list()

    n_snapshots = 0
    step_length = cell_l #vehicle_l
    snapshot_y_lim = np.zeros(2)

    # loop until the vehicle has reached the end of the row
    while q_vy < y_lim[1]+vehicle_l:
        print('vehicle\'s current location:', q_vy)
        # run 'vision system' to determine what fruit are in front of the vehicle
        # changes numFruit and sortedFruit to match 
        camera_l = vehicle_l + horizon_l  

        # obtain a sliced version of sortedFruit based on what the vehicle sees in front of it (snapshot)
        [sliced_numFruit, sliced_sortedFruit] = whatIsInFront(sortedFruit, q_vy, camera_l)
        # save a list with the sortedFruit indexes of the horizon fruit to compare picked fruit against it
        horizon_indexes = getHorizonIndex(sortedFruit, q_vy, vehicle_l, horizon_l)
        print()

        snapshot_y_lim[0] = q_vy
        snapshot_y_lim[1] = q_vy + vehicle_l

        ### init/reset IG scheduler module for this snapshot ###
        sched = IG_scheduling(v_vy, v_max, a_max, n_arm, n_cell, cell_l, x_lim, snapshot_y_lim, z_lim, vehicle_l, horizon_l)

        ## calculate multiple R and v_vy values based on multiple slices of the current view
        # return a list of fruit densities in each cell
        d = calcDensity(q_vy, v_vy, n_cell, n_arm, cell_l, cell_h, arm_reach, sliced_sortedFruit)
        print()
        ## I wonder if calculating the max number of fruit in a bunch would help...

        ## using the fruit densities, determine the vehicle speed to set a specific R value?
        # currently, the R value would be 
        R = calcR(v_vy, len(horizon_indexes), horizon_l, vehicle_h, arm_reach)  # calculated based on columns and the horizon length
        
        # sched.setFruitData(numFruit, sortedFruit)
        sched.setFruitData(sliced_numFruit, sliced_sortedFruit)
        # schedule the current view
        sched.initDummyNodes()
        sched.initBaseTimeIntervals()
        sched.chooseArm4Fruit()
        # snapshot_fruit_picked_by = sched.chooseArm4Fruit()
        print()
        sched.calcResults()
        # sched.calcResults(step_length)
        snapshot_fruit_picked_by = sched.fruitPickedBy(sliced_numFruit)

        # set picked fruit in sortedFruit as picked
        sortedFruit = setAsPicked(sortedFruit, sched.sortedFruit, n_arm, n_cell, snapshot_fruit_picked_by)

        sched.calcPCT(snapshot_fruit_picked_by)
        sched.calculateStateTime(snapshot_fruit_picked_by)
        # # combine results
        # FPT_snap.append(sched.FPT)
        # FPE_snap.append(sched.FPE)

        # save snapshot
        snapshot_list.append(sched)
        print('NUMBER OF SCHEDULED SNAPSHOTS:', len(snapshot_list))
        print()
        # save the snapshot's cell by cell density and R values
        snapshot_cell.append([d, R])

        n_snapshots += 1

        # vehicle takes a step (as big or small as desired)
        q_vy = vehicleStep(q_vy, step_length)

    # combine the results based on the various snapshots taken
    ## remember this is just the scheduling, so it's the theoretically best results (no missed scheduled fruit, etc.)
    results = IG_data_analysis(snapshot_list, snapshot_cell, step_length, y_lim)
    results.printSettings()
    results.realFPEandFPT(sortedFruit, y_lim, v_vy)
    results.avgFPTandFPE()
    results.avgPCT()
    print()
    results.plotValuesOverDistance()
    results.plotTotalStatePercent()

    snapshot_schedules_2_plot = range(n_snapshots)  
    results.plot2DSchedule(snapshot_schedules_2_plot)


if __name__ == '__main__':
    main()