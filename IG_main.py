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

    fruit_index     = np.where((sortedFruit[1,:] >= q_vy) & (sortedFruit[1,:] < vehicle_front))
    new_numFruit    = len(fruit_index[0])

    new_sortedFruit = np.copy(sortedFruit[:,fruit_index[0]]) 
    # need to chnage fruit to be in vehicle frame
    # new_sortedFruit[1,:] = new_sortedFruit[1,:] - q_vy

    print('Num fruit in front of vehicle:', new_numFruit, 'at coordinates')
    # print(new_sortedFruit)
    return([new_numFruit, new_sortedFruit])


def calcDensity(q_vy, v_v, n_row, n_arm, cell_l, cell_h, arm_reach, sortedFruit):
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
                        (sortedFruit[2,:] >= row_z) & (sortedFruit[2,:] < row_z + cell_h))
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


def calcR(d, v_v):
    '''Calculate the R value given a speed and density'''
    R = d * v_v # in fruit / (m^2 * s)
    print('Fruit incoming rate for each cell [fruit/(m^2 s)]:')
    print(R)
    return(R)    


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
    n_cell = 4       # total number of horizonatal rows s
    n_arm  = 5       # total number of arms in a row
    
    cell_l = 0.3     # in m, length of individual arm cell
    cell_h = (z_lim[1] - z_lim[0]) / n_cell # in m, height of each hor. row
    vehicle_l = n_arm * cell_l 

    arm_reach = x_lim[1] - x_lim[0]

    v_v    = 0.08      # in m/s vehicle velocity -> max 0.9 m/s
    q_vy   = y_lim[0]  # in m, vehicle's current position (backmost part) 

    # arm settings, also in calcTd function
    v_max = 0.5     # in m/s, Oriental Motor Co. value
    a_max = 1.      # in m/s^2, Oriental Motor Co. value
    d_max = a_max

    t_grab = 0.1  # in sec

    ### Create Fruit Distribution ###
    fruitD = fruitDistribution(x_lim, y_lim, z_lim)

    # settings/variables for the various allowed distributions
    density = 10      # fruit/m^3, average density of whole orchard
    fruit_in_cell = 3 # num of fruit in front of cell if using (equalCellDensity())
    csv_file = './TREE_FRUIT_DATA/apple_tree_data_2015/Applestotheright.csv'

    # [numFruit, sortedFruit] = fruitD.csvFile(csv_file, 0)
    # [numFruit, sortedFruit] = fruitD.column(v_v, v_max, a_max, t_grab, n_cell, n_arm, cell_h, z_seed)
    # [numFruit, sortedFruit] = fruitD.uniformRandom(density, x_seed, y_seed, z_seed)
    [numFruit, sortedFruit] = fruitD.equalCellDensity(n_cell, n_arm, cell_h, cell_l, arm_reach, fruit_in_cell, x_seed, y_seed, z_seed)

    # ### init IG scheduler module ###
    # sched = IG_scheduling(v_v, n_arm, n_cell, cell_l, y_lim, z_lim)
    ## create list to save each snapshot's schedule and data
    snapshot_list = list()
    snapshot_cell = list()

    n_snapshots = 0
    # FPT_snap = list() -> uneccessary if saving each scheduling object successfully
    # FPE_snap = list()
    step_length = vehicle_l
    snapshot_y_lim = np.zeros(2)

    # loop until the vehicle has reached the end of the row
    while q_vy < y_lim[1]:
        print('vehicle\'s current location:', q_vy)
        # run 'vision system' to determine what fruit are in front of the vehicle
        # changes numFruit and sortedFruit to match 
        [sliced_numFruit, sliced_sortedFruit] = whatIsInFront(sortedFruit, q_vy, vehicle_l)
        print()

        snapshot_y_lim[0] = q_vy
        snapshot_y_lim[1] = q_vy + vehicle_l

        ### init/reset IG scheduler module for this snapshot ###
        sched = IG_scheduling(v_v, v_max, a_max, n_arm, n_cell, cell_l, x_lim, snapshot_y_lim, z_lim)

        ## calculate multiple R and v_v values based on multiple slices of the current view
        # return a list of fruit densities in each cell
        d = calcDensity(q_vy, v_v, n_cell, n_arm, cell_l, cell_h, arm_reach, sliced_sortedFruit)
        print()
        ## I wonder if calculating the max number of fruit in a bunch would help...

        ## using the fruit densities, determine the vehicle speed to set a specific R value?
        # currently, the R value would be 
        R = calcR(d, v_v)
        
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

        sched.calcPCT(snapshot_fruit_picked_by)
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
    results = IG_data_analysis(snapshot_list, snapshot_cell)
    results.avgFPTandFPE()
    results.plotValuesOverDistance()


if __name__ == '__main__':
    main()