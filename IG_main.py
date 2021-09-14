import numpy as np
# import math

######## IMPORT MY OWN MODULES ########
from trajectory import *          # import the trajectory time calc (bang-bang) 
from plotStates_updated import *  # import module to plot % time each arm is in each state
from fruit_distribution import *  # import module to create the various desired fruit distributions
from IG_scheduling import *       # import module to perform interval graph scheduling similar to melon paper

def vehicleStep(q_vy, distance):
    '''Moves the vehicle a given distance'''
    new_q_vy = q_vy + distance
    return(new_q_vy)


def whatIsInFront(sortedFruit, q_vy, vehicle_l):
    '''Get the number of fruit in front of vehicle and their coordinates for scheduling'''
    vehicle_front = q_vy + vehicle_l

    fruit_index = np.where((sortedFruit[1,:] >= q_vy) & (sortedFruit[1,:] < vehicle_front))
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
    # starting position in the y_axis (front-back on robot)
    col_y = q_vy

    for n in range(n_row):
        # starting position on the z-axis (up-down on robot)
        row_z = 0.
        for k in range(n_arm):
            index = np.where((sortedFruit[1,:] >= col_y) & (sortedFruit[1,:] < col_y + cell_l) & 
                        (sortedFruit[2,:] >= row_z) & (sortedFruit[2,:] < row_z + cell_h))
            # save the number of fruit in this cell
            d[n,k] = len(index[0])
            print(d)
            # move up to the next cell on this column
            row_z += cell_h

        # move to the next column of cells
        col_y += cell_l

    # before calculating the true density, check total number of fruit
    print('which sums to', np.sum(d))

    # divide all the values by the volume of space in front of each cell 
    d = d / (arm_reach * cell_l * cell_h)

    print('fruit density in each cell:')
    print(d)

    return(d)


def calcR(d, v_v):
    '''Calculate the R value given a speed and density'''




def main(args=None):
    # create fruit distribution(s)
    ### environment constants
    x_lim = [0.2, 0.9]
    y_lim = [0., 10.]
    z_lim = [0., 2.7]

    # for the fruit distribution, want to keep it the same for these tests
    x_seed = PCG64(37428395352013185889194479428694397783)
    y_seed = PCG64(13250124924871709375127216220749555998)
    z_seed = PCG64(165440185943501291848242755689690423219)

    # density of "fruit" in the orchard
    density = 10     # fruit/m^3

    ### robot constants and variables
    n_cell = 4       # total number of horizonatal rows s
    n_arm  = 5       # total number of arms in a row
    
    cell_l = 0.3     # in m, length of individual arm cell
    cell_h = (z_lim[1] - z_lim[0]) / n_cell # in m, height of each hor. row
    vehicle_l = n_arm * cell_l 

    arm_reach = x_lim[1] - x_lim[0]

    v_v    = 0.13      # in m/s vehicle velocity
    q_vy   = y_lim[0]  # in m, vehicle's current position (backmost part) 

    # init IG scheduler module
    sched = IG_scheduling(v_v, n_arm, n_cell, cell_l, y_lim, z_lim)

    # Create Fruit distribution
    fruitD = fruitDistribution(x_lim, y_lim, z_lim)
    # [self.numFruit, self.sortedFruit] = fruitD.column(self.v, self.v_max, self.a_max, self.t_grab, self.n_cell, self.n_arm, self.cell_h, z_seed)
    [numFruit, sortedFruit] = fruitD.uniformRandom(density, x_seed, y_seed, z_seed)

    # loop until the vehicle has reached the end of the row
    while q_vy < y_lim[1]:
        print('vehicle\'s current location:', q_vy)
        # run 'vision system' to determine what fruit are in front of the vehicle
        # changes numFruit and sortedFruit to match 
        [sliced_numFruit, sliced_sortedFruit] = whatIsInFront(sortedFruit, q_vy, vehicle_l)
        print()

        ## calculate multiple R and v_v values based on multiple slices of the current view
        # return a list of fruit densities in each cell
        d = calcDensity(q_vy, v_v, n_cell, n_arm, cell_l, cell_h, arm_reach, sliced_sortedFruit)
        ## I wonder if calculating the max number of fruit in a bunch would help...

        ## using the fruit densities, determine the vehicle speed to set a specific R value?
        # currently, the R value would be 



        # sched.setFruitData(numFruit, sortedFruit)
        sched.setFruitData(sliced_numFruit, sliced_sortedFruit)
        # schedule the current view
        sched.initDummyNodes()
        sched.initBaseTimeIntervals()
        sched.chooseArm4Fruit()
        print()
        sched.calcResults()
        print()
        # combine results

        # vehicle takes a step (as big or small as desired)
        q_vy = vehicleStep(q_vy, vehicle_l)
    


if __name__ == '__main__':
    main()