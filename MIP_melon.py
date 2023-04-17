# from collections import defaultdict

## see examples in https://www.gurobi.com/resource/modeling-examples-using-the-gurobi-python-api-in-jupyter-notebook/
import gurobipy as gp
from gurobipy import GRB

import math
import numpy as np
# from datetime import datetime
import sys

from fruit_distribution import *   # import module to create the various desired fruit distributions 
from IG_data_analysis import *     # import module to analyze the data from the snapshots
from trajectory import *           # import module to calculate the trapezoidal/S-curve (S-curve not working yet) tajectory calculator

# tested with Python 3.7.0 & Gurobi 9.0

## based on the Gurobi technician routing scheduling example
# https://gurobi.github.io/modeling-examples/technician_routing_scheduling/technician_routing_scheduling.html

class MIP_melon(object):
    def __init__(self, q_vy, n_col, n_row, starting_row_n, set_distribution, v_vy_fruit_cmps, cell_l, cell_h, vehicle_h, hor_l, x_lim, y_lim, z_lim, density):
        # def __init__(self, q_vy, n_col, n_row, starting_row_n, set_distribution, set_algorithm, set_MIPsettings, set_edges, v_vy_fruit_cmps, cell_l, cell_h, vehicle_h, horizon_l, x_lim, y_lim, z_lim, density)

        '''
            Mixed integer programming model based on the MIP model in the melon combinatorial scheduling paper. 
            Modified to work with a matrix of arms, rather than just a line.

            starting_row_n is the row number at which this MIP run will start, usually set at 0 unless each row is run seperately
        '''

        ## Constants 
        ## to print out from data analysis
        self.print_out  = 1
        plot_out   = 1

        # set which digitized data file will be used, 'raj' or 'juan'
        # self.data_name = 'raj'

        ## settings
        self.Td    = 2.      # in s, fruit handling time when it's constant
        self.M     = 280     # arbitrarily large number, should be based on the largest difference between t[k, l, i] and t[k, l, j] plus some extra (upper bound)  
        self.q_vy  = q_vy    # in m, backmost, lowest coordinate of the vehicle
        self.q_vy0 = q_vy    # in m, the actual start (snapshots require negatve values)

        self.n_row            = n_row      # total number of horizontal rows with cells containg one arm per cell
        self.n_col            = n_col      # number of arms in one horizontal row, also the number of columns of arms

        self.starting_row_n   = starting_row_n # row number at which this MIP run will start, usually set at 0 unless each row is run seperately

        self.density          = density      # in fruit/m^2
       
        n_fruit               = 80           # in fruit, for melon column distribution, should clean this up

        self.cell_l           = cell_l     # in m, length of the cell along the orchard row (y-axis), parallel to vehicle travel
        # pick travel length gets set seperately in addArmTravelLimits function
        self.l_real_y_travel  = 0.       # in m, the amount of distance within the cell the arm can travel due to frame, motor placement, etc. assume centered
        self.cell_h           = cell_h     # in m, width/height of the horizontal row of arms (z-axis) perpendicular to vehicle travel
        self.reach            = 1          # in m, the length that the arm can extend out to pick a fruit
        self.hor_l            = hor_l      # in m, the length of the view horizon

        self.noRel_time_ub    = 15     # in s, no relaxation heuristic max time to solve before moving to branch and bound (varies)
        self.timLim_time_ub   = 30     # in s, the total amount of time the solver runs (includes NoRel)

        self.vehicle_l  = self.n_col * self.cell_l

        self.t_grab = 0.5

        ## Get fruit list based on desired distribution
        self.x_lim   = np.copy(x_lim)
        self.y_lim   = np.copy(y_lim)
        self.z_lim   = np.copy(z_lim)

        # calculate the real y_lim[1] based on the fruit distribution 
        # self.calcYlimMax(set_distribution, self.vehicle_l, v_vy_fruit_cmps, n_fruit) 

        # allow calculations for the x-axis, 1.0 m long worst case
        v_max_x   = 2         # in m/s, from Motor Sizing Google sheet calculations if we want to keep triangular profile to maximize speed
        a_max_x   = 4         # in m/s^2, from Motor Sizing Google sheet calculations if we want desired linear movement time to be 1 second
        d_max_x   = a_max_x   # in m/s^2, if motors allow, keep equal to a_max
        # initialize the ability to calculate trajectory
        self.traj_calc_x = Trajectory(v_max_x, a_max_x, d_max_x) 

        

## Functions
    def addArmTravelLimits(self, l_real_y_travel):
        '''The arm will only be able to travel some length of the total cell length (restricted by frame and motor placement). Set that value.'''
        self.l_real_y_travel = l_real_y_travel # in m


    
    def setTravelLength(self, l_step_m):
        '''Set the length of travel of vehicle for this run (either snapshot or overall)'''
        self.travel_l = l_step_m # in m, the travel length over this snapshot (or overall run)



    def setZlim(self, bot_edge, top_edge):
        '''set this run's z-axis limits (top and bottom) for each row'''
        self.z_row_bot_edges = bot_edge
        self.z_row_top_edges = top_edge


    # def calcSumPCTstates(self, fruit_picked_by, N, TX):
    #     '''
    #        Calculates the sum of each of the six PCT state (idle, pickYZ, pickX, grab, retract, unload) over
    #        the whole snapshot. Returns the sum over *all the arms* of the duration spent in each state.

    #        Can only be run after scheduling has finished.

    #        *** Depends on no cycles (figure out cycles later) *** 
    #        Assumes that fruit_picked_by contains the picking order (smallest to largest) which may not be true later
    #        Might need to switch MIP to save order if cycles added.
    #     '''
    #     sum_pickYZ = 0 # in s, the sum of move times
    #     sum_pickX  = 0 # in s, the sum of extension times
    #     sum_grab   = 0 # in s, the sum of grab times
    #     # retract will equal pickX unless proven otherwise :)
    #     # not using unload right now (don't have the math yet, assumes a vacuum gripper)

    #     state_time = np.zeros([self.n_row*self.n_col, 7]) # save each arm's percent time in each of the six states plus a total

    #     # iterate through fruit picked by to obtain the individual PCT/Td of every fruit
    #     for i_row in range(self.n_row):
    #         if self.n_row == 1:
    #             i_row = 0
    #         for i_col in range(self.n_col):
    #             tot_arm_index = i_col + (i_row*self.n_col)
    #             # skips not picked list
    #             if self.n_row > 1:
    #                 num_fruits = len(fruit_picked_by[i_row][i_col])
    #                 # print('the fruits in this row/column:', fruit_picked_by[i_row][i_col])
    #             else:
    #                 num_fruits = len(fruit_picked_by[i_col])
                
    #             if num_fruits > 1:
    #                 # get extension time of the first fruit in the list
    #                 if self.n_row > 1: 
    #                     zero_TX_i = N.index(fruit_picked_by[i_row][i_col][0])
    #                 else:
    #                     zero_TX_i = N.index(fruit_picked_by[i_col][0])
    #                 # print('TX[0] %.2f' % TX[fruit_picked_by[i_row][i_col][0]])

    #                 # obtain the extension and retraction of the first fruit (assumes it doesn't have to move to that fruit)
    #                 sum_pickX += TX[zero_TX_i]
    #                 sum_grab  += self.t_grab

    #                 # needs to take into account the order of harvest (move from fruit i to fruit j, figure out which is index belongs to i and which to j)
    #                 for i_list in range(num_fruits-1):
    #                     if self.n_row > 1:
    #                         curr_i = fruit_picked_by[i_row][i_col][i_list]
    #                         next_i = fruit_picked_by[i_row][i_col][i_list+1] 
    #                         # print('current i: %d and next i: %d' % (curr_i, next_i))
    #                     else:
    #                         curr_i = fruit_picked_by[i_col][i_list]
    #                         next_i = fruit_picked_by[i_col][i_list+1] 

    #                     # curr_TX_i = N.index(curr_i)
    #                     next_TX_i = N.index(next_i)
    #                     # print('which are %d and %d in N' % (curr_TX_i, next_TX_i))

    #                     curr_move = self.fruit_travel_matrix[curr_i, next_i]
    #                     # print('with move time %.2f' % curr_move)

    #                     # to get the indexes to work correctly, this part works with indexes (i+1), avoiding any doubling of the extension time
    #                     # add the extension time for the next fruit (i+1) 
    #                     # print('with extension time %.2f' % TX[next_TX_i])
    #                     sum_pickYZ += curr_move
    #                     sum_pickX  += TX[next_TX_i]
    #                     sum_grab   += self.t_grab

    #             elif num_fruits == 1:
    #                 # just worry about the one extension because only one fruit was harvested
    #                 if self.n_row > 1:
    #                     zero_TX_i = N.index(fruit_picked_by[i_row][i_col][0])
    #                 else:
    #                     zero_TX_i = N.index(fruit_picked_by[i_col][0])
    #                 sum_pickX += TX[zero_TX_i]
    #                 sum_grab  += self.t_grab
    #                 # sum_retr  += TX[zero_TX_i]

    #             state_time[tot_arm_index,1] = sum_pickYZ
    #             state_time[tot_arm_index,2] = sum_pickX
    #             state_time[tot_arm_index,3] = sum_grab
    #             state_time[tot_arm_index,4] = sum_pickX # same as pickX unless something comes up

    #             # reset the values
    #             sum_pickYZ = 0
    #             sum_pickX  = 0
    #             sum_grab   = 0

    #     # print('\nThe sum of extension times is %.2f s' % sum_of_ext_times)
    #     # print('The sum of move times is %.2f s\n' % sum_of_move_times)

    #     return(state_time)



    # def calcMeanTd(self, fruit_picked_by, fruit, total_picked):
    #     '''
    #        Calculates the mean Td of the snapshot based on the harvested fruits. Pass the final fruit_picked_by
    #        list, the one that already went through queue management, etc.

    #        Can only be run after scheduling has finished.

    #        *** Depends on no cycles (figure out cycles later) *** 
    #        Assumes that fruit_picked_by contains the picking order (smallest to largest) which may not be true later
    #        Might need to switch MIP to save order if cycles added.
    #     '''
    #     # # list of fruit extension times in fruit object
    #     N  = [i.real_index for i in fruit]    # list of fruit indexes
    #     TX = [i.Tx for i in fruit]            # list of precalculated fruit extension times

    #     state_time = self.calcSumPCTstates(fruit_picked_by, N, TX)

    #     sum_pickYZ = np.sum(state_time[:,1])
    #     sum_pickX  = np.sum(state_time[:,2])
    #     sum_grab   = np.sum(state_time[:,3])
    #     sum_retr   = np.sum(state_time[:,4])

    #     print('\nSum of pickYZ %.2f,   pickX %.2f,   grab %.2f,   retract %.2f' % (sum_pickYZ, sum_pickX, sum_grab, sum_retr))

    #     # get the total fruit handling time
    #     sum_Td_times = sum_pickYZ + sum_pickX + sum_grab + sum_retr # sum_of_ext_times + sum_of_move_times

    #     # divide by the number of harvested fruits to get the average
    #     mean_Td = sum_Td_times / total_picked
    #     print('Mean Td time for the snapshot %.2f\n' % mean_Td)

    #     return(mean_Td)


    # def timeBtwFruits(self):
    #     '''
    #        Create a matrix that saves the time it takes to move between every two fruits in the snapshot. Time calculated as max(Ty, Tz) using the trajectory.py
    #        module. One side of matrix is calculated and then mirrored over since the distance should be the same between i and j vs j and i. 

    #        Requires that buildOrchard() has already been run.
    #     '''
    #     # allow calculations for the y-axis, 0.7 m long worst case
    #     v_max_y   = 1.4 # m/s from Motor Sizing Google sheet calculations if we want to keep triangular profile to maximize speed
    #     a_max_y   = 2.8 # m/s^2 from Motor Sizing Google sheet calculations if we want desired linear movement time to be 1 second
    #     d_max_y   = a_max_y # if motors allow, keep equal to a_max
    #     # initialize the ability to calculate trajectory
    #     traj_calc_y = Trajectory(v_max_y, a_max_y, d_max_y) 

    #     # allow calculations for the z-axis, 0.61? m worst case
    #     v_max_z   = 1.3 # m/s from Motor Sizing Google sheet calculations if we want to keep triangular profile to maximize speed
    #     a_max_z   = 2.8 # m/s^2 from Motor Sizing Google sheet calculations if we want desired linear movement time to be 1 second
    #     d_max_z   = a_max_z # if motors allow, keep equal to a_max
    #     # initialize the ability to calculate trajectory
    #     traj_calc_z = Trajectory(v_max_z, a_max_z, d_max_z) 

    #     self.fruit_travel_matrix = np.zeros((self.numFruit, self.numFruit), dtype=float )  
    #     # fill the diagonal with np.inf (so it can never be used)
    #     np.fill_diagonal(self.fruit_travel_matrix, np.inf)

    #     for i in range(1,self.numFruit):
    #         for j in range(i):
    #             if i != j:
    #                 start_y = self.sortedFruit[1,i]  # in m, fruit i y_coordinate
    #                 start_z = self.sortedFruit[2,i]  # in m, fruit i z_coordinate

    #                 end_y   = self.sortedFruit[1,j]  # in m, fruit j y_coordinate
    #                 end_z   = self.sortedFruit[2,j]  # in m, fruit j z_coordinate

    #                 # calculate extension to fruit in y-axis
    #                 traj_calc_y.adjInit(start_y, 0.) # start moving from zero speed
    #                 traj_calc_y.noJerkProfile(traj_calc_y.q0, end_y, traj_calc_y.v0, v_max_y, a_max_y, d_max_y)
    #                 T_y = traj_calc_y.Ta + traj_calc_y.Tv + traj_calc_y.Td 
                    
    #                 # calculate extension to fruit in z-axis
    #                 traj_calc_z.adjInit(start_z, 0.) # start moving from zero speed
    #                 traj_calc_z.noJerkProfile(traj_calc_z.q0, end_z, traj_calc_z.v0, v_max_z, a_max_z, d_max_z)
    #                 T_z = traj_calc_z.Ta + traj_calc_z.Tv + traj_calc_z.Td 

    #                 # add to matrix and mirror it
    #                 self.fruit_travel_matrix[i,j] = max(T_y, T_z)
    #                 self.fruit_travel_matrix[j,i] = max(T_y, T_z)

    #     # print('travel times between fruits:')
    #     # print(self.fruit_travel_matrix)



    # def buildOrchard(self, n_runs, set_algorithm, set_distribution, seed_list):
    #     '''Creates the simulated environment, separated so that MIP run/row can happen'''

    #     for run in range(n_runs):
    #         # get seeds for x, y, and z RNG (probably unneccessary right now, especially for x)
    #         seed = [seed_list[run][0], seed_list[run][1], seed_list[run][2]]
    #         x_seed = PCG64(int(seed[0]))
    #         y_seed = PCG64(int(seed[1]))
    #         z_seed = PCG64(int(seed[2])) 

    #     # create fruit distribution and get total number of fruits
    #     fruitD = fruitDistribution(self.x_lim, self.y_lim, self.z_lim) # init fruit distribution script
    #     [self.numFruit, self.sortedFruit] = self.createFruitDistribution(fruitD, set_algorithm, set_distribution, self.density, x_seed, y_seed, z_seed)

    #     # print('Density chosen', self.density)
    #     print('x-lim', self.x_lim, 'y-lim', self.y_lim, 'z-lim', self.z_lim)
    #     print('Total fruit in the orchard row',self.numFruit)
    #     # print()
    #     # print('length of sortedFruit', len(self.sortedFruit[0]))
    #     # print()
    #     # print('List of the x, y, and z coordinates of the sorted fruit')
    #     # print(sortedFruit)

    #     ## for now, calculate time between fruits here (only needed once per snapshot)
    #     self.timeBtwFruits()


    def inputOrchard(self, sortedFruit):
        '''Input existing fruit distribution data to use as the fruit data. Used for snapshots.'''
        self.sortedFruit = np.copy(sortedFruit)


    def createArms(self):
        '''Create and populate all the arms' classes then put in appropriate list'''
        ## create arm object list
        arm = list()

        # check if being updated 
        # print('starting row number:', self.starting_row_n)

        if self.starting_row_n + 1 > self.n_row:
            row_n = self.starting_row_n + 1
        else:
            row_n = self.n_row

        # print('the starting row number is', self.starting_row_n, 'and the ending is', row_n)

        for r in range(self.starting_row_n, row_n):
            # print('row number', r)
            for k in range(self.n_col):
                this_arm = Arm(r, k)
                arm.append(this_arm)
        # print()
        return(arm)


    def createFruits(self, numFruit, sortedFruit):
        '''Create and populate all the fruits' classes then put in appropriate list. Fruit indexes are 'zeroed' so that it works with snapshots'''
        ## create fruit object list
        fruit = list()

        # only works by starting index at 0
        for index in range(numFruit):
            # if the fruit was not removed due to clustering
            x_coord      = sortedFruit[0][index]
            y_coord      = sortedFruit[1][index]
            z_coord      = sortedFruit[2][index]
            # use a 'zeroed' index so that it works with snapshots
            fruit_i      = index  
            # but make sure to save the fruit's 'real' index 
            fruit_i_real = int(sortedFruit[3][index])
            # create the object
            this_fruit   = Fruit(self.traj_calc_x, fruit_i, fruit_i_real, x_coord, y_coord, z_coord)
            # print('Fruit index', index, 'should match this index only if no snapshots', sortedFruit[3][index])
            # print('with y and z coordinates:', y_coord, z_coord)
            fruit.append(this_fruit)

        # print(fruit)
        return(fruit)

    
    def createJobs(self, arm, fruit, v_vy_curr, cell_l):
        '''Create and populate all the jobs' classes then put in appropriate list'''
        ## create job object list
        job = list()

        for k in arm:
            for i in fruit:  
                this_job = Job(i, k, self.q_vy, v_vy_curr, cell_l, self.l_real_y_travel)  # careful because makespan does not use the Jobs() TW values
                job.append(this_job)
                # print('for arm', this_job.arm_k.arm_n, 'in row', this_job.arm_k.row_n,'and fruit', this_job.fruit_i.index)
                # print('TW starts at', this_job.TW_start, 'and TW ends at', this_job.TW_end)  

        return(job)


    # def getHorizonIndex(self, sortedFruit, q_vy, vehicle_l):
    #     '''
    #     Saves this snapshot's horizon fruit indexes based on the sortedFruit indexes to 
    #     compare and remove picked fruit.
    #     '''
    #     # edges of the horizon based on vehicle location and length
    #     horizon_back  = q_vy + vehicle_l
    #     horizon_front = horizon_back + self.hor_l

    #     H_fruit_index = np.where((sortedFruit[1,:] >= horizon_back) & (sortedFruit[1,:] < horizon_front))

    #     return(H_fruit_index[0])


    def calcDensity(self, q_vy, v_vy, n_row, n_col, cell_l, arm_reach, sortedFruit):
        '''Get the fruit density, d, of each cell'''
        ## should the columns be based on cell length? number of arms? 
        #  should the columns be the same width? increase/decrease the closer to the front of vehicle?
        #  should I calculate R per horizontal row of arms?

        d = np.zeros([n_row, n_col])  # total number of cells
        # starting position on the z-axis (up-down on robot)
        row_z = 0.

        for n in range(n_row):
            # starting position in the y_axis (front-back on robot)
            col_y = q_vy
            cell_h = self.z_row_top_edges[0,n] - self.z_row_bot_edges[0,n] # for now all rows are the same
            # print('Cell height for this', n, 'loop', cell_h)
            # print('bottom', self.z_row_bot_edges[0,n], 'top', self.z_row_top_edges[0,n], '\n')

            for k in range(n_col):
                # print('col', n, 'row', k)
                # print('back', col_y, 'front', col_y + cell_l)
                # print('bottom', row_z, 'top', row_z + cell_h)
                index = np.where((sortedFruit[1,:] >= col_y) & (sortedFruit[1,:] < col_y + cell_l) & 
                            (sortedFruit[2,:] >= row_z) & (sortedFruit[2,:] < row_z + cell_h) & 
                            (sortedFruit[4,:] < 1))
                # save the number of fruit in this cell and divide all the values by the volume of space in front of each cell 
                d[n,k] = len(index[0]) / (arm_reach * cell_l * cell_h)

                # print(d)
                # move to the next column of cells
                col_y += cell_l

            # move up to the next cell on this column
            row_z += cell_h

        # before calculating the true density, check total number of fruit
        # print('which sums to', np.sum(d))   # has to be equal to numer of fruit
        # divide all the values by the volume of space in front of each cell 
        # d = d / (arm_reach * cell_l * cell_h)

        # print('fruit density in each cell [fruit/m^3]:')
        # print(d)

        return(d)


    def calcR(self, v_vy, fruit_in_horizon, vehicle_h, arm_reach):
        '''Calculate the R value given a speed and horizon volume and density'''
        try:
            density_H = fruit_in_horizon / (self.hor_l * vehicle_h * arm_reach)
            time_H    = self.hor_l / v_vy

            R         = density_H / time_H # in fruit / (m^3 * s)

        except ZeroDivisionError:
            R         = 0 

        # print('Fruit incoming rate based on the horizon [fruit/(m^3 s)]:')
        # print(R)
        return(R) 


    # def calcStateTime(self, fruit_picked_by, fruit, travel_l, v_vy, n_row, n_col):
    #         '''Calculates the time each arm is in each state so that it can plotState can plot the data'''
    #         total_time = travel_l / v_vy 
    #         # print('total move time:', total_time)

    #         # list of fruit extension times in fruit object
    #         N  = [i.real_index for i in fruit]    # list of fruit indexes
    #         TX = [i.Tx for i in fruit]            # list of precalculated fruit extension times
    
    #         ## states: idle, pick_yz, pick_x, grab, retract_x, move_z/unload
    #         # self.state_percent = np.zeros([self.total_arms, 6]) # save each arm's percent time in each of the six states 
    #         # state_time = np.zeros([total_arms, 7]) # save each arm's percent time in each of the six states plus a total

    #         state_time = self.calcSumPCTstates(fruit_picked_by, N, TX) 

    #         for r in range(n_row):
    #             for c in range(n_col):
    #                 tot_arm_index = c + (r*n_col)
    #                 # calculate idle by subtracting all numbes calculated above by total time: length_row / v
    #                 state_time[tot_arm_index,0] = total_time - np.sum(state_time[tot_arm_index,:])
    #                 # save the total time for this run to get total percent later
    #                 state_time[tot_arm_index,6] = total_time
                    
    #         return(state_time)
    
    

    # def set_zEdges(self, set_edges, z_lim, n_row, numFruit, sortedFruit):
    #     '''
    #     Calculate the z-coord for each horizontal row, assuming the whole row shares these edges.
    #     Returns a n_row x n_col matrix for both the bottom and top edges of each cell. 
    #     '''   
    #     # find what fruits are available to harvest (some may have already been picked or removed)
    #     index_available = np.where(sortedFruit[4,:] <= 1) 

    #     # edges for the nth horizontal row of cells
    #     if set_edges == 0 or len(index_available[0]) < 1: 
    #         # divided equally by distance along orchard height or there are no fruits in view
    #         # row bottom edge = n*self.cell_h
    #         bottom = np.linspace(0, (n_row*self.cell_h - self.cell_h), self.n_row, endpoint=True)

    #         bot_edge = np.tile(bottom, (self.n_col, 1))
    #         top_edge = np.copy(bot_edge) + self.cell_h
    #     #     print('bottom edges:', bot_edge)
    #     #     print()
    #     #     print('top edges:', top_edge)
    #     #     print()
    #     elif set_edges == 1:
    #         # divided by number of fruit
            
    #         # make zero arrays for top and bottom. Since edges shared along horizontal row, can tile it by n_col
    #         top    = np.zeros(n_row)
    #         bottom = np.zeros(n_row)

    #         # calculate how many fruits should be in each row
    #         fruit_in_row = math.floor(len(index_available[0]) / n_row)  # total fruit in each horizontal row (round down, one row could be heavier)
    #         # fruit_in_row = math.floor(numFruit / n_row)  # total fruit in each horizontal row (round down, one row could be heavier)
    #         # fruit_in_row = math.floor(self.numFruit / n_row)  # total fruit in each horizontal row (round down, one row could be heavier)
    #         print('number of fruit in each row, rounded down', fruit_in_row)
    #         print()
            
    #         # get the z-coord array of the remaining fruits
    #         # z_check = np.array(sortedFruit[2])
    #         z_coord = np.array(sortedFruit[2,index_available[0]])

    #         # print('z_check', z_check)
    #         # print()
    #         # print('z_coord', z_coord)
    #         # sort the array
    #         z_sorted = np.sort(z_coord)
    # #         print('sorted z-coord', z_sorted)

    #         # sys.exit(0)
            
    #         for row in range(n_row-1):
    #             top[row]      = z_sorted[fruit_in_row*(row+1)]-0.0001
    #             bottom[row+1] = z_sorted[fruit_in_row*(row+1)]+0.0001 
                
    #         top[-1] = z_lim[1]
                
    #         bot_edge = np.tile(bottom, (self.n_col, 1))
    #         top_edge = np.tile(top, (self.n_col, 1))
            
    #     else:
    #         print('Not an edge setting, please try again')
    #         return([0, 0])

    #     self.z_row_bot_edges = bot_edge
    #     self.z_row_top_edges = top_edge

    #     # print('bottom z-axis edges', self.z_row_bot_edges)
    #     # print()
    #     # print('top z-axis edges', self.z_row_top_edges)
    #     # print()
    #     # return([bot_edge, top_edge]), unneccessary to return them at the moment 


    # def calcYlimMax(self, set_distribution, vehicle_l, v_vy_fruit_cmps, n_fruit):
    #     '''Calculates the end of the orchard's coordinate based on the set fruit distribution'''
    #     # no snapshots, so the travel length is static at the orchard row's length
    #     if set_distribution == 0:
    #         # travel l changes as the vehicle length changes?
    #         if self.data_name == 'juan':
    #             y_max  = 16.5 # in m, for Juan's data
    #             self.density   = 53.97            # in fruit/m^2 (on avg.), constant, for Juan's data
    #         elif self.data_name == 'raj':
    #             y_max  = 10.9 # in m, for Raj's data
    #             self.density   = 48.167         # in fruit/m^2 (on avg.), constant, for Raj's data
    #         n_runs    = 1

    #     elif set_distribution == 1:
    #         y_max  = 6 # in m, usually 5 m + length

    #     elif set_distribution == 3:
    #         y_max  = 30 # in m
            
    #     elif set_distribution == 5:
    #         v_vy_fruit_mps = v_vy_fruit_cmps / 100
    #         self.d_y  = self.Td*v_vy_fruit_mps*(n_fruit+1)/(n_fruit+2) # kind of works 3/4 or 5/8 fruit with 1 arm: (Td/2)*v_vy
    #     #     d_y  = Td*v_vy_fruit*(n_fruit+1)/(n_fruit+2) # kind of works 3/4 or 5/8 fruit with 1 arm: (Td/2)*v_vy
    #         print('with Td', self.Td, 'and v_vy for fruit distribution', v_vy_fruit_cmps, 'cm/s')
    #         print('d_y for this line of fruit:', self.d_y, 'so the total distance they take up:', self.d_y*n_fruit)
    #         y_max  = self.d_y * n_fruit # in m
            
    #     elif set_distribution == 6:
    #         y_max = 6 # in m, should watch out because the total distance is actually 12

    #     else: 
    #         print('ERROR: that distribution', set_distribution, 'does not exist, exiting out')
    #         sys.exit(0)

    #     self.y_lim[1] = y_max #+ self.y_lim[1] # correct the travel length 

    
    # def createFruitDistribution(self, fruitD, set_algorithm, set_distribution, density, x_seed, y_seed, z_seed):
    #         if set_distribution == 0:
 
    #             if self.data_name == 'juan':
    #                 csv_file = './TREE_FRUIT_DATA/20220811_apples_Juan.csv'
    #                 is_meter = 1
    #             elif self.data_name == 'raj':
    #                 csv_file = './TREE_FRUIT_DATA/apple_tree_data_2015/Applestotheright.csv'
    #                 is_meter = 0

    #             [numFruit, sortedFruit] = fruitD.csvFile(csv_file, is_meter)

    #         elif set_distribution == 1:
    #             if set_algorithm == 1:
    #                 [numFruit, sortedFruit] = fruitD.uniformRandomMelon(density, y_seed, z_seed)
    #             else:
    #                 [numFruit, sortedFruit] = fruitD.uniformRandom(density, x_seed, y_seed, z_seed)
    #             # print()
    #             # print('--------------------------------------------')
    #             # print('Number of fruit:', numFruit)
    #             # print()

    #         elif set_distribution == 2: 
    #             fruit_in_cell = math.ceil(density * (self.cell_h*self.cell_l*self.arm_reach)) # num of fruit in front of cell if using (equalCellDensity())
    #             print('Number of fruit in each cell:', fruit_in_cell)
    #             print()
    #             [numFruit, sortedFruit] = fruitD.equalCellDensity(self.n_row, self.n_col, self.cell_h, self.cell_l, self.arm_reach, fruit_in_cell, x_seed, y_seed, z_seed)

    #         elif set_distribution == 3: 
    #             densities = np.array([5, 4, 3])
    #             [numFruit, sortedFruit] = fruitD.uniformRandomMelon_MultipleDensity(densities, y_seed, z_seed)

    #         # elif set_distribution == 4: 
    #         #     [numFruit, sortedFruit] = fruitD.column(v_vy, v_max, a_max, t_grab, self.n_row, self.n_col, self.cell_h, z_seed)
    #         elif set_distribution == 5:
    #             z_coord = (self.cell_h / 2) + 0.7
    #             [numFruit, sortedFruit] = fruitD.columnUniform_melon(numFruit, self.d_y, z_coord)
                
    #         elif set_distribution == 6:
    #             csv_file = './TREE_FRUIT_DATA/apple_tree_data_2015/Applestotheright.csv'
    #             [numFruit, sortedFruit] = fruitD.csvFile_reduced(csv_file, 0, density, x_seed)

    #         else: 
    #             print('not a correct fruit distribution, defaulting to uniform random')
    #             if set_algorithm == 1:
    #                 [numFruit, sortedFruit] = fruitD.uniformRandomMelon(density, y_seed, z_seed)
    #             else:
    #                 [numFruit, sortedFruit] = fruitD.uniformRandom(density, x_seed, y_seed, z_seed)

    #         return([numFruit, sortedFruit])
    


    
    def solve_mip(self, arm, fruit, n_snap, v_vy_curr, set_MIPsettings, fruit_travel_matrix, sortedFruit, FPE_min=0.5, v_vy_lb_cmps=1, v_vy_ub_cmps=5):
        '''
            Solve the MIP formulation based on already created arm and fruit object lists, the vehicle velocity in cm/s for the run, the MIP settings (makespan or not, etc.).
            Has default value for keyword FPE_min in case makespan is used, this is NOT a recommended value. The FPE_min value should be thoughtfully dtermined based on 
            on settings such as v_vy, length of the vehicle, horizon, and "recalculation" travel. Same deal with the upper and lower
            velocity bounds  

            n_snap is the available number of fruits in the snapshot       
        '''
        ## Build useful data structures
        # lists:
        K  = [*range(self.n_col)]        # list of columns containing the arms (number of arms in a column based on the number of rows)
        L  = [*range(self.n_row)]        # list of horizontal row numbers, uses the argument-unpacking operator *
        N  = [i.index for i in fruit]    # list of fruit indexes starting at 0, with an offset when needed
        Y  = [i.y_coord for i in fruit]  # list of fruits' y-coordinate (x-coordinate in the paper)
        Z  = [i.z_coord for i in fruit]  # list of fruits' z-coordinate
        TX = [i.Tx for i in fruit]       # list of fruit extension times

        self.job = self.createJobs(arm, fruit, v_vy_curr, self.cell_l)

        # save the real index of the 0th fruit (to get the difference)
        try:
            offset_fruit_index = self.job[0].fruit_i.real_index
            # print('job\'s first real index:', offset_fruit_index, '\n')
        except IndexError:
            # no fruits visible, not even already picked fruits, probably the start where there are no fruits (entering the row)
            offset_fruit_index = 0

        # if self.print_out == 1:
            # print('number of arms in each horizontal row:',K, 'with length', len(K))
            # print()
            # print('number of horizontal rows:',L, 'with length', len(L))
            # print()
            # print('number of fruits:\n',N, '\nwith length', len(N))
            # print()
            # print('fruit y-coordinate:', Y, 'with length', len(Y))
            # print()
            # print('fruit z-coordinate:', Z, 'with length', len(Z))
            # print()
            # print('number of jobs:', len(self.job))
            # print()
        
        if set_MIPsettings == 0:
            # makespan has to calculate it's own TW values, so this only needs to be done for non-makespan runs
            TW_start = {i : [j.TW_start for j in self.job if j.fruit_i.index == i] for i in N}
            TW_end   = {i : [j.TW_end for j in self.job if j.fruit_i.index == i] for i in N}

            if len(TW_start) != len(TW_end) or len(TW_start) != len(N) or len(self.job) != len(N)*len(K)*len(L):
                print('Error: The number of fruit and the number of jobs or TW times do not match, exiting out of system')
                print('Number of jobs', len(self.job), 'and number of fruits*arms', len(N)*len(K)*len(L))
                print()
                sys.exit(0)


        if len(N) != len(Y) or len(N) != len(Z) or len(Y) != len(Z):
            print('Error: Indexes for fruit index, y, and z-coordinates do not match, exiting out of system')
            print()
            sys.exit(0)

        ### Create model
        m = gp.Model("v_vy_loop_mip")

        ### change needed model parameters
        ### see https://www.gurobi.com/documentation/9.5/refman/python_parameter_examples.html#PythonParameterExamples
        
        if set_MIPsettings == 1:
            # if n_snap = 0, change to n_snap = .0001 to avoid division by 0
            if n_snap == 0:
                n_snap = 0.0001
            # if velocity becomes a variable, it is multiplied with another variable requiring the following settings
            m.params.NonConvex = 2
            m.setParam('NonConvex', 2)

        # Due to *very* high complexity, limits time in the no relaxation heuristic
        # see https://www.gurobi.com/documentation/9.5/refman/norelheurtime.html
        m.setParam('NoRelHeurTime', self.noRel_time_ub)

        # limit the maximum amount of time the solver takes to find a solution -> if gap isn't 0% then "no solution"
        # if NoRel == TimeLimit, only NoRel used (https://support.gurobi.com/hc/en-us/community/posts/4414052781073-NoRel-and-setting-time-limits)
        m.setParam('TimeLimit', self.timLim_time_ub) # stop after half an hour


        ### Decision variables
        # to create the variables, see https://www.gurobi.com/documentation/9.5/refman/py_model_addvars.html
        # Arm-fruit assignment (is fruit i picked by arm k in row l)
        x = m.addVars(K, L, N, vtype=GRB.BINARY, name="x")

        # upper bound when arm can pick the fruit, set at the time when the vehicle will reach the end of it's travel length
        # print('\nthe velocity being used to calculate time bounds:', v_vy_curr) 
        distance_traveled = abs(self.q_vy - self.q_vy0) # in m, the distance the system has traveled from the start
        t_move = (self.travel_l) / (v_vy_curr/100) # in s, the duration of the snapshot

        if self.travel_l >= self.vehicle_l:
            # if the travel length is larger than the vehicle
            t_ub = (self.travel_l + self.cell_l + self.hor_l + distance_traveled) / (v_vy_curr/100)   # in m/s
        elif self.travel_l < self.vehicle_l and self.travel_l > 0:
            # when the travel length is smaller than the vehicle
            t_ub = (self.vehicle_l + self.cell_l + self.hor_l + distance_traveled) / (v_vy_curr/100)  # in m/s
        elif self.travel_l <= 0:
            print('ERROR: travel length is zero, exiting out of system')
            sys.exit(0)

        if self.print_out == 1:
            print('\nThe travel length being processed is {:.3f} m'.format(self.travel_l))
            print(f'The distance traveled from the start: %4.2f m' % distance_traveled)
            print(f'The potential travel distance window without the horizon: %4.2f s' % t_move)
            print('The upper bound for when an arm can pick a fruit is {:.3f} sec\n'.format(t_ub))

        # Time arm k, l reaches fruit i
        t = m.addVars(K, L, N, lb=0, ub=t_ub, name="t")

        if set_MIPsettings == 1:
            # any TW start and end should be less than the total move time since nothing can be picked after this
            # the lower bound allows for negatives only because the aux values that take place after will switch to 0 if negative
            tw_s          = m.addVars(K, N, lb=-t_ub, ub=t_ub, name="tw_s")
            tw_e          = m.addVars(K, N, lb=-t_ub, ub=t_ub, name="tw_e")

            # required because gurobi doesn't like >= or <= constraints that deal with two variables
            aux_end       = m.addVars(K, N, lb=0, name="aux_end")
            aux_start     = m.addVars(K, N, lb=0, name="aux_start")
            aux_comp      = m.addVars(K, N, lb=0, name="aux_compare")
            aux_comp_end  = m.addVars(K, N, lb=0, name="aux_compare_end")

            # in cm/s, vehicle velocity along orchard row
            # bounded by the cell length and Td (melon paper) and bounded by max velocity of the lift (90 cm/s)
            # v_vy          = m.addVar(lb=v_vy_lb_cmps, ub=v_vy_ub_cmps, name="v_vy")
            v_vy          = m.addVar(vtype=GRB.INTEGER, lb=v_vy_lb_cmps, ub=v_vy_ub_cmps, name="v_vy")

            # variable t_move calculated with currently chosen v_vy
            t_move_var    = m.addVar(lb=0, name="var_t_move")

            FPE_var       = m.addVar(lb=0, name="FPE")
            FPT_var       = m.addVar(lb=0, name="FPT")
            
            # need slack variables for soft constraints 
            # see https://support.gurobi.com/hc/en-us/community/posts/5628368009233-Soft-Constraints-being-treated-as-Hard-Constraints-
            minFPE_slack     = m.addVar(lb=0, name="FPEslackMin")
            minFPT_slack     = m.addVar(lb=0, name="FPTslackMin")

            # add a starting guess value to the variable
            # see https://www.gurobi.com/documentation/9.5/refman/start.html#attr:Start
            # v_vy.start = 2 # in cm/s

            # create a variable that saves the last picking time, or makespan
            x_weighted    = m.addVars(K, L, N, lb=0, name="x_weighted")   # used to get the makespan of harvested fruits, not all fruits
        

        ### Constraints
        # At most one arm can be assigned to a fruit (1)
        m.addConstrs((x.sum('*', '*', i) <= 1 for i in N), name="assignOne")

        # Time elapsed between pickup of any two fruit reached by the same arm is at least Td (2)
        # m.addConstrs((t[k, l, i] + self.Td - t[k, l, j] <= self.M * (2 - x[k, l, j] - x[k, l, i]) for i in N for j in N for k in K for l in L if Y[j] > Y[i]), name="atLeast")
        m.addConstrs((t[k, l, i] + TX[i] + fruit_travel_matrix[i+offset_fruit_index, j+offset_fruit_index] + TX[j] + self.t_grab - t[k, l, j] <= self.M * (2 - x[k, l, j] - x[k, l, i]) for i in N for j in N for k in K for l in L if Y[j] > Y[i]), name="atLeast")
        
        # If fruit z-coord is outside of arm's range, do not pick it
        if self.starting_row_n >= self.n_row:
            # if the mip is divided so each row gets a mip run, the row number has to be changed
            n_row = self.starting_row_n
            m.addConstrs(((x[k, l, i] == 0) for i in N for l in L for k in K if Z[i] < self.z_row_bot_edges[k,n_row] or Z[i] > self.z_row_top_edges[k,n_row]), name="verticalWorkArea")
        else:
            m.addConstrs(((x[k, l, i] == 0) for i in N for l in L for k in K if Z[i] < self.z_row_bot_edges[k,l] or Z[i] > self.z_row_top_edges[k,l]), name="verticalWorkArea")

        # If fruit was removed because it's scheduled and picked, do not schedule it again
        # offset added because the indexes have to start at 0 for every run, but when scheduling snapshots, the index of the fruits may not start at 0
        m.addConstrs(((x[k, l, i] == 0) for i in N for l in L for k in K if sortedFruit[4,i+offset_fruit_index] == 2), name="removeScheduledAndPicked")
        # if the travel distance between snapshots is less than the view distance, don't harvest fruits that are too far forward for arms in column k to pick given that limited travel distance
        m.addConstrs(((x[k, l, i] == 0) for i in N for l in L for k in K if Y[i] - (self.q_vy + (k + 1)*self.cell_l) >= self.travel_l), name="fruitInHorizon")
        # if TW_end is negative, the fruit has passed the column k’s back edge and cannot be harvested by any arm in that column
        m.addConstrs(((x[k, l, i] == 0) for i in N for l in L for k in K if Y[i] - (self.q_vy + k*self.cell_l) <= 0 ), name="fruitStartPassed")

        # If the fruit is at the back edge, then it cannot be picked because the arm has to extend out and grab the fruit
        m.addConstrs(((t[k, l, i] - TX[i] - self.t_grab >= -(1.5 + self.t_grab)*(1 - x[k, l, i]) + 0.001*x[k, l, i]) for i in N for l in L for k in K), name="minHarvestTime")
        
        if set_MIPsettings == 0:   
            # the time of harvest for fruit i by arm in column k and row l has to be within the calculated time windows for that fruit and that column 
            m.addConstrs((t[k, l, i] >= min(TW_start[i][k], TW_end[i][k]) for i in N for l in L for k in K), name="timeWinStart")
            m.addConstrs((t[k, l, i] <= max(TW_start[i][k], TW_end[i][k]) for i in N for l in L for k in K), name="timeWinEnd")

        elif set_MIPsettings == 1:
            # min values for slack constranints
            FPE_min_val = .35    # the minimum FPE in a soft constraint
            FPT_min_val = 2.5   # in fruits/s, the minimum FPT in a soft constraint
            # calculate the time windows for each fruit given the tested/chosen velocity 
            m.addConstrs(((tw_s[k, i] * (v_vy / 100) == (Y[i] - (self.q_vy + (k + 1)*self.cell_l))) for i in N for k in K), name="TW_start")
            m.addConstrs(((tw_e[k, i] * (v_vy / 100) == (Y[i] - (self.q_vy + k*self.cell_l))) for i in N for k in K), name="TW_end")
            
            # turns x[k, l, i] == 0 if t[k, l, i] * (v_vy/100) >= travel_l, or the fruit cannot be picked if the time of picking is larger than the time between snapshots
            # to learn about building this constraint see https://or.stackexchange.com/questions/5860/link-a-binary-variable-to-continuous-variable-in-java-gurobi
            m.addConstrs(((self.travel_l - t[k, l, i]*(v_vy / 100) >= -(self.vehicle_l + self.travel_l + self.hor_l) * (1 - x[k, l, i]) + 0.0001 * x[k, l, i]) for i in N for l in L for k in K), name="tCannotPasstMove")

            # to learn how to deal with max/min with variables
            # see https://support.gurobi.com/hc/en-us/community/posts/360076808711-how-to-add-a-max-constr-General-expressions-can-only-be-equal-to-a-single-var
            # comparing to 0 because the value should never be negative
            # if the time window start or end times are negative, then they should be zeroed out because negative means that time has been passed
            m.addConstrs((aux_end[k, i]      == gp.max_(0, tw_e[k, i]) for i in N for k in K), name="fixEndNegative")
            m.addConstrs((aux_start[k, i]    == gp.max_(0, tw_s[k, i]) for i in N for k in K), name="fixStartNegative")
            # if aux_end <= 0, then aux_start should also be < 0, so choose the min value of the two
            m.addConstrs((aux_comp[k, i]     == gp.min_(aux_end[k, i], aux_start[k, i]) for i in N for k in K), name="ifEndZeroStartZero")
            m.addConstrs((aux_comp_end[k, i] == gp.max_(aux_end[k, i], aux_start[k, i]) for i in N for k in K), name="maxIsEnd")

            # Ensure each node is visited within the given time window (3) and (4)
            # TW_start and TW_end are matching the fruit index number exactly (disctionary), so [2][0] == index 2 (even 
            # though it starts at zero, second arm back from 0th arm)  
            m.addConstrs(((t[k, l, i] >= aux_comp[k, i]) for i in N for l in L for k in K), name="timeWinStart")
            m.addConstrs(((t[k, l, i] <= aux_comp_end[k, i]) for i in N for l in L for k in K), name="timeWinEnd")

            # Ensure at least 90% (or desired percentage) of available fruit are harvested, not needed if multi-objective setup works
            # m.addConstr((gp.quicksum(x[k, l, i] for i in N for l in L for k in K)/n_snap >= FPE_min), name="percentHarvest")

            # # set makespan as the latest t^k_i value/ for every arm no matter the row or fruit
            # # see https://support.gurobi.com/hc/en-us/community/posts/360071830171-Use-index-of-decision-variable-in-max-constraint
            # m.addConstrs(((t_harvested[k, l, i] == t[k, l, i] * x[k, l, i]) for i in N for l in L for k in K), name='timeHarvestedFruits')
            # m.addConstrs((t_max_arm[k]          == gp.max_(t_harvested.select(k, '*', '*')) for k in K), name='max_value')

            # opposite of penalty added to the sum(x) objective so that a weight that changes with the velocity can be used to get better results
            m.addConstrs(((x_weighted[k, l, i] == x[k, l, i] * (v_vy / 100)) for i in N for l in L for k in K), name='xWeighted')

            # FPT * FPE
            m.addConstr((FPT_var <= (1/self.travel_l) * gp.quicksum(x_weighted[k, l, i] for i in N for l in L for k in K)), name='FPT')
            m.addConstr((FPE_var <= (1/n_snap) * gp.quicksum(x[k, l, i] for i in N for l in L for k in K)), name='FPE')
            # m.addConstr((FPT_var - maxFPT_slack <= (1/self.travel_l) * gp.quicksum(x_weighted[k, l, i] for i in N for l in L for k in K)), name='FPT')
            # m.addConstr((FPE_var - maxFPE_slack <= (1/n_snap) * gp.quicksum(x[k, l, i] for i in N for l in L for k in K)), name='FPE')
            # m.addConstr((FPE_var <= 0.85), name='FPEmax')
            if n_snap > 10:
                m.addConstr((FPE_var + minFPE_slack >= FPE_min_val), name='FPEmin')
                m.addConstr((FPE_var + minFPT_slack >= FPT_min_val), name='FPTmin')

            # m.addConstrs((t_max_arm[k] == gp.max_(t.select(k, '*', '*')) for k in K), name='max_value')  # doesn't take into account if fruit harvested or not
            
            # # obtain the makespan and then normalize it over the total vailable travel time
            # m.addConstrs((makespan >= t_max_arm[k] for k in K), name='makespan_constraint')
            # m.addConstr((makespan_norm == makespan * (v_vy / 100) / self.travel_l), name='normalized_makespan')

            # m.addConstrs((((v_vy / 100) * aux_var[k, l, i] == x[k, l, i]) for i in N for l in L for k in K), name="aux")
            # m.addConstrs(((v_vy * aux_var[k, l, i] == (self.travel_l / n_snap) * x[k, l, i]) for i in N for l in L for k in K), name="aux")


        ### Objective function
        if set_MIPsettings == 0:
            m.setObjective(gp.quicksum(x[k, l, i] for i in N for l in L for k in K), GRB.MAXIMIZE)
            
        elif set_MIPsettings == 1:
            # makespan will use multiple objectives: minimize the makespan and maximize the FPE 
            # to define multiple hierarchical objectives see https://stackoverflow.com/questions/56120143/how-to-write-a-multi-objective-function-in-gurobi-python
            # Gurobi's specifying of https://www.gurobi.com/documentation/9.5/refman/specifying_multiple_object.html
            # Gurobi's working with https://www.gurobi.com/documentation/9.5/refman/working_with_multiple_obje.html 
            # example: https://www.gurobi.com/documentation/10.0/examples/workforce5_py.html#subsubsection:workforce5.py
            # set priority by index and priority, index 0 get automatic priority, how much priority can be set with priority--The higher (up to 10) the more priority 
            # makespan objective which affects the FPT 
            m.ModelSense = GRB.MAXIMIZE 
            m.setObjective((FPE_var*FPT_var - (1/n_snap)*minFPE_slack) - (1/n_snap)*minFPT_slack)# - (1/n_snap)*maxFPE_slack - maxFPT_slack)
            # m.setObjective((FPE_var * makespan))

        ## see https://github.com/jckantor/ND-Pyomo-Cookbook/blob/master/notebooks/04.03-Job-Shop-Scheduling.ipynb
        ## and https://support.gurobi.com/hc/en-us/community/posts/360071830171-Use-index-of-decision-variable-in-max-constraint

        ## write model into a file
        # see https://www.gurobi.com/documentation/9.5/refman/py_model_write.html
        # https://www.gurobi.com/documentation/9.5/refman/model_file_formats.html
        title_lp  = './mip_files/v_vy_' + str(v_vy_curr) + '_loop_mip.lp'
        title_mps = './mip_files/v_vy_' + str(v_vy_curr) + '_loop_mip.mps'
        m.write(title_lp)
        m.write(title_mps)
        m.optimize()

        print('lp and mps files saved under: v_vy =', str(v_vy_curr))
        
        status = m.Status
        if status in [GRB.INF_OR_UNBD, GRB.INFEASIBLE, GRB.UNBOUNDED]:
            print("Model is either infeasible or unbounded.")
            print("...")
            # obtain model Irreducible Inconsistent Subsystem (IIS)
            print('computing model IIS and writing it to an ipl doc') # see https://www.gurobi.com/documentation/10.0/refman/py_model_computeiis.html
            m.computeIIS()
            m.write("model.ilp")
            print("Exiting simulator.")
            sys.exit(0)
        elif status != GRB.OPTIMAL:
            print("Optimization terminated with status {}".format(status))
    #         sys.exit(0)

        if set_MIPsettings == 1:
            # print('Minimized makespan                                                 = %0.4f seconds' %makespan.X)

            # print out the slack value
            print('\nFPE\n    scheduled = {:.3f},    min provided= {:.3f},      slack value = {:.3f}'.format(FPE_var.X, FPE_min_val, minFPE_slack.X))
            print('FPT\n    scheduled = {:.3f},    min provided= {:.3f},      slack value = {:.3f}\n'.format(FPT_var.X, FPT_min_val, minFPT_slack.X))
            # print('FPE', FPE_var.X, 'min FPE', FPE_min_val ,'FPE slack value = ', minFPE_slack.X)
            # print('FPT', FPT_var.X, 'min FPT', FPT_min_val ,'FPT Slack value = ', minFPT_slack.X)

        fruit_picked_by = list()                             # list that saves which arm picks which fruit
        fruit_picked_at = list()                             # list that saves at what time an arm picks a fruit, based on fruit_picked_by "topography"
        self.curr_j     = np.zeros([self.n_row, self.n_col]) # array to save the sum of fruits picked by each arm

        for n in range(self.n_row):
            if self.n_row > 1:
                fruit_picked_by.append([])
                fruit_picked_at.append([])

            for k in range(self.n_col+1):
                if self.n_row > 1:
                    fruit_picked_by[n].append([])
                    fruit_picked_at[n].append([])
                else:
                    fruit_picked_by.append([])
                    fruit_picked_at.append([])

        ### Print results
        # print()
        # print('###############################')
        # print('how time picked is saved')
        # print(t)
        # print()
        # print('###############################')
        # Assignments    
    #     print()
        for j in self.job:
            if self.starting_row_n >= self.n_row:
                n_row = 0
            else:
                n_row = j.arm_k.row_n

            if x[j.arm_k.arm_n, n_row, j.fruit_i.index].X > 0:
                # if fruit was scheduled to be harvested
    #             print('fruit', j.fruit_i.index, 'assigned to arm', j.arm_k.arm_n, 'at t = ', t[j.arm_k.arm_n, j.fruit_i.index].X)
                # save picked to sortedFruit
                sortedFruit[4, j.fruit_i.real_index] = 1  # save to the real index on sortedFruit
                # self.curr_j[n_row, j.arm_k.arm_n] += 1
                if self.n_row > 1:
                    self.curr_j[n_row, j.arm_k.arm_n] += 1
                    fruit_picked_by[n_row][j.arm_k.arm_n].append(j.fruit_i.real_index)
                    fruit_picked_at[n_row][j.arm_k.arm_n].append(t[j.arm_k.arm_n, n_row, j.fruit_i.index].X)

                else:
                    # print(self.curr_j)
                    self.curr_j[0,j.arm_k.arm_n] += 1
                    fruit_picked_by[j.arm_k.arm_n].append(j.fruit_i.real_index)
                    fruit_picked_at[j.arm_k.arm_n].append(t[j.arm_k.arm_n, n_row, j.fruit_i.index].X)

        no_pick = np.where(sortedFruit[4,:] == 0)  # flag for scheduled == 1, scheduled and picked == 2
    #     print('not picked indexes:', no_pick[0])

        for no_pick_i in no_pick[0]:
            # Adding the indexes of non-picked fruit to a sublist at the end of the first 
            # horizontal row's list of sublists
            if self.n_row > 1:
                # if multiple horizontal rows, append the non-picked sublist to the first horizontal row's list of lists
                fruit_picked_by[0][self.n_col].append(no_pick_i)
            else:
                fruit_picked_by[self.n_col].append(no_pick_i)

        if self.print_out == 1:
        #     print('model variables:', m.getAttr("x", m.getVars()))
            print()
            if set_MIPsettings == 1:
                print('Makespan\'s chosen velocity:', v_vy.X, 'cm/s')
            print('set M value:', self.M)
            print()

        # check that TW and t^k_i match indexes and arms
        # print()
        # for i in N:
        #     for k in K:
        #         row = 0
        #         # for row in range(self.n_row):
        #         # TW start and end are the same for every row, but not t, so depending on what's being checked, add or remove this for loop and add to t[].X
        #         print('For row', row, 'arm',k, 'and fruit', i, 'TW start:', TW_start[i][k], 'TW end:', TW_end[i][k], 'and t^k_i', t[k, row, i].X)
        #     print('Fruit harvested?', self.sortedFruit[4,i])
        #     print('----')

        # print('fruit picked by [0]', fruit_picked_by[0])
        # print('fruit picked by [1]', fruit_picked_by[1])
        if set_MIPsettings == 0:
            # only need which fruit were picked by what and when
            return([fruit_picked_by, fruit_picked_at])
        elif set_MIPsettings == 1:
            # needs the above as well as the chosen velocity
            return([fruit_picked_by, fruit_picked_at, v_vy.X])
        
    


## Required classes for the arms, fruits, and jobs
class Arm():
    def __init__(self, row_n, arm_n):
        self.row_n = row_n
        self.arm_n = arm_n

    # def __str__(self):
    #     return f"Arm: {self.arm_n}\n Horizontal Row Number: {self.row_n}"


class Fruit():
    def __init__(self, traj_calc, index, real_index, x_coord, y_coord, z_coord):#, job, tStart, tEnd, tDue):
        self.index = index            # fruit's index when ordered by y-coordinate
        self.real_index = real_index  # fruit's real index before it's zeroed if using snapshots
        self.x_coord = x_coord        # x-coordinate of the fruit
        self.y_coord = y_coord        # y-coordinate of the fruit
        self.z_coord = z_coord        # z-coordiante of the fruit
        # calculate the extension + grab + retract of the arm (use trajectory.py module)
        start_x = 0
        v_max   = 2 # m/s from Motor Sizing Google sheet calculations if we want to keep triangular profile to maximize speed
        a_max   = 4 # m/s^2 from Motor Sizing Google sheet calculations if we want desired linear movement time to be 1 second
        d_max   = a_max # if motors allow, keep equal to a_max

        # calculate extension to fruit in x-axis
        traj_calc.adjInit(start_x, 0.) # start moving from zero speed
        traj_calc.noJerkProfile(traj_calc.q0, self.x_coord, traj_calc.v0, v_max, a_max, d_max)
        
        self.Tx = traj_calc.Ta + traj_calc.Tv + traj_calc.Td  # in s, the total extensiontime for this fruit
        
        # self.Tx = t_ext + t_grab + t_ext 
        # print('the extension time of this fruit takes {:.3f} sec\n'.format(self.Tx))
        
    # def __str__(self):
    #     return f"Fruit Index: {self.index}\n  Y-axis location: {self.y_coord}\n"



class Job():
    def __init__(self, fruit_i, arm_k, q_vy, v_vy, cell_l, pick_travel_l):
        self.fruit_i  = fruit_i
        self.arm_k    = arm_k
        self.v_vy     = v_vy
        offset        = (cell_l - pick_travel_l) / 2  # in m, assume centered
        # k+1 was added because the MIP model in paper assumes k starts at 1
        # self.TW_start = (self.fruit_i.y_coord + (self.arm_k.arm_n)*cell_l + 1/2 * offset) / (v_vy/100)
        # self.TW_end   = (self.fruit_i.y_coord + (self.arm_k.arm_n + 1)*cell_l - 1/2 * offset) / (v_vy/100)

        # fix to start the back of the vehicle as (0,0)
        self.TW_start = (self.fruit_i.y_coord - (q_vy + (self.arm_k.arm_n + 1)* cell_l) ) / (v_vy/100) 
        if self.TW_start < 0:
            # correction since it cannot be harvested at negative values, so make the window smaller by making it able to start at 0? -> might require the addition of q_vy_start
            self.TW_start = 0

        self.TW_end   = (self.fruit_i.y_coord - (q_vy + self.arm_k.arm_n * cell_l) ) / (v_vy/100)
        if self.TW_end <= 0:
            # if TW_end is less than zero, the fruit cannot be harvested so just set it all to zero
            # was causing problems for the TW constraints because the max and min were not always the start and end. 
            self.TW_start = 0
            self.TW_end = 0


## create snapshot object for data analysis
class Snapshot(object):
    def __init__(self, n_col, n_row, horizon_l, vehicle_l, cell_l, v_max, a_max, set_algorithm, Td, v_vy, FPE, FPT, y_lim, numFruit, curr_j, sortedFruit, fruit_picked_by, state_time):
        # constants for the whole run
        self.n_col      = n_col
        self.n_row      = n_row
        self.horizon_l  = horizon_l
        self.vehicle_l  = vehicle_l
        self.cell_l     = cell_l
        self.v_max      = v_max
        self.a_max      = a_max
        if set_algorithm == 1:
            self.Td     = Td
            
        # constants and results for each snapshot in the run
        self.v_vy       = v_vy
        self.FPE        = FPE
        self.FPT        = FPT
        self.y_lim      = y_lim
        self.actual_numFruit = numFruit
        self.curr_j          = curr_j
        self.avg_PCT         = 0.
        self.state_time      = state_time
        self.fruit_picked_by = fruit_picked_by
        self.sortedFruit     = sortedFruit

    def __str__(self):
        return f"Arm: {self.n_col}\n Horizontal Row Number: {self.n_row}"