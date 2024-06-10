# import math
import numpy as np
import sys

from trajectory import *           # import module to calculate the trapezoidal/S-curve (S-curve not working yet) tajectory calculator

class FCFS(object):
    def __init__(self):
        '''
            First come, first serve (FCFS) implementation that takes a list of jobs, handling times, job time windows, multi-armed robot configuration
            and returns the arm and the order at which the fruits are harvested, and when they are harvested by that arm.
        '''


    def main(self, n_col, n_row, fruit, job, V, Q, d_cell, d_o, fruit_travel_matrix, sortedFruit, z_row_bot_edges, z_row_top_edges):
        V_m    = V / 100 # change once to m/s
        t_grab = 0.5     # in s, the constant time it takes to harvest a fruit once reached 

        # save the real index of the 0th fruit (to get the difference) when using snapshots
        try:
            # offset_fruit_index = self.job[0].fruit_i.real_index
            offset_fruit_index = job[0].fruit_i.real_index
            # print('job\'s first real index:', offset_fruit_index, '\n')
            # print('job\'s last real index:', job[-1].fruit_i.real_index, '\n')
        except IndexError:
            # no fruits visible, not even already picked fruits, probably the start where there are no fruits (entering the row)
            offset_fruit_index = 0  


        ## Build useful data structures
        # lists:
        # C  = [*range(n_col)]             # list of columns containing the arms (number of arms in a column based on the number of rows)
        R  = [*range(n_row)]             # list of horizontal row numbers, uses the argument-unpacking operator *
        N  = [i.index for i in fruit]    # list of fruit indexes starting at 0, with an offset when needed
        Y  = [i.y_coord for i in fruit]  # list of fruits' y-coordinate (x-coordinate in the paper)
        Z  = [i.z_coord for i in fruit]  # list of fruits' z-coordinate
        TX = [i.Tx for i in fruit]       # list of fruit extension times

        busy_till       = np.zeros([n_row, n_col])                  # when does this arm (c,r) stop being busy and is available for a new job
        busy_with       = np.zeros([n_row, n_col], dtype=np.int32)    # previous chosen fruit to be harvested for the arm (c,r)

        # lists of the desired results
        fruit_picked_by = list()                      # list that saves which arm picks which fruit
        fruit_picked_at = list()                      # list that saves at what time an arm picks a fruit, based on fruit_picked_by "topography"
        self.curr_j     = np.zeros([n_row, n_col])    # array to save the sum of fruits picked by each arm
        # build out the lists to be the right size (tried replication *, but the lists point at each other)
        for r in range(n_row):
            if n_row  > 1:
                fruit_picked_by.append([])
                fruit_picked_at.append([])

            for c in range(n_col+1):
                if n_row > 1:
                    fruit_picked_by[r].append([])
                    fruit_picked_at[r].append([])
                else:
                    fruit_picked_by.append([])
                    fruit_picked_at.append([])


        for i_fruit in N:
            # print('\n\nfruit index being scheduled %d' %i_fruit)
    
            # calculate the start and end times of the time window during which the fruit is available to be harvested (column 0 to column n_col-1)
            # although the last column is indexed at n_col-1, arange does not include the last value so we want to calulate the last column as n_col (which will not be included)
            tw_s0 = (Y[i_fruit] - (Q + (0 + 1)*d_cell + 0*d_o)) / V_m
            tw_e0 = (Y[i_fruit] - (Q + (0)*d_cell + 0*d_o)) / V_m
            tw_s_last = (Y[i_fruit] - (Q + (n_col + 1)*d_cell + n_col*d_o)) / V_m  # it should be (n_col-1) but using n_col to add another step to tw_s and tw_e because arange doesn't include the stop value 
            tw_e_last = (Y[i_fruit] - (Q + n_col*d_cell + (n_col-1)*d_o)) / V_m        # it should be (n_col-1) but using n_col to add another step to tw_s and tw_e because arange doesn't include the stop value 
            # print(f'first and last tw_s: %4.2f, %4.2f s' %(tw_s0, tw_s_last+(d_cell + d_o)/V_m))
            # print(f'first and last tw_e: %4.2f, %4.2f s' %(tw_e0, tw_e_last+(d_cell + d_o)/V_m))

            # calculate all time window starts and ends for this fruit
            tw_s = np.arange(tw_s0, tw_s_last, step=-(d_cell + d_o)/V_m) # included an extra step to tw_last because arange doesn't include the stop value
            tw_e = np.arange(tw_e0, tw_e_last, step=-(d_cell + d_o)/V_m)
            # print('number of columns %d' %n_col)
            # print('start times:', tw_s)
            # print('end times:', tw_e)
 
            for c_col in range(n_col-1, -1, -1): # needs to include index 0
                # go backwards because the frontmost arm is most likely to hit fruits first or not be able to harvest them
                # if tw_s[c_col] <= 0 or tw_e[c_col] <= 0:
                    # the fruit cannot be harvested by this column

                if sortedFruit[4, i_fruit+offset_fruit_index] > 0: 
                    # if the fruit has not been scheduled or harvested then this can proceed, otherwise break out and schedule the next fruit
                    break

                for r_row in R:
                    # check what row the fruit is in
                    if Z[i_fruit] > z_row_bot_edges[r_row, c_col] and Z[i_fruit] < z_row_top_edges[r_row, c_col]:

                        j_fruit = busy_with[r_row, c_col] # previously harvested fruit
                        # print('previously scheduled fruit index: %d' %j_fruit)

                        if i_fruit > 0:
                            j_to_i_travel = fruit_travel_matrix[j_fruit+offset_fruit_index, i_fruit+offset_fruit_index]

                        elif i_fruit == 0:
                            # if the fruit index is zero, there is no previous fruit to compare against so I should calculate the movement distance from Q and the middle of the row
                            
                            ###### IF CHANGING v_max, etc., CHANGE IT IN fruit_handler.py AS WELL!! ########

                            v_max_y   = 1.4 # m/s from Motor Sizing Google sheet calculations if we want to keep triangular profile to maximize speed
                            a_max_y   = 2.8 # m/s^2 from Motor Sizing Google sheet calculations if we want desired linear movement time to be 1 second
                            d_max_y   = a_max_y # if motors allow, keep equal to a_max
                            # initialize the ability to calculate trajectory
                            traj_calc_y = Trajectory(v_max_y, a_max_y, d_max_y) 

                            # allow calculations for the z-axis, 0.61? m worst case
                            v_max_z   = 1.3 # m/s from Motor Sizing Google sheet calculations if we want to keep triangular profile to maximize speed
                            a_max_z   = 2.8 # m/s^2 from Motor Sizing Google sheet calculations if we want desired linear movement time to be 1 second
                            d_max_z   = a_max_z # if motors allow, keep equal to a_max
                            # initialize the ability to calculate trajectory
                            traj_calc_z = Trajectory(v_max_z, a_max_z, d_max_z) 

                            start_y = Q  # in m, vehicle start location
                            start_z = (z_row_bot_edges[r_row, c_col] + z_row_top_edges[r_row, c_col]) / 2 # in m, centered between the bot and top edge of this row

                            end_y   = sortedFruit[1,0]  # in m, fruit 0's y_coordinate
                            end_z   = sortedFruit[2,0]  # in m, fruit 0's z_coordinate

                            # calculate side-to-side movement to fruit in y-axis
                            traj_calc_y.adjInit(start_y, 0.) # start moving from zero speed
                            traj_calc_y.noJerkProfile(traj_calc_y.q0, end_y, traj_calc_y.v0, v_max_y, a_max_y, d_max_y)
                            T_y = traj_calc_y.Ta + traj_calc_y.Tv + traj_calc_y.Td 
                            
                            # calculate up and down movement to fruit in z-axis
                            traj_calc_z.adjInit(start_z, 0.) # start moving from zero speed
                            traj_calc_z.noJerkProfile(traj_calc_z.q0, end_z, traj_calc_z.v0, v_max_z, a_max_z, d_max_z)
                            T_z = traj_calc_z.Ta + traj_calc_z.Tv + traj_calc_z.Td 

                            # add to matrix and mirror it
                            j_to_i_travel = max(T_y, T_z)
                       
                        # check if the arm is busy up to tw_e of the current fruit plus the amount of time it would take to move to the fruit, extend, and grab it
                        if busy_till[r_row, c_col] + j_to_i_travel + TX[i_fruit] + t_grab <= tw_e[c_col]:
                            # mark the fruit as picked, save by which arm, and increase that arm's count of harvested fruits
                            if n_row > 1:
                                fruit_picked_by[r_row][c_col].append(i_fruit+offset_fruit_index)
                            else:
                                fruit_picked_by[c_col].append(i_fruit+offset_fruit_index)

                            self.curr_j[r_row, c_col] += 1
                            # sortedFruit[4, j.fruit_i.real_index] = 1  # save to the real index on sortedFruit
                            sortedFruit[4, i_fruit+offset_fruit_index] = 1  # save to the real index on sortedFruit

                            # if busy_till[r_row, c_col] <= tw_s[c_col]:
                            if busy_till[r_row, c_col] <= tw_s[c_col] - (j_to_i_travel + TX[i_fruit]): 
                                # will have to wait until it harvests the next fruit, but as FCFS it would still be chosen as the fruit it will harvest next
                                # first steps for harvesting can be started before the fruit even reaches the work volume as long as the arm is waiting sp that 
                                # the time of harvest, t_harvest, is at tw_s + grabbing
                                t_harvest = tw_s[c_col] + t_grab
                                # t_harvest = tw_s[c_col] + (j_to_i_travel + TX[i_fruit] + t_grab)

                            else:
                                # does not wait between one fruit and the other, will start whenever the arm finishes harvesting the previous fruit
                                t_harvest = busy_till[r_row, c_col] + j_to_i_travel + TX[i_fruit] + t_grab

                            # print('\narm in col %d and row %d is busy until %0.2f with fruit %d and the next harvest time is %0.2f for fruit %d' %(c_col, r_row, busy_till[r_row, c_col], busy_with[r_row, c_col], t_harvest, i_fruit))
                                
                            # update that the arm will be busy until the pick time plus an amount of time for retraction (assumes a vacuum gripper is being used)
                            if n_row > 1:
                                fruit_picked_at[r_row][c_col].append(t_harvest)
                            else:
                                fruit_picked_at[c_col].append(t_harvest)
                            busy_till[r_row, c_col] = t_harvest + TX[i_fruit] # in s, the time at which the arm will be ready for the next fruit
                            busy_with[r_row, c_col] = i_fruit

                            # print('arm in col %d and row %d is scheduled to harvest fruit %d' %(c_col, r_row, i_fruit))
                            # print('      it will now be busy until %0.2f seconds' %busy_till[r_row,c_col])
                            break # get out of the row loop early because the fruit has been harvested


        no_pick = np.where(sortedFruit[4,:] == 0)  # flag for scheduled == 1, scheduled and picked == 2
    #     print('not picked indexes:', no_pick[0])

        for no_pick_i in no_pick[0]:
            # Adding the indexes of non-picked fruit to a sublist at the end of the first 
            if (no_pick_i >= offset_fruit_index) & (no_pick_i <= job[-1].fruit_i.real_index):
                # horizontal row's list of sublists
                if n_row > 1:
                    # if multiple horizontal rows, append the non-picked sublist to the first horizontal row's list of lists
                    fruit_picked_by[0][n_col].append(no_pick_i)
                else:
                    fruit_picked_by[n_col].append(no_pick_i)

        # print('\nThe list of what arms pick what fruits:')
        # print(fruit_picked_by)
        # print('Unpicked fruit indexes')
        # print(fruit_picked_by[0][n_col])
        # print('\nThe list of when an arm picks the next fruit:')
        # print(fruit_picked_at)
        # print('and the array showing how many fruit were picked by each arm:')  
        # print(self.curr_j)   

        # init cannot return a list, currently being used as sys.exit(0)
        return([fruit_picked_by, fruit_picked_at])
