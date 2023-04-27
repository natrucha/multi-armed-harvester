# import math
import numpy as np

class FCFS(object):
    def __init__(self):
        '''
            First come, first serve (FCFS) implementation that takes a list of jobs, handling times, job time windows, multi-armed robot configuration
            and returns the arm and the order at which the fruits are harvested, and when they are harvested by that arm.
        '''


    def main(self, n_col, n_row, fruit, job, V, Q, d_cell, fruit_travel_matrix, sortedFruit, z_row_bot_edges, z_row_top_edges):
        V_m    = V / 100 # change once to meters
        t_grab = 0.5     # in s, the constant time it takes to harvest a fruit once reached 

        # save the real index of the 0th fruit (to get the difference) when using snapshots
        try:
            # offset_fruit_index = self.job[0].fruit_i.real_index
            offset_fruit_index = job[0].fruit_i.real_index
            # print('job\'s first real index:', offset_fruit_index, '\n')
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

        busy_till       = np.zeros([n_row, n_col])    # until when is this arm busy
        busy_with       = np.zeros([n_row, n_col], dtype=np.int)    # previous chosen fruit for the arm

        # lists of the desired results
        fruit_picked_by = list()                      # list that saves which arm picks which fruit
        fruit_picked_at = list()                      # list that saves at what time an arm picks a fruit, based on fruit_picked_by "topography"
        self.curr_j          = np.zeros([n_row, n_col])    # array to save the sum of fruits picked by each arm
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
            # for now assume 1 row and 1 column (only one arm)

            # calculate the start and end times of the time window during which the fruit is available to be harvested for COLUMN 0
            tw_s0 = (Y[i_fruit] - (Q + (0 + 1)*d_cell)) / V_m
            tw_e0 = (Y[i_fruit] - (Q + (0)*d_cell)) / V_m
            tw_s_last = (Y[i_fruit] - (Q + (n_col + 1)*d_cell)) / V_m  
            tw_e_last = (Y[i_fruit] - (Q + (n_col)*d_cell)) / V_m

            # calculate all time window starts and ends for this fruit
            tw_s = np.arange(tw_s0, tw_s_last, step=-(d_cell)/V_m)
            tw_e = np.arange(tw_e0, tw_e_last, step=-(d_cell)/V_m)
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
                    if Z[i_fruit] > z_row_bot_edges[c_col,r_row] and Z[i_fruit] < z_row_top_edges[c_col,r_row]:

                        j_fruit = busy_with[r_row, c_col]
                        # print('previously scheduled fruit index: %d' %j_fruit)

                        # figure out how to deal with not all fruit being scheduled to fruit index 0?

                        # check if the arm is busy up to tw_e of the current fruit plus the amount of time it would take to move to the fruit, extend, and grab it
                        if busy_till[r_row, c_col] + fruit_travel_matrix[j_fruit+offset_fruit_index, i_fruit+offset_fruit_index] + TX[i_fruit] + t_grab <= tw_e[c_col]:
                            # mark the fruit as picked, save by which arm, and increase that arm's count of harvested fruits
                            fruit_picked_by[r_row][c_col].append(i_fruit+offset_fruit_index)
                            self.curr_j[r_row, c_col] += 1
                            # sortedFruit[4, j.fruit_i.real_index] = 1  # save to the real index on sortedFruit
                            sortedFruit[4, i_fruit+offset_fruit_index] = 1  # save to the real index on sortedFruit

                            if busy_till[r_row, c_col] <= tw_s[c_col]: 
                                # will have to wait until it harvests the next fruit, but as FCFS it would still be chosen as the fruit it will harvest next
                                t_harvest = tw_s[c_col] + fruit_travel_matrix[j_fruit+offset_fruit_index, i_fruit+offset_fruit_index] + TX[i_fruit] + t_grab

                            else:
                                # does not wait between one fruit and the other, will start whenever the last fruit finishes
                                t_harvest = busy_till[r_row, c_col] + fruit_travel_matrix[j_fruit+offset_fruit_index, i_fruit+offset_fruit_index] + TX[i_fruit] + t_grab

                            # print('\narm in col %d and row %d is busy until %0.2f with fruit %d and the next harvest time is %0.2f for fruit %d' %(c_col, r_row, busy_till[r_row, c_col], busy_with[r_row, c_col], t_harvest, i_fruit))
                                
                            # update that the arm will be busy until the pick time plus an amount of time for retraction (assumes a vacuum gripper is being used)
                            fruit_picked_at[r_row][c_col].append(t_harvest)
                            busy_till[r_row, c_col] = t_harvest + TX[i_fruit] # in s, the time at which the arm will be ready for the next fruit
                            busy_with[r_row, c_col] = i_fruit

                            # print('arm in col %d and row %d is scheduled to harvest fruit %d' %(c_col, r_row, i_fruit))
                            # print('      it will now be busy until %0.2f seconds' %busy_till[r_row,c_col])
                            break # get out of the row loop early because the fruit has been harvested


        no_pick = np.where(sortedFruit[4,:] == 0)  # flag for scheduled == 1, scheduled and picked == 2
    #     print('not picked indexes:', no_pick[0])

        for no_pick_i in no_pick[0]:
            # Adding the indexes of non-picked fruit to a sublist at the end of the first 
            # horizontal row's list of sublists
            if n_row > 1:
                # if multiple horizontal rows, append the non-picked sublist to the first horizontal row's list of lists
                fruit_picked_by[0][n_col].append(no_pick_i)
            else:
                fruit_picked_by[n_col].append(no_pick_i)

        # print('\nThe list of what arms pick what fruits:')
        # print(fruit_picked_by)
        # print('\nThe list of when an arm picks the next fruit:')
        # print(fruit_picked_at)
        # print('and the array showing how many fruit were picked by each arm:')  
        # print(self.curr_j)                              

        # init cannot return a list, currently being used as sys.exit(0)
        return([fruit_picked_by, fruit_picked_at])
