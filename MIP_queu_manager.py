import math
import numpy as np

class MIP_queu_manager(object):
    def __init__(self, q_vy, q_vy_start, v_vy, l_step_m, fruit_picked_a, fruit_when):
    # def __init__(self, q_vy, q_hy, v_vy, row_n, column_n, l_cell, l_step_m, fruit_picked_a, fruit_when):
        '''
            Works with the mixed integer programming model to make it dynamic. Takes queue created by scheduler and determines which fruits are actually picked 
            based on vehicle, not camera (includes horizonn), locations.

            row_n is the row number, 0th on the bottom
            column_n is the column number, 0th at the back 
            fruit_picked_a queue of fruits this arm a 'will' harvest based on scheduler results, including horizon (would not all be harvested on this run)
            q_vy in m, vehicle location on y-axis. Located at the back of the vehicle
            q_vy_start in m, global start location of the vehicle
            l_step_l in m, the distance the vehicle moves for each snapshot
            v_vy in m/s, vehicle velocity in the y-axis for the snapshot
        '''
        # identify the arm by row and column number
        # self.row_n = row_n
        # self.column_n = column_n

        # identify which fruits the arm picks and when
        queue = np.array(fruit_picked_a)
        when  = np.array(fruit_when)    # linked to fruit_picked by location on the list. VERY IMPORTANT

        # identify the location of important points
        # self.q_ay = q_vy + l_cell*self.column_n # in m, where is the back of this arm's frame on the y-axis
        # self.q_hy = q_hy                        # in m, where is the back of the horizon on the y_axis

        # identify travel distance and velocity
        # self.v_vy = v_vy # in m/s, vehicle velocity in the y-axis
        # self.l_step_m = l_step_m # in m, the distance the vehicle moves per snapshot

        t_move = (l_step_m) / v_vy # in s, the total time of movement for this snapshot
        
        # get two lists: indexes of fruits that were picked in time and indexes of fruits that were not picked in time
        [self.picked_queue, self.unpicked_queue] = self.pickedInTime(t_move, when, queue)



    def pickedInTime(self, t_move, when, queue):
        '''
            Update which fruits have been scheduled and picked based on vehicle start and end location translated to the arm's start and end locations 
            as well as time. Time to travel can determine what the last possible fruit picked will be by the arm if we also give the function at which times 
            the fruits are picked by the arm.
        '''
        # queue can be a single value if only one fruit is harvested, causing index error exceptions. Check if it has a length to avoid that problem 
        if not hasattr(queue, "__len__"):  
            print('There\'s at most one value in the queue')
            # check if the one value was harvested or not
            index_picked     = np.where(when < t_move)
            index_not_picked = np.where(when > t_move)

            picked_queue   = list()
            unpicked_queue = list()

            if len(index_picked[0]) == 1:
                # the one fruit was harvested so unpicked should stay an empty list
                picked_queue.append(queue)

            if len(index_not_picked[0]) == 1:
                # the one fruit was NOT harvested so picked should stay an empty list
                unpicked_queue.append(queue)

        else:
            # print('queue of picked fruits', queue)
            index_picked = np.where(when < t_move)
            # print('comparing when fruits would be harvested\n', when) 
            # print(f'to when they could not be picked anymore: %4.2f' % t_move)
            # print('indexes that can be picked within travel time', index_picked[0])  # print the real indexes based on the given queue of fruis instead
            picked_queue = queue[index_picked]

            index_not_picked = np.where(when > t_move)
            # print('indexes that cannot be picked within travel time', index_not_picked[0]) # print the real indexes based on the given queue of fruis instead
            unpicked_queue = queue[index_not_picked]

        # if len(picked_queue) > 0:
        #     # only print if there are fruits being picked
        #     print('queue of fruits harvested within the limited travel time:\n', picked_queue) 
        # print()

        return([picked_queue, unpicked_queue])



    def updateUnpicked(self, fruit_missed_list):
        '''
            Update the outer list of unpicked fruits with the fruits unpicked due to the time constraints
        '''
        # print('check correct fruit_missed_list was obtained', fruit_missed_list)

        # see https://stackoverflow.com/questions/48776902/numpy-fastest-way-to-insert-value-into-array-such-that-arrays-in-order 
        # for insertion of a value into an array while keeping array sorted
        # searchsorted between arrays https://numpy.org/devdocs/reference/generated/numpy.searchsorted.html

        temp_array = np.array(fruit_missed_list) # array where values will be inserted

        idx = np.searchsorted(temp_array, self.unpicked_queue, side='right')
        # print('the indexes where the values should be inserted:', idx)
        for i in range(len(self.unpicked_queue)):
            new_index = idx[i] + i # add the i because we are concactenating new values into the array and the indexes are sorted so they'll be added after each other
            # print()
            # print('new index',new_index )
            # print('start',temp_array[:new_index])
            # print('new value', self.unpicked_queue[i])
            # print('end', temp_array[new_index:])
            temp_array = np.concatenate((temp_array[:new_index], [self.unpicked_queue[i]], temp_array[new_index:]))
            # print('updated temp', temp_array)

        new_fruit_missed_list = list(temp_array)

        return(new_fruit_missed_list)


    def updateSumPicked(self, queue_picked):
        '''
            Update the array saving the sum of fruits, curr_j, picked by *an* arm. Use provided picked queue to keep up with any changes to the column, row.
            Provided queue can be an array or a list.
        '''
        # print('list of fruits picked by this arm in this row and column', queue_picked)   
        # arm_j = len(queue_picked)

        # print('This arm\'s j value (sum of harvested fruits)', arm_j)
        # print()

        return(len(queue_picked))




        
        
