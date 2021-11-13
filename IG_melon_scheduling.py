## based on https://dynetworkx.readthedocs.io/en/latest/tutorial.html
import dynetworkx as dnx
import networkx as nx

import numpy as np
from numpy.random import PCG64
# use matplotlib to visualize graph?
# see https://www.geeksforgeeks.org/python-visualize-graphs-generated-in-networkx-using-matplotlib/
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D   # plot in 3D

######## IMPORT MY OWN MODULES ########
from trajectory import *          # import the trajectory time calc (bang-bang) 
from plotStates_updated import *  # import module to plot % time each arm is in each state
from fruit_distribution import *  # import module to create the various desired fruit distributions


class IG_melon_scheduling(object):
    def __init__(self, v_vy, Td, v_max, a_max, n_arm, n_cell, cell_l, x_lim, y_lim, z_lim, vehicle_l, travel_l, horizon_l):

        '''
            Interval graph scheduling algorithm for automated orchard fruit picking based on automated 
            melon picking (insert citation) paper. 
        '''
        ### environment constants
        # x_lim         = [0.2, 0.9]
        # self.y_lim    = [0., 10.]
        # z_lim         = [0., 2.7]

        self.x_lim = x_lim
        self.y_lim = y_lim
        self.z_lim = z_lim

        # for the fruit distribution, want to keep it the same for these tests
        # x_seed = PCG64(37428395352013185889194479428694397783)
        # y_seed = PCG64(13250124924871709375127216220749555998)
        # z_seed = PCG64(165440185943501291848242755689690423219)

        ### robot constants
        self.n_arm  = n_arm    # K in melon paper
        self.n_cell = n_cell   # keeping it simple for now
        self.total_arms = self.n_arm*self.n_cell

        self.n_fruit_row = np.zeros([n_cell]) # for now, this is the number of horizonatal rows

        # vehicle speed
        self.v_vy = v_vy   # in m/s 

        # arm settings, also in calcTd function
        self.v_max = v_max
        self.a_max = a_max
        self.d_max = self.a_max

        self.t_grab   = 0.1
        self.t_unload = 0.1   # if we assume arm moves to the side frame to unload, unloading becomes constant

        self.Td = Td          # in s, fruit handling time

        # cell width/height (perpendicular to movement) and length (parallel to movement)
        self.cell_h = (self.z_lim[1] - self.z_lim[0]) / self.n_cell  # w in paper, not true once we know dimensions
        self.cell_l = cell_l                  # length of individual arm cell 

        self.vehicle_l = vehicle_l  # in m, the length (in y) of the vehicle
        self.travel_l  = travel_l   # in m, the distance the vehicle moves in this snapshot
        self.horizon_l = horizon_l  # in m, the length (in y) of the horizon

        self.Ts_end = self.travel_l / self.v_vy # in s, the time when the next snapshot will be taken so nothing can be scheduled after
        # print('Horizon length:', self.horizon_l, 'travel length:', self.travel_l)
        # print('TS_END:', self.Ts_end)

        # arm starting locations
        arm_location = np.zeros([self.n_cell, self.n_arm, 3])
        offset       = 0.2

        arm_location[:,:,0] = 0. # x-coordinate start
        arm_location[:,:,1] = np.linspace(self.y_lim[0], self.n_arm*offset + self.y_lim[0], self.n_arm, endpoint=True)
        # print('ARM Y START', arm_location[:,:,1])

        for n in range(self.n_cell):
            arm_location[n,:,2] = 0. + n*self.cell_h # z-coordinate start
        # print(arm_location)

        # edges for the nth horizontal row of cells
        # row bottom edge = n*self.cell_h
        self.row_bot_edge = np.linspace(0, (self.n_cell*self.cell_h - self.cell_h), self.n_cell, endpoint=True)
        # row_top_edge = row_bot_edge + self.cell_h
        self.row_top_edge = np.copy(self.row_bot_edge) + self.cell_h
        # print('each bottom frame z-val:', row_bot_edge)
        # print('each top frame z-val:', row_top_edge)

        ################# Modulers and Libraries #################
        # Create Fruit distribution
        # fruitD = fruitDistribution(x_lim, self.y_lim, z_lim)
        # # [self.numFruit, self.sortedFruit] = fruitD.column(self.v_vy, self.v_max, self.a_max, self.t_grab, self.n_cell, self.n_arm, self.cell_h, z_seed)
        # [self.numFruit, self.sortedFruit] = fruitD.uniformRandom(density, x_seed, y_seed, z_seed)
        
        # # create an array (or list if it'll need to be dynamic later) for node objects
        # # node_array  = np.ndarray(self.numFruit+self.n_arm, dtype=object)  # self.numFruit+arm_node for the initial dummy nodes for each arm
        # self.node_array  = np.ndarray(self.numFruit+self.total_arms, dtype=object)  # self.numFruit+arm_node for the initial dummy nodes for each arm

        ## Initialize the interval graph
        self.IG = dnx.IntervalGraph()
        ## initialize the ability to calculate trajectory
        self.traj_calc = Trajectory(self.v_max, self.a_max, self.d_max)


    def fixNumFruit(self):
        '''Function needed to remove picked fruit from numFruit due to indexing complications outside of scheduling'''
        #### NOTE: after adding an array row of world frame indexes to sortedFruit, this should change to removing 
        #          horizon double counting 
        actual_available_fruit = np.where(self.sortedFruit[4,:] < 1)

        self.actual_numFruit = len(actual_available_fruit[0])


    def setFruitData(self, numFruit, sortedFruit):
        '''Gives the current total fruit and all the fruit locations, can't run without it'''
        self.numFruit    = numFruit
        self.sortedFruit = sortedFruit

        # create an array (or list if it'll need to be dynamic later) for node objects
        # node_array  = np.ndarray(self.numFruit+self.n_arm, dtype=object)  # self.numFruit+arm_node for the initial dummy nodes for each arm
        self.node_array  = np.ndarray(self.numFruit+self.total_arms, dtype=object)  # self.numFruit+arm_node for the initial dummy nodes for each arm

        # create a variable (actual_numFruit) that has the picked fruit subtracted from it. Used for results 
        # and data analysis -> remove once fruit disappear when picked ;)
        self.fixNumFruit()


    def calcTm(self, traj_calc, start_y, start_z, fruit_y, fruit_z):
        '''
           Calculate Tm (moving time -> move in y,z to next fruit) for node i for arm k.
        '''      
        # calculate move in y-axis
        traj_calc.adjInit(start_y, 0.) # start moving from zero speed
        traj_calc.noJerkProfile(traj_calc.q0, fruit_y, traj_calc.v0, self.v_max, self.a_max, self.d_max)
        
        t_y = traj_calc.Ta + traj_calc.Tv + traj_calc.Td 
        
        # calculate move in z-axis
        traj_calc.adjInit(start_z, 0.) 
        traj_calc.noJerkProfile(traj_calc.q0, fruit_z, traj_calc.v0, self.v_max, self.a_max, self.d_max)
        
        t_z = traj_calc.Ta + traj_calc.Tv + traj_calc.Td 
        
        # calculate which will dominate
        Tm = max(t_y, t_z)
        
        return(Tm)


    def calcTw(self, traj_calc, fruit_x, fruit_z, bottom_z):
        '''
           Calculate handling time -> extension + pick time + retraction + conveyor drop off 
           for node i. Seperated into two values, before picking (Td0) and after picking (Td1) 
        '''      
        # calculate extension (retraction)
        traj_calc.adjInit(0., 0.)      # starts at zero for x each time (extend to fruit)
        traj_calc.noJerkProfile(traj_calc.q0, fruit_x, traj_calc.v0, self.v_max, self.a_max, self.d_max)
        
        t_x = traj_calc.Ta + traj_calc.Tv + traj_calc.Td 
        
        # calculate conveyor drop off
        # traj_calc.adjInit(fruit_z, 0.) # starts at fruit's location (move to conveyor from fruit)
        # traj_calc.noJerkProfile(traj_calc.q0, bottom_z, traj_calc.v0, self.v_max, self.a_max, self.d_max)

        # t_z = traj_calc.Ta + traj_calc.Tv + traj_calc.Td 

        ## if conveyor is to the side, we plan to finish Tw at the back frame where unloading happens 
        # so t_z is "cancelled out"
        t_z = 0.
        
        # add them together to get before picking and after picking
        Tw0 = self.t_grab + t_x
        Tw1 = t_x + t_z
        
        return([Tw0, Tw1])


    def intervalGraph(self, k, ystart, ystop, color='b'):
        '''Interval graph of the edges with given k arm and color.''' 
        # plot the interval
        plt.hlines(k, ystart, ystop, color, lw=4)
        # plots the whiskers/ends
        plt.vlines(ystart, k+0.03, k-0.03, color, lw=2)
        plt.vlines(ystop, k+0.03, k-0.03, color, lw=2)


    def initDummyNodes(self):
        '''initialize dummy 0 nodes for each arm k in each horizontal row'''
        n = -1
        for k in range(self.total_arms): 
            k_pool = k % self.n_arm
            if k_pool == 0:
                n += 1
            # print('n:', n, 'k_pool:', k_pool)
            
            # node array's index will go from k = 0, 1, ... , self.total_arms (all the 0th values for each arm)
            self.node_array[k] = fruitNode(0, 0, k_pool, n, 0)
            self.IG.add_node(self.node_array[k])
            
        # self.IG.nodes(data=True) # show the attributes for each node (maybe not necessary)
        # print('did the dummy node for k=0 get created?', self.IG.has_node(self.node_array[0]))
        # print('k value for dummy node for k=0 (should be = 0):', self.node_array[0].k)

        ### PROCESS THE NODES ###
        # set all fruit as unpicked
        for i in range(self.numFruit):
        #     self.node_array[i+self.n_arm] = fruitNode(i, 0, self.n_arm, 0) # k = self.n_arm which is > n-1 (index) to identify unpicked fruit
            self.node_array[i+self.total_arms] = fruitNode(i, 0, self.n_arm, self.n_cell, 0) # k = self.n_arm which is > n-1 (index) to identify unpicked fruit
            # actual fruit will also be set to non-existent pool (self.n_cell > range(self.n_cell))
        #     self.IG.add_node(self.node_array[i+self.self.n_arm])
            self.IG.add_node(self.node_array[i+self.total_arms])
            
        # print('number of fruit:', self.numFruit)    
        # print('number of nodes after adding all zero nodes for the arms:', len(self.IG.nodes()))


    def initBaseDistIntervals(self):
        '''
            Creating location intervals for each fruit in the cell based on fruit location and handling time Td: when the arm 
            can pick it and how long extension+grab+retract+drop off of fruit will take without running through fruit 
            list for every pool/horizontal row
        '''
        # figure out what fruit get matched to each row based on z-axis location
        self.edge_list = list()

        # calculate how many fruit in each row
        self.n_fruit_row[:] = 0. # zero it out per run

        if self.n_cell > 1:
            for n in range(self.n_cell):
                self.edge_list.append([])

        for index, y_i in enumerate(self.sortedFruit[1]):
            # check that the zi value is available to the nth horizontal row of cells
                
            ## calculate pool/horizontal, n, row value by comparing zi vs cell height 
            n = math.floor(self.sortedFruit[2,index] / self.cell_h)
        #     print(n)

            # fruit could be located higher than the arms can go... 
            # if n < self.n_cell:
            #     self.n_fruit_row[n] += 1
            if n > self.n_cell:
                print('ERROR: fruit higher than available cells')

            self.n_fruit_row[n] += 1
            # print('n_fruit_row')
            # print(self.n_fruit_row)
            # print()

            ## can set fruit node's pool value here:
            self.node_array[index+self.total_arms].n = n

            # calculate the picking location interval for each fruit 
            start_y_i = self.sortedFruit[1,index]
            end_y_i   = start_y_i + self.v_vy * self.Td

            if self.sortedFruit[4,index] < 1:
                self.edge_list.append([start_y_i, end_y_i])


    # def initBaseTimeIntervals(self):
    #     '''
    #         Creating time intervals for each fruit for each arm in the cell based on fruit location: when the arm 
    #         can pick it and how long extension+grab+retract+drop off of fruit will take without running through fruit 
    #         list for every pool/horizontal row
    #     '''
    #     # figure out what fruit get matched to each row based on z-axis location
    #     self.edge_list = list()
    #     k_edges   = list()
    #     self.Tw_values = list() # the same value for every arm, separated into Tw0 (extend+grab) and Tw1 (retract+drop off)

    #     self.n_fruit_row[:] = 0. # zero it out per run

    #     for n in range(self.n_cell):
    #         self.edge_list.append([])
    #     #     Tw_values.append([]) 
            
    #     for index, y_i in enumerate(self.sortedFruit[1]):
    #         # check that the zi value is available to the nth horizontal row of cells
                
    #         ## calculate pool/horizontal, n, row value by comparing zi vs cell height 
    #         n = math.floor(self.sortedFruit[2,index] / self.cell_h)
    #     #     print(n)

    #         # fruit could be located higher than the arms can go... 
    #         # if n < self.n_cell:
    #         #     self.n_fruit_row[n] += 1
    #         if n > self.n_cell:
    #             print('ERROR: fruit higher than available cells')

    #         self.n_fruit_row[n] += 1

    #         ## can set fruit node's pool value here:
    #         self.node_array[index+self.total_arms].n = n

    #         #### CHECK IF THE FRUIT IS IN HORIZON/CAN'T BE PICKED BY THIS ARM ####
            
    #         # calculate y_i / v which is constant for this fruit

    #         # handling time will be constant here, based on the pool's/row's bottom edge
    #         [Tw0, Tw1] = self.calcTw(self.traj_calc, self.sortedFruit[0,index], self.sortedFruit[2,index], self.row_bot_edge[n])
    #         self.Tw_values.append([Tw0, Tw1]) # save values since it will be used to calc picking time later
    #         ## Tw based on fruit i, and it will be based on which horizontal row it fits in (row_bot_edge[n]) but 
    #         #   not saved by row (n)

    #         Tw1 += self.t_unload # if the arm moves to the side frame to unload, unloading is constant

    #         Tw = Tw0 + Tw1

    #         #### NOTE: SHOULD THERE BE AN OFFSET FOR THE SNAPSHOT'S OFFSET STARTING POSITION? -> kinda is one ####
    #         # values of fruit location at the start and end, as well as the handling time
    #         t_start_0 = (y_i - self.y_lim[0]) / self.v_vy - Tw # adding the calculated handling time 
    #         t_end_0   = (y_i - self.y_lim[0]) / self.v_vy      # the end time will be when the back frame is reached by the fruit 

    #         # print('Snapshot ends at', self.Ts_end)
    #         # print('Cell movement limit:',  self.cell_l/self.v_vy, 'and Tw:', Tw)
    #         # print('non-offset start and end times:', t_start_0, 'and', t_end_0)

    #         if self.sortedFruit[4,index] < 1:
    #             # if the schedule would happen before the end of the snapshot, continue (otherwise nothing happens)

    #             k_edges.append(index)

    #             for k in range(self.n_arm):
    #                 # add the offset based on the arm number (assuming back arm is k=0 to front arm k=n_arm)
    #         #         offset = (cell_l*(k+1)) / v  # (k+1) to indicate it's the front frame location we're looking for
    #                 offset = (self.cell_l*k) / self.v_vy  # looking at the back part of the frame 

    #                 ## Saying here that the fruit can only be picked if arm is not busy when the front of the frame reaches t
    #                 #  the fruit

    #                 t_start_k = t_start_0 - offset
    #                 t_end_k   = t_end_0 - offset

    #                 # if k == 0:
    #                 #     print('Backmost arm un-appended Tw start and end times:', t_start_k, t_end_k)

    #                 #### NOTE: check if interval too long versus the amount of time fruit is in cell (t = cell_l/v) 
    #                 if (t_start_k > 0) and (t_end_k > 0) and (Tw < self.cell_l/self.v_vy) and (t_end_k < self.Ts_end):
    #                 # if t_start_k > 0 and t_end_k > 0 and t_end_k - (t_start_k + Tw0) < self.cell_l/self.v_vy:
    #                     # the interval has to be positive or it cannot be used (impossible to pick that fruit)
    #                     k_edges.append([k, t_start_k, t_end_k])
    #                     # if k == 0:
    #                     #     print('Backmost arm is added to the list :)')
    #                     #     print('With Tw start and end times:', t_start_k, t_end_k)
    #                     #     print()
    #                     # print('for fruit located at', y_i, 'arm', k, 'start and end time:', t_start_k, 'and', t_end_k)

    #             if len(k_edges) > 1:
    #         #         print(k_edges)
    #                 self.edge_list[n].append(k_edges.copy()) # if not a copy, values in edge_list also get deleted in next line

    #             # delete values in k_edges
    #             del k_edges[:]

    #     # print(self.edge_list[0]) # prints pool 0 time intervals


    def chooseArm4Fruit(self):
        '''now have to figure out which arm in which row/pool picks what fruit based on previously calculated time intervals'''
        # (b) process nodes in order of increasing y-coord
        # Td = np.zeros(n_arm)
        t  = np.zeros([self.n_cell, self.n_arm, self.numFruit+1]) # t of arm k when *finished picking* (Tm+Td) fruit i

        # save Tm values for state % plot
        self.Tm_values = list()

        fruit_picked_by = list()

        last_i = np.ndarray([self.n_cell, self.n_arm, self.numFruit], dtype=object)      # previously picked fruit's node which provides information 
        self.curr_j = np.ndarray([self.n_cell, self.n_arm], dtype=object) # current j value of the fruit picked by arm k

        for n in range(self.n_cell):
            if self.n_cell > 1:
                self.Tm_values.append([])
                fruit_picked_by.append([])
            
            for k in range(self.n_arm):
                if self.n_cell > 1:
                    self.Tm_values[n].append([])
                else:
                    self.Tm_values.append([])
                fruit_picked_by.append([])
        #         print('should be 0, 1, 2, ...., total arms:', k+(n*3))
                real_index = k + (n*self.n_arm)
                # saves the fruit id of the fruit last picked by the kth arm in pool n
                last_i[n,k,:] = self.node_array[real_index]
                # saves the current number of picked fruit by the kth arm in pool n
                self.curr_j[0,k] = self.node_array[real_index].j

        for i, e in enumerate(self.edge_list):
            # save picking interval start and end locations 
            start_pick = e[0]
            end_pick   = e[1]

            # careful with reference, the return should actually be ((u,v), begin, end)
            # https://dynetworkx.readthedocs.io/en/latest/reference/classes/generated/dynetworkx.IntervalGraph.edges.html#dynetworkx.IntervalGraph.edges              
            n_interval = self.IG.edges(begin=start_pick, end=end_pick) # check if there are edges that lie between the new edges?
            # print('number of intervals that fall within the current calculated edges',len(n_interval))

            min_U_star = [0,0,0,10000]
            k_star     = 0

            if len(n_interval) < self.n_arm:
                # add to the subgraph and calculate time to pick and unpicking edges, U, for every arm
                for k in range(self.n_arm):
                    # while there is only one cell, cell_n = 0
                    #### NOTE: my arms start at the back and are indexed 0
                    t[0,k,i] = max(t[0,k,i-1] + self.Td, (self.sortedFruit[1,i] + k*self.cell_l)/self.v_vy)

                    U_edge_start = e[0]
                    U_edge_end   = self.v_vy * (self.Td + t[0,k,i]) - k*self.cell_h

                    # if i == 3:
                    #     print('for fruit:', i, 'and arm:',k)
                    #     print('Edges to be added:', U_edge_start, U_edge_end)

                    if U_edge_start < U_edge_end:
                        # go through the possible edges and find the one with the leftmost right end of the intervals
                        if min_U_star[3] > U_edge_end:
                            # the current edge is smaller than the previously calculated edge, so new desirable edge
                            min_U_star = [last_i[0,k,i], self.node_array[i+self.total_arms], U_edge_start, U_edge_end]
                            k_star     = k    
                            # print('Arm that picks this fruit is', k_star)                
                            # self.IG.add_edge(last_i[0,k,i], self.node_array[i+self.total_arms], U_edge_start, U_edge_end)

                    # update all the saved prev node data
                    last_i[0,k,i] = self.node_array[i+self.total_arms]
                
                # add the interval with the leftmost right end of the intervals
                self.IG.add_edge(min_U_star[0], min_U_star[1], min_U_star[2], min_U_star[3])

                # update the node
                self.node_array[i+self.total_arms].t = t[0,k,i]
                self.node_array[i+self.total_arms].j = self.curr_j[0,k_star] + 1
                self.node_array[i+self.total_arms].k = k_star

                # finish updating all the saved prev node data
                self.curr_j[0,k_star] = self.node_array[i+self.total_arms].j # maybe don't need to save this array, just use last_i?
                # print()

                # save the Tm value for state plot
                # self.Tm_values[n][k].append(Tm)

                # save that this fruit was picked by this arm
                fruit_picked_by[k_star].append(i)

        # print(self.curr_j)
        print(fruit_picked_by)
        return(fruit_picked_by)

        
       


    def calcResults(self):
    # def calcResults(self, total_distance):
        '''Calculate and print basic results like total picked fruit, FPE, FPT, etc.'''
        # total_distance = self.y_lim[1] - self.y_lim[0]
        total_distance = self.travel_l

        try:
            self.FPE = np.sum(self.curr_j)/self.actual_numFruit
        except ZeroDivisionError:
            self.FPE = 0

        self.FPT = np.sum(self.curr_j) / (total_distance / self.v_vy)

        self.FPE_row = list()
        self.FPT_row = list()

        # print('Total number of fruit:', self.actual_numFruit)
        # print('Total time:', total_distance / self.v_vy, 'sec')
        # print('Vehicle velocity:', self.v_vy, 'm/s')
        # print('Number of rows:', self.n_cell)
        # print('Number of arms in row:', self.n_arm)
        # print()
        # print('Total harvested fruit:', np.sum(self.curr_j))
        # print('FPE = fruit picked / total fruit:          ', self.FPE)
        # print('FPT = total fruit / total vehicle run time:', self.FPT, 'fruit/sec')
        # print()

        for n in range(self.n_cell):
            arm_string = ''
            # calculate the row's FPE and FPT
            self.FPT_row.append(np.sum(self.curr_j[n,:]) / (total_distance / self.v_vy))
            self.FPE_row.append(np.sum(self.curr_j[n,:]) / self.n_fruit_row[n])

            for k in range(self.n_arm):
                arm_string += ' arm ' + str(k) + ': ' + str(self.curr_j[n,k]) + ' '
                
                
            # print('Number of fruit picked by row/pool', n, arm_string)

            # print('Row FPE', self.FPE_row[n], 'and FPT', self.FPT_row[n], 'fruit/sec')
            # print()
        

    def fruitPickedBy(self, numFruit):
        '''Separates fruit index by which arm is set to pick that fruit'''
        self.IG.edges()

        self.fruit_picked_by = list()
        for n in range(self.n_cell):
            if self.n_cell > 1:
                self.fruit_picked_by.append([])
            for k in range(self.n_arm+1):
                self.fruit_picked_by.append([])
                # self.fruit_picked_by[n].append([])

        for i in range(self.total_arms, self.numFruit+self.total_arms):
        #     print('row number:', node_array[i].n, 'arm number:', node_array[i].k)
            self.fruit_picked_by[self.node_array[i].k].append(self.node_array[i].i)
            # self.fruit_picked_by[self.node_array[i].n][self.node_array[i].k].append(self.node_array[i].i)

        # print(fruit_picked_by)
        return(self.fruit_picked_by)


    def calcPCT(self, fruit_picked_by):
        '''Calculate average PCT for each arm for a run -- snapshot or total'''
        # matrix to save each arms's PCT
        self.avg_PCT = np.zeros([self.n_cell, self.n_arm])

        # PCT is composed of move_yz, extend, grab, retract, move_z -> remove all idle time
        # this is the sum of T_m and T_w values divided by total fruit picked
        for n in range(self.n_cell):
            for k in range(self.n_arm):
                ##### MELON #####

                # for tm in self.Tm_values[n][k]:
                #     # add in move_yz 
                #     self.avg_PCT[n,k] += self.Td

                # for i in fruit_picked_by[n][k]:
                #     # Tw was saved for every possible fruit for each possible arm. Only use the ones used. 
                #     self.avg_PCT[n,k] +=  self.Tw_values[i][0] + self.Tw_values[i][1] # add in retract and unload

                # divide the calculated total time by number of fruit picked to get sec/fruit
                self.avg_PCT[n,k] = self.Td
        
        # print('Average PCT for this snapshot [sec/fruit]')
        # print(self.avg_PCT)
        # print()


    def calculateStateTime(self, fruit_picked_by):
    # def calculateStateTimePercent(self, fruit_picked_by, total_distance):
        '''Calculates the time each arm is in each state so that it can plotState can plot the data'''
        # total_distance = self.y_lim[1] - self.y_lim[0]
        total_distance = self.travel_l

        total_time = total_distance / self.v_vy  # for snapshots? -> I'm deleting Tm and Tw data at each snapshot, problem

        ## states: idle, pick_yz, pick_x, grab, retract_x, move_z/unload
        # self.state_percent = np.zeros([self.total_arms, 6]) # save each arm's percent time in each of the six states 
        self.state_time = np.zeros([self.total_arms, 7]) # save each arm's percent time in each of the six states 

        for n in range(self.n_cell):
            for k in range(self.n_arm):
                tot_arm_index = k + (n*self.n_arm)
                # calculate arm's move_yz using Tm
                for tm in self.Tm_values[n][k]:
                    self.state_time[tot_arm_index,1] += tm

                for i in fruit_picked_by[n][k]:  
        #             print(Tw_values[i])

                    # calculate extend from Tw0 and final j for each arm k       
                    move_x = self.Tw_values[i][0] - self.t_grab
                    self.state_time[tot_arm_index,2] += move_x

                    # calculate grab from Tw and final j for each arm k
                    self.state_time[tot_arm_index,3] += self.t_grab

                    # calculate unload from Tw1 and final j for each arm k
                    # self.state_time[tot_arm_index,5] += self.Tw_values[i][1] - move_x
                    self.state_time[tot_arm_index,5] += self.t_unload

                # since this is ideal, retract == extend
                self.state_time[tot_arm_index,4] = self.state_time[tot_arm_index,2]

                # calculate idle by subtracting all before by total time: length_row / v
                self.state_time[tot_arm_index,0] = total_time - np.sum(self.state_time[tot_arm_index,:])
                # save the total time for this run to get total percent later
                self.state_time[tot_arm_index,6] = total_time
                

                # self.state_time[tot_arm_index,:] = (self.state_time[tot_arm_index,:] / total_time) * 100

        #         print('For arm', tot_arm_index,)
        #         print('idle:', state_percent[tot_arm_index,0], 'pick_yz:', state_percent[tot_arm_index,1], 
        #               'pick_x', state_percent[tot_arm_index,2], 'grab:', state_percent[tot_arm_index,3], 'retract_x:', 
        #               state_percent[tot_arm_index,4], 'unload:', state_percent[tot_arm_index,5])

                # does each k's state % add up to 100%?
            # percent_sum = np.sum(self.state_time[tot_arm_index,:])
            # print('Sum total of all times', percent_sum)
            # print()

        # state_percent_transpose = self.state_percent
        # state_percent_list = state_percent_transpose.tolist()
        # # print(state_percent_list)
        # plot_states = plotStates(state_percent_list)

    
## Interval graph node setup
class fruitNode:
    def __init__(self, i, j, k, n, t): 
        # ith fruit
        self.i  = i
        # jth fruit picked by this arm
        self.j  = j
        # arm for which edges are being calculated
        self.k  = k
        # arm's pool/horizontal row
        self.n  = n
        # jth fruit i that the arm k has picked up, denoted t^{k}_{i(j)}, previous fruit 
        # would be t^{k}_{i(j-1)}
        # time at which kth arm reaches the jth fruit it has picked up
        self.t  = t 

