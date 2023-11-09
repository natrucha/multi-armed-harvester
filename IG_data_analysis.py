## based on https://dynetworkx.readthedocs.io/en/latest/tutorial.html
# import dynetworkx as dnx
# import networkx as nx

import numpy as np
# from numpy.random import PCG64
# use matplotlib to visualize graph?
# see https://www.geeksforgeeks.org/python-visualize-graphs-generated-in-networkx-using-matplotlib/
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D   # plot in 3D

######## IMPORT MY OWN MODULES ########
# from trajectory import *          # import the trajectory time calc (bang-bang) 
from plotStates_updated import *  # import module to plot % time each arm is in each state
# from fruit_distribution import *  # import module to create the various desired fruit distributions


class IG_data_analysis(object):
    def __init__(self, snapshot_list, snapshot_cell, step_l, y_lim, algorithm, print_out):

        '''
            Obtain list of snapshot/camer frame calculated schedule and fruit density and fruit R at each cell
            which are combined and analyzed to provide results.
        '''

        self.print_out = print_out # determines if results are printed out, 1 is yes

        if self.print_out == 1:
            print('--------------------- FINAL RESULTS ---------------------')
        
        self.algorithm = 0 # default to not using the melon algorithm

        if algorithm == 1:
            # if it is based on the melon algorithm
            self.algorithm = 1
        else:
            # who knows, default to not melon algorithm
            print('Defaulting to not using the melon algorithm')

        self.schedule_data       = snapshot_list
        self.fruit_per_cell_data = snapshot_cell

        self.n_col  = 0
        self.n_row  = 0
        self.total_arms = 0

        self.step_l = step_l

        self.y_lim = y_lim # in m, needed to calculate real FPT from the individual snapshot FPT results  

        ### values/lists saved as lists for easier access ###
        # will save each snapshot's schedule as a list of lists
        self.fruit_picked_by = list()    # only available if IG_sched function fruitPickedByFunction() called 
        self.fruit_list      = list()    # sorted fruit in slices -> since that's closer to what will actually exist
        self.density         = list()
        self.R               = list()
        self.PCT             = list()
        self.state_time      = list()    # list of arrays with the % of time each arm spent in each state during snapshot
        
        ### values saved as arrays for easier manipulation/calculations ###
        self.v_vy       = np.zeros(len(self.schedule_data))
        self.y0         = np.zeros(len(self.schedule_data))  # snapshot's y 0th coordinate
        self.FPE        = np.zeros(len(self.schedule_data))
        self.FPEavg     = np.zeros(len(self.schedule_data))
        self.FPT        = np.zeros(len(self.schedule_data))
        self.FPTavg     = np.zeros(len(self.schedule_data))
        self.tot_fruit  = np.zeros(len(self.schedule_data)) # total fruit available in snapshot
        self.pick_fruit = np.zeros(len(self.schedule_data))

        # extract individual pieces of information from the master lists
        self.extractData()


    def extractData(self):
        '''Extracts relevant values from the schedule_data and fruit_per_cell_data lists'''
        # for snapshot in self.schedule_data:
        for index, snapshot in enumerate(self.schedule_data):
            if index == 0:
                # obtain constant values of the whole run
                self.n_col      = snapshot.n_col
                self.n_row      = snapshot.n_row
                self.total_arms = self.n_col * self.n_row
                self.horizon_l  = snapshot.horizon_l
                self.vehicle_l  = snapshot.vehicle_l
                self.cell_l     = snapshot.cell_l
                self.Td         = snapshot.Td

            # extract scheduling data per snapshot
            self.v_vy[index]       = snapshot.v_vy
            self.FPE[index]        = snapshot.FPE
            self.FPEavg[index]     = snapshot.FPEavg
            self.FPT[index]        = snapshot.FPT
            self.FPTavg[index]     = snapshot.FPTavg
            self.y0[index]         = snapshot.y_lim[0]
            self.tot_fruit[index]  = snapshot.actual_numFruit
            self.pick_fruit[index] = np.sum(snapshot.curr_j)

            self.PCT.append(snapshot.avg_PCT)
            self.state_time.append(snapshot.state_time)
            self.fruit_picked_by.append(snapshot.fruit_picked_by)
            self.fruit_list.append(snapshot.sortedFruit)

        for snapshot in self.fruit_per_cell_data:
            # extract cell fruit data per snapshot
            self.density.append(snapshot[0])
            self.R.append(snapshot[1])


    def plotValuesOverDistance(self):
        '''Plot all the relevant values as vehicle moves'''
        # fig, ax = plt.subplots()
        x = np.arange(len(self.schedule_data))  # snapshot number, can change later

        fig, axs = plt.subplots(6, 1)
        # want subplots to stack in columns so they can be compared, see
        # see https://matplotlib.org/stable/gallery/lines_bars_and_markers/cohere.html#sphx-glr-gallery-lines-bars-and-markers-cohere-py
        axs[0].plot(x, self.FPE*100, color='r')
        # axs[0].set_xlabel('time')
        axs[0].set_ylabel('FPE [%]')
        # axs[0].grid(True)

        axs[1].plot(x, self.FPT, color='c')
        axs[1].set_ylabel('FPT [fruit/s]')

        x_pct = list()

        for n in range(self.n_row):
            for k in range(self.n_col):
                y = list()

                for i in range(len(self.schedule_data)):
                    y.append(self.PCT[i][n][k])

                axs[2].plot(x, y)

        axs[2].set_ylabel('PCT [s/fruit]')

        axs[3].plot(x, self.v_vy, color='g')
        axs[3].set_ylabel('vehicle velocity [m/s]')

        axs[4].plot(x, self.tot_fruit, color='k')
        axs[4].set_ylabel('available fruit')

        axs[5].plot(x, self.R, color='b')
        axs[5].set_ylabel('R [fruit/(m^3 s)]')

        axs[5].set_xlabel('Snapshot No.')

        fig.tight_layout()

        # plt.plot(self.sortedFruit[1][fruit_picked_by[n][k]], 
        #         self.sortedFruit[2][fruit_picked_by[n][k]], linestyle=linestyle, color=line_color, marker='o', label=arm_label)

        file_name = './plots/test0.png'
        print('Saving plot of FPT and FPE vs snapshot number in', file_name)
        plt.savefig(file_name,dpi=300)
        # plt.show()


    def plotTotalStatePercent(self):
        '''Takes the average percent time each arm spent in each of the six states'''
        self.state_percent = np.zeros([self.total_arms, 7])
            
        for snapshot_percent in self.state_time:
            self.state_percent = self.state_percent + snapshot_percent
            # print('snapshot state percent:', self.state_percent)
        # print('snapshot state times:')
        # print(self.state_percent)
        
        # get percentage by dividing over total time
        self.state_percent = self.state_percent / self.state_percent[0,6] * 100
        # print('*not* snapshot state perce nt:', self.state_percent)
        # print('Overall average percent amount of time each arm spent in each state:')
        # print(self.state_percent)
        # print()

        file_name = './plots/state_percent.png'
        print('Saving plot of the mean state percent of each arm in', file_name)

        # Create and save the plot
        state_percent_list = self.state_percent.tolist()
        # print(state_percent_list)
        plot_states = plotStates(state_percent_list, file_name, self.n_col, self.n_row)


    def printSettings(self):
        '''Prints out constant robot settings such as number of arms, etc.'''
        if self.print_out == 1:
            print('Settings for these results:')
            print('Number of rows', self.n_row, 'number of arms:', self.n_col)
            print('Fruit handling time:', self.Td, 'sec')    
            print()
            print('Vehicle length: {0:.2f}'.format(self.vehicle_l), 'm, with cell length:', self.cell_l, 'm')
            print('Horizon length:', self.horizon_l, 'm')
            print('Step length:', self.step_l , 'm')
            print()


    def avgFPTandFPE(self):
        '''Takes the various snapshots and combines their FPT and FPE values to get overall average FPT and FPE'''
        avg_FPE = np.average(self.FPE)*100
        avg_FPT = np.average(self.FPT)
        # calculate the "real FPT" value from individual FPT results
        # sum_FPT = np.sum(self.FPT)
        # orchard_veh = (self.y_lim[1] - self.y_lim[0]) / self.vehicle_l  # number of vehicle lengths that fit in orchard row
        if self.print_out == 1:
            print('Total Picked Fruit', np.sum(self.pick_fruit), 'out of', np.sum(self.tot_fruit))
            # print('Does not delete potential doubling between snapshot (realism)')
            print()

            print('Based on known pickable fruit by system:')
            print("Average final FPE {0:.2f}".format(avg_FPE), "%")
            print("Average final FPT {0:.2f}".format(avg_FPT), "fruit/s")
            print('--------------------------------------------------------')
            print()
            #### NOT WORKING SINCE FIX TO Ts_end
            # print('Sum of FPT values {0:.2f}'.format(sum_FPT), 'fruit/s, and number of times the vehicle length fits in the orchard row:', orchard_veh)
            # print('divide the two to get the REAL FPT: {0:.2f}'.format(sum_FPT/orchard_veh), 'fruit/s')
            # print()

        return([avg_FPE, avg_FPT])


    def realFPEandFPT(self, real_sortedFruit, y_lim, v_vy):
        '''
            Takes created fruit distribution matrix and calculates the real overall FPE and FPT 
            assuming that the snapshots will never have the complete picture (newly revealed data, double 
            counting due to the horizon, etc.)
        '''
        total_fruit = len(real_sortedFruit[4,:])
        total_time  = (y_lim[1] - y_lim[0]) / v_vy

        picked_index = np.where(real_sortedFruit[4,:] > 1)  # scheduled *and* harvested is flagged as 2, 1 means it was only scheduled

        real_FPE = len(picked_index[0]) / total_fruit * 100
        real_FPT = len(picked_index[0]) / total_time

        if self.print_out == 1:
            print('--------------------------------------------------------')
            print('Real total fruit:', total_fruit)
            print('Real total fruit picked:', len(picked_index[0]))
            print("Real overall FPE: {0:.2f}".format(real_FPE), "%, and FPT: {0:.2f}".format(real_FPT), "fruit/s")
            print('--------------------------------------------------------')

        return([real_FPE, real_FPT])


    def avgPCT(self):
        '''Calculates each arm's average PCT over all the snapshots'''
        #### FIGURE OUT HOW TO DEAL WITH NAN ####
        sum_PCT = np.zeros([self.n_row, self.n_col])

        for snapshot_PCT in self.PCT:
            sum_PCT = np.nansum(np.dstack((sum_PCT,snapshot_PCT)),2)

        avg_PCT = sum_PCT / len(self.schedule_data)

        if self.print_out == 1:
            print('Average PCT for each arm over the whole run:')
            print('**Lower than it should be, nan values are converted to 0 which lowers the average')
            print(avg_PCT)
            # print()
            # print('Sum of PCT matrices')
            # print(sum_PCT)


    def plot2DSchedule(self, snapshot_list):
        '''Plot the path of the schedule in 2D based on y and z axes based on the list of *desired* snapshots'''

        fig, ax = plt.subplots()

        # see https://matplotlib.org/stable/gallery/lines_bars_and_markers/linestyles.html
        # https://matplotlib.org/stable/tutorials/colors/colors.html
        # https://thispointer.com/matplotlib-line-plot-with-markers/

        # used to indicate horizontal row
        line_type = ['--', '-.', '-', (0, (1, 1)), (0, (3, 1, 1, 1, 1, 1)), (0, (5, 10))]
        # used to indicate arm number, see https://matplotlib.org/stable/gallery/color/named_colors.html
        color     = ['blue', 'red', 'purple', 'chartreuse', 'black', 'aqua', 'pink', 'sienna', 'deepskyblue', 'teal', 'tomato', 'slategrey']

        if self.n_row > 1:
            for snapshot_i in snapshot_list:
                for n in range(self.n_row):
                    for k in range(self.n_col+1):
                        try:
                            # some snapshots will have empty lists which lead to an IndexError
                            x = self.fruit_list[snapshot_i][1][self.fruit_picked_by[snapshot_i][n][k]]
                            y = self.fruit_list[snapshot_i][2][self.fruit_picked_by[snapshot_i][n][k]]

                                # add modulo later so it works with n and k > 3 
                            if k == self.n_col:
                                line_color = 'gold'
                                linestyle = ''
                                arm_label = 'unpicked'
                            elif k == 0:
                                line_color = str(color[k])
                                linestyle = line_type[n]
                                arm_label = 'back arm'
                            elif k == self.n_col-1:
                                line_color = str(color[k])
                                linestyle = line_type[n]
                                arm_label = 'front arm'
                            else:
                                line_color = str(color[k])
                                linestyle = line_type[n]
                                arm_label = 'middle arm ' + str(k)

                            if snapshot_i == 0 and n == 0:
                                # limits the labels for the legend
                                # plt.plot(self.sortedFruit[1][fruit_picked_by[n][k]], 
                                #         self.sortedFruit[2][fruit_picked_by[n][k]], linestyle=linestyle, color=line_color, marker='o', label=arm_label)

                                plt.plot(x, y, linestyle=linestyle, color=line_color, marker='o', label=arm_label)

                            else:
                                # plt.plot(self.sortedFruit[1][fruit_picked_by[n][k]], 
                                #         self.sortedFruit[2][fruit_picked_by[n][k]], marker='o')
                                plt.plot(x, y, linestyle=linestyle, color=line_color, marker='o')
                        
                        except IndexError:
                            print('\nWARNING: current row', n, 'and column', k, 'in snapshot', snapshot_i, 'has an empty list and has to be skipped')
                            # return(0)

                        

        elif self.n_row == 1 and self.algorithm == 1:  # if using the melon algorithm
            for snapshot_i in snapshot_list:
                for k in range(self.n_col,-1,-1):
                # for k in range(self.n_col+1):
                    x = self.fruit_list[snapshot_i][1][self.fruit_picked_by[snapshot_i][k]]
                    y = self.fruit_list[snapshot_i][2][self.fruit_picked_by[snapshot_i][k]]

                    linestyle = line_type[2]

                    # add modulo later so it works with n and k > 3 
                    if k == self.n_col:
                        line_color = 'gold'
                        linestyle = ''
                        arm_label = 'unpicked'
                    elif k == 0:
                        line_color = str(color[k])
                        # linestyle = line_type[n]
                        arm_label = 'front arm (k=0)'     
                    elif k == self.n_col-1:
                        line_color = str(color[k])
                        # linestyle = line_type[n]
                        arm_label = 'back arm (k=' + str(k) + ')'      
                    else:
                        line_color = str(color[k])
                        # linestyle = line_type[n]
                        arm_label = 'k=' + str(k)

                    if snapshot_i == 0:
                        # limits the labels for the legend
                        # plt.plot(self.sortedFruit[1][fruit_picked_by[n][k]], 
                        #         self.sortedFruit[2][fruit_picked_by[n][k]], linestyle=linestyle, color=line_color, marker='o', label=arm_label)

                        plt.plot(x, y, linestyle=linestyle, color=line_color, marker='o', label=arm_label)

                    else:
                        # plt.plot(self.sortedFruit[1][fruit_picked_by[n][k]], 
                        #         self.sortedFruit[2][fruit_picked_by[n][k]], marker='o')
                        plt.plot(x, y, linestyle=linestyle, color=line_color, marker='o')


        plt.xlabel('Distance along orchard row (m)')
        plt.ylabel('Height from ground (m)')

        legend = ax.legend(bbox_to_anchor=(1.1, 1),loc='upper right')
        ax.grid(True)

        file_name = './plots/2dschedule.png'

        print('Saving 2D plot of the schedule', file_name)
        plt.savefig(file_name,dpi=300)
                        
        # plt.show()

    
    # def plot3DSchedule(self, fruit_picked_by): 
    #     '''Plot the path of the schedule in 3D alongside the fruit'''
    #     fig = plt.figure()
    #     ax = plt.axes(projection ='3d')

    #     line_type = ['--', '-.', '-', '.']
    #     color     = ['c', 'r', 'b', 'g']

    #     for n in range(self.n_row):
    #         for k in range(self.n_col+1):
    #             # add modulo later so it works with n and k > 3 
    #             if k == self.n_col:
    #                 line = 'oy'
    #                 arm_label = 'row ' + str(n) + ', unpicked'
    #             elif k == 0:
    #                 line = 'o' + line_type[n] + color[k]
    #                 arm_label = 'row ' + str(n) + ', back arm'
    #             elif k == n_col-1:
    #                 line = 'o' + line_type[n] + color[k]
    #                 arm_label = 'row ' + str(n) + ', front arm'
    #             else:
    #                 line = 'o' + line_type[n] + color[k]
    #                 arm_label = 'row ' + str(n) + ', middle arm ' + str(k)
                    
    #             plt.plot(self.sortedFruit[1][fruit_picked_by[n][k]], 
    #                     self.sortedFruit[0][fruit_picked_by[n][k]],
    #                     self.sortedFruit[2][fruit_picked_by[n][k]], line, label=arm_label)

    #     ax.set_xlabel('Distance along orchard row (m)')
    #     ax.set_ylabel('Distance into tree crown (m)')
    #     ax.set_zlabel('Height from ground (m)')

    #     plt.show()



