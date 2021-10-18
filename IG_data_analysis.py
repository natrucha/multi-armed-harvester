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
    def __init__(self, snapshot_list, snapshot_cell):

        '''
            Obtain list of snapshot/camer frame calculated schedule and fruit density and fruit R at each cell
            which are combined and analyzed to provide results.
        '''

        print('-------------------FINAL RESULTS-------------------')

        self.schedule_data       = snapshot_list
        self.fruit_per_cell_data = snapshot_cell

        self.n_arm  = 0
        self.n_cell = 0
        self.total_arms = 0

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
        self.FPT        = np.zeros(len(self.schedule_data))
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
                self.n_arm  = snapshot.n_arm
                self.n_cell = snapshot.n_cell
                self.total_arms = self.n_arm * self.n_cell

            # extract scheduling data per snapshot
            self.v_vy[index]       = snapshot.v
            self.FPE[index]        = snapshot.FPE
            self.FPT[index]        = snapshot.FPT
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
        fig, ax = plt.subplots()

        x = np.arange(len(self.schedule_data))  # snapshot number, can change later

        fig, axs = plt.subplots(5, 1)
        # want subplots to stack in columns so they can be compared, see
        # see https://matplotlib.org/stable/gallery/lines_bars_and_markers/cohere.html#sphx-glr-gallery-lines-bars-and-markers-cohere-py
        axs[0].plot(x, self.FPE*100, color='r')
        # axs[0].set_xlabel('time')
        axs[0].set_ylabel('FPE [%]')
        # axs[0].grid(True)

        axs[1].plot(x, self.FPT, color='c')
        axs[1].set_ylabel('FPT [fruit/s]')

        x_pct = list()

        for n in range(self.n_cell):
            for k in range(self.n_arm):
                y = list()

                for i in range(len(self.schedule_data)):
                    y.append(self.PCT[i][n][k])

                axs[2].plot(x, y)

        axs[2].set_ylabel('PCT [s/fruit]')

        axs[3].plot(x, self.v_vy, color='g')
        axs[3].set_ylabel('vehicle velocity [m/s]')

        axs[4].plot(x, self.tot_fruit, color='k')
        axs[4].set_ylabel('available fruit')

        axs[4].set_xlabel('Snapshot No.')

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
        # print('snapshot state times:')
        # print(self.state_percent)
        
        # get percentage by dividing over total time
        self.state_percent = self.state_percent / self.state_percent[0,6] * 100
        # print('Overall average percent amount of time each arm spent in each state:')
        # print(self.state_percent)
        # print()

        file_name = './plots/state_percent.png'
        print('Saving plot of the mean state percent of each arm in', file_name)

        # Create and save the plot
        state_percent_list = self.state_percent.tolist()
        # print(state_percent_list)
        plot_states = plotStates(state_percent_list, file_name)


    def avgFPTandFPE(self):
        '''Takes the various snapshots and combines their fPT and FPE values to get overall average FPT and FPE'''
        avg_FPE = np.average(self.FPE)
        avg_FPT = np.average(self.FPT)

        print('Total Picked Fruit', np.sum(self.pick_fruit), 'out of', np.sum(self.tot_fruit))
        print()

        print('Based on known pickable fruit by system:')
        print("Average final FPT {0:.2f}".format(avg_FPT), "fruit/s")
        print("Average final FPE {0:.2f}".format(avg_FPE*100), "%")


    def avgPCT(self):
        '''Calculates each arm's average PCT over all the snapshots'''
        #### FIGURE OUT HOW TO DEAL WITH NAN ####
        avg_PCT = np.zeros([self.n_cell, self.n_arm])

        for snapshot_PCT in self.PCT:
            avg_PCT = avg_PCT + snapshot_PCT

        avg_PCT = avg_PCT / len(self.schedule_data)
        print('Average PCT for each arm over the whole run:')
        print(avg_PCT)


    def plot2DSchedule(self, snapshot_list):
        '''Plot the path of the schedule in 2D based on y and z axes based on the list of desired snapshots'''

        fig, ax = plt.subplots()

        # see https://matplotlib.org/stable/gallery/lines_bars_and_markers/linestyles.html
        # https://matplotlib.org/stable/tutorials/colors/colors.html
        # https://thispointer.com/matplotlib-line-plot-with-markers/

        # used to indicate horizontal row
        line_type = ['--', '-.', '-', (0, (1, 1)), (0, (3, 1, 1, 1, 1, 1)), (0, (5, 10))]
        # used to indicate arm number
        color     = ['blue', 'red', 'purple', 'chartreuse', 'black', 'aqua', 'pink']

        for snapshot_i in snapshot_list:
            for n in range(self.n_cell):
                for k in range(self.n_arm+1):
                    x = self.fruit_list[snapshot_i][1][self.fruit_picked_by[snapshot_i][n][k]]
                    y = self.fruit_list[snapshot_i][2][self.fruit_picked_by[snapshot_i][n][k]]

                    # add modulo later so it works with n and k > 3 
                    if k == self.n_arm:
                        line_color = 'gold'
                        linestyle = ''
                        arm_label = 'unpicked'
                    elif k == 0:
                        line_color = str(color[k])
                        linestyle = line_type[n]
                        arm_label = 'back arm'
                    elif k == self.n_arm-1:
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

    #     for n in range(self.n_cell):
    #         for k in range(self.n_arm+1):
    #             # add modulo later so it works with n and k > 3 
    #             if k == self.n_arm:
    #                 line = 'oy'
    #                 arm_label = 'row ' + str(n) + ', unpicked'
    #             elif k == 0:
    #                 line = 'o' + line_type[n] + color[k]
    #                 arm_label = 'row ' + str(n) + ', back arm'
    #             elif k == n_arm-1:
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



