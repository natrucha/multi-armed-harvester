## based on https://dynetworkx.readthedocs.io/en/latest/tutorial.html
# import dynetworkx as dnx
# import networkx as nx

import numpy as np
# from numpy.random import PCG64
# use matplotlib to visualize graph?
# see https://www.geeksforgeeks.org/python-visualize-graphs-generated-in-networkx-using-matplotlib/
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D   # plot in 3D

# from datetime import datestime
from datetime import date
from pathlib import Path

######## IMPORT MY OWN MODULES ########
# from trajectory import *          # import the trajectory time calc (bang-bang) 
from plotStates_updated import *  # import module to plot % time each arm is in each state
# from fruit_distribution import *  # import module to create the various desired fruit distributions


class IG_data_analysis(object):
    def __init__(self, snapshot_list, snapshot_cell, print_out):

        '''
            Obtain list of snapshot/camer frame calculated schedule and fruit density and fruit R at each cell
            which are combined and analyzed to provide results.
        '''

        self.print_out = print_out # determines if results are printed out, 1 is yes

        if self.print_out == 1:
            print('--------------------- FINAL RESULTS ---------------------')

        self.schedule_data       = snapshot_list
        self.fruit_per_cell_data = snapshot_cell

        # self.n_col  = 0
        # self.n_row  = 0
        # self.total_arms = 0
        # self.D = 0

        ### values/lists saved as lists for easier access ###
        # will save each snapshot's schedule as a list of lists
        self.fruit_picked_by = list()    # only available if IG_sched function fruitPickedByFunction() called 
        self.fruit_list      = list()    # sorted fruit in slices -> since that's closer to what will actually exist
        self.density         = list()
        self.R_melon         = list()
        self.PCT             = list()
        self.state_time      = list()    # list of arrays with the % of time each arm spent in each state during snapshot
        self.z_bot_bounds    = list()
        self.z_top_bounds    = list()
        
        ### values saved as arrays for easier manipulation/calculations ###
        self.v_vy       = np.zeros(len(self.schedule_data))
        self.q0         = np.zeros(len(self.schedule_data)) # whole run's starting y-coordinate
        self.FPE        = np.zeros(len(self.schedule_data))
        self.FPEavg     = np.zeros(len(self.schedule_data))
        self.FPT        = np.zeros(len(self.schedule_data))
        self.FPTavg     = np.zeros(len(self.schedule_data))
        self.N_snap     = np.zeros(len(self.schedule_data)) # total fruit available in snapshot
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
                self.d_hrz      = snapshot.d_hrz
                self.d_vehicle  = snapshot.d_vehicle
                self.d_cell     = snapshot.d_cell
                self.D          = snapshot.D
                self.plan       = snapshot.d_plan
                self.Td         = snapshot.Td
                self.N          = len(snapshot.sortedFruit[0]) # total number of fruits in the whole run

            # extract scheduling data per snapshot
            self.v_vy[index]         = snapshot.v_vy
            self.FPE[index]          = snapshot.FPE_global
            self.FPEavg[index]       = snapshot.FPEavg
            self.FPT[index]          = snapshot.FPT_global
            self.FPTavg[index]       = snapshot.FPTavg
            self.q0[index]           = snapshot.q_plan[0] # in m, whole run's starting coordinate
            self.N_snap[index]       = snapshot.N_snap
            self.pick_fruit[index]   = np.sum(snapshot.curr_j)

            self.z_bot_bounds.append(snapshot.z_bot_bounds)
            self.z_top_bounds.append(snapshot.z_top_bounds)
            self.PCT.append(snapshot.avg_PCT)
            self.state_time.append(snapshot.state_time)
            self.fruit_picked_by.append(snapshot.fruit_picked_by)
            self.fruit_list.append(snapshot.sortedFruit)

        for snapshot in self.fruit_per_cell_data:
            # extract cell fruit data per snapshot
            self.density.append(snapshot[0])
            self.R_melon.append(snapshot[1])



    def filePath(self, run_n):
        '''Setup for plot path and name based on date and run. Allows plots to be saved for every run.'''
        # date object of today's date
        today = date.today() 
        # print("Current year:", today.year)
        # print("Current month:", today.month)
        # print("Current day:", today.day)
        # print()

        if today.month < 10:
            month = str(0) + str(today.month)
        else:
            month = str(today.month)

        date_today = str(today.year) + month + str(today.day)
        # date_today = '20220815' # if hardcoded is necessary

        self.file_base = './plots/' + date_today + '_run' + str(run_n) + '_' # save the base of the file name



    def fileExists(self, file_name, extension):
        '''Check if file already exists, if so, add a [number] up to [num_checks] so the files are not overwritten'''

        file_base = file_name

        num_checks = 10        # how many times it'll check if the file name exists 
        # add check to see if the file already exists, if so, add a value at the end
        for i_check in range(num_checks):
            # do the check around 10 times, after which just put up an error message that the last file was overwritten
            check_file = Path(file_name + extension)
            # run 10 checks after which just send up an error message 
            if check_file.is_file():
                # file exists so add an extension to the name
                file_name = file_base + '[' + str(i_check) + ']'
                if i_check == num_checks - 1:
                    # no more checks, the first file will be overwritten
                    print('WARNING: Files will be overwritten after this run.')
                    file_name = file_base + 'OVERWRITE'
            else:
                # file doesn't exist so can use this name
                file_full = file_name + extension
                return(file_full)
                # break



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

        axs[4].plot(x, self.N_snap, color='k')
        axs[4].set_ylabel('available fruit')

        axs[5].plot(x, self.R_melon, color='b')
        axs[5].set_ylabel('R_melon [fruit/(m^3 s)]')

        axs[5].set_xlabel('Snapshot No.')

        fig.tight_layout()

        # plt.plot(self.sortedFruit[1][fruit_picked_by[n][k]], 
        #         self.sortedFruit[2][fruit_picked_by[n][k]], linestyle=linestyle, color=line_color, marker='o', label=arm_label)

        file_fct_name = self.file_base + 'over_distance'
        file_name = self.fileExists(file_fct_name, '.png')

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

        file_fct_name = self.file_base + 'state_percent'
        file_name = self.fileExists(file_fct_name, '.pdf')# '.png')

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
            print('Vehicle length: {0:.2f}'.format(self.d_vehicle), 'm, with cell length:', self.d_cell, 'm')
            print('Horizon length:', self.d_hrz, 'm')
            print('Step length between reschedules:', self.D , 'm')
            print()
            print('Total harvested fruit', np.sum(self.pick_fruit), 'out of', self.N)


    def avgFPTandFPE(self):
        '''Takes the various snapshots and combines their FPT and FPE values to get overall average FPT and FPE'''
        avg_FPE = np.average(self.FPEavg)*100
        avg_FPT = np.average(self.FPTavg)
        # calculate the "real FPT" value from individual FPT results
        # sum_FPT = np.sum(self.FPT)
        if self.print_out == 1:
            print()
            print('Based on known pickable fruit by system:')
            print('--------------------------------------------------------')
            print("Average snapshot FPE {0:.2f}".format(avg_FPE), "% ")
            print("Average snapshot FPT {0:.2f}".format(avg_FPT), "fruit/s \n")
            #### NOT WORKING SINCE FIX TO Ts_end
            # print('Sum of FPT values {0:.2f}'.format(sum_FPT), 'fruit/s, and number of times the vehicle length fits in the orchard row:', orchard_veh)
            # print('divide the two to get the REAL FPT: {0:.2f}'.format(sum_FPT/orchard_veh), 'fruit/s')
            # print()

        return([avg_FPE, avg_FPT])


    def runFPEandFPT(self):
        '''
            Takes created fruit distribution matrix and calculates the real overall FPE and FPT 
            assuming that the snapshots will never have the complete picture (newly revealed data, double 
            counting due to the horizon, etc.)
        '''
        # the last snapshot will have the global total FPE and FPT
        run_FPE = self.FPE[-1]*100
        run_FPT = self.FPT[-1]

        if self.print_out == 1:
            print("Run's global FPE value: {0:.2f}".format(run_FPE), "% \nRun's global FPT value: {0:.2f}".format(run_FPT), "fruit/s")
            print('--------------------------------------------------------')

        # return([run_FPE, run_FPT])


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

        array_of_first_populated_list = np.zeros(self.n_col+1)  # flag with values 0, 1, 2 used to determine the first populated list so it can be used to create the legend labels  

        if self.n_row > 1:
            for snapshot_i in snapshot_list:
                # plot the z bounds to see where they end up at each snapshot
                y_bound_array= [self.q0[snapshot_i]+(self.D*snapshot_i), self.q0[snapshot_i]+(self.D*(snapshot_i+1))] # calculate planning window y-bounds
                # print('start and end y_coord of snapshot', snapshot_i)
                # print(y_bound_array)

                for i_row in range(self.n_row):
                    for i_col in range(self.n_col+1):
                        # print('COLUMN NUMBER (WANT IT EQUAL TO 3 AT SOME POINT):', i_col)
                        # print('indexes of the fruits picked in the snapshot_i by the arm in i_row, i_col', self.fruit_picked_by[snapshot_i][i_row][i_col])
                        # print()

                        if self.fruit_picked_by[snapshot_i][i_row][i_col]: # if there are harvested fruits by this arm in this planning window
                            if array_of_first_populated_list[i_col] == 0:
                                # this will be used as the label/legend setup dataset
                                array_of_first_populated_list[i_col] = 1

                            # need to turn everything in the list to ints so they can work as indexes. Just turn it into an array to make life easier
                            index_array = np.int_(self.fruit_picked_by[snapshot_i][i_row][i_col])

                            # obtain the y and z-coordinates
                            y_coord = self.fruit_list[snapshot_i][1][index_array]  # y-coordinates of the fruits indicated by fruits_picked_by in the snapshot_i by the arm in i_row, i_col
                            z_coord = self.fruit_list[snapshot_i][2][index_array]  # z-coordinates of the fruits indicated by fruits_picked_by in the snapshot_i by the arm in i_row, i_col
                            
                            # determine the color and line style so that they can be  identified in the plot
                            if i_col == self.n_col:
                                line_color = 'gold'
                                line_style = ''
                                arm_label = 'unpicked'
                            elif i_col == 0:
                                line_color = str(color[i_col])
                                line_style = line_type[i_row]
                                arm_label = 'back arm'
                            elif i_col == self.n_col-1:
                                line_color = str(color[i_col])
                                line_style = line_type[i_row]
                                arm_label = 'front arm'
                            else:
                                line_color = str(color[i_col])
                                line_style = line_type[i_row]
                                arm_label = 'middle arm ' + str(i_col)

                            # plot the z-bounds for each snapshot
                            if i_col < self.n_col:
                                # i_col == n_col doesn't really exist---it's saving the unpicked fruit---so there is no such thing as z-bounds for it and it returns an index error
                                z_bound_array_bot = np.ones(2) * self.z_bot_bounds[snapshot_i][i_row][i_col]
                                z_bound_array_top = np.ones(2) * self.z_top_bounds[snapshot_i][i_row][i_col]
                                # print('z_bound arrays')
                                # print('bot\n', z_bound_array_bot)
                                # print('top\n', z_bound_array_top)

                            # if snapshot_i == 0 and i_row == 0:
                            if array_of_first_populated_list[i_col] == 1:
                                # limits the labels for the legend
                                # plt.plot(self.sortedFruit[1][fruit_picked_by[n][k]], 
                                #         self.sortedFruit[2][fruit_picked_by[n][k]], linestyle=linestyle, color=line_color, marker='o', label=arm_label)
                                plt.plot(y_coord, z_coord, linestyle=line_style, color=line_color, marker='o', label=arm_label)
                                # plot the z-bounds
                                if i_col < self.n_col:
                                    plt.plot(y_bound_array, z_bound_array_bot, linestyle='--', color=line_color, linewidth=0.5, label='z_bounds')
                                    plt.plot(y_bound_array, z_bound_array_top, linestyle='--', color=line_color, linewidth=0.5)

                                array_of_first_populated_list[i_col] = 2

                            else:
                                # plt.plot(self.sortedFruit[1][fruit_picked_by[n][k]], 
                                #         self.sortedFruit[2][fruit_picked_by[n][k]], marker='o')
                                plt.plot(y_coord, z_coord, linestyle=line_style, color=line_color, marker='o')
                                # plot the z-bounds
                                if i_col < self.n_col:
                                    plt.plot(y_bound_array, z_bound_array_bot, linestyle='--', color=line_color, linewidth=0.5)
                                    plt.plot(y_bound_array, z_bound_array_top, linestyle='--', color=line_color, linewidth=0.5) 

        elif self.n_row == 1:
            for snapshot_i in snapshot_list:
                for i_col in range(self.n_col,-1,-1):
                # for k in range(self.n_col+1):
                    y_coord = self.fruit_list[snapshot_i][1][self.fruit_picked_by[snapshot_i][i_col]]
                    z_coord = self.fruit_list[snapshot_i][2][self.fruit_picked_by[snapshot_i][i_col]]

                    line_style = line_type[2]

                    # add modulo later so it works with n and k > 3 
                    if i_col == self.n_col:
                        line_color = 'gold'
                        line_style = ''
                        arm_label = 'unpicked'
                    elif i_col == 0:
                        line_color = str(color[i_col])
                        # linestyle = line_type[n]
                        arm_label = 'front arm (c=0)'     
                    elif i_col == self.n_col-1:
                        line_color = str(color[i_col])
                        # linestyle = line_type[n]
                        arm_label = 'back arm (c=' + str(i_col) + ')'      
                    else:
                        line_color = str(color[i_col])
                        # linestyle = line_type[n]
                        arm_label = 'c=' + str(i_col)

                    if snapshot_i == 0:
                        # limits the labels for the legend
                        # plt.plot(self.sortedFruit[1][fruit_picked_by[n][k]], 
                        #         self.sortedFruit[2][fruit_picked_by[n][k]], linestyle=linestyle, color=line_color, marker='o', label=arm_label)

                        plt.plot(y_coord, z_coord, linestyle=line_style, color=line_color, marker='o', label=arm_label)

                    else:
                        # plt.plot(self.sortedFruit[1][fruit_picked_by[n][k]], 
                        #         self.sortedFruit[2][fruit_picked_by[n][k]], marker='o')
                        plt.plot(y_coord, z_coord, linestyle=line_style, color=line_color, marker='o')        

        plt.xlabel('Distance along orchard row (m)')
        plt.ylabel('Height from ground (m)')

        legend = ax.legend(bbox_to_anchor=(1.3, 1),loc='upper right')
        fig.tight_layout()
        ax.grid(True)

        file_fct_name = self.file_base + '2dschedule'
        file_name = self.fileExists(file_fct_name, '.png')

        print('Saving 2D plot of the schedule', file_name)
        plt.savefig(file_name,dpi=300)
                        
        plt.show()

    
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



