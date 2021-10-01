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
        
        self.v_vy    = np.zeros(len(self.schedule_data))
        self.y0      = np.zeros(len(self.schedule_data))  # snapshot's y 0th coordinate
        self.FPE     = np.zeros(len(self.schedule_data))
        self.FPT     = np.zeros(len(self.schedule_data))
        self.density = np.zeros(len(self.schedule_data))
        self.R       = np.zeros(len(self.schedule_data))
        self.PCT     = np.zeros(len(self.schedule_data))

        # extract individual pieces of information from the master lists
        self.extractData()


    def extractData(self):
        '''Extracts relevant values from the schedule_data and fruit_per_cell_data lists'''
        # for snapshot in self.schedule_data:
        for index, snapshot in enumerate(self.schedule_data):
            # extract scheduling data per snapshot
            self.v_vy[index]    = snapshot.v
            self.FPE[index]     = snapshot.FPE
            self.FPT[index]     = snapshot.FPT
            # self.PCT[index]     = snapshot.avg_PCT
            self.y0[index]      = snapshot.y_lim[0]

        # for snapshot in self.fruit_per_cell_data:
        #     # extract cell fruit data per snapshot
        #     self.density = snapshot[0]
        #     self.R       = snapshot[1]


    def plotValuesOverDistance(self):
        '''Plot all the relevant values as vehicle moves'''
        fig, ax = plt.subplots()

        x = np.arange(len(self.schedule_data))  # snapshot number, can chnage later

        fig, axs = plt.subplots(3, 1)
        # want subplots to stack in columns so they can be compared, see
        # see https://matplotlib.org/stable/gallery/lines_bars_and_markers/cohere.html#sphx-glr-gallery-lines-bars-and-markers-cohere-py
        axs[0].plot(x, self.FPE*100)
        # axs[0].set_xlabel('time')
        axs[0].set_ylabel('FPE [%]')
        # axs[0].grid(True)

        axs[1].plot(x, self.FPT)
        axs[1].set_ylabel('FPT [fruit/s]')

        axs[2].plot(x, self.v_vy)
        axs[2].set_ylabel('vehicle velocity [m/s]')
        axs[2].set_xlabel('Snapshot No.')

        fig.tight_layout()

        # plt.plot(self.sortedFruit[1][fruit_picked_by[n][k]], 
        #         self.sortedFruit[2][fruit_picked_by[n][k]], linestyle=linestyle, color=line_color, marker='o', label=arm_label)

        plotName = './plots/test0.png'

        print('Saving plot of FPT and FPE vs snapshot number in', plotName)
        plt.savefig(plotName,dpi=300)
       
        # plt.show()



    def avgFPTandFPE(self):
        '''Takes the various snapshots and combines their fPT and FPE values to get overall average FPT and FPE'''
        avg_FPE = np.average(self.FPE)
        avg_FPT = np.average(self.FPT)

        print('Based on known pickable fruit by system:')
        print("Average final FPT {0:.2f}".format(avg_FPT), "fruit/s")
        print("Average final FPE {0:.2f}".format(avg_FPE*100), "%")



