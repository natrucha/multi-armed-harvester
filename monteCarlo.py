import csv                      # read and write CSV file-type
from enum import Enum           # allows for the creation of enumeration object-types
import math                     # use math library functions such as

import matplotlib.pyplot as plt # plotting

import numpy as np
from numpy.random import PCG64, SeedSequence  # random number seed generator based on PCG64
import time                     # use OS clock

######## IMPORT MY OWN MODULES ########
from sim_loop import *          # import the simulator loop
from drawRobot import *         # import code that plots the vehicle and arm extension
from plotStates import *        # import module to plot percent time each arm spends in each of the six states
from simulation_config import * # import the JSON creation module to create the settings for each simulator run

## Set Flag values
class spaceConf(Enum):
    '''Flag values for if the workspace in each row is individual or shared'''
    INDIVIDUAL = 0
    SHARED     = 1

class calendar(Enum):
    '''Flag values for the type of scheduler to use, name seems like a bad idea :P'''
    SINGLE_FRUIT = 0
    EDF          = 1 # Earliest Deadline First, batch

class treeCreation(Enum):
    '''Flags values for the type of data being used as fruit coordinates'''
    CSV_RAJ  = 0
    SYN_LINE = 1
    UNIFORM  = 2

class noiseMaker(Enum):
    TURN_OFF = 0
    TURN_ON  = 1

class reality(Enum):
    '''Flag values determining how many fruit are real and how many are fake in the simulation'''
    TURN_OFF  = 0
    DENSITY   = 1


class monteCarlo(object):
    def __init__(self):
        self.N         = 10 # default number of runs per fruit distribution
        self.seed_list = []

        # lists for results
        self.fpe = []
        self.fpt = []
        self.running_time = []

        self.indep_var =[]

        # initialize the configuration file for easier paramater changes
        self.json_data = simulation_config()

        # open and save seeds from previously created RNG seed list
        self.getRNGSeedList()


    def plotAll(self, xLabel):
        '''
            Plot FPE and FPT results for all distributions vs. set of
            parameter Values.
        '''
        # Plot time versus number of fruit picked per arm
        fig, ax = plt.subplots(2, figsize=(7, 11))

        # change y-label depending on variable tested
        if xLabel == 'v_v':
            label = "Vehicle Velocity [m/sec]"
        elif xLabel == 'v_a':
            label = "Arm Max Velocity [m/sec]"
        elif xLabel == 'arm':
            label = "No. of Arms per Row"

        for i in range(self.N):
            ax[0].plot(self.indep_var, self.fpe[i], alpha=0.9, label="run "+str(i))
        # ax[0].plot(vel, fpe_semi, alpha=0.9, label="look ahead 20")
        # ax[0].set_xlabel("Vehicle Velocity [m/sec]")
        ax[0].set_xlabel(label)
        ax[0].set_ylabel("Percent Fruit Picked [%]")

        for i in range(self.N):
            ax[1].plot(self.indep_var, self.fpt[i], alpha=0.9, label="run "+str(i))
        # ax[1].plot(vel, fpt_semi, alpha=0.9, label="look ahead 20")
        ax[1].set_xlabel(label)
        ax[1].set_ylabel("Throughput [fruit/sec]")

        #         ax[rows].set_title("No. of fruit picked versus time")
        ax[0].legend(bbox_to_anchor=(1.2, 1), loc='upper right', ncol=1)
        ax[1].legend(bbox_to_anchor=(1.2, 1), loc='upper right', ncol=1)

        fig.subplots_adjust(bottom=0.05, top=0.95, right=0.8)

        plt.show()


    def plotMean(self, xLabel):
        '''Plot the mean and std of the fpt and fp results'''
        fig, ax = plt.subplots(2, figsize=(7, 11))

        # change y-label depending on variable tested
        if xLabel == 'v_v':
            label = "Vehicle Velocity [m/sec]"
        elif xLabel == 'v_a':
            label = "Arm Max Velocity [m/sec]"
        elif xLabel == 'arm':
            label = "No. of Arms per Row"

        mean_fpt = []
        std_fpt  = []

        mean_fpe = []
        std_fpe  = []

        for v in range(len(self.indep_var)):
            fpe_per_v = []
            fpt_per_v = []

            for r in range(self.N):
                fpe_per_v.append(self.fpe[r][v])
                fpt_per_v.append(self.fpt[r][v])

            mean_fpe.append(np.mean(fpe_per_v))
            std_fpe.append(np.std(fpe_per_v))

            mean_fpt.append(np.mean(fpt_per_v))
            std_fpt.append(np.std(fpt_per_v))

        ax[0].errorbar(self.indep_var, mean_fpe, std_fpe, marker='o', capsize=3, label="Fruit Picking Efficiency mean over "+str(self.N)+" runs")
        ax[1].errorbar(self.indep_var, mean_fpt, std_fpt, marker='o', capsize=3, label="Fruit Picking Throughput mean over "+str(self.N)+" runs")

        ax[0].set_xlabel(label)
        ax[0].set_ylabel("Percent Fruit Picked [%]")

        ax[1].set_xlabel(label)
        ax[1].set_ylabel("Throughput [fruit/sec]")

        ax[0].legend(bbox_to_anchor=(1, 1.1) ,loc='upper right', ncol=1)
        ax[1].legend(bbox_to_anchor=(1, 1.1) ,loc='upper right', ncol=1)

        plt.show()


    def runVariableArms(self, N):
        '''
           Run the simulation over N number of different uniform fruit
           distributions over a set list of number of arms per arm row.

           Clears fpt and fpe from previous runs.
        '''
        self.N    = N
        # clear previously obtained results
        self.fpe = []
        self.fpt = []
        # set any non-default values for scheduling
        self.json_data.appointment = calendar.SINGLE_FRUIT
        seeds     = [] # saves the rng seed list used for each run
        tot_time  = 0.
        # To print the number of runs done
        r         = 1

        # run the simulation
        for n in range(self.N):
            # init or clear the lists this fruit distributions' results
            fpe_semi = []
            fpt_semi = []
            running_time_semi = []

            # set up the seeds needed for this run
            seeds = self.seed_list[n]
            # arn velocities to be tested
            self.indep_var = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10] # in number of arms
            if n == 0:
                print("Total number of runs for completion:", N)
                print("----------------------------------------")

            for v in self.indep_var:
                # run the simulation, each time changing the velocity and calculating the results
                self.json_data.changeNumArms(v)
                # re-init sim_loop
                sim = sim_loop(seeds)
            #     print()
            #     sim.sysData()
                # print()
                sim.results() # just calculates all the results, doesn't print anything out.
                fpe_semi.append(sim.all_percent_harvest*100)
                try:
                    fpt_semi.append(1/sim.all_sec_per_fruit)

                except ZeroDivisionError as e0:
                    fpt_semi.append(0)

                running_time_semi.append(sim.prog_time)
                tot_time += sim.prog_time
        #         print("Done with:", v, "number of arms")
        #         print("Took", sim.prog_time, "sec to run")

            # save this fruit distributions's results to be compared with the other distributions.
            self.fpe.append(fpe_semi)
            self.fpt.append(fpt_semi)
            self.running_time.append(running_time_semi)

            print("Done with run:", r)
            print("System time passed up to now {0:.2f}".format(tot_time/60), "min")
            print()
            r += 1

        print("Total system time to run {0:.2f}".format(tot_time/60), "min")


    def runVariableV_a(self, N):
        '''
           Run the simulation over N number of different uniform fruit
           distributions over a set list of arm maximum velocities.

           Clears fpt and fpe from previous runs.
        '''

        self.N    = N
        # clear previously obtained results
        self.fpe = []
        self.fpt = []
        # set any non-default values for scheduling
        self.json_data.appointment = calendar.SINGLE_FRUIT
        seeds     = [] # saves the rng seed list used for each run
        tot_time  = 0.
        # To print the number of runs done
        r         = 1

        # run the simulation
        for n in range(self.N):
            # init or clear the lists this fruit distributions' results
            fpe_semi = []
            fpt_semi = []
            running_time_semi = []

            # set up the seeds needed for this run
            seeds = self.seed_list[n]
            # arn velocities to be tested
            self.indep_var = [0.01, 0.05, 0.1, 0.2, 0.4, 0.6, 0.8, 1., 1.2, 1.4] # in m
            if n == 0:
                print("Total number of runs for completion:", N)
                print("----------------------------------------")

            for v in self.indep_var:
                # run the simulation, each time changing the velocity and calculating the results
                self.json_data.changeV_a(v)
                # re-init sim_loop
                sim = sim_loop(seeds)
            #     print()
            #     sim.sysData()
                # print()
                sim.results() # just calculates all the results, doesn't print anything out.
                fpe_semi.append(sim.all_percent_harvest*100)
                try:
                    fpt_semi.append(1/sim.all_sec_per_fruit)

                except ZeroDivisionError as e0:
                    fpt_semi.append(0)

                running_time_semi.append(sim.prog_time)
                tot_time += sim.prog_time
        #         print("Done with vehicle velocity:", v)
        #         print("Took", sim.prog_time, "sec to run")

            # save this fruit distributions's results to be compared with the other distributions.
            self.fpe.append(fpe_semi)
            self.fpt.append(fpt_semi)
            self.running_time.append(running_time_semi)

            print("Done with run:", r)
            print("System time passed up to now {0:.2f}".format(tot_time/60), "min")
            print()
            r += 1

        print("Total system time to run {0:.2f}".format(tot_time/60), "min")


    def runVariableV_v(self, N):
        '''
           Run the simulation over N number of different uniform fruit
           distributions over a set list of vehicle velocities.

           Clears fpt and fpe from previous runs.
        '''
        self.N    = N
        # clear previously obtained results
        self.fpe = []
        self.fpt = []
        # set any non-default values for scheduling
        self.json_data.appointment = calendar.SINGLE_FRUIT
        seeds     = [] # saves the rng seed list used for each run
        tot_time  = 0.
        # keeps track of the row number of the csv being read (each row contains the seeds for one run)
        csv_i     = 0
        # To print the number of runs done
        r         = 1

        # run the simulation
        for n in range(self.N):
            # init or clear the lists this fruit distributions' results
            fpe_semi = []
            fpt_semi = []
            running_time_semi = []

            # set up the seeds needed for this run
            seeds = self.seed_list[n]
            # vehicle velocities to be tested
            self.indep_var = [0.01, 0.015, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.15, 0.2, 0.3, 0.5] # in m
            if n == 0:
                print("Total number of runs for completion:", N)
                print("----------------------------------------")

            for v in self.indep_var:
                # run the simulation, each time changing the velocity and calculating the results
                self.json_data.changeV_v(v)
                # re-init sim_loop
                sim = sim_loop(seeds)
            #     print()
            #     sim.sysData()
                # print()
                sim.results() # just calculates all the results, doesn't print anything out.
                fpe_semi.append(sim.all_percent_harvest*100)
                try:
                    fpt_semi.append(1/sim.all_sec_per_fruit)

                except ZeroDivisionError as e0:
                    fpt_semi.append(0)

                running_time_semi.append(sim.prog_time)
                tot_time += sim.prog_time
        #         print("Done with vehicle velocity:", v)
        #         print("Took", sim.prog_time, "sec to run")

            # save this fruit distributions's results to be compared with the other distributions.
            self.fpe.append(fpe_semi)
            self.fpt.append(fpt_semi)
            self.running_time.append(running_time_semi)

            print("Done with run:", r)
            print("System time passed up to now {0:.2f}".format(tot_time/60), "min")
            print()
            r += 1

        print("Total system time to run {0:.2f}".format(tot_time/60), "min")


    def getRNGSeedList(self):
        '''
           Open the random seed list rngseed_list_20200901.csv with 200 seeds for each of the 3 real fruit coordinate axis
           and 3 fake fruit coordinate axis.
        '''
        # keeps track of the row number of the csv being read (each row contains the seeds for one run)
        csv_i     = 0

        with open('rngseed_list_20200901.csv') as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
            for row in reader:
                self.seed_list.append(row)
                if csv_i == self.N:
                    break

                csv_i += 1
