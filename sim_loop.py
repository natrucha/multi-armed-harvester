#!/usr/bin/env python
import csv
from enum import Enum
import math
# import copy                    # when copying lists, important to look at shallow and deep copies

# Plotting
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
# from matplotlib.collections import Poly3DCollection # https://stackoverflow.com/questions/10599942/drawing-a-rectangle-or-bar-between-two-points-in-a-3d-scatter-plot-in-python-and
# and https://stackoverflow.com/questions/37585340/plotting-3d-polygons-in-python-3

import numpy as np
from numpy.random import PCG64
import time

# import my own libraries
from arm import *               # import the arm state library
from camera import *            # import the simulated vision system
from fruitTreeCreator import *  # import the library that creates the array with fruit location + other info
from scheduler import *         # import the scheduler functions
from simulation_config import * # import the JSON creation file so that it can be read back to set parameters
from trajectory import *        # import the trajectory generation "library" that I'm writing
from drawRobot import *         # import code that plots the vehicle and arm extension

import json # configuration file encode and decode
# see https://realpython.com/python-json/#decoding-custom-types

# Testing and Debugging
# Example: https://stackoverflow.com/questions/40172281/unit-tests-for-functions-in-a-jupyter-notebook
import unittest             # docs https://docs.python.org/3/library/unittest.html
import pdb #; pdb.set_trace() # docs https://docs.python.org/3/library/pdb.html

# interesting/useful websites
# talks about implementing robot simulator: https://www.toptal.com/robotics/programming-a-robot-an-introductory-tutorial
#    said robot simulator code: https://github.com/nmccrea/sobot-rimulator/blob/v1.0.0/models/supervisor.py

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
    '''Flags values for the type of noise'''
    TURN_OFF = 0
    TURN_ON  = 1

class reality(Enum):
    '''Flag values determining how many fruit are real and how many are fake in the simulation'''
    TURN_OFF  = 0
    DENSITY   = 1

class sim_loop(object):
    def __init__(self):
        ################ Default Variable Values ###################
        # fruit row depth, in ft  -- x-axis
        self.fruit_row_ed   = 0.3
        self.fruit_row_tk   = 1.3
        # fruit row length, in ft -- y-axis
        self.fruit_row_st   = 0.
        self.fruit_row_end  = 25.
        # fruit row height, in ft -- z-axis
        self.fruit_row_bt   = 0.
        self.fruit_row_tp   = 9.
        # values for fruit density (currently supports only one value overall)
        self.rho_real       = 1.
        self.rho_fake       = 0.3
        # decide on the number of arms and rows
        self.num_arms       = 3
        self.num_row        = 3
        # arm's max velocity and acceleration values apparently in ft/s
        self.max_v          = 1.
        self.max_a          = 10.
        # vehicle's velocity (constant), in ft/s
        self.v_vx           = 0.
        self.v_vy           = 0.05
        # number of goals the semionline scheduler will look for
        self.n_goals        = 20
        # when working with the fruit ribbon, how high above the conveyors will the ribbon be
        self.ribbon_z       = 1.
        # Configuration parameter DEFAULTS
        # decide if the arms are in individual or shared spaces
        self.space_config   = spaceConf.SHARED
        # decide on the type of scheduler
        self.appointment    = calendar.EDF
        # decide what fruit distribution to use to create fruit coordinates
        self.data_config    = treeCreation.UNIFORM
        # decide if fake fruit will be added
        self.existance      = reality.TURN_OFF
        # decide if noise should be added and the distribution
        self.noise_level    = noiseMaker.TURN_OFF

        # Read and set new variable settings
        self.readJSON()

        ##################### Other Variables #######################
        # Orchard row settings
        self.t            = []        # 'global' internal time list
        self.t_step       = 0.
        self.dt           = 0.01      # 'global' time step size
        self.runs         = 0         # saves the number of loops in one simulation run
        self.prog_time    = 0.        # how long the program takes to run
        self.start_time   = 0.        # OS time at which the program started to calculate total running time
        self.tot_num_arms = self.num_arms*self.num_row

        # variables to save and analyze final results
        self.avg_pick_cycle     = [] # saves the average picking cycle for each individual arm
        self.percent_goal       = [] # saves the percent of fruit reached by each arm
        self.row_percent        = [] # saves the % harvestable harvested fruit by each row
        self.sec_per_fruit      = [] # saves the average sec per picked fruit for each arm
        self.row_sec_per_fruit  = [] # saves the average sec per picked fruit for each row
        self.states_percent     = [] # multi-dimensional list that saves the amount of time each arm spent in each state
        self.mean_state_percent = [] # mean of percent time the arms spend in each state
        self.total_fruit_picked = 0  # by the whole system
        self.all_PCT            = 0. # average PCT of the whole simulation
        self.all_percent_goal   = 0. # average percent reached ofthe whole system
        self.all_sec_per_fruit  = 0. # overall internal time over overall fruit reached

        # Lists
        self.arm_states  = [] # saves the state of the arm at every loop of the simulation
        self.row_picture = [] # saves the fruit seen by the row
        # used to plot the vehicle
        self.qv0         = []
        self.qv1         = []
        # for plotting data, saves the location of the edges of the vehicle
        self.left_edge   = []
        self.right_edge  = []
        self.front_edge  = []
        self.back_edge   = []

        ####### Vehicle Init Values for parameter setting #######
        # create the vehicle speed array
        self.v_v = np.array([self.v_vx, self.v_vy])  # in ft, constant velocity only in y-axis
        # make the arm take up space within the space (slowly added)
        column_width = 0.2               # width of column holding the arm in place
        # calculate the height of the frame based on number of rows, height of orchard, and add column width to make sure all
        # fruit can be reached
        frame_height = (self.fruit_row_tp - self.fruit_row_bt + 2*column_width) / self.num_row # if the rows are evenly spaced
        # configure the arm's space, length added to vehicle per arm in the y-direction
        if self.space_config == spaceConf.INDIVIDUAL:
            frame_width  = 3.
        elif self.space_config == spaceConf.SHARED:
            frame_width  = 3.
        #     frame_width  = 1.9
        width_v  = 1.                    # vehicle width (x-dir, parallel to arms going into canopy) (only for plotting)
        length_v = frame_width*self.num_arms  # vehicle length (y-dir parallel to row of trees)

        ##################### Based on Flags #####################
        # initializes required variables based on the distribution creation method
        if self.data_config != treeCreation.CSV_RAJ:
            self.q_v = np.array([self.fruit_row_ed-1.0/2-0.3, self.fruit_row_st])
            # init a z-coordinate list of where a fruit ribbon will be placed on each row
            fruit_lines = []
            # if using the line distribution, this will set the z height for each line
            for n in range(self.num_row):
                fruit_lines.append(n*frame_height + self.ribbon_z) # in ft
            # initialize the synthetic fruit distribution creator
            self.fruit = fruitTreeCreator([self.fruit_row_st,self.fruit_row_end])
            # settings for an angle offset if the tree data has the fruit set at a diagonal
            arm_offset = 0.
            # create list for fruit and fake fruit, only needed when not using digitized fruit
            fruit_density = []
            fake_density  = []

            for n in range(self.num_row):
                # set fruit density and percent of fake fruit to add (will have to change later to add sections
                # with different densities)
                fruit_density.append([self.rho_real])  # in fruit/ft^3 (unless using LINE, then it's fruit/ft^2)
                if self.existance == reality.DENSITY:
                    fake_density.append([self.rho_fake])
                else:
                    fake_density.append([0.0])

        elif self.data_config == treeCreation.CSV_RAJ:
            self.q_v = np.array([4.5,7.]) # fruits are in very specific locations
            # CSV fruit are diagonally placed (fix that later?)
            self.v_v = np.array([0.01,0.05])  # in ft, constant velocity   => for single fruit got almost 100% fruit at 0.007m/s with 0.73s/fruit (too slow)
            # use Raj's digitized fruit from real trees
            self.fruit = csvRead()
            # settings for an angle offset if the tree data has the fruit set at a diagonal
            arm_offset = 0.75 / (self.num_row-1)

        if self.data_config == treeCreation.SYN_LINE:
            # create synthetic fruit data in a line to test the robot when fruit are set at one specific height
            for r in range(self.num_row):
                self.fruit.fruitLine(self.num_row, fruit_density[r], [self.fruit_row_ed,self.fruit_row_tk], fruit_lines[r], fake_density[r])
        #     fruit.fruitLine(fruit_lines)
            self.fruit.fruitTreeOut()

        elif self.data_config == treeCreation.UNIFORM:
            # test the robot when there is variation in distance between fruit
        #     fruit.fruitUniform()
            for r in range(self.num_row):
                self.fruit.fruitUniform(self.num_row,fruit_density[r],[self.fruit_row_ed,self.fruit_row_tk],[self.fruit_row_bt,self.fruit_row_tp], fake_density[r])
            self.fruit.fruitTreeOut()

        ##################### init environment #####################
        # end of the row (when the back of vehicle reaches this point it should stop)
        self.end_row = self.fruit_row_end + width_v
        # used to plot basic outlines of the robot for better visualization
        self.dr = drawRobot()

        ######################## init camera #######################
        for rows in range(self.num_row): # one "camera" object per row on vehicle
            self.pic = camera(length_v, rows, frame_height)
            self.row_picture.append(self.pic)

        ######################### init arms ########################
        # array of arm object setup
        self.arm_obj = np.ndarray((self.num_row,self.num_arms), dtype=object)
        # setup for arm initialization
        arm0start_y = self.q_v[1] - (length_v - frame_width)/2  # arm starting position in the y-dir
        q_a_new = np.array([0.,0.,0.])                     # used as a temporary value while calculating arm init positions

        # initialize the arm objects in a matrix set for each row and for the number of arms in  said row
        for rows in range(self.num_row):
            for count in range(self.num_arms):
                # calculate where each new arm should go
                # if fruit's in a diagonal, get the bottom arms closer to the fruit
                # the 0.3 is the zero starting point since it will never be exactly zero if they're extending cylinders
                q_a_new[0] = self.q_v[0] + 0.3 + ((self.num_row-1)*arm_offset - arm_offset*rows)
                q_a_new[1] = arm0start_y + frame_width*count # places the arms either in their space or far from each other
                q_a_new[2] = self.row_picture[rows].row_mid   # place it in the already calculated middle of the row
                # last one this makes sure it matches the "camera" object's z-location
                # initialize the new arm
                x = arm(q_a_new, self.q_v, count, self.num_arms, frame_width, frame_height, column_width, rows, self.max_v, self.max_a, self.space_config.value)
                # add the arm to the a-list
                self.arm_obj[rows,count] = x

        ###################### init scheduler ######################
        # goal = np.zeros(3) # does this go here? it's an empty array for the goal values for each arm...
        self.sched_obj = scheduler(self.n_goals, self.num_row, self.num_arms, self.max_v, self.max_a)
        # make a scheduler function that sets n_goals only if it's synthetic data

        # run main loop
        self.simulationLoop()



    def simulationLoop(self):
        '''Main loop that runs through all the nodes'''
        ## start timer to see how long code takes to execute
        self.start_time = time.time()

        ##### while loooop!
        while(self.q_v[1] < self.end_row):
            # env.step
            self.t_step = self.step(self.t_step, self.dt)
            self.t.append(float(self.t_step))

            # vehicle.step
            self.q_v = self.vehicleStep(self.q_v, self.v_v, self.dt) # calculate "instantaneous" location

            # Have the camera take a new "picture" at new location ##FLASH!##
            for rows in range(self.num_row):
                self.row_picture[rows].cameraStep(self.end_row, self.q_v[1], self.fruit.sortedFruit) # resulting in a B-Tree of index/coordinates in world frame

            if (self.noise_level == noiseMaker.TURN_ON):
                # add some noissse! (to the fruit locations)
                self.noise_level = noiseMaker.TURN_OFF # but not right now

            # scheduler setting
            if self.appointment == calendar.SINGLE_FRUIT:
                self.sched_obj.singleFruitScheduler(self.num_row, self.num_arms, self.arm_obj, self.row_picture, self.fruit, self.t_step)

            elif self.appointment == calendar.EDF:
                if self.runs % 100 == 0: # don't want to run the scheduler every millisecond
                    self.sched_obj.edfScheduler(self.arm_obj, self.row_picture, self.fruit, self.v_v, self.space_config.value)
                # go through the list and give each arm a goal
                for rows in range(self.num_row):
                    # give each free arm in that row a goal
                        for arm_free in range(self.num_arms):
                            if self.arm_obj[rows,arm_free].free == 1:
                                # need to check if there are goals
                                if (not self.sched_obj.goal_queue[rows, arm_free].empty()):
                                    goal_new = self.sched_obj.goal_queue[rows, arm_free].get()
                                    # give the arm the index so that it can set it back to 0 if it does not reach it
                                    self.arm_obj[rows,arm_free].goal_index = goal_new
                                    # set the new goal
                                    self.arm_obj[rows,arm_free].setGoal(self.fruit.sortedFruit[0:3,goal_new], self.t_step)


            # have each arm take a step
            for rows in range(self.num_row):
                for arm2step in range(self.num_arms):
                    q_a = self.arm_obj[rows,arm2step].armStep(self.v_v, self.arm_obj[rows,:], self.dt, self.t_step, self.fruit, self.row_picture)

            # save plotting data
            self.qv0.append(float(self.q_v[0]))
            self.qv1.append(float(self.q_v[1]))

            self.left_edge.append(float(self.q_v[0]))  # set at the center of the vehicle in case the arm is set as diagonal to rach farther fruit
            self.front_edge.append(float(self.arm_obj[0,0].y_edges_f[0]))
            self.back_edge.append(float(self.arm_obj[0,0].y_edges_f[1]))

            arm_index = 0
            # list to save the state of all the arms during this loop
            arm_list  = []

            # compile at which state each arm ends the loop to add to plot later
            for rows in range(self.num_row):
                for arm2state in range(self.num_arms):
                    arm_list.append([arm_index, self.arm_obj[rows, arm2state].stateFLAG])
                    arm_index += 1

            # congregation of all the arm states at every loop
            self.arm_states.append(arm_list)
            # add to the main loop counter
            self.runs+=1

        # obtain the amount of time it took to run the simulator
        self.prog_time = time.time() - self.start_time


    def sysData(self):
        '''Print system values such as running time, number of fruit, internal simulator time, etc.'''
        tot_internal_time = self.t[-1]
        tot_vehicle_dist  = self.q_v[1]-self.q_v[0]

        try:
            print("program took: {0:.2f}".format(self.prog_time), "sec")
        except NameError:
            self.prog_time = time.time() - start_time
            print("***prog_time, and thus the total time, maybe incorrect because the main loop terminated early***")

        print("total internal time: {0:.2f}".format(tot_internal_time), "sec")
        print("total vehicle distance moved: {0:.2f}".format(tot_vehicle_dist), "ft")
        print()
        print("vehicle speed (if constant) in the y-axis:", self.v_v[1], "ft/s")
        print("max arm velocity:", self.arm_obj[0,0].v_max, "ft/s, max arm acceleration:", self.arm_obj[0,0].a_max, "ft/s^2")
        print("total number of fruit in CSV file:", len(self.fruit.x_fr)) # reality check


    def results(self):
        '''Compile all the results'''
        sum_individual_PCT = 0.
        # row_sec_per_fruit  = 0.
        # if it runs more than one time, the same list gets appended onto the first one
        self.listCleanup()

        # compile arm state data and calculate the percent time each arm is in each state throughtout the
        # simulation run
        self.armStateResults()

        # Rear arms are arm no. 0, bottom row is row no. 0
        for rows in range(self.num_row):
            # calculate the % harvestable harvested fruit by each row
            self.calcPercentHarvested(rows)
            self.rowSecPerFruit(rows)

            for count in range(self.num_arms):
                # Obtain individual arm picking cycle times
                sum_individual_PCT += self.calcAvgPCT(rows, count)
                # Obtain individual arm % fruit reached out of goals given by arm
                self.calcPercentGoals(rows, count)
                # calculate how many fruit were picked overall
                self.total_fruit_picked += self.arm_obj[rows,count].reached_goals
                # calculate the average seconds per fruit for each arm
                self.secPerFruit(rows, count)

        ### Calc totals for overall system ###
        # calculate the overall average PCT for the system
        self.all_PCT = sum_individual_PCT / (self.num_row * self.num_arms)
        # calculate the overall average percent reached
        self.all_percent_goal = sum(self.percent_goal) / self.tot_num_arms
        # calculate the overall average sec/fruit
        self.all_sec_per_fruit = self.t[-1] / self.total_fruit_picked
        # calculate average % reached harvestable fruit
        self.all_percent_harvest = self.total_fruit_picked / self.fruit.tot_fruit
        # calculate the mean percent of time the arms are in each states
        self.meanStatePercent() # don't run unless armStateResults() already ran


    def secPerFruit(self, rows, count):
        '''Calculate the average amount of seconds an arm takes to pick a fruit'''
        numPicked         = self.arm_obj[rows, count].reached_goals
        tot_internal_time = self.t[-1]

        self.sec_per_fruit.append(tot_internal_time / numPicked)



    def rowSecPerFruit(self, rows):
        '''Calculate the average amount of seconds per picked fruit for each row of arms'''
        picked = 0
        tot_internal_time = self.t[-1]

        for arms in range(self.num_arms):
            picked += self.arm_obj[rows, arms].reached_goals

        self.row_sec_per_fruit.append(tot_internal_time / picked)


    def armStateResults(self):
        '''
            Compiles the information of what state each arm is at at each time step in the main
            loop. Results in the percent time each arm is in each state.
        '''
        # calculate how long each arm spent in each state
        state_step = 0     # used to save the state in the correct order matching the arm order
        state_data = np.zeros((self.tot_num_arms, len(self.arm_states))) # used to separate the states based on the arm number
        # list that has the sum of loops during which the arm was in each state
        arm_states          = []
        self.states_percent = [] # in case this function is run more than one time during testing

        # obtain the state for each arm at each time step of the simulation loop
        for time_step in self.arm_states:  # has all the arms' states at each loop of the main code
            for arm_num in time_step:      # for each arm in that time step
                # save the state values
                state_data[arm_num[0],state_step] = arm_num[1]

            state_step += 1

        for arms in range(self.tot_num_arms):
            # calculate the total amount of simulation loops (to get percentages)
            total_loops = len(state_data[arms,:])

            for state in range(6): # number of states
                loops_in_state = np.where(state_data[arms,:] == state)
                sum_loops      = len(loops_in_state[0])
                # print("Arm", arms, "was in state", state, "this many loops:", sum_loops)
                arm_states.append(sum_loops)

            total_loops    = sum(arm_states)
            percent = [x / total_loops for x in arm_states]

            self.states_percent.append(percent)
            arm_states = []


    def meanStatePercent(self):
        '''
           Calculate the mean of the percent time the arms are in each state.

           Depends on results from armStateResults() which has to be run before
           meanStatePercent() can be run.
        '''
        self.mean_state_percent = []
        state_tot = 0

        for states in range(6):
            for arms in self.states_percent:
                state_tot += arms[states]

            self.mean_state_percent.append(state_tot/self.tot_num_arms)
            state_tot = 0


    def calcPercentHarvested(self, rows):
        '''
            Calculate the percent reachable fruit harvested by the system

            INPUT: row number
        '''
        row_harvested = 0
        # obtain number of fruits reached by the arms in the row (goes one by one)
        for i in range(self.num_arms):
            row_harvested += self.arm_obj[rows, i].reached_goals
        # calculate the obtained reached goals per row vs the total fruit created in the row
        self.row_percent.append(row_harvested / self.fruit.fruit_in_row[rows])


    def calcAvgPCT(self, rows, count):
        '''Calculate the individual arm and overall average picking cycle time values'''
        # calculate picking cycle average for each arm
        sum_individual_PCT = 0.

        try:
            avg_individual_PCT = sum(self.arm_obj[rows,count].pick_cycle)/len(self.arm_obj[rows,count].pick_cycle)
            self.avg_pick_cycle.append(avg_individual_PCT)
            # only add the individual PCT values that are not NaN
            sum_individual_PCT += avg_individual_PCT
        except ZeroDivisionError: # in case no fruit were picked by an arm
            avg_individual_PCT = np.nan
            self.avg_pick_cycle.append(avg_individual_PCT)
            # don't really know how to deal with this.
            # if avg_individual_PCT = 0, the average drops, but it shouldn't?
            # if avg_individual_PCT = NaN, it makes arthmetic = NaN

        # calculate the total average PCT
        return sum_individual_PCT


    def calcPercentGoals(self, rows, count):
        '''
           Obtain the data for percent reached fruit out of given goals per individual
           arm and per the harvesting system as a whole.
         '''
        # calculate the percent reached goals for each arm
        given = self.arm_obj[rows, count].goals_given
        reached = self.arm_obj[rows, count].reached_goals
        self.percent_goal.append((reached / given) * 100)


    def readJSON(self):
        '''Read the JSON configuration file and set all the variables to the
           correct values and settings'''
        ###### create JSON configuration file to read from #####
        # init function
        json_data = simulation_config()
        # creates the data file based on default values
        json_data.convertJSON()
        # load the json file
        data = json.load(open("data.json"))

        ###### Parameter settings from the JSON configuration file ######
        # fruit row depth, in ft  -- x-axis
        self.fruit_row_ed   = data['orchard']['x']['start']  # how far the vehicle will be from the edges of the tree
        self.fruit_row_tk   = data['orchard']['x']['end']    # how far the arms can reach into the canopy/ where the trunk is
        # fruit row length, in ft -- y-axis
        self.fruit_row_st   = data['orchard']['y']['start']
        self.fruit_row_end  = data['orchard']['y']['end']
        # fruit row height, in ft -- z-axis
        self.fruit_row_bt   = data['orchard']['z']['start']
        self.fruit_row_tp   = data['orchard']['z']['end']
        # values for fruit density (currently supports only one value overall)
        self.rho_real       = data['orchard']['rho_real']
        self.rho_fake       = data['orchard']['rho_fake']
        # decide on the number of arms and rows
        self.num_arms       = data['vehicle']['num_arms']      # set number of arms on robot, will determine the length of the robot (for now)
        self.num_row        = data['vehicle']['num_rows']      # set the number of rows of arms
        # arm's max velocity and acceleration values apparently in ft/s
        self.max_v          = data['arms']['max_v']
        self.max_a          = data['arms']['max_a']
        # vehicle's velocity (constant), in ft/s
        self.v_vx           = data['vehicle']['v_vx']
        self.v_vy           = data['vehicle']['v_vy']
        # number of goals the semionline scheduler will look for
        self.n_goals        = data['num_goals']
        # when working with the fruit ribbon, how high above the conveyors will the ribbon be
        self.ribbon_z       = data['ribbon_z']
        # decide if the arms are in individual or shared spaces
        self.space_config   = spaceConf(data['space_config'])
        # decide on the type of scheduler
        self.appointment    = calendar(data['appointment'])
        # decide what fruit distribution to use to create fruit coordinates
        self.data_config    = treeCreation(data['data_config'])
        # decide if fake fruit will be added
        self.existance      = reality(data['existance'])
        # decide if noise should be added and the distribution
        self.noise_level    = noiseMaker(data['noise_level'])


    def step(self, t, dt):
        '''
           Step function for the time steps.

           INPUT: last time value, t, and the time step size, dt

           OUTPUT: new time value
        '''
        t = t + self.dt
        return t


    def vehicleStep(self, q_curr, v, dt):
        '''
           Step function for the vehicle when the velocity is constant.

           INPUT: last vehicle location, q_curr, vehicle velocity, and time step

           OUTPUT: new vehicle location
        '''
        # for now it's constant velocity
        q_new = np.array([q_curr[0] + v[0]*self.dt, q_curr[1] + v[1]*self.dt])
        return q_new


    def listCleanup(self):
        '''
           Empties out result lists so that they don't keep growing if the results are calculated multiple times.
        '''
        ## commented out lists
        self.avg_pick_cycle    = []
        self.percent_goal      = []
        self.row_percent       = []
        self.sec_per_fruit     = []
        self.row_sec_per_fruit = []
