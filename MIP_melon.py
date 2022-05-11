from collections import defaultdict

## see examples in https://www.gurobi.com/resource/modeling-examples-using-the-gurobi-python-api-in-jupyter-notebook/
import gurobipy as gp
from gurobipy import GRB

import math
import numpy as np
from datetime import datetime
import sys

from fruit_distribution import *   # import module to create the various desired fruit distributions 
from IG_data_analysis import *     # import module to analyze the data from the snapshots

# tested with Python 3.7.0 & Gurobi 9.0

## based on the Gurobi technician routing scheduling example
# https://gurobi.github.io/modeling-examples/technician_routing_scheduling/technician_routing_scheduling.html

class MIP_melon(object):
    def __init__(self, n_arm, n_row, starting_row_n, set_distribution, set_algorithm, set_MIPsettings, set_edges, v_vy_fruit_cmps, cell_l, cell_h, vehicle_h, horizon_l, x_lim, y_lim, z_lim):

        '''
            Mixed integer programming model based on the MIP model in the melon combinatorial scheduling paper. 
            Modified to work with a matrix of arms, rather than just a line.

            starting_row_n is the row number at which this MIP run will start, usually set at 0 unless each row is run seperately
        '''

        ## Constants 
        ## to print out from data analysis
        print_out  = 1
        plot_out   = 1

        ## settings
        self.Td    = 2.      # fruit handling time
        self.M     = 80     # arbitrarily large number, set 600 from paper    
        self.q_vy  = 0.      # in m, backmost, lowest coordinate of the vehicle

        # v_max      = 0.5
        # a_max      = 1.
        # t_grab     = 0.1 

        self.n_row      = n_row      # total number of horizontal rows with cells containg one arm per cell
        self.n_arm      = n_arm      # number of arms in one horizontal row

        self.starting_row_n = starting_row_n # row number at which this MIP run will start, usually set at 0 unless each row is run seperately

        self.density    = 16       # in fruit/m^2, makespan is being limited to rho = 2 with random placement
        n_fruit         = 80      # in fruit, for melon column distribution

        self.FPE_min    = 0.95

        self.cell_l     = cell_l     # in m, length of the cell along the orchard row (y-axis), parallel to vehicle travel
        self.cell_h     = cell_h     # in m, width/height of the horizontal row of arms (z-axis) perpendicular to vehicle travel
        self.reach      = 1          # in m, the length that the arm can extend out to pick a fruit

        self.noRel_time_ub  = 80     # no relaxation heuristic max time to solve before moving to branch and bound (varies)
        self.timLim_time_ub = 3600/3 # 3600*2

        ## set at function creation
        # v_vy_fruit_cmps = 8  # in cm/s, assumed vehicle velocity along orchard row to build line distribution

        # velocities if going thorugh a list of velocities
        # self.v_vy_ub_cmps   = 90     # in cm/s, when testing many velocities, this detemrines the top velocity tested
        # # v_vy_lb_cmps = math.ceil(cell_l / Td * 100) # chose to make into integer, ceil because smaller won't work
        # self._vy_lb_cmps   = 9
        # use a loop of potential v_vy values to remove v_vy as a variable from the MIP formulation
        # v_vy_cmps_try = np.arange(self.v_vy_lb_cmps, self.v_vy_ub_cmps+1)
        # print('velocities being attempted:', v_vy_cmps_try)

        vehicle_l  = self.n_arm * self.cell_l
        # vehicle_h  = self.n_row * self.cell_h #### SET AT INIT ####

        ## for future addition of snapshots
        # horizon_l  = 0. #### SET AT INIT ####

        # array to save which arm picked which fruit
        self.curr_j   = np.zeros([self.n_row, self.n_arm])


        #### SET AT INIT ####
        # ## set fruit distribution flag
        # # 0     == Raj's digitized fruits
        # # 1     == uniform random  (if algorithm == 1, use melon version)
        # # 2     == uniform random, equal cell density
        # # 3     == multiple densities separated by some space (only melon for now)
        # # 4     == fruit in vertical columns
        # # 5     == "melon" version of columns (user inputs desired no. fruits, z height, and distance between fruit in y-coord)
        # # 6     == Raj's digitized fruits, but which can reduce the density to a desired density
        # set_distribution = 6

        # ## set algorithm being used 
        # # 1     == melon
        # # not 1 == not melon
        # set_algorithm    = 1

        # ## set MIP model settings
        # # 0     == basic MIP model from the melon paper with arms not sharing space
        # # 1     == basic MIP model with velocity as a variable
        # # 2     == makespan MIP (have seperate code, don't use this until proven the same as the other)
        # # set_MIPsettings = set_MIPsettings ## SET AT FUNCTION CREATION

        # ## set how z-coord edges are calculated
        # # 0     == edges are divided equally along orchard height
        # # 1     == edges are divided so each row has equal number of fruit (or close to equal)
        # set_edges = 1

        n_snapshots = 1 # for now a constant

        ## Get fruit list based on desired distribution
        n_runs = 1
        # seed_list = self.getRNGSeedList(n_runs)

        # for run in range(n_runs):
        #     # get seeds for x, y, and z RNG (probably unneccessary right now, especially for x)
        #     seed = [seed_list[run][0], seed_list[run][1], seed_list[run][2]]
        #     x_seed = PCG64(int(seed[0]))
        #     y_seed = PCG64(int(seed[1]))
        #     z_seed = PCG64(int(seed[2]))
        
        self.calcTravel_l(set_distribution, vehicle_l, v_vy_fruit_cmps, n_fruit)
            
        # x_lim        = [0.2, 1.2]
        # self.y_lim   = [0. , self.travel_l - vehicle_l]
        # z_lim        = [0., vehicle_h] 

        self.x_lim   = x_lim
        self.y_lim   = y_lim
        self.z_lim   = z_lim

        self.y_lim[1] = self.travel_l - self.y_lim[1] # set all the limits outside the class, just add the correct travel length

        # # create fruit distribution and get total number of fruits
        # fruitD = fruitDistribution(x_lim, self.y_lim, z_lim)
        # [self.numFruit, self.sortedFruit] = self.createFruit(fruitD, set_algorithm, set_distribution, self.density, x_seed, y_seed, z_seed)

        # calculate the top and bottom z-coordinates for the horizontal rows
        # [self.z_row_bot_edges, self.z_row_top_edges] = self.set_zEdges(set_edges, z_lim)   
        


## Functions
    def buildOrchard(self, n_runs, set_algorithm, set_distribution):
        '''Creates the simulated environment, separated so that MIP run/row can happen'''
        seed_list = self.getRNGSeedList(n_runs)

        for run in range(n_runs):
            # get seeds for x, y, and z RNG (probably unneccessary right now, especially for x)
            seed = [seed_list[run][0], seed_list[run][1], seed_list[run][2]]
            x_seed = PCG64(int(seed[0]))
            y_seed = PCG64(int(seed[1]))
            z_seed = PCG64(int(seed[2])) 

        # create fruit distribution and get total number of fruits
        fruitD = fruitDistribution(self.x_lim, self.y_lim, self.z_lim) # init fruit distribution script
        [self.numFruit, self.sortedFruit] = self.createFruit(fruitD, set_algorithm, set_distribution, self.density, x_seed, y_seed, z_seed)

        print('Total fruit in the orchard row',self.numFruit)
        print()
        print('length of sortedFruit', len(self.sortedFruit[0]))
        print()
        # print('List of the x, y, and z coordinates of the sorted fruit')
        # print(sortedFruit)



    def createArms(self):
        '''Create and populate all the arms' classes then put in appropriate list'''
        ## create arm object list
        arm = list()

        if self.starting_row_n + 1 > self.n_row:
            row_n = self.starting_row_n + 1
        else:
            row_n = self.n_row

        # print('the starting row number is', self.starting_row_n, 'and the ending is', row_n)

        for r in range(self.starting_row_n, row_n):
            # print('row number', r)
        # for r in range(1, n_row+1):
            for k in range(self.n_arm):
        #     for k in range(1, n_arm+1):
                this_arm = Arm(r, k)
                arm.append(this_arm)
        # print()
        return(arm)


    def createFruits(self):
        '''Create and populate all the fruits' classes then put in appropriate list'''
        ## create fruit object list
        fruit = list()

        for index in range(self.numFruit):
            y_coord = self.sortedFruit[1][index]
            z_coord = self.sortedFruit[2][index]
            this_fruit = Fruit(index, y_coord, z_coord)
        #     print('Fruit index', index, 'should match this index', sortedFruit[3][index])
        #     print('with y and z coordinates:', y_coord, z_coord)

            fruit.append(this_fruit)

        # print(fruit)
        return(fruit)

    
    def createJobs(self, arm, fruit, v_vy_curr, cell_l):
        '''Create and populate all the jobs' classes then put in appropriate list'''
        ## create job object list
        job = list()

        for k in arm:
            for i in fruit:  
                # if self.starting_row_n >= self.n_row or self.n_row == 1:
                #     # check if the mip is being divided by row
                #     n_row = self.starting_row_n
                #     if (i.z_coord > self.z_row_bot_edges[0,n_row] and i.z_coord < self.z_row_top_edges[0,n_row]):
                #         # only create the job if the fruit's location is within the row's limits
                #         this_job = Job(i, k, v_vy_curr, cell_l)
                #         job.append(this_job)
                #         # print('for arm', this_job.arm_k.arm_n, 'and fruit', this_job.fruit_i.index)
                #         # print('TW starts at', this_job.TW_start, 'and TW ends at', this_job.TW_end)

                # else:
                this_job = Job(i, k, v_vy_curr, cell_l)
                job.append(this_job)
                # print('for arm', this_job.arm_k.arm_n, 'in row', this_job.arm_k.row_n,'and fruit', this_job.fruit_i.index)
                # print('TW starts at', this_job.TW_start, 'and TW ends at', this_job.TW_end)  

        return(job)

    
    def solve_melon_mip(self, arm, fruit, v_vy_curr, set_MIPsettings):
        ## Build useful data structures
        # lists:
        K = [*range(self.n_arm)]        # list of arms in each horizontal row
        L = [*range(self.n_row)]        # list of horizontal row numbers, uses the argument-unpacking operator *

        # if self.starting_row_n >= self.n_row or self.n_row == 1:
        #     # check if the mip is being divided by row
        #     n_row = self.starting_row_n
        #     print('edges to be compared against', self.z_row_bot_edges[0,n_row], self.z_row_top_edges[0,n_row])

        #     # only add the fruit and coordinates if the fruit's location is within the row's limits
        #     N = [i.index for i in fruit if (i.z_coord > self.z_row_bot_edges[0,n_row] and i.z_coord < self.z_row_top_edges[0,n_row])]    # list of fruit indexes
        #     Y = [i.y_coord for i in fruit if (i.z_coord > self.z_row_bot_edges[0,n_row] and i.z_coord < self.z_row_top_edges[0,n_row])]  # list of fruits' y-coordinate (x-coordinate in the paper)
        #     Z = [i.z_coord for i in fruit if (i.z_coord > self.z_row_bot_edges[0,n_row] and i.z_coord < self.z_row_top_edges[0,n_row])]  # list of fruits' z-coordinate
        # else:
        N = [i.index for i in fruit]    # list of fruit indexes
        Y = [i.y_coord for i in fruit]  # list of fruits' y-coordinate (x-coordinate in the paper)
        Z = [i.z_coord for i in fruit]  # list of fruits' z-coordinate
        
        # print('number of arms in each horizontal row:',K, 'with length', len(K))
        # print()
        # print('number of horizontal rows:',L, 'with length', len(L))
        # print()
        print('number of fruits:',N, 'with length', len(N))
        print()
        # print('fruit y-coordinate:', Y, 'with length', len(Y))
        # print()
        # print('fruit z-coordinate:', Z, 'with length', len(Z))
        # print()

        total_fruit = len(N) # needed to constraint FPE to a high picking percentage

        self.job = self.createJobs(arm, fruit, v_vy_curr, self.cell_l)
    #     ## create job object list
    #     job = list()

    #     for k in arm:
    #         for i in fruit:  
    #             this_job = Job(i, k, v_vy_curr, cell_l)
    #     #         
    # #             print('for arm', this_job.arm_k.arm_n, 'and fruit', this_job.fruit_i.index)
    # #             print('TW starts at', this_job.TW_start, 'and TW ends at', this_job.TW_end)
    #             job.append(this_job)
        
        if set_MIPsettings == 0 or set_MIPsettings == 1:
            TW_start = {i : [j.TW_start for j in self.job if j.fruit_i.index == i] for i in N}
            TW_end   = {i : [j.TW_end for j in self.job if j.fruit_i.index == i] for i in N}

        if len(N) != len(Y) or len(N) != len(Z) or len(Y) != len(Z):
            print('Error: Indexes for fruit index, y, and z-coordinates do not match')
            print()

        if len(TW_start) != len(TW_end) or len(TW_start) != len(N) or len(self.job) != len(N)*len(K)*len(L):
            print('Error: The number of fruit and the number of jobs or TW times do not match')
            print('Number of jobs', len(self.job), 'and number of fruits*arms', len(N)*len(K)*len(L))
            print()


        ### Create model
        m = gp.Model("v_vy_loop_mip")

        ### change needed model parameters
        ### see https://www.gurobi.com/documentation/9.5/refman/python_parameter_examples.html#PythonParameterExamples
        
        if set_MIPsettings == 2:
            # if velocity becomes a variable, it is multiplied with another variable requiring the following settings
            m.params.NonConvex = 2
            m.setParam('NonConvex', 2)

        # Due to *very* high complexity, limits time in the no relaxation heuristic
        # see https://www.gurobi.com/documentation/9.5/refman/norelheurtime.html
        m.setParam('NoRelHeurTime', self.noRel_time_ub)

        # limit the maximum amount of time the solver takes to find a solution -> if gap isn't 0% then "no solution"
        # if NoRel == TimeLimit, only NoRel used (https://support.gurobi.com/hc/en-us/community/posts/4414052781073-NoRel-and-setting-time-limits)
        m.setParam('TimeLimit', self.timLim_time_ub) # stop after half an hour


        ### Decision variables
        # Arm-fruit assignment (is fruit i picked by arm k)
        x = m.addVars(K, L, N, vtype=GRB.BINARY, name="x")

        # Time arm k, l reaches fruit i
        t_ub = (self.travel_l + self.cell_l) / (v_vy_curr/100)

        t = m.addVars(K, L, N, lb=0, ub=t_ub, name="t")
        
        if set_MIPsettings == 2:
            # Start and end of time window arm k can reach fruit i
            tw_s_ub = self.travel_l / (self.cell_l / self.Td) 
            tw_e_ub = (self.travel_l + self.cell_l) / (self.cell_l/ self.Td) 

            tw_s = m.addVars(K, N, lb=0, ub=tw_s_ub, name="tw_s")
            tw_e = m.addVars(K, N, lb=0, ub=tw_e_ub, name="tw_e")

            # required because gurobi doesn't like >= or <= constraints that deal with two variables
            aux_max = m.addVars(K, N, lb=0, name="aux_max")
            aux_min = m.addVars(K, N, lb=0, name="aux_min")

            # in cm/s, vehicle velocity along orchard row
            # bounded by the cell length and Td (melon paper) and bounded by max velocity of the lift (90 cm/s)
            v_vy = m.addVar(vtype=GRB.INTEGER, lb=self.v_vy_lb_cmps, ub=self.v_vy_ub_cmps, name="v_vy")
            
            # add a starting guess value to the variable
            # see https://www.gurobi.com/documentation/9.5/refman/start.html#attr:Start
        #     v_vy.start = 61 # in cm/s

            # create a variable that saves the last picking time, or makespan
            makespan  = m.addVar(lb=0, name="makespan")
            t_max_arm = m.addVars(K, name='t_max')  # max t value for each arm
        

        ### Constraints
        # At most one arm can be assigned to a fruit (1)
        m.addConstrs((x.sum('*', '*', i) <= 1 for i in N), name="assignOne")

        # Time elapsed between pickup of any two fruit reached by the same arm is at least Td (2)
        m.addConstrs((t[k, l, i] + self.Td - t[k, l, j] <= self.M * (2 - x[k, l, j] - x[k, l, i]) for i in N for j in N for k in K for l in L if Y[j] > Y[i]), name="atLeast")
        
        # If fruit z-coord is outside of arm's range, do not pick it
        if self.starting_row_n >= self.n_row:
            # if the mip is divided so each row gets a mip run, the row number has to be changed
            n_row = self.starting_row_n
            m.addConstrs(((x[k, l, i] == 0) for i in N for l in L for k in K if Z[i] < self.z_row_bot_edges[k,n_row] or Z[i] > self.z_row_top_edges[k,n_row]), name="verticalWorkArea")
        else:
            m.addConstrs(((x[k, l, i] == 0) for i in N for l in L for k in K if Z[i] < self.z_row_bot_edges[k,l] or Z[i] > self.z_row_top_edges[k,l]), name="verticalWorkArea")
        
        if set_MIPsettings == 0 or set_MIPsettings == 1:
            m.addConstrs((t[k, l, i] <= max(TW_start[i][k], TW_end[i][k]) for i in N for l in L for k in K), name="timeWinA")
            m.addConstrs((t[k, l, i] >= min(TW_start[i][k], TW_end[i][k]) for i in N for l in L for k in K), name="timeWinB")
            
            # If fruit z-coord is outside of arm's range, do not pick it
    #         m.addConstrs(((x[k, l, i] == 0) for i in N for l in L for k in K if Z[i] < z_row_bot_edges[k,l] or Z[i] > z_row_top_edges[k,l]), name="verticalWorkArea")

        elif set_MIPsettings == 2:
            m.addConstrs(((tw_s[k, i] * (v_vy / 100) == (Y[i] + k * self.cell_l)) for i in N for k in K), name="TW_start")
            m.addConstrs(((tw_e[k, i] * (v_vy / 100) == (Y[i] + (k + 1) * self.cell_l)) for i in N for k in K), name="TW_end")

            # to learn how to deal with max/min with variables
            # see https://support.gurobi.com/hc/en-us/community/posts/360076808711-how-to-add-a-max-constr-General-expressions-can-only-be-equal-to-a-single-var
            m.addConstrs((aux_max[k, i] == gp.max_(tw_s[k, i], tw_e[k, i]) for i in N for k in K), name="auxMax")
            m.addConstrs((aux_min[k, i] == gp.min_(tw_s[k, i], tw_e[k, i]) for i in N for k in K), name="auxMin")

            # Ensure each node is visited within the given time window (3) and (4)
            # TW_start and TW_end are matching the fruit index number exactly (disctionary), so [2][0] == index 2 (even 
            # though it starts at zero, second arm back from 0th arm)  
            m.addConstrs(((t[k, l, i] <= aux_max[k, i]) for i in N for l in L for k in K), name="timeWinA")
            m.addConstrs(((t[k, l, i] >= aux_min[k, i]) for i in N for l in L for k in K), name="timeWinB")

            # Ensure at least 90% (or desired percentage) of available fruit are harvested
            m.addConstr((gp.quicksum(x[k, l, i] for i in N for l in L for k in K)/total_fruit >= self.FPE_min), name="percentHarvest")

            # set makespan as the latest t^k_i value
            # see https://support.gurobi.com/hc/en-us/community/posts/360071830171-Use-index-of-decision-variable-in-max-constraint
            m.addConstrs((t_max_arm[k] == gp.max_(t.select(k, '*', '*')) for k in K), name='max_value')

        #     makespan = m.addVar(name='makespan')
            m.addConstrs((makespan >= t_max_arm[k] for k in K), name='makespan_contraint')


        ### Objective function
        if set_MIPsettings == 0 or set_MIPsettings == 1:
            m.setObjective(gp.quicksum(x[k, l, i] for i in N for l in L for k in K), GRB.MAXIMIZE)
            
        elif set_MIPsettings == 2:
            m.setObjective((makespan), GRB.MINIMIZE)

        ## see https://github.com/jckantor/ND-Pyomo-Cookbook/blob/master/notebooks/04.03-Job-Shop-Scheduling.ipynb
        ## and https://support.gurobi.com/hc/en-us/community/posts/360071830171-Use-index-of-decision-variable-in-max-constraint

        ## write model into a file
        # see https://www.gurobi.com/documentation/9.5/refman/py_model_write.html
        # https://www.gurobi.com/documentation/9.5/refman/model_file_formats.html
        title_lp  = './mip_files/v_vy_' + str(v_vy_curr) + '_loop_mip.lp'
        title_mps = './mip_files/v_vy_' + str(v_vy_curr) + '_loop_mip.mps'
        m.write(title_lp)
        m.write(title_mps)
        m.optimize()
        
        status = m.Status
        if status in [GRB.INF_OR_UNBD, GRB.INFEASIBLE, GRB.UNBOUNDED]:
            print("Model is either infeasible or unbounded.")
            print("Exiting simulator.")
            sys.exit(0)
        elif status != GRB.OPTIMAL:
            print("Optimization terminated with status {}".format(status))
    #         sys.exit(0)

        fruit_picked_by = list()    

        for n in range(self.n_row):
            if self.n_row > 1:
                fruit_picked_by.append([])

            for k in range(self.n_arm+1):
                if self.n_row > 1:
                    fruit_picked_by[n].append([])
                else:
                    fruit_picked_by.append([])

        ### Print results
        # Assignments    
    #     print()
        for j in self.job:
            if self.starting_row_n >= self.n_row:
                n_row = 0
            else:
                n_row = j.arm_k.row_n

            if x[j.arm_k.arm_n, n_row, j.fruit_i.index].X > 0:
    #             print('fruit', j.fruit_i.index, 'assigned to arm', j.arm_k.arm_n, 'at t = ', t[j.arm_k.arm_n, j.fruit_i.index].X)
    #             if set_MIPsettings == 1:
    #                 print('with tw start:', aux_min[j.arm_k.arm_n, j.fruit_i.index].X, 'and tw end:', aux_max[j.arm_k.arm_n, j.fruit_i.index].X)
                # save picked to sortedFruit
                self.sortedFruit[4, j.fruit_i.index] = 1
                self.curr_j[n_row, j.arm_k.arm_n] += 1
                if self.n_row > 1:
                    fruit_picked_by[n_row][j.arm_k.arm_n].append(j.fruit_i.index)

                else:
                    fruit_picked_by[j.arm_k.arm_n].append(j.fruit_i.index)

        no_pick = np.where(self.sortedFruit[4,:] == 0)
    #     print('not picked indexes:', no_pick[0])

        for no_pick_i in no_pick[0]:
            # Adding the indexes of non-picked fruit to a sublist at the end of the first 
            # horizontal row's list of sublists
            if self.n_row > 1:
                # if multiple horizontal rows, append the non-picked sublist to the first horizontal row's list of lists
                fruit_picked_by[0][self.n_arm].append(no_pick_i)
            else:
                fruit_picked_by[self.n_arm].append(no_pick_i)

    #     print('model variables:', m.getAttr("x", m.getVars()))
        print()
    #     print('chosen velocity:', v_vy.X, 'cm/s')
        print('set M value:', self.M)
        print()


        # check that TW and t^k_i match indexes and arms
    #     print()
    #     for k in K:
    #         for i in N:
    #             print('TW start:', TW_start[i][k], 'TW end:', TW_end[i][k], 'and t^k_i', t[k, i].X)

        # print('fruit picked by [0]', fruit_picked_by[0])
        # print('fruit picked by [1]', fruit_picked_by[1])
        
        return(fruit_picked_by)



    def calcTravel_l(self, set_distribution, vehicle_l, v_vy_fruit_cmps, n_fruit):
        '''Calculates travel length for the orchard row size for the chosen distribution and vehicle length'''
        if set_distribution == 0:
            self.travel_l  = 12 + vehicle_l # in m
            self.density   = 48.167         # in fruit/m^2 (on avg.), constant
            n_runs    = 1

        elif set_distribution == 1:
            self.travel_l  = 3 + vehicle_l # in m, usually 5 m + length

        elif set_distribution == 3:
            self.travel_l  = 30 + vehicle_l # in m
            
        elif set_distribution == 5:
            v_vy_fruit_mps = v_vy_fruit_cmps / 100
            self.d_y  = self.Td*v_vy_fruit_mps*(n_fruit+1)/(n_fruit+2) # kind of works 3/4 or 5/8 fruit with 1 arm: (Td/2)*v_vy
        #     d_y  = Td*v_vy_fruit*(n_fruit+1)/(n_fruit+2) # kind of works 3/4 or 5/8 fruit with 1 arm: (Td/2)*v_vy
            print('with Td', self.Td, 'and v_vy for fruit distribution', v_vy_fruit_cmps, 'cm/s')
            print('d_y for this line of fruit:', self.d_y, 'so the total distance they take up:', self.d_y*n_fruit)
            self.travel_l  = self.d_y * n_fruit + vehicle_l # in m
            
        elif set_distribution == 6:
            self.travel_l = 6 + vehicle_l # in m, should watch out because the total distance is actually 12

        else: 
            self.travel_l  = 10 + vehicle_l # in m



    def getRNGSeedList(self, n_runs):
            '''
            Open the random seed list rngseed_list_20200901.csv with 200 seeds for each of the 3 real fruit coordinate axis
            and 3 fake fruit coordinate axis.
            '''
            # keeps track of the row number of the csv being read (each row contains the seeds for one run)
            csv_i     = 0

            seed_list = list()

            with open('./rngseed_list_20200901.csv') as csvfile:
                reader = csv.reader(csvfile, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
                for row in reader:
                    seed_list.append(row)
                    if csv_i == n_runs:
                        break

                    csv_i += 1

            # print(seed_list)
            return(seed_list)
    
    
    def createFruit(self, fruitD, set_algorithm, set_distribution, density, x_seed, y_seed, z_seed):
            if set_distribution == 0:
                csv_file = './TREE_FRUIT_DATA/apple_tree_data_2015/Applestotheright.csv'
                [numFruit, sortedFruit] = fruitD.csvFile(csv_file, 0)

            elif set_distribution == 1:
                if set_algorithm == 1:
                    [numFruit, sortedFruit] = fruitD.uniformRandomMelon(density, y_seed, z_seed)
                else:
                    [numFruit, sortedFruit] = fruitD.uniformRandom(density, x_seed, y_seed, z_seed)
                # print()
                # print('--------------------------------------------')
                # print('Number of fruit:', numFruit)
                # print()

            elif set_distribution == 2: 
                fruit_in_cell = math.ceil(density * (self.cell_h*self.cell_l*arm_reach)) # num of fruit in front of cell if using (equalCellDensity())
                print('Number of fruit in each cell:', fruit_in_cell)
                print()
                [numFruit, sortedFruit] = fruitD.equalCellDensity(self.n_row, self.n_arm, self.cell_h, self.cell_l, self.arm_reach, fruit_in_cell, x_seed, y_seed, z_seed)

            elif set_distribution == 3: 
                densities = np.array([5, 4, 3])
                [numFruit, sortedFruit] = fruitD.uniformRandomMelon_MultipleDensity(densities, y_seed, z_seed)

            # elif set_distribution == 4: 
            #     [numFruit, sortedFruit] = fruitD.column(v_vy, v_max, a_max, t_grab, self.n_row, self.n_arm, self.cell_h, z_seed)
                
            elif set_distribution == 5:
                z_coord = (self.cell_h / 2) + 0.7
                [numFruit, sortedFruit] = fruitD.columnUniform_melon(n_fruit, self.d_y, z_coord)
                
            elif set_distribution == 6:
                csv_file = './TREE_FRUIT_DATA/apple_tree_data_2015/Applestotheright.csv'
                [numFruit, sortedFruit] = fruitD.csvFile_reduced(csv_file, 0, density, x_seed)

            else: 
                print('not a correct fruit distribution, defaulting to uniform random')
                if set_algorithm == 1:
                    [numFruit, sortedFruit] = fruitD.uniformRandomMelon(density, y_seed, z_seed)
                else:
                    [numFruit, sortedFruit] = fruitD.uniformRandom(density, x_seed, y_seed, z_seed)

            return([numFruit, sortedFruit])
    
    
    def calc_TW(self, arm_n, y_coord, v_vy):
        TW_start = (y_coord + (arm_n - 1)*self.cell_l) / v_vy
        TW_end   = (y_coord + arm_n*self.cell_l) / v_vy
        return([TW_start, TW_end])


    def getHorizonIndex(self, sortedFruit, q_vy, vehicle_l, horizon_l):
        '''
        Saves this snapshot's horizon fruit indexes based on the sortedFruit indexes to 
        compare and remove picked fruit.
        '''
        # edges of the horizon based on vehicle location and length
        horizon_back  = q_vy + vehicle_l
        horizon_front = horizon_back + horizon_l

        H_fruit_index = np.where((sortedFruit[1,:] >= horizon_back) & (sortedFruit[1,:] < horizon_front))

        return(H_fruit_index[0])


    def calcDensity(self, q_vy, v_vy, n_row, n_arm, cell_l, cell_h, arm_reach, sortedFruit):
        '''Get the fruit density, d, of each cell'''
        ## should the columns be based on cell length? number of arms? 
        #  should the columns be the same width? increase/decrease the closer to the front of vehicle?
        #  should I calculate R per horizontal row of arms?

        d = np.zeros([n_row, n_arm])  # total number of cells
        # starting position on the z-axis (up-down on robot)
        row_z = 0.

        for n in range(n_row):
            # starting position in the y_axis (front-back on robot)
            col_y = q_vy

            for k in range(n_arm):
                # print('col', n, 'row', k)
                # print('back', col_y, 'front', col_y + cell_l)
                # print('bottom', row_z, 'top', row_z + cell_h)
                index = np.where((sortedFruit[1,:] >= col_y) & (sortedFruit[1,:] < col_y + cell_l) & 
                            (sortedFruit[2,:] >= row_z) & (sortedFruit[2,:] < row_z + cell_h) & 
                            (sortedFruit[4,:] < 1))
                # save the number of fruit in this cell
                d[n,k] = len(index[0])
                # print(d)
                # move to the next column of cells
                col_y += cell_l

            # move up to the next cell on this column
            row_z += cell_h

        # before calculating the true density, check total number of fruit
        # print('which sums to', np.sum(d))   # has to be equal to numer of fruit
        # divide all the values by the volume of space in front of each cell 
        d = d / (arm_reach * cell_l * cell_h)

        # print('fruit density in each cell [fruit/m^3]:')
        # print(d)

        return(d)


    def calcR(self, v_vy, fruit_in_horizon, horizon_l, vehicle_h, arm_reach):
        '''Calculate the R value given a speed and horizon volume and density'''
        try:
            density_H = fruit_in_horizon / (horizon_l * vehicle_h * arm_reach)
            time_H    = horizon_l / v_vy

            R         = density_H / time_H # in fruit / (m^3 * s)

        except ZeroDivisionError:
            R         = 0 

        # print('Fruit incoming rate based on the horizon [fruit/(m^3 s)]:')
        # print(R)
        return(R) 


    def calcStateTime(self, fruit_picked_by, travel_l, v_vy, total_arms, n_row, n_arm, Td):
        # def calculateStateTimePercent(self, fruit_picked_by, total_distance):
            '''Calculates the time each arm is in each state so that it can plotState can plot the data'''
            # total_distance = self.y_lim[1] - self.y_lim[0]
            total_distance = travel_l
            total_time = total_distance / v_vy  # for snapshots? -> I'm deleting Tm and Tw data at each snapshot, problem
    #         print('movement distance:', total_distance)
    #         print('total move time:', total_time)

            ## states: idle, pick_yz, pick_x, grab, retract_x, move_z/unload
            # self.state_percent = np.zeros([self.total_arms, 6]) # save each arm's percent time in each of the six states 
            state_time = np.zeros([total_arms, 7]) # save each arm's percent time in each of the six states plus a total

            for n in range(n_row):
                for k in range(n_arm):
                    tot_arm_index = k + (n*n_arm)
                    # # calculate arm's move_yz using Tm
                    # for tm in self.Tm_values[n][k]:
                    #     self.state_time[tot_arm_index,1] += tm

                    if n_row > 1:
                        num_picked = len(fruit_picked_by[n][k])
    #                     print('arm', k, 'picked', num_picked, 'fruit, from', fruit_picked_by[n][k])
                    else:
                        num_picked = len(fruit_picked_by[k])
    #                     print('arm', k, 'picked', num_picked, 'fruit, from', fruit_picked_by[k])
                        
                    busy = num_picked * Td
    #                 print('so was busy', busy, 'total seconds')
    #                 print()
                    state_time[tot_arm_index,1] = busy  # setting Tm as the "handling time"

                    # calculate idle by subtracting all before by total time: length_row / v
                    state_time[tot_arm_index,0] = total_time - np.sum(state_time[tot_arm_index,:])
                    # save the total time for this run to get total percent later
                    state_time[tot_arm_index,6] = total_time
                    
            return(state_time)
    
    
    def set_zEdges(self, set_edges, z_lim, n_row):
        '''
        Calculate the z-coord for each horizontal row, assuming the whole row shares these edges.
        Returns a n_row x n_arm matrix for both the bottom and top edges of each cell. 
        '''   
        # edges for the nth horizontal row of cells
        
        if set_edges == 0: 
            # divided equally by distance along orchard height
            # row bottom edge = n*self.cell_h
            bottom = np.linspace(0, (n_row*self.cell_h - self.cell_h), self.n_row, endpoint=True)

            bot_edge = np.tile(bottom, (self.n_arm, 1))
            top_edge = np.copy(bot_edge) + self.cell_h
        #     print('bottom edges:', bot_edge)
        #     print()
        #     print('top edges:', top_edge)
        #     print()
        elif set_edges == 1:
            # divided by number of fruit
            
            # make zero arrays for top and bottom. Since edges shared along horizontal row, can tile it by n_arm
            top    = np.zeros(n_row)
            bottom = np.zeros(n_row)
            
            fruit_in_row = math.floor(self.numFruit / n_row)  # total fruit in each horizontal row (round down, one row could be heavier)
            print('number of fruit in each row, rounded down', fruit_in_row)
            
            # get z-coord array
            z_coord = np.array(self.sortedFruit[2]) 
            # sort the array
            z_sorted = np.sort(z_coord)
    #         print('sorted z-coord', z_sorted)
            
            for row in range(n_row-1):
                top[row]      = z_sorted[fruit_in_row*(row+1)]+0.0001
                bottom[row+1] = z_sorted[fruit_in_row*(row+1)]+0.0001 
                
            top[-1] = z_lim[1]
                
            bot_edge = np.tile(bottom, (self.n_arm, 1))
            top_edge = np.tile(top, (self.n_arm, 1))
            
        else:
            print('Not an edge setting, please try again')
            return([0, 0])

        self.z_row_bot_edges = bot_edge
        self.z_row_top_edges = top_edge

        print('bottom z-axis edges', self.z_row_bot_edges)
        print()
        print('top z-axis edges', self.z_row_top_edges)
        print()

        # return([bot_edge, top_edge])




## Required classes for the arms, fruits, and jobs
class Arm():
    def __init__(self, row_n, arm_n):
        self.row_n = row_n
        self.arm_n = arm_n

    def __str__(self):
        return f"Arm: {self.arm_n}\n Horizontal Row Number: {self.row_n}"


class Fruit():
    def __init__(self, index, y_coord, z_coord):#, job, tStart, tEnd, tDue):
        self.index = index       # fruit's index when ordered by y-coordinate
        self.y_coord = y_coord   # y-coordinate of the fruit
        self.z_coord = z_coord
        
    def __str__(self):
        return f"Fruit Index: {self.index}\n  Y-axis location: {self.y_coord}\n"


class Job():
    def __init__(self, fruit_i, arm_k, v_vy, cell_l):
        self.fruit_i  = fruit_i
        self.arm_k    = arm_k
        self.v_vy     = v_vy
        # k+1 was added because the MIP model in paper assumes k starts at 1
        self.TW_start = (self.fruit_i.y_coord + (self.arm_k.arm_n - 1 + 1)*cell_l) / (v_vy/100)
        self.TW_end   = (self.fruit_i.y_coord + (self.arm_k.arm_n + 1)*cell_l) / (v_vy/100)
        
class Job_v_vy():
    # for basic MIP with vehicle velocity as a variable
    def __init__(self, fruit_i, arm_k):
        self.fruit_i = fruit_i
        self.arm_k   = arm_k


## create snapshot object for data analysis
class Snapshot(object):
    def __init__(self, n_arm, n_row, horizon_l, vehicle_l, cell_l, v_max, a_max, set_algorithm, Td, v_vy, FPE, FPT, y_lim, numFruit, curr_j, sortedFruit, fruit_picked_by, state_time):
        # constants for the whole run
        self.n_arm      = n_arm
        self.n_row      = n_row
        self.horizon_l  = horizon_l
        self.vehicle_l  = vehicle_l
        self.cell_l     = cell_l
        self.v_max      = v_max
        self.a_max      = a_max
        if set_algorithm == 1:
            self.Td     = Td
            
        # constants and results for each snapshot in the run
        self.v_vy       = v_vy
        self.FPE        = FPE
        self.FPT        = FPT
        self.y_lim      = y_lim
        self.actual_numFruit = numFruit
        self.curr_j          = curr_j
        self.avg_PCT         = 0.
        self.state_time      = state_time
        self.fruit_picked_by = fruit_picked_by
        self.sortedFruit     = sortedFruit

    def __str__(self):
        return f"Arm: {self.arm_n}\n Horizontal Row Number: {self.row_n}"