# Gurobi MIP for the multi-armed harvester
# Copyright (C) 2024  Natalie C. Pueyo Svoboda

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>. 
# 

# from collections import defaultdict

## see examples in https://www.gurobi.com/resource/modeling-examples-using-the-gurobi-python-api-in-jupyter-notebook/
import gurobipy as gp
from gurobipy import GRB

# import math
import numpy as np
# from datetime import datetime
import sys

from fruit_distribution import *   # import module to create the various desired fruit distributions 
from data_analysis import *        # import module to analyze the data from the snapshots
from trajectory import *           # import module to calculate the trapezoidal/S-curve (S-curve not working yet) tajectory calculator

# tested with Python 3.7.0 & Gurobi 9.0

## based on the Gurobi technician routing scheduling example
# https://gurobi.github.io/modeling-examples/technician_routing_scheduling/technician_routing_scheduling.html

class MIP_algorithm(object):
    def __init__(self, q_vy, n_col, n_row, starting_row_n, d_cell, d_o, d_hrzn, x_lim, y_lim, z_lim, density):

        '''
            Mixed integer programming model based on the MIP model in the melon combinatorial scheduling paper. 
            Modified to work with a matrix of arms, rather than just a line.

            starting_row_n is the row number at which this MIP run will start, usually set at 0 unless each row is run seperately
        '''

        ## Constants 
        ## to print out from data analysis
        self.print_out  = 1
        plot_out   = 1

        # set which digitized data file will be used, 'raj' or 'juan'
        # self.data_name = 'raj'

        ## settings
        self.Td    = 2.      # in s, fruit handling time when it's constant
        self.M     = 280     # arbitrarily large number, should be based on the largest difference between t[c, r,i] and t[c, r,j] plus some extra (upper bound)  
        self.q_vy  = q_vy    # in m, backmost, lowest coordinate of the vehicle
        self.q_vy0 = q_vy    # in m, the actual start (snapshots require negatve values)

        self.n_row            = n_row      # total number of horizontal rows with cells containg one arm per cell
        self.n_col            = n_col      # number of arms in one horizontal row, also the number of columns of arms

        self.starting_row_n   = starting_row_n # row number at which this MIP run will start, usually set at 0 unless each row is run seperately

        self.density          = density      # in fruit/m^2
       
        self.d_cell           = d_cell     # in m, length of the cell along the orchard row (y-axis), parallel to vehicle travel
        self.d_o              = d_o        # in m, the space between column workspaces (includes frame and any space)

        # self.h_cell           = h_cell     # in m, width/height of the horizontal row of arms (z-axis) perpendicular to vehicle travel
        self.reach            = 1          # in m, the length that the arm can extend out to pick a fruit
        self.d_hrzn           = d_hrzn      # in m, the length of the view horizon

        self.noRel_time_ub    = 35     # in s, no relaxation heuristic max time to solve before moving to branch and bound (varies)
        self.timLim_time_ub   = 10*60     # in s, the total amount of time the solver runs (includes NoRel)

        self.d_vehicle  = self.n_col*self.d_cell + (self.n_col-1)*self.d_o  # in m, the full length of the vehicle, including space between the column workspaces

        self.t_grab = 0.5

        ## Get fruit list based on desired distribution
        self.x_lim   = np.copy(x_lim)
        self.y_lim   = np.copy(y_lim)
        self.z_lim   = np.copy(z_lim)

        # calculate the real y_lim[1] based on the fruit distribution 
        # self.calcYlimMax(set_distribution, self.d_vehicle, v_vy_fruit_cmps, n_fruit) 

        # allow calculations for the x-axis, 1.0 m long worst case
        v_max_x   = 2         # in m/s, from Motor Sizing Google sheet calculations if we want to keep triangular profile to maximize speed
        a_max_x   = 4         # in m/s^2, from Motor Sizing Google sheet calculations if we want desired linear movement time to be 1 second
        d_max_x   = a_max_x   # in m/s^2, if motors allow, keep equal to a_max
        # initialize the ability to calculate trajectory
        self.traj_calc_x = Trajectory(v_max_x, a_max_x, d_max_x) 

        

## Functions   
    def setTravelLength(self, D_):
        '''Set the length of travel of vehicle for this run (either snapshot or overall)'''
        self.D_ = D_ # in m, the travel length over this snapshot (or overall run)



    def setZlim(self, bot_edge, top_edge):
        '''set this run's z-axis limits (top and bottom) for each row'''
        self.z_row_bot_edges = bot_edge
        self.z_row_top_edges = top_edge



    def inputOrchard(self, sortedFruit):
        '''Input existing fruit distribution data to use as the fruit data. Used for snapshots.'''
        self.sortedFruit = np.copy(sortedFruit)


    
    def solve_mip(self, fruit, job, n_snap, v_vy_curr, set_distribution, set_algorithm, set_MPC, fruit_travel_matrix, sortedFruit, FPE_min=0.5, v_vy_lb_cmps=1, v_vy_ub_cmps=5):
    # def solve_mip(self, arm, fruit, job, n_snap, v_vy_curr, set_algorithm, fruit_travel_matrix, sortedFruit, FPE_min=0.5, v_vy_lb_cmps=1, v_vy_ub_cmps=5):
        '''
            Solve the MIP formulation based on already created arm and fruit object lists, the vehicle velocity in cm/s for the run, the MIP settings (makespan or not, etc.).
            Has default value for keyword FPE_min in case makespan is used, this is NOT a recommended value. The FPE_min value should be thoughtfully dtermined based on 
            on settings such as v_vy, length of the vehicle, horizon, and "recalculation" travel. Same deal with the upper and lower
            velocity bounds  

            n_snap is the available number of fruits in the snapshot       
        '''
        ## Build useful data structures
        # lists:
        C_  = [*range(self.n_col)]        # list of columns containing the arms (number of arms in a column based on the number of rows)
        R_  = [*range(self.n_row)]        # list of horizontal row numbers, uses the argument-unpacking operator *
        N  = [i.index for i in fruit]    # list of fruit indexes starting at 0, with an offset when needed
        Y  = [i.y_coord for i in fruit]  # list of fruits' y-coordinate (x-coordinate in the paper)
        Z  = [i.z_coord for i in fruit]  # list of fruits' z-coordinate
        TX = [i.Tx for i in fruit]       # list of fruit extension times

        # self.job = self.createJobs(arm, fruit, v_vy_curr, self.d_cell)

        # lists for results 
        fruit_picked_by = list()                             # list that saves which arm picks which fruit
        fruit_picked_at = list()                             # list that saves at what time an arm picks a fruit, based on fruit_picked_by "topography"
        self.curr_j     = np.zeros([self.n_row, self.n_col]) # array to save the sum of fruits picked by each arm
        # build out the lists to be the right size (tried replication *, but the lists point at each other)
        for n in range(self.n_row):
            if self.n_row > 1:
                fruit_picked_by.append([])
                fruit_picked_at.append([])

            for c in range(self.n_col+1):
                if self.n_row > 1:
                    fruit_picked_by[n].append([])
                    fruit_picked_at[n].append([])
                else:
                    fruit_picked_by.append([])
                    fruit_picked_at.append([])

        # save the real index of the 0th fruit (to get the difference)
        try:
            # offset_fruit_index = self.job[0].fruit_i.real_index
            offset_fruit_index = job[0].fruit_i.real_index
            # print('job\'s first real index:', offset_fruit_index, '\n')
        except IndexError:
            # no fruits visible, not even already picked fruits, probably the start where there are no fruits (entering the row)
            offset_fruit_index = 0

        # if self.print_out == 1:
            # print('number of arms in each horizontal row:',C_, 'with length', len(C_))
            # print()
            # print('number of horizontal rows:',R_, 'with length', len(R_))
            # print()
            # print('number of fruits:\n',N, '\nwith length', len(N))
            # print()
            # print('fruit y-coordinate:', Y, 'with length', len(Y))
            # print()
            # print('fruit z-coordinate:', Z, 'with length', len(Z))
            # print()
            # print('number of jobs:', len(self.job))
            # print()
        
        if set_algorithm == 0:
            # makespan has to calculate it's own TW values, so this only needs to be done for non-makespan runs
            TW_start = {i : [j.TW_start for j in job if j.fruit_i.index == i] for i in N}
            TW_end   = {i : [j.TW_end for j in job if j.fruit_i.index == i] for i in N}

            if len(TW_start) != len(TW_end) or len(TW_start) != len(N) or len(job) != len(N)*len(C_)*len(R_):
                print('Error: The number of fruit and the number of jobs or TW times do not match, exiting out of system')
                print('Number of jobs', len(job), 'and number of fruits*arms', len(N)*len(C_)*len(R_))
                print()
                sys.exit(0)


        if len(N) != len(Y) or len(N) != len(Z) or len(Y) != len(Z):
            print('Error: Indexes for fruit index, y, and z-coordinates do not match, exiting out of system')
            print()
            sys.exit(0)

        ### Create model
        m = gp.Model("v_vy_loop_mip")

        ### change needed model parameters
        ### see https://www.gurobi.com/documentation/9.5/refman/python_parameter_examples.html#PythonParameterExamples
        
        if set_algorithm == 1 or set_algorithm == 2 or set_algorithm == 4 or set_algorithm == 5:
            # if n_snap = 0, change to n_snap = .0001 to avoid division by 0
            if n_snap == 0:
                n_snap = 0.0001
            # if velocity becomes a variable, it is multiplied with another variable requiring the following settings
            m.params.NonConvex = 2
            m.setParam('NonConvex', 2)

        # Due to *very* high complexity, limits time in the no relaxation heuristic
        # see https://www.gurobi.com/documentation/9.5/refman/norelheurtime.html
        m.setParam('NoRelHeurTime', self.noRel_time_ub)

        # limit the maximum amount of time the solver takes to find a solution -> if gap isn't 0% then "no solution"
        # if NoRel == TimeLimit, only NoRel used (https://support.gurobi.com/hc/en-us/community/posts/4414052781073-NoRel-and-setting-time-limits)
        m.setParam('TimeLimit', self.timLim_time_ub) # stop after half an hour
        
        # m.setParam('MIPGap', .4) # default is 1e-4, the relative gap between the two bounds
        # m.setParam('MIPGapAbs', .1) # bsolute difference between the lower and upper bounds

        ### Decision variables
        # to create the variables, see https://www.gurobi.com/documentation/9.5/refman/py_model_addvars.html
        # Arm-fruit assignment (is fruit i picked by arm c in row l)
        x = m.addVars(C_, R_, N, vtype=GRB.BINARY, name="x")

        # upper bound when arm can pick the fruit, set at the time when the vehicle will reach the end of it's travel length
        # print('\nthe velocity being used to calculate time bounds:', v_vy_curr) 
        distance_traveled = abs(self.q_vy - self.q_vy0) # in m, the distance the system has traveled from the start
        # need to have a way to switch to MPC vs. non MPC where the vehicle travel length changes
        if set_MPC == 0:
            # non-MPC => travel length is just the travel length
            d_solve = self.D_
        else:
            # MPC     => travel length is whole vehicle+horizon view
            d_solve = self.d_vehicle + self.d_hrzn

        t_move = (d_solve) / (v_vy_curr/100) # in s, the duration of the snapshot

        if d_solve >= self.d_vehicle:
            # if the travel length is larger than the vehicle
            t_ub = (d_solve + self.d_cell + self.d_hrzn + distance_traveled) / (v_vy_curr/100)   # in m/s
        elif d_solve < self.d_vehicle and d_solve > 0:
            # when the travel length is smaller than the vehicle
            t_ub = (self.d_vehicle + self.d_cell + self.d_hrzn + distance_traveled) / (v_vy_curr/100)  # in m/s
        elif d_solve <= 0:
            print('ERROR: travel length is zero, exiting out of system from MIP_melon.py')
            sys.exit(0)

        if self.print_out == 1:
            print('\nThe travel length being processed is {:.3f} m'.format(d_solve))
            print(f'The distance traveled from the start: %4.2f m' % distance_traveled)
            print(f'The potential travel distance window without the horizon: %4.2f s' % t_move)
            print('The upper bound for when an arm can pick a fruit is {:.3f} sec\n'.format(t_ub))

        # Time arm c, l reaches fruit i
        t = m.addVars(C_, R_, N, lb=0, ub=t_ub, name="t")

        if set_algorithm == 1 or set_algorithm == 2 or set_algorithm == 4 or set_algorithm == 5:
            # any TW start and end should be less than the total move time since nothing can be picked after this
            # the lower bound allows for negatives only because the aux values that take place after will switch to 0 if negative
            tw_s          = m.addVars(C_, N, lb=-t_ub, ub=t_ub, name="tw_s")
            tw_e          = m.addVars(C_, N, lb=-t_ub, ub=t_ub, name="tw_e")

            # required because gurobi doesn't like >= or <= constraints that deal with two variables
            aux_end       = m.addVars(C_, N, lb=0, name="aux_end")
            aux_start     = m.addVars(C_, N, lb=0, name="aux_start")
            aux_comp      = m.addVars(C_, N, lb=0, name="aux_compare")
            aux_comp_end  = m.addVars(C_, N, lb=0, name="aux_compare_end")

            ## mean handling time (generally seen btw 2-3 s/f)
            T_handle = 2.75
            
            # in cm/s, vehicle velocity along orchard row
            # bounded by the cell length and Td (melon paper) and bounded by max velocity of the lift (90 cm/s)
            if set_distribution == 1 and len(N) >= 15:
                V_lb_cmps = (self.n_row * self.n_col)*((d_solve-self.d_vehicle)*100) / (len(N)*T_handle)
            else:
                V_lb_cmps = (self.n_row * self.n_col)*(d_solve*100) / (len(N)*T_handle)

            V_ub_cmps = V_lb_cmps + 5

            if V_lb_cmps > v_vy_ub_cmps:
                V_lb_cmps = v_vy_ub_cmps - 5
                V_ub_cmps = v_vy_ub_cmps


            # print('##################### N = %d CR= %d, d_solve=%0.1f, T_h=%0.1f' %(len(N), int(self.n_row * self.n_col), ((d_solve-self.d_vehicle)*100), T_handle))
            
            
            v_vy          = m.addVar(lb=V_lb_cmps, ub=V_ub_cmps, name="v_vy")
            # v_vy          = m.addVar(lb=v_vy_lb_cmps, ub=v_vy_ub_cmps, name="v_vy")
            # v_vy          = m.addVar(vtype=GRB.INTEGER, lb=v_vy_lb_cmps, ub=v_vy_ub_cmps, name="v_vy")

            # variable t_move calculated with currently chosen v_vy
            # # t_move_var    = m.addVar(lb=0, name="var_t_move")
            # t_harvested   = m.addVars(C_, R_, N, lb=0, ub=t_ub, name="t_harvested")   # used to get the makespan of harvested fruits, not all fruits
            # makespan      = m.addVar(lb=0, name="makespan")
            # t_max_arm     = m.addVars(C_, name="t_max")  # max t value for each arm
            # makespan_norm = m.addVar(lb=0, name="makespan_norm")
            
            FPE_var       = m.addVar(lb=0, name="FPE")
            FPT_var       = m.addVar(lb=0, name="FPT")

            # need slack variables for soft constraints 
            # see https://support.gurobi.com/hc/en-us/community/posts/5628368009233-Soft-Constraints-being-treated-as-Hard-Constraints-
            # min values for slack constranints
            FPE_min_val = FPE_min    # the minimum FPE in a soft constraint
            if set_distribution == 1:
                FPT_min_val = V_lb_cmps*len(N)/((d_solve-self.d_vehicle)*100)   # in fruits/s, the minimum FPT desired. Trying to get something a bit higher than max FPT at 95%
            else:
                FPT_min_val = V_lb_cmps*len(N)/((d_solve)*100)
            # FPT_min_val = (self.n_row * self.n_col) / T_handle # theoretical max
            FPT_theo_max = (self.n_row * self.n_col) / (T_handle-0.2)

            # lb_FPE_slack     = 1.0 - FPE_min_val
            lb_FPT_slack     = FPT_min_val - FPT_theo_max

            # print('############## FPTmin %0.4f, theoretical max %0.4f and V lower bound %0.4f' %(FPT_min_val, FPT_theo_max, V_lb_cmps))

            minFPE_slack     = m.addVar(lb=0, name="FPEslackMin") # lb = difference btw 0.95 and 1.0 

            minFPT_slack     = m.addVar(lb=lb_FPT_slack, name="FPTslackMin") # lb = lb_FPE times slope

            # add a starting guess value to the variable
            # see https://www.gurobi.com/documentation/9.5/refman/start.html#attr:Start
            # v_vy.start = 2 # in cm/s

            # create a variable that saves the last picking time, or makespan
            x_weighted    = m.addVars(C_, R_, N, lb=0, name="x_weighted")   # used to get the makespan of harvested fruits, not all fruits
        

        ### Constraints
        # At most one arm can be assigned to a fruit (1)
        m.addConstrs((x.sum('*', '*', i) <= 1 for i in N), name="assignOne")

        # Time elapsed between pickup of any two fruit reached by the same arm is at least Td (2)
        # m.addConstrs((t[c, r,i] + self.Td - t[c, r,j] <= self.M * (2 - x[c, r,j] - x[c, r,i]) for i in N for j in N for c in C_ for r in R_ if Y[j] > Y[i]), name="atLeast")
        m.addConstrs((t[c, r, j] + TX[j] + fruit_travel_matrix[j+offset_fruit_index, i+offset_fruit_index] + TX[i] + self.t_grab - t[c, r, i] <= self.M * (2 - x[c, r, i] - x[c, r, j]) for i in N for j in N for c in C_ for r in R_ if Y[j] < Y[i]), name="atLeast")
        
        # If fruit z-coord is outside of arm's range, do not pick it
        if self.starting_row_n >= self.n_row:
            # if the mip is divided so each row gets a mip run, the row number has to be changed
            n_row = self.starting_row_n
            m.addConstrs(((x[c, r, i] == 0) for i in N for r in R_ for c in C_ if Z[i] < self.z_row_bot_edges[n_row,c] or Z[i] > self.z_row_top_edges[n_row,c]), name="verticalWorkArea")
        else:
            m.addConstrs(((x[c, r, i] == 0) for i in N for r in R_ for c in C_ if Z[i] < self.z_row_bot_edges[r,c] or Z[i] > self.z_row_top_edges[r,c]), name="verticalWorkArea")

        # If fruit was removed because it's scheduled and picked, do not schedule it again
        # offset added because the indexes have to start at 0 for every run, but when scheduling snapshots, the index of the fruits may not start at 0
        m.addConstrs(((x[c, r, i] == 0) for i in N for r in R_ for c in C_ if sortedFruit[4,i+offset_fruit_index] == 2), name="removeScheduledAndPicked")
        # if the travel distance between snapshots is less than the view distance, don't harvest fruits that are too far forward for arms in column k to pick given that limited travel distance
        if set_MPC == 0:
            #### THIS ONE to be removed if using MPC, or used seperately
            m.addConstrs(((x[c, r, i] == 0) for i in N for r in R_ for c in C_ if Y[i] - (self.q_vy + (c + 1)*self.d_cell + c*self.d_o) >= d_solve), name="fruitInHorizon")
        # if TW_end is negative, the fruit has passed the column k’s back edge and cannot be harvested by any arm in that column
        m.addConstrs(((x[c, r, i] == 0) for i in N for r in R_ for c in C_ if Y[i] - (self.q_vy + c*self.d_cell + c*self.d_o) <= 0 ), name="fruitStartPassed")

        # If the fruit is at the back edge, then it cannot be picked because the arm has to extend out and grab the fruit
        m.addConstrs(((t[c, r, i] - TX[i] - self.t_grab >= -(1.5 + self.t_grab)*(1 - x[c, r, i]) + 0.001*x[c, r, i]) for i in N for r in R_ for c in C_), name="minHarvestTime")
        
        if set_algorithm == 0:   
            # the time of harvest for fruit i by arm in column k and row l has to be within the calculated time windows for that fruit and that column 
            m.addConstrs((t[c, r, i] >= min(TW_start[i][c], TW_end[i][c]) for i in N for r in R_ for c in C_), name="timeWinStart")
            m.addConstrs((t[c, r, i] <= max(TW_start[i][c], TW_end[i][c]) for i in N for r in R_ for c in C_), name="timeWinEnd")

        elif set_algorithm == 1 or set_algorithm == 2 or set_algorithm == 4 or set_algorithm == 5:
            # calculate the time windows for each fruit given the tested/chosen velocity 
            m.addConstrs(((tw_s[c, i] * (v_vy / 100) == (Y[i] - (self.q_vy + (c + 1)*self.d_cell + c*self.d_o))) for i in N for c in C_), name="TW_start")
            m.addConstrs(((tw_e[c, i] * (v_vy / 100) == (Y[i] - (self.q_vy + c*self.d_cell + c*self.d_o))) for i in N for c in C_), name="TW_end")
            
            # turns x[c, r, i] == 0 if t[c, r, i] * (v_vy/100) >= D_, or the fruit cannot be picked if the time of picking is larger than the time between snapshots
            # to learn about building this constraint see https://or.stackexchange.com/questions/5860/link-a-binary-variable-to-continuous-variable-in-java-gurobi
            if set_MPC == 0:
                #### THIS ONE to be removed if using MPC, or used seperately
                m.addConstrs(((d_solve - t[c, r, i]*(v_vy / 100) >= -(self.d_vehicle + d_solve + self.d_hrzn) * (1 - x[c, r, i]) + 0.0001 * x[c, r, i]) for i in N for r in R_ for c in C_), name="tCannotPasstMove")

            # to learn how to deal with max/min with variables
            # see https://support.gurobi.com/hc/en-us/community/posts/360076808711-how-to-add-a-max-constr-General-expressions-can-only-be-equal-to-a-single-var
            # comparing to 0 because the value should never be negative
            # if the time window start or end times are negative, then they should be zeroed out because negative means that time has been passed
            m.addConstrs((aux_end[c, i]      == gp.max_(0, tw_e[c, i]) for i in N for c in C_), name="fixEndNegative")
            m.addConstrs((aux_start[c, i]    == gp.max_(0, tw_s[c, i]) for i in N for c in C_), name="fixStartNegative")
            # if aux_end <= 0, then aux_start should also be < 0, so choose the min value of the two
            m.addConstrs((aux_comp[c, i]     == gp.min_(aux_end[c, i], aux_start[c, i]) for i in N for c in C_), name="ifEndZeroStartZero")
            m.addConstrs((aux_comp_end[c, i] == gp.max_(aux_end[c, i], aux_start[c, i]) for i in N for c in C_), name="maxIsEnd")

            # Ensure each node is visited within the given time window (3) and (4)
            # TW_start and TW_end are matching the fruit index number exactly (disctionary), so [2][0] == index 2 (even 
            # though it starts at zero, second arm back from 0th arm)  
            m.addConstrs(((t[c, r, i] >= aux_comp[c, i]) for i in N for r in R_ for c in C_), name="timeWinStart")
            m.addConstrs(((t[c, r, i] <= aux_comp_end[c, i]) for i in N for r in R_ for c in C_), name="timeWinEnd")

            # Ensure at least 90% (or desired percentage) of available fruit are harvested, not needed if multi-objective setup works
            # m.addConstr((gp.quicksum(x[c, r, i] for i in N for r in R_ for c in C_)/n_snap >= FPE_min), name="percentHarvest")

            # # set makespan as the latest t^k_i value/ for every arm no matter the row or fruit
            # # see https://support.gurobi.com/hc/en-us/community/posts/360071830171-Use-index-of-decision-variable-in-max-constraint
            # m.addConstrs(((t_harvested[c, r, i] == t[c, r, i] * x[c, r, i]) for i in N for r in R_ for c in C_), name='timeHarvestedFruits')
            # m.addConstrs((t_max_arm[c]          == gp.max_(t_harvested.select(c, '*', '*')) for c in C_), name='max_value')

            # opposite of penalty added to the sum(x) objective so that a weight that changes with the velocity can be used to get better results
            m.addConstrs(((x_weighted[c, r, i] == x[c, r, i] * (v_vy / 100)) for i in N for r in R_ for c in C_), name='xWeighted')

            # FPT * FPE
            m.addConstr((FPT_var <= (1/d_solve) * gp.quicksum(x_weighted[c, r, i] for i in N for r in R_ for c in C_)), name='FPT')
            m.addConstr((FPE_var <= (1/n_snap) * gp.quicksum(x[c, r, i] for i in N for r in R_ for c in C_)), name='FPE')
            # m.addConstr((FPT_var - maxFPT_slack <= (1/d_solve) * gp.quicksum(x_weighted[c, r, i] for i in N for r in R_ for c in C_)), name='FPT')
            # m.addConstr((FPE_var - maxFPE_slack <= (1/n_snap) * gp.quicksum(x[c, r, i] for i in N for r in R_ for c in C_)), name='FPE')
            # m.addConstr((FPE_var <= 0.85), name='FPEmax')
            if n_snap > 10 and (set_algorithm == 1 or set_algorithm == 4):
                m.addConstr((FPE_var + minFPE_slack >= FPE_min_val), name='FPEmin')
                m.addConstr((FPT_var + minFPT_slack >= FPT_min_val), name='FPTmin')

            elif n_snap > 10 and (set_algorithm == 2 or set_algorithm == 5):
                m.addConstr((FPE_var + minFPE_slack >= FPE_min_val), name='FPEmin')
                # m.addConstr((FPT_var + minFPT_slack >= FPT_min_val), name='FPTmin')

            # m.addConstrs((t_max_arm[c] == gp.max_(t.select(c, '*', '*')) for c in C_), name='max_value')  # doesn't take into account if fruit harvested or not
            
            # # obtain the makespan and then normalize it over the total vailable travel time
            # m.addConstrs((makespan >= t_max_arm[c] for c in C_), name='makespan_constraint')
            # m.addConstr((makespan_norm == makespan * (v_vy / 100) / d_solve), name='normalized_makespan')

            # m.addConstrs((((v_vy / 100) * aux_var[c, r, i] == x[c, r, i]) for i in N for r in R_ for c in C_), name="aux")
            # m.addConstrs(((v_vy * aux_var[c, r, i] == (d_solve / n_snap) * x[c, r, i]) for i in N for r in R_ for c in C_), name="aux")
            # set makespan as the latest t^k_i value/ for every arm no matter the row or fruit
            # see https://support.gurobi.com/hc/en-us/community/posts/360071830171-Use-index-of-decision-variable-in-max-constraint
            # m.addConstrs(((t_harvested[c, r, i] == t[c, r, i] * x[c, r, i]) for i in N for r in R_ for c in C_), name='timeHarvestedFruits')
            # m.addConstrs((t_max_arm[c] == gp.max_(t_harvested.select(c, '*', '*')) for c in C_), name='max_value')

            # # m.addConstrs((t_max_arm[c] == gp.max_(t.select(c, '*', '*')) for c in C_), name='max_value')  # doesn't take into account if fruit harvested or not
            # if set_algorithm == 2 or set_algorithm == 5:
            #     m.addConstrs((makespan >= t_max_arm[c] for c in C_), name='makespan_constraint')
            #     m.addConstr((makespan_norm == makespan * (v_vy / 100) / d_solve), name='normalized_makespan')

            # m.addConstrs((((v_vy / 100) * aux_var[c, r, i] == x[c, r, i]) for i in N for r in R_ for c in C_), name="aux")
            # m.addConstrs(((v_vy * aux_var[c, r, i] == (d_solve / total_fruit) * x[c, r, i]) for i in N for r in R_ for c in C_), name="aux")



        ### Objective function
        if set_algorithm == 0:
            # single run MIP
            m.setObjective(gp.quicksum(x[c, r, i] for i in N for r in R_ for c in C_), GRB.MAXIMIZE)
            
        elif set_algorithm == 1 or set_algorithm == 4:
            # FPE*FPT 
            weight_fpe = 1#(3/n_snap)
            weight_fpt = (d_solve/n_snap) 
            m.ModelSense = GRB.MINIMIZE 
            m.setObjective(weight_fpe*minFPE_slack + weight_fpt*minFPT_slack)
            # m.ModelSense = GRB.MAXIMIZE 
            # m.setObjective(FPE_var*FPT_var - (1/n_snap)*minFPE_slack - (1/n_snap)*minFPT_slack) # old FPT*FPE objective

        elif set_algorithm == 2 or set_algorithm == 5:
            # makespan
            slack_weight = 1
            # makespan might use multiple objectives: minimize the makespan and maximize the FPE 
            # to define multiple hierarchical objectives see https://stackoverflow.com/questions/56120143/how-to-write-a-multi-objective-function-in-gurobi-python
            # Gurobi's specifying of https://www.gurobi.com/documentation/9.5/refman/specifying_multiple_object.html
            # Gurobi's working with https://www.gurobi.com/documentation/9.5/refman/working_with_multiple_obje.html 
            # example: https://www.gurobi.com/documentation/10.0/examples/workforce5_py.html#subsubsection:workforce5.py
            # set priority by index and priority, index 0 get automatic priority, how much priority can be set with priority--The higher (up to 10) the more priority 
            # makespan objective which affects the FPT 
            m.ModelSense = GRB.MAXIMIZE
            m.setObjective(FPT_var - slack_weight*minFPE_slack)
            # m.ModelSense = GRB.MINIMIZE
            # m.setObjective(makespan_norm + slack_weight*minFPE_slack)


        ## see https://github.com/jckantor/ND-Pyomo-Cookbook/blob/master/notebooks/04.03-Job-Shop-Scheduling.ipynb
        ## and https://support.gurobi.com/hc/en-us/community/posts/360071830171-Use-index-of-decision-variable-in-max-constraint

        ## write model into a file
        # see https://www.gurobi.com/documentation/9.5/refman/py_model_write.html
        # https://www.gurobi.com/documentation/9.5/refman/model_file_formats.html
        title_lp  = './mip_files/v_vy_' + str(v_vy_curr) + '_loop_mip.lp'
        title_mps = './mip_files/v_vy_' + str(v_vy_curr) + '_loop_mip.mps'
        m.write(title_lp)
        m.write(title_mps)
        print('lp and mps files saved under: v_vy =', str(v_vy_curr))
        m.optimize()

        status = m.Status
        if status in [GRB.INF_OR_UNBD, GRB.INFEASIBLE, GRB.UNBOUNDED]:
            print("Model is either infeasible or unbounded.")
            print("...")
            # obtain model Irreducible Inconsistent Subsystem (IIS)
            print('computing model IIS and writing it to an ipl doc') # see https://www.gurobi.com/documentation/10.0/refman/py_model_computeiis.html
            m.computeIIS()
            m.write("model.ilp")
            print("Exiting simulator.")
            sys.exit(0)

        elif status in [GRB.TIME_LIMIT]:
            print('No solution was found within the specified time limit, use the last solution found')
            # get number of solutions found by Gurobi 
            nSolutions = m.SolCount
            if nSolutions > 0:
                # choose the last solution found as the solution we'll use
                m.setParam(GRB.Param.SolutionNumber, nSolutions-1)
                # print('\nFPE\n    scheduled = {:.3f},    min provided= {:.3f},      slack value = {:.3f}'.format(FPE_var.X, FPE_min_val, minFPE_slack.X))
                # print('FPT\n    scheduled = {:.3f},    min provided= {:.3f},      slack value = {:.3f}\n'.format(FPT_var.X, FPT_min_val, minFPT_slack.X))
                # print('The chosen velocity is', v_vy.X, 'cm/s')
                # sys.exit(0)
            else:
                # no solution possible, build up and send back empty results and the max speed
                no_pick = np.where(sortedFruit[4,:] == 0)  # flag for scheduled == 1, scheduled and picked == 2
                for no_pick_i in no_pick[0]:
                    # Adding the indexes of non-picked fruit to a sublist at the end of the first 
                    # horizontal row's list of sublists
                    if self.n_row > 1:
                        # if multiple horizontal rows, append the non-picked sublist to the first horizontal row's list of lists
                        fruit_picked_by[0][self.n_col].append(no_pick_i)
                    else:
                        fruit_picked_by[self.n_col].append(no_pick_i)

                return([fruit_picked_by, fruit_picked_at, 0])

        elif status != GRB.OPTIMAL:
            print("Optimization terminated with status {}".format(status))               
    #         sys.exit(0)

        if set_algorithm == 1 or set_algorithm == 4:
            # print out the slack value
            print('\nFPE\n    scheduled = {:.3f},    min provided= {:.3f},      slack value = {:.3f}'.format(FPE_var.X, FPE_min_val, minFPE_slack.X))
            print('FPT\n    scheduled = {:.3f},    min provided= {:.3f},      slack value = {:.3f}\n'.format(FPT_var.X, FPT_min_val, minFPT_slack.X))
            # print('FPE', FPE_var.X, 'min FPE', FPE_min_val ,'FPE slack value = ', minFPE_slack.X)
            # print('FPT', FPT_var.X, 'min FPT', FPT_min_val ,'FPT Slack value = ', minFPT_slack.X)

        elif set_algorithm == 2 or set_algorithm == 5:
            print('FPT\n    scheduled = {:.3f}\n'.format(FPT_var.X))
            print('FPE\n    scheduled = {:.3f},     min desired = {:.3f},     slack value = {:.3f}'.format(FPE_var.X, FPE_min_val, minFPE_slack.X))

        # fruit_picked_by = list()                             # list that saves which arm picks which fruit
        # fruit_picked_at = list()                             # list that saves at what time an arm picks a fruit, based on fruit_picked_by "topography"
        # self.curr_j     = np.zeros([self.n_row, self.n_col]) # array to save the sum of fruits picked by each arm
        # build out the lists to be the right size (tried replication *, but the lists point at each other)
        # for n in range(self.n_row):
        #     if self.n_row > 1:
        #         fruit_picked_by.append([])
        #         fruit_picked_at.append([])

        #     for c in range(self.n_col+1):
        #         if self.n_row > 1:
        #             fruit_picked_by[n].append([])
        #             fruit_picked_at[n].append([])
        #         else:
        #             fruit_picked_by.append([])
        #             fruit_picked_at.append([])

        ### Print results
        # print()
        # print('###############################')
        # print('how time picked is saved')
        # print(t)
        # print()
        # print('###############################')
        # Assignments    
    #     print()
        for j in job:
            if self.starting_row_n >= self.n_row:
                n_row = 0
            else:
                # n_row = j.arm_k.row_n
                n_row = j.arm_cr.arm_r

            # if x[j.arm_k.arm_n, n_row, j.fruit_i.index].X > 0:
            if x[j.arm_cr.arm_c, n_row, j.fruit_i.index].X > 0:
                # if fruit was scheduled to be harvested
    #             print('fruit', j.fruit_i.index, 'assigned to arm', j.arm_k.arm_n, 'at t = ', t[j.arm_k.arm_n, j.fruit_i.index].X)
                # save picked to sortedFruit
                sortedFruit[4, j.fruit_i.real_index] = 1  # save to the real index on sortedFruit
                # self.curr_j[n_row, j.arm_k.arm_n] += 1
                if self.n_row > 1:
                    self.curr_j[n_row, j.arm_cr.arm_c] += 1
                    fruit_picked_by[n_row][j.arm_cr.arm_c].append(j.fruit_i.real_index)
                    fruit_picked_at[n_row][j.arm_cr.arm_c].append(t[j.arm_cr.arm_c, n_row, j.fruit_i.index].X)

                else:
                    self.curr_j[0,j.arm_cr.arm_c] += 1
                    fruit_picked_by[j.arm_cr.arm_c].append(j.fruit_i.real_index)
                    fruit_picked_at[j.arm_cr.arm_c].append(t[j.arm_cr.arm_c, n_row, j.fruit_i.index].X)

        no_pick = np.where(sortedFruit[4,:] == 0)  # flag for scheduled == 1, scheduled and picked == 2
    #     print('not picked indexes:', no_pick[0])

        for no_pick_i in no_pick[0]:
            # Adding the indexes of non-picked fruit to a sublist at the end of the first 
            # horizontal row's list of sublists
            if self.n_row > 1:
                # if multiple horizontal rows, append the non-picked sublist to the first horizontal row's list of lists
                fruit_picked_by[0][self.n_col].append(no_pick_i)
            else:
                fruit_picked_by[self.n_col].append(no_pick_i)

        if self.print_out == 1:
        #     print('model variables:', m.getAttr("x", m.getVars()))
            print()
            if set_algorithm == 1 or set_algorithm == 2 or set_algorithm == 4 or set_algorithm == 5:
                print('The chosen velocity is', v_vy.X, 'cm/s')
            print('set M value:', self.M)
            print()

        # check that TW and t^k_i match indexes and arms
        # print()
        # for i in N:
        #     for c in C_:
        #         row = 0
        #         # for row in range(self.n_row):
        #         # TW start and end are the same for every row, but not t, so depending on what's being checked, add or remove this for loop and add to t[].X
        #         print('For row', row, 'arm',c, 'and fruit', i, 'TW start:', TW_start[i][c], 'TW end:', TW_end[i][c], 'and t^k_i', t[c, row, i].X)
        #     print('Fruit harvested?', self.sortedFruit[4,i])
        #     print('----')

        # print('fruit picked by [0]', fruit_picked_by[0])
        # print('fruit picked by [1]', fruit_picked_by[1])
            
        if v_vy.X > v_vy_ub_cmps:
            V = v_vy_ub_cmps
        else:
            V = v_vy.X

        if set_algorithm == 0:
            # only need which fruit were picked by what and when
            return([fruit_picked_by, fruit_picked_at])
        elif set_algorithm == 1 or set_algorithm == 2 or set_algorithm == 4 or set_algorithm == 5:
            # needs the above as well as the chosen velocity
            return([fruit_picked_by, fruit_picked_at, V])