import numpy as np
import math
import queue                       # see https://docs.python.org/3/library/queue.html#module-queue

from arm import *                  # import the arm state library
from camera import *               # import the simulated vision system
from trajectory import *           # import the trajectory generation "library" that I'm writing

class scheduler(object):

    def __init__(self, n_goals, num_row, num_arms, v_max, a_max):
        # need to figure out the type of schedule being used

        #################### CLASS VARIABLES ####################
        self.v_max = v_max
        self.a_max = a_max
        self.d_max = a_max

        # robotic system configuration
        self.num_rows     = num_row
        self.num_arms_row = num_arms

        total_arms = self.num_rows * self.num_arms_row

        # arm mounting frame width, since arms aren't just points in space
        self.mounting_width = 0.06 # in m, should match the cylinder drawn by the draw class

        # variables for the goals
        self.n_goals = n_goals # number of goals the greedy scheduler will find
        # create a matrix of queues that hold the next fruit to pick, one queue per each arm
        self.goal_queue = np.ndarray((self.num_rows, self.num_arms_row), dtype=object) # create the matrix

        for rows in range(self.num_rows):
            for count in range(self.num_arms_row):
                self.goal_queue[rows, count] = queue.Queue() # populate the matrix with queue objects

        ######################### FLAGS #########################
        # until I can figure out Enum

        # Arm cofiguration flags
        self.conf_INDIVIDUAL = 0
        self.conf_SHARED     = 1

        ######################## MODULES ########################


        ################ SELF FUNCTIONS AT INIT #################
        self.trajCalc = Trajectory(self.v_max, self.a_max, self.d_max) # to be able to calculate trajectories

    def edfScheduler(self, a, row_picture, fruit, v_v, arm_conf):
        '''Earliest deadline first algorithm with batches'''
        # base setting is with arms as individuals

        # keyword parameters in functions, https://treyhunner.com/2018/04/keyword-arguments-in-python/

        # get arm or for each arm?
        for rows in range(self.num_rows):

            arm_location = np.zeros([self.num_rows, self.num_arms_row])
            back_edge    = np.zeros([self.num_arms_row, 1])
            front_edge   = np.zeros([self.num_arms_row, 1])
            # variable that adds up times to goal to figure out the edges for the next arm
            time_tot = []

            # list of all the goals that will be put into a queue at the end
            pregoals = []

            for i in range (self.num_arms_row):
                # create a matrix with the starting location values for all three arms
                arm_location[i] = np.copy(a[rows,i].q_a)
                # create arrays for the front and back edges of each arm
                back_edge[i]  = np.copy(a[rows,i].y_edges_f[1])
                front_edge[i] = np.copy(a[rows,i].y_edges_f[0])
                # create a new list in time_tot for each new arm
                time_tot.append([0.])
                # make a list of pregoals in the row list, for each arm
                pregoals.append([])

            unpick = [] # list to make sure goals don't repeatedly get checked but are unlisted as picked later

            # get list of next X number of fruit and add them to a queue
            for pears in range(self.n_goals):
                for arm_free in range((self.num_arms_row-1), -1, -1): # as long as the arms are moving back to front picking fruits,
#                 for arm_free in range(self.num_arms_row):
                    # we need to figure out the back arm's location first?
                    if len(row_picture[rows].fruitBTree) > 0:

                        # get the location of this arm's back edge and convert to integer to get a value to compare keys
                        potential_key = math.floor(back_edge[arm_free]*1000)

                        # find a value in the B-tree that fits the first location
                        try: # if no key exists after the potential key, break out rather than crash the program
                            key = row_picture[rows].fruitBTree.minKey(potential_key)
                            index = row_picture[rows].fruitBTree[key]

                            # check that the goal/fruit does not pass the front part of the frame if frontmost arm
                            # or if the arm's are in individual work space
                            if arm_free == self.num_arms_row-1 or arm_conf == self.conf_INDIVIDUAL:
                                if fruit.sortedFruit[1,index] > front_edge[arm_free]:
                                    # if it does, stahp
                                    break
                            # choose the first fruit as a goal to be given to the arm
                            goal_new = row_picture[rows].fruitBTree.pop(key)
                            goal_coord = np.copy(fruit.sortedFruit[0:3,goal_new])
                            # calculate the time to the fruit
                            p_time = self.pickTime(arm_location[arm_free,:], goal_coord)

                            # compare with back_edge's location at that point, remove if not possible reach (okay if grabbed before vehicle passes)
                            temp_back_edge = back_edge[arm_free] + v_v[1]*p_time # maybe need to add arm speed?

                            if temp_back_edge < goal_coord[1] and fruit.sortedFruit[3,index] == 0.:  # for now only the y-coordinate
#                                 print("")
#                                 print("arm", arm_free, "in row", rows)
#                                 print("arm location", arm_location[arm_free,1], "goal y-coord",goal_coord[1], "temp back", temp_back_edge)
#                                 print("")

                                # add to list, index is given, not the coordinates
                                pregoals[arm_free].append(goal_new)
                                # calculate retraction and drop-off times
                                r_time = self.returnTime(arm_location[arm_free,:], goal_coord, a[rows,arm_free].z_edges_f[1])
                                # set newest location
                                arm_location[arm_free,1] = goal_coord[1]          # y-coord goal was last moved y location
                                arm_location[arm_free,2] = a[rows,arm_free].z_edges_f[1] # b/c of drop off, that's the new z-coordinate
                                # x location should always end at the starting point x-coord

                                if arm_conf == self.conf_INDIVIDUAL:
                                    # if the arms work in individual spaces, they are only affected by the vehicles's speed
                                    back_edge[arm_free] = back_edge[arm_free] + v_v[1]*(p_time + r_time)
                                    front_edge[arm_free] = front_edge[arm_free] + v_v[1]*(p_time + r_time)

                                elif arm_conf == self.conf_SHARED:
                                    # if the arms are sharing the row space, their edges are based on the other arm's velocity
                                    time_tot[arm_free].append([arm_location[arm_free,1], p_time + r_time])

                                    if arm_free == self.num_arms_row-1:
                                        # don't care about the front edge
                                        front_edge[arm_free] = front_edge[arm_free] + v_v[1]*(p_time + r_time)
                                        # maybe: (prolly will have to calculate it?)
                                        back_edge[arm_free]  = arm_location[arm_free-1,1]+ v_v[1]*(p_time + r_time)

                                    elif arm_free == 0:
                                        # don't care about the back edge
                                        back_edge[arm_free] = back_edge[arm_free] + v_v[1]*(p_time + r_time)
                                        # maybe: (prolly will have to calculate it?)
                                        front_edge[arm_free] = arm_location[arm_free+1,1] + v_v[1]*(p_time + r_time)

                                    else:
                                        # care about both
                                        front_edge[arm_free] = arm_location[arm_free+1,1] + v_v[1]*(p_time + r_time)
                                        back_edge[arm_free]  = arm_location[arm_free-1,1] + v_v[1]*(p_time + r_time)

                                # set the index's fruit as picked
                                fruit.sortedFruit[3,goal_new] = 1.
                                fruit.sortedFruit[4,goal_new] = arm_free
#                             else:
                            elif fruit.sortedFruit[3,goal_new] == 0:
                                fruit.sortedFruit[3,goal_new] = 1.
                                # however, populate a list of fruit that will go back to being not picked after this
                                unpick.append(goal_new)

                        except ValueError:
                            ## Error checking print statement:
                            # print("*** No value came up, broke out of the if statement ***")
#                             break
                            pass

                # unpick all the unpickable fruit... not doing very much
                for i in range(len(unpick)):
                    fruit.sortedFruit[3,i] = 0.
#                     print("Clearing some space", i)

            # we can do some post-processing here to make the list of goals better :)
#             print("GOAL LIST")
#             print(pregoals)
            # add the goals to the queue
            for arms_to_go in range(self.num_arms_row):
                for x in pregoals[arms_to_go]:
                    self.goal_queue[rows, arms_to_go].put(x)


    def pickTime(self, a_coord, goal):
        '''Calculate move and extend time for an arm to move from curr position to a fruit position'''
        x_time = self.calcTrapTimes(a_coord[0], goal[0], 0.)
        y_time = self.calcTrapTimes(a_coord[1], goal[1], 0.)
        z_time = self.calcTrapTimes(a_coord[2], goal[2], 0.)

        time_pick = x_time + max(y_time, z_time)

        return(time_pick)


    def returnTime(self, a_coord, goal, z_bottom):
        '''Calculate total retract and drop off cycle time for a fruit'''
        # maybe add grip time later?

        # calculate x, y and z times as the arm moves from curent position to the goal
        x_time = self.calcTrapTimes(goal[0], a_coord[0], 0.) # opposite :)
        z_time = self.calcTrapTimes(goal[2], z_bottom, 0.)

        time_ret = x_time + z_time

        return(time_ret)


    def calcTrapTimes(self, q_start, q_end, v_start):
        '''Get the time it takes to move in trapezoidal trajectory'''
        self.trajCalc.adjInit(q_start, v_start)
        self.trajCalc.noJerkProfile(self.trajCalc.q0, q_end, self.trajCalc.v0, self.v_max, self.a_max, self.d_max)

        total_time = self.trajCalc.Ta + self.trajCalc.Tv + self.trajCalc.Td

        return(total_time)


    def singleFruitScheduler(self, num_row, num_arms, a, row_picture, fruit, t_step):
        '''Assigns fruit one at a time to each fruit. Not really a scheduler, but useful for testing'''

        # for each row
        for rows in range(num_row):
            # check if there are any arms that are free
            for arm_free in range(num_arms):
                if a[rows,arm_free].free == 1 and len(row_picture[rows].fruitBTree) > 0:
                    # get the location of this arm's back edge and convert to integer to get a value to compare keys
                    potential_key = math.floor(a[rows,arm_free].y_edges_f[1]*1000)
                    # find a value in the B-tree that fits it's location
                    try: # if no key exists after the potential key, break out rather than crash the program
                        key = row_picture[rows].fruitBTree.minKey(potential_key)
                        index = row_picture[rows].fruitBTree[key]

                        # check that the goal does not pass the front part of the frame
                        if fruit.sortedFruit[1,index] < a[rows,arm_free].y_edges_f[0] and fruit.sortedFruit[3,index] == 0.:
#                         if fruit.sortedFruit[1,index] < a[rows,arm_free].y_edges_f[0]+1:
                            # choose the first fruit as a goal to be given to the arm
                            goal_new = row_picture[rows].fruitBTree.pop(key)
                            # set the index's fruit as scheduled to be picked
                            fruit.sortedFruit[3,goal_new] = 1.
                            # give the arm the index so that it can set it back to 0 if it does not reach it
                            a[rows,arm_free].goal_index = goal_new
                            # set the new goal
                            a[rows,arm_free].setGoal(fruit.sortedFruit[0:3,goal_new], t_step, fruit.sortedFruit[3,index])

                    except ValueError:
                        # print("*** No value came up, broke out of the if statement ***")
                        break
