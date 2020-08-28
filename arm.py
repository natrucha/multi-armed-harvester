import numpy as np

from camera import *
from trajectory import *

class arm(object):
    '''Object that contains functions to calculate the arm location and state at each time step'''

    def __init__(self, q, q_v, n, num_arms, frame_width, frame_height, column_width, row_n, max_vel, max_accel, row_conf):

        ## next level would be for them to be seperate threads with access to the global clock -Stavros

        ######################### FLAGS #########################
        self.pickFLAG        = 0     # checks if the fruit has been picked
        self.goalFLAG        = 0     # checks if there is no goal available
        self.stateFLAG       = 0

        # Arm configuration flag values set at initialization
        self.conf_INDIVIDUAL = 0
        self.conf_SHARED     = 1

        # Fruit picked flag values
        self.state_IDLE      = 0
        self.state_PICKYZ    = 1
        self.state_PICKX     = 2
        self.state_GRAB      = 3
        self.state_RETRACT   = 4
        self.state_UNLOAD    = 5

        # Goal being picked or not (for when there are no goals)
        self.goal_FALSE      = 0
        self.goal_TRUE       = 1

        # Fruit picked flag values
        self.fruit_UNPICKED  = 0
        self.fruit_PICKED    = 1
        self.fruit_MISSED    = 2
        self.fruit_NONEXIST  = 3


        #################### CLASS VARIABLES ####################
        self.n             = n                           # arm number
        self.row_n         = row_n                       # which row this arm is located in
        self.q_a           = np.array([q[0],q[1],q[2]])  # arm's location in the world's frame of reference
        self.v_a           = np.array([0.,0.,0.])        # arm's current velocity in the three axis
        self.state         = 0                           # arm's state to log % time used for each activity
        self.row_conf      = row_conf                    # sets if the row space is shared or not
        self.num_arms      = num_arms                    # total number of arms in a row

        # list of all locations where arm was
        self.qax           = []
        self.qay           = []
        self.qaz           = []

        # list of time it took to finish each picking cycle
        self.pick_cycle    = []
        self.pick_cycle_s  = 0.  # when the picking cycle timer started

        # which fruits were picked and at what time, might only need times if it's being used to determine
        # the arm's throughput
        self.time_of_picking = []

        # frame size
        self.length_f      = 1.           # how far can it go into trees, in m
        self.width_f       = frame_width  # size of enclosing frame, width in m -> when shared this includes the shared space
        self.height_f      = frame_height # size of enclosing frame, height in m
        self.width_c       = column_width # width of the columns supporting the arms

        if self.row_conf == self.conf_SHARED:
            self.width_f  = frame_width*self.num_arms # if it's shared space the width is much larger and based on how many arms

        # frame center coordinates
        self.q_f           = np.array([q[0],q_v[1],q[2]])  # frame's center at row and vehicle's center

        if self.row_conf == self.conf_INDIVIDUAL:
            self.q_f[1]    = q[1]  # frame's center moves to be individual, rather than the vehicle's center

        # edges of the frame
        self.x_edges_f     = np.array([0.,0.]) # 0 is the starting position, 1 is max extension
        self.y_edges_f     = np.array([0.,0.]) # 0 is forward, 1 is in the back
        self.z_edges_f     = np.array([0.,0.]) # 0 is the top, 1 is the bottom

        if self.row_conf == self.conf_SHARED:
            self.y_edge_end = np.array([0.,0.])

        # values for trapezoid trajectory (parameters to play with...)
        self.v_max         = max_vel
        self.a_max         = max_accel
        self.d_max         = max_accel

        # can the arm be given a goal, or does it already have one
        self.free          = 1     # setting that determines when a new goal can be given, used by outside functions
        self.goal          = np.array([0.,0.,0.])
        self.reached_goals = 0     # how many goals were successfully reached
        self.goal_index    = 0     # what index did the goal come from
        self.goal_list     = []    # list the index of each goal reached (check if goals were repeated somewhere)
        self.goals_given   = 0     # how many goals were given to the arm (to check against how many it reached)
        self.numMissed     = 0     # will follow how many times one goal is missed and cancels that goal once too many misses have happened

        # following the trapezoidal trajectory
        self.t             = 0.      # time at which the arm will begin moving to the goal
        self.t_grab        = 0.


        ######################## MODULES ########################
        self.x = Trajectory(self.v_max, self.a_max, self.d_max)
        self.y = Trajectory(self.v_max, self.a_max, self.d_max)
        self.z = Trajectory(self.v_max, self.a_max, self.d_max)

        ################ SELF FUNCTIONS AT INIT #################
        self.initFrame()


    ######################## FUNCTIONS ########################
    def armStep(self, v_v, a, dt, t, fruit, row_picture):
        '''Funtion detailing and following the decision tree for each arm based on the arm state.

           INPUTS: vehicle velocity, arm object, global time step, and the global time

           OUTPUTS: returns the new location calculated for the arm as a list with three items.'''

        q_curr     = np.array([self.q_a[0],self.q_a[1],self.q_a[2]]) # actually q_{t-1}
        goal_r     = np.array([0,0,0]) # has the goal been rached at each axis

        epsilon    = 0.001             # allowable error when reaching fruit
        t2Grab     = 0.2               # amount of time it takes to grab a fruit during GRABBING
        can_miss   = 1                 # number of allowed misses before looking for new goal

        goal_time  = t - self.t        # time since the x, y, or z self timer was started

        # move frame to match vehicle's location
        self.moveFrame(v_v, a, dt)

        # defaul to move with vehicle, velocities will be changed depending on state
        self.v_a[0] = 0.
        self.v_a[1] = 0.
        self.v_a[2] = 0.

        if self.goalFLAG == self.goal_FALSE:
            # When the arm is waiting for a new goal
            self.stateFLAG   = self.state_IDLE

            self.calcLocation(q_curr, v_v, dt)

        elif self.goalFLAG == self.goal_TRUE:
            # when the arm has a goal
            # check if fruit can still be reached according to frame's location
            can_reach = self.reachable(fruit)

            if (not can_reach) and (self.stateFLAG == self.state_PICKX or self.stateFLAG == self.state_GRAB):
                # need to retract back to the start position before resetting
                self.stateFLAG = self.state_RETRACT
                self.retract(t)
                goal_time = 0.

            if (not can_reach) and self.stateFLAG == self.state_PICKYZ:
                self.resetFlags()
                # remove the picking flag from the fruit scheduler so it can be scheduled again later
                fruit.sortedFruit[3,self.goal_index] = 0.
                # print("Reseting goal index, can't reach", self.goal_index)

            # start going through the states
            if self.stateFLAG == self.state_PICKYZ:
                # when moving to in the y, z-coordinates and the goal can be reached
                goal_r[1] = self.calcYVel(goal_time, v_v)
                goal_r[2] = self.calcZVel(goal_time)      # not affected by the vehicle's velocity

                if goal_r[1] == 1 and goal_r[2] == 1:
                    # check if the arm's location is at the goal +/- epsilon
                    check = self.accCheckYZ()

                    if check == 2:
                        # if both y and z goal locations successfully reached, start moving in the x-dir to grab the fruit
                        self.t    = t           # start time for the new timer for x
                        goal_time = 0.          # restart the timer for x
                        # move to the next state
                        self.stateFLAG = self.state_PICKX

                    elif check < 2 and self.numMissed < can_miss:
                        # at least one coordinate was not successful
                        self.stateFLAG  = self.state_PICKYZ
                        self.numMissed += 1
                        # recalculate movement to the fruit
                        self.startTrap(t)

                    elif check < 2 and self.numMissed > can_miss:
                        # too many missed attempts, reset flags and find new goal
                        self.resetFlags()
                        # remove the picking flag from the fruit scheduler so it can be scheduled again later
                        fruit.sortedFruit[3,self.goal_index] = 0.
                        # print("Reseting goal index, pickyz", self.goal_index)

                self.calcLocation(q_curr, v_v, dt)

            elif self.stateFLAG == self.state_PICKX:
                # when extending out to the fruit in the x-coordinates and the goal can be reached
                goal_r[0] = self.calcXVel(goal_time, v_v)
                # the velocity for the y and z coordinates should be stationary with respect to the world frame
                self.v_a[1] = -v_v[1]

                if goal_r[0] == 1:
                    # once again check that the arm is within the goal's location +/- epsilon
                    check = self.accCheckXYZ(fruit)

                    if check == 3:
                        # if all 3 goal coordinates successfully reached move to grabbing the fruit
                        self.stateFLAG = self.state_GRAB
                        self.t_grab = 0.         # amount of time spent grabbing fruit, increases each step in GRAB state

                    elif check < 3 and self.numMissed < can_miss:
                        self.pickFLAG   = self.fruit_MISSED
                        self.stateFLAG  = self.state_RETRACT
                        self.numMissed += 1
                        # will need to retract to recalculate positioning
                        self.retract(t)
                        goal_time       = 0.

                    elif check < 3 and self.numMissed > can_miss:
                        # too many missed attempts, reset flags and find new goal
                        self.resetFlags()
                        # remove the picking flag from the fruit scheduler so it can be scheduled again later
                        fruit.sortedFruit[3,self.goal_index] = 0.
                        # print("Reseting goal index, pickx", self.goal_index)

                self.calcLocation(q_curr, v_v, dt)


            elif self.stateFLAG == self.state_GRAB:# and can_reach:
                # take some amount of time as machine "picks" the fruit
                self.t_grab += dt

                # arm should be stationary at the fruit for the grab amount of time
                self.v_a[0] = -v_v[0]
                self.v_a[1] = -v_v[1]

                if self.t_grab >= t2Grab:
                    # need to check that the arm is in the correct location
                    check = self.accCheckXYZ(fruit)


                    if check == 3:
                        # successful harvest and the fruit has been grabbed
                        self.pickFLAG  = self.fruit_PICKED
                        self.stateFLAG = self.state_RETRACT
                        # init retracting variables
                        self.retract(t)
                        # reset timer start for x
                        goal_time = 0.
                        # add to reached goals and remove fruit from schedulable set
                        self.reached_goals += 1
                        self.goal_list.append(self.goal_index)
                        fruit.sortedFruit[3,self.goal_index] = 2. # set environment fruit as picked
                        # "pick" the fruit and remove it from the vision system's b-tree
                        row_picture[self.row_n].fruitPicked(self.q_a, t)
                        # save the time of picking
                        self.time_of_picking.append(float(t))

                    elif check < 3 and self.numMissed < can_miss:
                        self.pickFLAG   = self.fruit_MISSED
                        self.stateFLAG  = self.state_RETRACT
                        self.numMissed += 1
                        # will need to retract to recalculate positioning
                        self.retract(t)
                        goal_time       = 0.

                    elif check < 3 and self.numMissed > can_miss:
                        # too many missed attempts, reset flags and find new goal
                        self.resetFlags()
                        # remove the picking flag from the fruit scheduler so it can be scheduled again later
                        fruit.sortedFruit[3,self.goal_index] = 0.
                        # print("Reseting goal index, grab", self.goal_index)

                self.calcLocation(q_curr, v_v, dt)


            elif self.stateFLAG == self.state_RETRACT:
                # state where the arm moves back until it reaches the starting x-coordinate, functions to reset
                # the x-coordinate when fruit is picked or the fruit becomes unreachable
                retracted = self.calcXVel(goal_time, v_v)
                # the velocity for the y and z coordinates should be stable
                self.v_a[1] = -v_v[1]   # NOTE: this might be problematic because it takes up more vehicle "space"

                if retracted and self.pickFLAG == self.fruit_PICKED:
                    # unloading will only happen if the fruit was successfully picked
                    self.stateFLAG = self.state_UNLOAD
                    # init unloading variables
                    self.unload(t)
                    goal_time = 0.

                elif retracted and self.pickFLAG == self.fruit_MISSED and self.numMissed < can_miss:
                    # If the fruit has been missed, recaulculate the trap times to get to the correct location
                    self.startTrap(t)
                    # now that it's back to the "start" and times have been recalculated, go back to picking in y and z
                    self.pickFLAG   = self.fruit_UNPICKED
                    self.stateFLAG  = self.state_PICKYZ

                elif retracted and ((not can_reach) or (self.pickFLAG == self.fruit_MISSED and self.numMissed > can_miss)):
                    # If the fruit cannot be reached, reset all the flags to get a new goal
                    self.resetFlags()
                    # remove the picking flag from the fruit scheduler so it can be scheduled again later
                    fruit.sortedFruit[3,self.goal_index] = 0.

                self.calcLocation(q_curr, v_v, dt)

            elif self.stateFLAG == self.state_UNLOAD:
                # state where the arm down to the bottom of the frame to "unload" the fruit
                # happens when the fruit was successfully obtained
                unloaded = self.calcZVel(goal_time)

                self.calcLocation(q_curr, v_v, dt)

                if unloaded:
                    self.resetFlags()
                    # save how long it took to finish this picking cycle
                    pick_cycle_e = t - self.pick_cycle_s
                    self.pick_cycle.append(pick_cycle_e)
#                     if self.n == 0 and self.row_n == 0:
#                         print("")
#                         print("PICK CYCLE END")
#                         print("Goal Index:", self.goal_index)
#                         print("Has the fruit been picked?", fruit.sortedFruit[3,self.goal_index])
#                         print("")

            else:
                # maybe an error, though usually it's that the arm is idle
                # print("*** Is it just idle? ***")
                # print("*** BAD FLAG:", self.stateFLAG, " ***")

                # keep in place?
                self.v_a[0] = -v_v[0]
                self.v_a[1] = -v_v[1]

                self.calcLocation(q_curr, v_v, dt)

        # if self.n == 0 and self.row_n == 3:
        #     print("")
        #     print("Row:", self.row_n, "Arm:", self.n)
        #     print("Edges, X:", self.x_edges_f, "Y:", self.y_edges_f, "Z:", self.z_edges_f)
        #     print("Goal: {0:.4f}".format(self.goal[0]), " {0:.4f}".format(self.goal[1]), " {0:.4f}".format(self.goal[2]))
        #     print("Current location: {0:.4f}".format(self.q_a[0]), " {0:.4f}".format(self.q_a[1]), " {0:.4f}".format(self.q_a[2]))

#             print("")
#             print("Is the arm free?", self.free)
#             print("Has the fruit been picked?", fruit.sortedFruit[3,self.goal_index])
#             print("Goals reached:", goal_r)
#             print("Arm state:", self.stateFLAG)
#             print("Where 0:IDLE, 1:PICKYZ, 2:PICKX, 3:GRAB, 4:RETRACT, 5:UNLOAD")
#             print("can it reach?", can_reach)
#             print("")

        return self.q_a


    def calcLocation(self, q_curr, v_v, dt):
        '''
           Function that calculates the arm location based on the calculated arm velocity, vehicle velocity,
           and q_{t-1} for each of the tree axis.

           INPUTS: q_curr which is q_{t-1}, the vehicle velocity and the global time step size.
        '''
        # calculate the new location based on calculated velocities
        self.q_a[0] = q_curr[0] + self.v_a[0]*dt + v_v[0]*dt # coordinate affected by the vehicle's speed (need to change to reflect the axis)
        self.q_a[1] = q_curr[1] + self.v_a[1]*dt + v_v[1]*dt # coordinate affected by the vehicle's speed
        self.q_a[2] = q_curr[2] + self.v_a[2]*dt

        # fix any edge constraints of the arms trying to go past their frame/other arms
        self.edgeConstraintCheck()
        # update the location history (for plotting :) )
        self.qax.append(float(self.q_a[0]))
        self.qay.append(float(self.q_a[1]))
        self.qaz.append(float(self.q_a[2]))


    def pickData(self):
        '''
           Function to compile data of when fruit are picked. Still unfinished. Used in plotting.
           Only compiling time data at the moment.
        '''
#         x_fr = np.array(self.x)
#         y_fr = np.array(self.y)
#         z_fr = np.array(self.z)
        time_pick = np.array(self.time_of_picking)

        # need a matrix to sort x, y, and z based on the y-axis (to know what fruit show up earlier)
#         inv_fruit = np.stack([x_fr, y_fr, z_fr, t])

        return time_pick  # until I know that the coordinates are needed, just use this


    def resetFlags(self):
        '''
           Resets all the flags so as to restart the process of getting a goal.
        '''
        # end of the picking cycle, reset the state, goal, and fruit flags
        self.stateFLAG    = self.state_IDLE
        self.goalFLAG     = self.goal_FALSE
        self.pickFLAG     = self.fruit_UNPICKED
        # set arm as free so that the scheduler can see it
        self.free         = 1


    def unload(self, t):
        '''
           Calculates the trajectory for the arm from curr position to bottom of frame (conveyor location).
           should only run if fruit has been obtained and the arm has retracted
        '''
        self.z.adjInit(self.q_a[2], self.v_a[2])
        self.z.noJerkProfile(self.q_a[2], self.z_edges_f[1], self.z.v0, self.v_max, self.a_max, self.d_max)
        # restart the timer
        self.t  = t


    def calcZVel(self, goal_time):
        '''
           Calculates the arm velocity in the z-coordinate based on the trapezoidal times calculated beforhand.
           Goes through three stages, accel, constant v, deaccel and then a stopped version.

           INPUTS:  count up "global" timer to determine when to switch from each section and the velocity of the vehicle
                    as inputs.

           OUTPUTS: 0 as it runs, 1 when the total time has passed to show it finished
        '''
        out = 0

        if goal_time <= self.z.Ta:
            self.v_a[2] = self.z.v0 + self.z.ar*goal_time

        elif goal_time <= self.z.Ta + self.z.Tv:
            self.v_a[2] = self.z.vr

        elif goal_time <= self.z.Ta + self.z.Tv + self.z.Td:
            self.v_a[2] = self.z.vr - self.z.dr*(goal_time - (self.z.Ta + self.z.Tv))
        else:
            self.v_a[2] = 0.
            out = 1

        return out


    def calcYVel(self, goal_time, v_v):
        '''
           Calculates the arm velocity in the y-coordinate based on the trapezoidal times calculated beforhand.
           Goes through three stages, accel, constant v, deaccel and then a stopped version.

           INPUTS:  count up "global" timer to determine when to switch from each section and the velocity of the vehicle
                    as inputs.

           OUTPUTS: 0 as it runs, 1 when the total time has passed to show it finished
        '''
        out = 0

        if goal_time <= self.y.Ta:
            self.v_a[1] = self.y.v0 + self.y.ar*goal_time - v_v[1]

        elif goal_time <= self.y.Ta + self.y.Tv:
            self.v_a[1] = self.y.vr  - v_v[1]

        elif goal_time <= self.y.Ta + self.y.Tv + self.y.Td:
            self.v_a[1] = self.y.vr - self.y.dr*(goal_time - (self.y.Ta + self.y.Tv))  - v_v[1]

        else:
            self.v_a[1] = -v_v[1] # cancel the vehicle's motion while waiting to grab the fruit
            out = 1

        return out


    def calcXVel(self, goal_time_x, v_v):
        '''
           Calculates the arm velocity in the x-coordinate based on the trapezoidal times calculated beforhand.
           Goes through three stages, accel, constant v, deaccel and then a stopped version.

           INPUTS:  count up "global" timer to determine when to switch from each section and the velocity of the vehicle
                    as inputs.

           OUTPUTS: 0 as it runs, 1 when the total time has passed to show it finished
        '''
        out = 0

        if goal_time_x <= self.x.Ta:
            self.v_a[0] = self.x.v0 + self.x.ar*goal_time_x - v_v[0]

        elif goal_time_x <= self.x.Ta + self.x.Tv:
            self.v_a[0] = self.x.vr - v_v[0]

        elif goal_time_x <= self.x.Ta + self.x.Tv + self.x.Td:
            self.v_a[0] = self.x.vr - self.x.dr*(goal_time_x - (self.x.Ta + self.x.Tv)) - v_v[0]

        else:
            self.v_a[0] = -v_v[0] # cancel the vehicle's motion while waiting to grab the fruit
            out = 1

        return out


    def calcXYZTrap(self):
        '''
           Calculates the trap times for all three coordinates
        '''
        self.x.adjInit(self.q_a[0], 0)
        self.x.noJerkProfile(self.q_a[0], self.q_f[0], self.x.v0, self.v_max, self.a_max, self.d_max)

        self.y.adjInit(self.q_a[1], 0)
        self.y.noJerkProfile(self.q_a[1], self.q_f[1], self.y.v0, self.v_max, self.a_max, self.d_max)

        self.z.adjInit(self.q_a[2], 0)
        self.z.noJerkProfile(self.q_a[2], self.q_f[2], self.z.v0, self.v_max, self.a_max, self.d_max)


    def retract(self, t):
        '''
           Begins the process of retracting by calculating the required movement times in the x-coordinate
           and resetting the movement timer.
        '''
        # retracts the arm
        self.x.adjInit(self.q_a[0], 0)
        self.x.noJerkProfile(self.q_a[0], self.q_f[0], self.x.v0, self.v_max, self.a_max, self.d_max)
        # restart the timer
        self.t = t
#         if self.q_a[0] < self.q_f[0]-0.001:
#             print(" ")
#             print("RETRACTING")
#             print("ARM:", self.n, "ROW:", self.row_n)
#             print("X-axis, Ta:", self.x.Ta, "Tv:", self.x.Tv, "Td:", self.x.Td)
#             print(" ")
#             print("goal:         {0:.4f}".format(self.goal[0]), " {0:.4f}".format(self.goal[1]), " {0:.4f}".format(self.goal[2]))


    def reachable(self, fruit):
        '''
           Checks if a fruit is still reachable by checking if the fruit has passed the back edge of the fruit
           workspace or is beyond the extension range. Returns a 1 if the fruit remains reachable, 0 if not.
           Also used to reset the value of the fruit if it has not been picked.
        '''
        if self.goal[0] < self.x_edges_f[1] and self.goal[1] > self.y_edges_f[1]:
            # check if the goal is behind the back edges of the frame or over the extension range of the arm
            return 1

        else:
            if self.pickFLAG == self.fruit_UNPICKED: # and fruit.sortedFruit[3,self.goal_index] != 2.:
                # if it's not reachable before it reached it's goal, reset the scheduler's state on that fruit:
                # It's not scheduled anymore.
                fruit.sortedFruit[3,self.goal_index] = 0.
#                 print("Fruit skipped")

            return 0


    def accCheckXYZ(self, fruit):
        '''
           Adds up the number of coordinates where the arm is within the goal location +/- epsilon.
           Returns the number of successes, up to 3
        '''
        x = self.accuracyCheck(self.q_a[0],self.goal[0])
        y = self.accuracyCheck(self.q_a[1],self.goal[1])
        z = self.accuracyCheck(self.q_a[2],self.goal[2])
        # check that the fruit has not been picked
        if fruit.sortedFruit[3,self.goal_index] != 1:
            self.numMissed = 10
            x = 0
            y = 0
            z = 0

        return x + y + z


    def accCheckYZ(self):
        '''
           Adds up the number of coordinates, but only for Y, Z, where the arm
           is within the goal location +/- epsilon. Returns the number of successes, up to 2
        '''
        y = self.accuracyCheck(self.q_a[1],self.goal[1])
        z = self.accuracyCheck(self.q_a[2],self.goal[2])
        return y + z


    def accuracyCheck(self, arm, goal):
        '''
           Checks if the arm's location is located near enough to the given goal to pick the fruit.
           Based on an allowable error set as epsilon. Returns a zero if too far, one if successful.
        '''
        epsilon = 0.001 # allowable error

        if arm > (goal-epsilon) and arm < (goal+epsilon):
            # if successful in reaching the correct coordinates
            return 1
        else:
            return 0


    def edgeConstraintCheck(self):
        '''
           Stops the arm from moving beyond its workspace. If a constraint is ignored, the function will
           change the arm location to be within the bounds, causing error in location but avoiding
           collisions
        '''
        # stop the arm from extending backwards beyond the back of the frame
        if self.q_a[0] < self.q_f[0]: # totally retracted position
            self.q_a[0] = self.q_f[0]

        # stop the arm from moving below or above the bottom of the frame (saw it happen)
        if self.q_a[2] < self.z_edges_f[1]: # bottom
            self.q_a[2] = self.z_edges_f[1]

        elif self.q_a[2] > self.z_edges_f[0]: # top
            self.q_a[2] = self.z_edges_f[0]

        # stop the arm from moving front or back beyond it's y-edges
        if self.q_a[1] < self.y_edges_f[1]: # back
            self.q_a[1] = self.y_edges_f[1]

        elif self.q_a[1] > self.y_edges_f[0]: # front
            self.q_a[1] = self.y_edges_f[0]


    def setGoal(self, goal, t, check):
        '''
           Function to give arm a new goal. Initializes movement for each axis and calculates the required
           trapezoidal times to go from a current location to the goal.

           INPUTS: goal coordinates in x, y, and z and the current global time.
        '''
#         if self.n == 0 and self.row_n == 0:
#             print("")
#             print("GOAL GIVEN")
#             print("Goal Index:", self.goal_index)
#             print("Has the fruit been picked?", fruit.sortedFruit[3,self.goal_index])
#             print("")
        if check == 1.:
            self.free     = 0              # global setting that makes arm's busy state visible to other functions
            self.goal     = goal
            self.goalFLAG = self.goal_TRUE

            self.startTrap(t)
            # increase goals given counter
            self.goals_given += 1
            # start picking cycle time measurement here
            self.pick_cycle_s = t


    def startTrap(self, t):
        '''
           Function which initializes and performs the trapezoidal time calculations.

           INPUTS: global time
        '''
        # calculate the trapezoidal times for each of the three movement stages (change to S-curve later)
        # init each axis with current location and velocity (will need velocity at some point?)
        self.x.adjInit(self.q_a[0], 0.)
        self.y.adjInit(self.q_a[1], 0.)
        self.z.adjInit(self.q_a[2], 0.)

        # get trapezoidal times
        self.x.noJerkProfile(self.x.q0, self.goal[0], self.x.v0, self.v_max, self.a_max, self.d_max)
        self.y.noJerkProfile(self.y.q0, self.goal[1], self.y.v0, self.v_max, self.a_max, self.d_max)
        self.z.noJerkProfile(self.z.q0, self.goal[2], self.z.v0, self.v_max, self.a_max, self.d_max)

#         print(" ")
#         print("X-axis, Ta:", self.x.Ta, "Tv:", self.x.Tv, "Td:", self.x.Td)
#         print("Y-axis, Ta:", self.y.Ta, "Tv:", self.y.Tv, "Td:", self.y.Td)
#         print("Z-axis, Ta:", self.z.Ta, "Tv:", self.z.Tv, "Td:", self.z.Td)
#         print(" ")
#         print("reached values, X:", self.x.vr, self.x.ar, self.x.dr)
#         print("reached values, Y:", self.y.vr, self.y.ar, self.y.dr)
#         print("reached values, Z:", self.z.vr, self.z.ar, self.z.dr)
#         print(" ")

        # reset state as picking in the y and z axis to start the picking cycle
        self.stateFLAG = self.state_PICKYZ
        # maybe start time here? => evaluate how that could change things once in real time
        self.t     = t


    def moveFrame(self, v_v, a, dt):
        '''
           Function which calculates the frame location based on the vehicle's speed. Used to
           set the limits of the frames for each arm.

           INPUTS: vehicle speed, the arm object to know the location of all the arms in case
                   the row is in the shared configuration, and the global time step size
        '''
        # x-values are not affected by the other arms (yet?)
        x_coor       = self.q_f[0]           # center position of the arm in x-axis
        self.q_f[0]  = x_coor + v_v[0]*dt    # move the center point as the vehicle moves (if it does in x-dir)

        retract_edge = self.x_edges_f[0]     # at frame
        extend_edge  = self.x_edges_f[1]     # at furthest extension

        # move x-dir the edges if vehicle has x-axis velocity
        self.x_edges_f[0] = retract_edge + v_v[0]*dt
        self.x_edges_f[1] = extend_edge + v_v[0]*dt

        # calculate the changes in y
        if self.row_conf == self.conf_INDIVIDUAL:

            y_coor      = self.q_f[1]        # center of the y-direction in the frame
            self.q_f[1] = y_coor + v_v[1]*dt # move the center point as the vehicle moves

            # save the old location values of the edges, don't have to add column width cause it was added at the frame init
            back_edge   = self.y_edges_f[1]
            front_edge  = self.y_edges_f[0]

        elif self.row_conf == self.conf_SHARED:
            # set the value for the edges
            # got a version of *arm_obj* that has the other arms in only that row

            self.y_edge_end += v_v[1]*dt # move the ends with the vehicle

            if self.n == 0:
                # backmost arm doesn't deal with arms behind it, so the edge is the frame minus half the column width
                back_edge   = self.y_edges_f[1]
                front_edge  = a[self.n+1].q_a[1] - self.width_c + a[self.n+1].v_a[1]*dt # next arm forwards' location

            elif self.n == self.num_arms-1:
                # frontmost arm doesn't deal with arms in front of it
                back_edge   = a[self.n-1].q_a[1] + self.width_c + a[self.n-1].v_a[1]*dt
                front_edge  = self.y_edges_f[0]

            else:
                back_edge   = a[self.n-1].q_a[1] + self.width_c + a[self.n-1].v_a[1]*dt
                front_edge  = a[self.n+1].q_a[1] - self.width_c + a[self.n+1].v_a[1]*dt

            # check that it never goes into space at the edges that would be taken up by the other arms
            if front_edge > self.y_edge_end[0]:
                front_edge = self.y_edge_end[0]

            if back_edge < self.y_edge_end[1]:
                back_edge = self.y_edge_end[1]

        # move the edges knowing the vehicle and other arms' velocities, the last bit calculate ahead of time
        self.y_edges_f[0] = front_edge + v_v[1]*dt
        self.y_edges_f[1] = back_edge + v_v[1]*dt

        ############ NOTE: ARM Y-EDGES ARE FINE (ADD column_width), STANDARDIZE THE FRAME EDGES ############
        # print("Arm ", self.n, "back edge {0:.2f}".format(self.y_edges_f[1]), "front edge {0:.2f}".format(self.y_edges_f[0]))
        # print("Arm location{0:.2f}".format(self.q_a[1]))
        # print("Frame based on arm ", self.n, "back edge {0:.2f}".format(self.y_edge_end[1]), "front edge {0:.2f}".format(self.y_edge_end[0]))


    def initFrame(self):
        '''
           Calculate the initial location of the edges of the frame based on the vehicle starting position.
           Only runs once during the initialization of the arm object.
        '''
        # the x-dir edge is more of a limit on the extension capabilities of the arm. so it's not symetrical
        self.x_edges_f[0] = self.q_f[0]
        self.x_edges_f[1] = self.q_f[0] + self.length_f

        # the height will stay constant throughout
        self.z_edges_f[0] = self.q_f[2] + self.height_f / 2
        self.z_edges_f[1] = self.q_f[2] - self.height_f / 2

        # y-dir edges change according to the row space configuration
        if self.row_conf == self.conf_INDIVIDUAL: # the y-dir edge make up individual rectangles
            # front
            self.y_edges_f[0] = self.q_f[1] + self.width_f / 2 - self.width_c/2
            # back
            self.y_edges_f[1] = self.q_f[1] - self.width_f / 2 + self.width_c/2

        if self.row_conf == self.conf_SHARED:
            # the y-dir edge are initialized as the whole row minus some space for the other arms
            # front
            self.y_edges_f[0] = self.q_f[1] + self.width_f / 2 - (self.num_arms - self.n+1)*self.width_c
            # back
            self.y_edges_f[1] = self.q_f[1] - self.width_f / 2 + self.n*self.width_c
            # saves the edges of the frame, not individual arms
            self.y_edge_end = np.copy(self.y_edges_f)
