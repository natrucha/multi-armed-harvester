from BTrees.IIBTree import IIBTree # documentation, https://pythonhosted.org/BTrees/
import numpy as np
import math

from trajectory import *           # import the trajectory generation "library" that I'm writing

class camera(object):
    def __init__(self, length_v, n_row, frame_height):

        #################### CLASS VARIABLES ####################
        self.width = length_v  # width of camera view, in m
        self.i_lastFruit = 0   # last visited fruit index

        # lists of coordinates of picked fruits
        self.x      = []
        self.y      = []
        self.z      = []
        self.time   = []        # time when fruit was picked

        # Settings for the row
        self.row_n  = n_row     # row number, in one sided-version, lower number means lower on the vehicle
        self.height = frame_height # how high of fruit can this row see
        self.row_mid = (self.row_n)*self.height + self.height/2 # z-coordinate of the middle of the arm row

        # calculate the up-down edges of the camera once
        self.view_bot = self.row_mid - self.height / 2 # up-down edges
        self.view_top = self.row_mid + self.height / 2

#         print("top:", self.view_top, "bottom:", self.view_bot)

        # side edges of the camera
        self.view_min = 0.
        self.view_max = 0.


        ######################## MODULES ########################
        self.fruitBTree    = IIBTree()



    ######################## FUNCTIONS ########################
    def cameraStep(self, end_row, q_curr, sortedFruit):
        # centered (for now) on the vehicle, perpendicular to data's y-axis
        self.view_min = q_curr - self.width / 2 # side edges
        self.view_max = q_curr + self.width / 2

        # fix side end constraints of beginning and end
        if self.view_min < 0:
            self.view_min = 0 # will need to change this to a beginning value like below

        # find where the picking flag array is stating there are unpicked fruit
        unpicked_indexes = np.where(sortedFruit[3,:] == 0) # should be a 1D array of indexes

        for f_index in unpicked_indexes[0]:
            try:
                if sortedFruit[1,f_index] > self.view_min and sortedFruit[1,f_index] < self.view_max:
                    # now check vertical edges
                    if sortedFruit[2,f_index] > self.view_bot and sortedFruit[2,f_index] < self.view_top:
                        # now check if it has been picked before
                        if sortedFruit[3,f_index] == 0.:
                            # convert the y-location into an integer so it can be used as a key, ceiling because it's at the edge
                            # so it can't really be used either way as a goal
                            key = math.ceil(sortedFruit[1,f_index]*1000)
                            value = int(f_index) # have to go from int64 (from array) to a base int
                            # add the value to the b-tree
                            self.fruitBTree.update({key:value})

                elif sortedFruit[1,f_index] > self.view_max:
                    # no fruit left to add
                    break
#                 j = j + 1
            except IndexError:
                print("*** Finished going through available fruit ***")
                break


    def fruitPicked(self, pointCoord, t):
        '''Saves the coordinates and time at which a fruit was picked for plotting'''
        self.x.append(float(pointCoord[0]))
        self.y.append(float(pointCoord[1]))
        self.z.append(float(pointCoord[2]))
        self.time.append(float(t))


    def packFruit(self):
        x_fr = np.array(self.x)
        y_fr = np.array(self.y)
        z_fr = np.array(self.z)
        t    = np.array(self.time)

        # need a matrix to sort x, y, and z based on the y-axis (to know what fruit show up earlier)
        inv_fruit = np.stack([x_fr, y_fr, z_fr, t])

        return inv_fruit
