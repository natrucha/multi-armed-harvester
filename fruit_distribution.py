import csv                      # read and write CSV file-type
import numpy as np
from numpy.random import PCG64

import math

from trajectory import *          # import the trajectory time calc (bang-bang)


class fruitDistribution(object):
    def __init__(self, x_lim, y_lim, z_lim):
        '''
            Module to create the various fruit distributions used in testing the orchard fruit harvester
        '''
        ## Environmental variables
        self.len_x = x_lim[1] - x_lim[0]            
        self.len_y = y_lim[1] - y_lim[0]  
        self.len_z = z_lim[1] - z_lim[0]

        self.x_lim = x_lim
        self.y_lim = y_lim
        self.z_lim = z_lim


    def sortNstack(self, x, y, z):
        '''
            Sorts x, y, and z fruit coordinate arrays based on the y-coordinates and stacks them into a matrix
        '''
        # need a matrix to sort x, y, and z based on the y-axis (to know what fruit show up earlier)
        fruit = np.stack([x, y, z])

        axis_to_sort = np.argsort(y) # sort based on y-axis
        sortedFruit = fruit[:,axis_to_sort]

        return(sortedFruit)


    def csvFile(self, file_name, is_meter):
        '''
            Import fruit coordinates from given CSV file, where each fruit is a row, columns are x, y, z
            coordinates. 

            file_name has to be string with '...' 

            is_meter = 0 means ft, 1 is meter (other values for other measurements?).
        '''
        x_list = list()
        y_list = list()
        z_list = list()

        numFruit = 0

        with open(file_name) as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
            for row in reader:
                # need to check that x, y, z, coordinates match my definition of x, y, z -> apples 2015 do
                x_list.append(row[0])
                y_list.append(row[1])
                z_list.append(row[2])

                numFruit += 1

        # convert list to array
        x = np.array(x_list)
        y = np.array(y_list)
        z = np.array(z_list) 

        if is_meter == 0:
            # if is_meter is zero, convert from feet to meters
            x = x * 0.3048
            y = y * 0.3048
            z = z * 0.3048

        # check if need to translate fruit in to get it to correct frame if vehicle is at 0 m and fruit starts...
        # something like 0.2 m away in the x-direction
        x_translate = np.amin(x) # in m
        print('x smallest value, in m', x_translate)
        x = x - x_translate + 0.2
        # something like 0 m long in the y-direction
        y_translate = np.amin(y)
        y = y - y_translate
        # something like 0 m high in the z-direction
        z_translate = np.amin(z)
        z = z - z_translate

        # remove any fruit that is outside of the robot's max limits
        index_out_of_x_bounds = np.where(x >= self.x_lim[1])
        index_out_of_y_bounds = np.where(y >= self.y_lim[1])
        index_out_of_z_bounds = np.where(z >= self.z_lim[1])
        # print('out of z bounds (before):')
        # print(index_out_of_z_bounds[0])

        out_of_bounds = np.concatenate((index_out_of_x_bounds[0], index_out_of_y_bounds[0], index_out_of_z_bounds[0]), axis=None)
        u = np.unique(out_of_bounds) # make sure there are no dulplicates, see https://numpy.org/doc/stable/reference/generated/numpy.unique.html
        # print('indexes to be deleted:',u, 'with size', len(u))
        x = np.delete(x, u)
        y = np.delete(y, u)
        z = np.delete(z, u)
        # ## check if it worked
        # # remove any fruit that is outside of the robot's max limits
        # index_out_of_x_bounds = np.where(x >= self.x_lim[1])
        # index_out_of_y_bounds = np.where(y >= self.y_lim[1])
        # index_out_of_z_bounds = np.where(z >= self.z_lim[1])
        # print('out of z bounds (after):')
        # print(index_out_of_z_bounds[0])

        sortedFruit = self.sortNstack(x, y, z)

        return([numFruit, sortedFruit])        


    def uniformRandom(self, density, x_seed, y_seed, z_seed):
        '''
           Fruit distribution set uniform random along total x, y, and z given x, y, and z limits, seeds, 
           and desired density
        '''
        numFruit = int(density * (self.len_y*self.len_x*self.len_z))  

        x = np.random.default_rng(x_seed).uniform(self.x_lim[0], self.x_lim[1], numFruit)
        y = np.random.default_rng(y_seed).uniform(self.y_lim[0], self.y_lim[1], numFruit)
        z = np.random.default_rng(z_seed).uniform(self.z_lim[0], self.z_lim[1], numFruit)

        # # need a matrix to sort x, y, and z based on the y-axis (to know what fruit show up earlier)
        # fruit = np.stack([x, y, z])

        # axis_to_sort = np.argsort(y) # sort based on y-axis
        # sortedFruit = fruit[:,axis_to_sort]

        sortedFruit = self.sortNstack(x, y, z)

        # return numFruit and the sortedFruit
        return([numFruit, sortedFruit])


    def equalCellDensity(self, n_row, n_arm, cell_h, cell_l, arm_reach, fruit_in_cell, x_seed, y_seed, z_seed):
        '''
           Fruit distribution set uniform random within each cell, making sure that the density in every 
           cell is the same. Fruit coordinates in all three axis are randomly chosen. 
        '''
        # calculate the density of fruit within every cell
        
        cell_density = fruit_in_cell / (cell_h*cell_l*arm_reach)
        print('cell density:', cell_density, 'num of fruit in each cell', fruit_in_cell)
        print()

        total_cells = n_row * n_arm

        num_snapshots = int(self.len_y / (cell_l * n_arm)) + 1 # won't get the total otherwise...
        # print('total snapshots to get to the end of the orchard row:', num_snapshots)

        numFruit = fruit_in_cell*total_cells * num_snapshots
        # print('number of fruit in front of vehicle', numFruit)

        x = np.random.default_rng(x_seed).uniform(self.x_lim[0], self.x_lim[1], numFruit) # independent of cell

        # not independent of cell
        y = np.zeros(numFruit)
        z = np.zeros(numFruit)

        i = 0

        for o in range(num_snapshots):
            # set initial cell z-axis values
            z_cell_lim = np.array([self.z_lim[0], self.z_lim[0] + cell_h])

            for n in range(n_row):
                # set initial cell y-axis values, at each snapshot moving a vehicle length forward 
                y_cell_lim = np.array([self.y_lim[0] + o*cell_l*n_arm, self.y_lim[0] + cell_l + o*cell_l*n_arm])

                for k in range(n_arm):
                    i_end = i + fruit_in_cell

                    y[i:i_end] = np.random.default_rng(y_seed).uniform(y_cell_lim[0], y_cell_lim[1], fruit_in_cell)
                    z[i:i_end] = np.random.default_rng(z_seed).uniform(z_cell_lim[0], z_cell_lim[1], fruit_in_cell)

                    y_cell_lim = y_cell_lim + cell_l

                    i += fruit_in_cell

                z_cell_lim = z_cell_lim + cell_h

        # print('length of x', len(x), 'y', len(y), 'z', len(z))  # should match numFruit

        # # need a matrix to sort x, y, and z based on the y-axis (to know what fruit show up earlier)
        # fruit = np.stack([x, y, z])

        # axis_to_sort = np.argsort(y) # sort based on y-axis
        # sortedFruit = fruit[:,axis_to_sort]

        sortedFruit = self.sortNstack(x, y, z)

        # return numFruit and the sortedFruit
        return([numFruit, sortedFruit])


    def column(self, v_v, v_a_max, a_a_max, t_grab, n_cell, n_arm, cell_h, z_seed):
        '''
           Fruit distribution where fruit are set in columns to test idle time in "ideal" conditions
           for the arms. Distance between columns determined by vehicle speed, extension amount(?), and 
           max arm speed. 
        '''
        # total_arms = n_cell*n_arm # how many total fruit in a column

        d_a_max = a_a_max
        self.traj_calc = Trajectory(v_a_max, a_a_max, d_a_max)

        mid_x = (self.x_lim[1] - self.x_lim[0]) / 2 + self.x_lim[0] # Delta_x, x-axis mid point
        mid_z = (self.z_lim[1] - self.z_lim[0]) / 2                 # Delta_z, z-axis mid point; the same distance for all rows

        t_x = self.calcTime(self.traj_calc, self.x_lim[0], mid_x, v_a_max, a_a_max, d_a_max)
        t_z = self.calcTime(self.traj_calc, self.z_lim[0], mid_z, v_a_max, a_a_max, d_a_max)

        # distance between columns dependent on extension, unloading, picking time, etc
        Delta_y_columns = v_v * (2*t_x + 2*t_z + t_grab) # assume t_z > t_y moving in y and z

        # total distance divided by the number of columns 
        n_columns       = math.floor((self.y_lim[1] - self.y_lim[0]) / Delta_y_columns)
        column_location = self.y_lim[0] + Delta_y_columns # first column's location

        numFruit = n_columns * (n_arm * n_cell)

        x = np.ones([numFruit]) * mid_x 

        for m in range(n_columns):
            y_col            = np.ones([n_arm*n_cell]) * column_location
            column_location += Delta_y_columns

            if m == 0:
                y = np.copy(y_col)
            else:
                y = np.concatenate([y, y_col])

            cell_z = np.array([self.z_lim[0], self.z_lim[0] + cell_h])

            for n in range(n_cell):
                z_col  = np.random.default_rng(z_seed).uniform(cell_z[0], cell_z[1], n_arm)
                cell_z = cell_z + cell_h

                # print(z_col)

                if n == 0 and m == 0:
                    z = np.copy(z_col)
                else:
                    z = np.concatenate([z,z_col])
                
        
        # print('y:')
        # print(y)
        # print('Z with size:', np.size(z))
        # print(z)
        
        # # need a matrix to sort x, y, and z based on the y-axis (to know what fruit show up earlier)
        # fruit = np.stack([x, y, z])

        # axis_to_sort = np.argsort(y) # sort based on y-axis
        # sortedFruit = fruit[:,axis_to_sort]

        sortedFruit = self.sortNstack(x, y, z)

        # return numFruit and the sortedFruit
        # print()
        # print('Number of fruit', numFruit)
        # print('Sorted array for fruit along y-axis')
        # print(sortedFruit)
        return([numFruit, sortedFruit])


    def calcTime(self, traj_calc, start, end, v_max, a_max, d_max):
        '''Use trajectoyr module to determine movement times for a start and end time'''

        traj_calc.adjInit(start, 0.) # start moving from zero speed
        traj_calc.noJerkProfile(traj_calc.q0, end, traj_calc.v0, v_max, a_max, d_max)

        t = traj_calc.Ta + traj_calc.Tv + traj_calc.Td

        return(t)

