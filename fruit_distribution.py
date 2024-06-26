# Creates or reads the desired fruit distributions
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

import csv                      # read and write CSV file-type
import numpy as np
# from numpy.random import PCG64

import math
# import sys

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

        self.x_lim = np.copy(x_lim)
        self.y_lim = np.copy(y_lim)
        self.z_lim = np.copy(z_lim)
        # print('length of orchard row within the fruit distribution creator:', self.y_lim)


    def sortNstack(self, x, y, z):
        '''
            Sorts x, y, and z fruit coordinate arrays based on the y-coordinates and stacks them into a matrix
        '''
        # create another array to indicate picked or not picked
        isPicked = np.zeros(len(y))
        index    = np.zeros(len(y))
    
        # need a matrix to sort x, y, and z based on the y-axis (to know what fruit show up earlier)
        fruit = np.stack([x, y, z, index, isPicked])

        axis_to_sort = np.argsort(y) # sort based on y-axis
        sortedFruit = fruit[:,axis_to_sort]

        sortedFruit[3,:] = range(len(y))

        return(sortedFruit)


    def csvFile(self, file_name, is_meter, file_delimiter=' '):
        '''
            Import fruit coordinates from given CSV file, where each fruit is a row, columns are x, y, z
            coordinates. 
            4.765 +/-2.612
            file_name has to be string with '...' 

            is_meter = 0 means ft, 1 is meter (other values for other measurements?).
            file_delimiter, 2022 data uses space (default) while the rest use commas
        '''
        x_list = list()
        y_list = list()
        z_list = list()

        numFruit = 0

        with open(file_name) as csvfile:
            reader = csv.reader(csvfile, delimiter=file_delimiter, quoting=csv.QUOTE_NONNUMERIC)
            for row in reader:
                if file_delimiter == ' ':
                    # the 2022 data is the only data set that has different x, y, z definitions than the rest of the project
                    x_list.append(row[2]) # depth into the canopy
                    y_list.append(row[0]) # length along orchard row
                    z_list.append(row[1]) # height

                else:
                    # datasets match the code's definition of the x, y, and z axes
                    x_list.append(row[0])
                    y_list.append(row[1])
                    z_list.append(row[2])

                numFruit += 1

        # convert list to array
        x = np.array(x_list)
        y = np.array(y_list)
        z = np.array(z_list) 

        if is_meter == 0:
            # if is_meter is zero (not in meters), convert from feet to meters
            x = x * 0.3048
            y = y * 0.3048
            z = z * 0.3048

        # calculate the real density in the file instead (make it volume later)
        volume = abs(np.amax(y) - np.amin(y)) * abs(np.amax(z) - np.amin(z)) * abs(np.amax(x) - np.amin(x))

        density = len(y) / volume
        print('\nThe density of fruits in the file is:', density, 'f/m^3 \n')

        x_translate = np.amin(x) - 0.01 # in m
        x = x - x_translate 

        y_translate = np.amin(y) - 0.01
        y = y - y_translate
        # fix the self.y_lim[1] value because hard-coding values isn't a good idea
        real_y_max   = np.amax(y) + 0.01

        z_translate = np.amin(z) - 0.01
        z = z - z_translate

        # remove any fruit that is outside of the robot's max limits
        index_out_of_x_bounds = np.where(x >= self.x_lim[1])
        index_out_of_y_bounds = np.where(y >= real_y_max)
        index_out_of_z_bounds = np.where(z >= self.z_lim[1])
        # print('out of x bounds (before):')
        # print(index_out_of_x_bounds[0])
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
        # print('out of x bounds (after):')
        # print(index_out_of_x_bounds[0])
        # print('out of z bounds (after):')
        # print(index_out_of_z_bounds[0])

        sortedFruit = self.sortNstack(x, y, z)
        # print(f'after removing out of bounds, the number of fruits left is %d' %len(sortedFruit[0]))

        if len(sortedFruit[0]) != numFruit:
            numFruit = len(sortedFruit[0])

        return([numFruit, sortedFruit, real_y_max])        



    def csvFile_segment(self, file_name, is_meter, y_limit, file_delimiter=' '):
        '''
            Import fruit coordinates from given CSV file, where each fruit is a row, columns are x, y, z
            coordinates then remove any fruit that is ouside of the given segment start and end coordinates.
            
            file_name has to be string with '...' 

            is_meter = 0 means ft, 1 is meter (other values for other measurements?).
            file_delimiter, 2022 data uses space (default) while the rest use commas
        '''
        x_list = list()
        y_list = list()
        z_list = list()

        print(f'Harvesting a segment of the CSV file between y-coordinates %3.3f and %3.3f' %(y_limit[0], y_limit[1]))

        numFruit = 0

        with open(file_name) as csvfile:
            reader = csv.reader(csvfile, delimiter=file_delimiter, quoting=csv.QUOTE_NONNUMERIC)
            for row in reader:
                if file_delimiter == ' ':
                    # the 2022 data is the only data set that has different x, y, z definitions than the rest of the project
                    x_list.append(row[2]) # depth into the canopy
                    y_list.append(row[0]) # length along orchard row
                    z_list.append(row[1]) # height

                else:
                    # datasets match the code's definition of the x, y, and z axes
                    x_list.append(row[0])
                    y_list.append(row[1])
                    z_list.append(row[2])

                numFruit += 1
   
        # convert list to array
        x = np.array(x_list)
        y = np.array(y_list)
        z = np.array(z_list) 

        if is_meter == 0:
            # if is_meter is zero (not in meters), convert from feet to meters
            x = x * 0.3048
            y = y * 0.3048
            z = z * 0.3048

        # calculate the real density in the file instead
        # volume = abs(np.amax(y) - np.amin(y)) * abs(np.amax(z) - np.amin(z)) * abs(np.amax(x) - np.amin(x))
        # density = len(y) / volume
        # print('\nThe density of fruits in the file is:', density, 'f/m^3 before removing fruits outside of the work volume\n')

        # translate fruit so it starts at 0 (some files are negative, etc.)
        # something like 0.2 m away in the x-direction
        x_translate = np.amin(x) - 0.01 # in m
        # print('x smallest value, in m', x_translate)
        x = x - x_translate 
        # something like 0 m long in the y-direction
        y_translate = np.amin(y) - 0.01
        y = y - y_translate
        # something like 0 m high in the z-direction
        z_translate = np.amin(z) - 0.01
        z = z - z_translate

        # remove any fruit that is outside of the robot's max limits
        index_out_of_x_bounds = np.where(x >= self.x_lim[1])
        index_out_of_y_bounds = np.where((y <= y_limit[0]) | (y >= y_limit[1]))  # only taking the fruits that fit within the segment (needs to be after translating the fruits, so not at CSV file read)
        index_out_of_z_bounds = np.where(z >= self.z_lim[1])
        # print('out of y bounds (before):')
        # print(index_out_of_y_bounds[0])
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
        # index_out_of_y_bounds = np.where((y <= y_lim[0]) | (y >= y_lim[1])) 
        # index_out_of_z_bounds = np.where(z >= self.z_lim[1])
        # print('out of x bounds (after):')
        # print(index_out_of_x_bounds[0])
        # print('out of y bounds (after):')
        # print(index_out_of_y_bounds[0])
        # print('out of z bounds (after):')
        # print(index_out_of_z_bounds[0])

        if len(y) > 0:
            # check if there are any fruits left, if so, continue

            # make the segment start at 0 in the y-axis relative to the vehicle
            # print('y-coordinates before translating 2nd time', y)
            # y_translate_again = np.amin(y)
            y = y - y_limit[0]
            # print('y-coordinates after translating 2nd time', y)

            # print('There are %d fruits in this segment' %len(y))
            # print('The first fruit is at', np.amin(y))

            # calculate the density in the segment
            volume = abs(np.amax(y) - np.amin(y)) * abs(np.amax(z) - np.amin(z)) * abs(np.amax(x) - np.amin(x))
            density = len(y) / volume
            # print('\nThe density of fruits in the segment is:', density, 'f/m^3 after removing fruits outside of the work volume\n')

            sortedFruit = self.sortNstack(x, y, z)
            # print(f'after removing out of bounds, the number of fruits left is %d' %len(sortedFruit[0]))

            if len(sortedFruit[0]) != numFruit:
                numFruit = len(sortedFruit[0])
            # print('####################### The number of fruits passed back is %d\n\n' %numFruit)

        else:
            # there are no more fruits, at least not in this segment
            numFruit = 0
            sortedFruit = []

        return([numFruit, sortedFruit]) 



    # def csvFile_reduced(self, file_name, is_meter, desired_density, x_seed):
    #     '''
    #         Import fruit coordinates from given CSV file, where each fruit is a row, columns are x, y, z
    #         coordinates. The average density in Raj's file is around 48 fruit/m^2.

    #         file_name has to be string with '...' 

    #         is_meter = 0 means ft, 1 is meter (other values for other measurements?).

    #         desired_density is the total desnity that should be returned. Reducetion is done by uniform 
    #         randomly choosing indexes and removing them from the list of fruit.

    #         x_seed is a seed used to remove fruits randomly, but with a known seed. 
    #     '''
    #     x_list = list()
    #     y_list = list()
    #     z_list = list()

    #     numFruit = 0

    #     with open(file_name) as csvfile:
    #         reader = csv.reader(csvfile, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
    #         for row in reader:
    #             # need to check that x, y, z, coordinates match my definition of x, y, z -> apples 2015 do
    #             x_list.append(row[0])
    #             y_list.append(row[1])
    #             z_list.append(row[2])

    #             numFruit += 1

    #     # convert list to array
    #     x = np.array(x_list)
    #     y = np.array(y_list)
    #     z = np.array(z_list) 

    #     # in Raj's data set, the average denisty is around 48 fruit/m^2
    #     # Raj_density = 48
    #     # calculate the real density in the file instead (make it volumn late)
    #     volume = abs(y.max() - y.min()) * abs(z.max() - z.min()) * abs(x.max() - x.min())

    #     density = len(y) / volume
    #     print('\nThe original density of the file is:', density, 'f/m^3 \n  The desired density is:', desired_density, 'f/m^3 \n')

    #     # if we want the final density to be 20 fruit/m^2, that's 42% of the density, so we want to remove 1-20/48 
    #     percent_2_remove = 1 - desired_density/density
    #     # print('percent of fruit to remove', percent_2_remove, 'out of total', numFruit, 'fruit')
    #     # print()
    #     total_2_remove  = math.floor(len(y) * percent_2_remove)
    #     # print('total fruit to remove', total_2_remove)
    #     # print()

    #     for remove in range(total_2_remove):
    #         # tried making an array of random integers to remove, but not all of them were unique
    #         index_2_remove = np.random.default_rng(x_seed).uniform(0, len(x)-1, 1)
    #         i2r = np.asarray(index_2_remove, dtype = int)
    #         x = np.delete(x, i2r)
    #         y = np.delete(y, i2r)
    #         z = np.delete(z, i2r)

    #     # if (numFruit - total_2_remove) == len(x):
    #     #     # check if the number of fruit in the array is correct
    #     #     print('got the correct number of fruit')

    #     if is_meter == 0:
    #         # if is_meter is zero, convert from feet to meters
    #         x = x * 0.3048
    #         y = y * 0.3048
    #         z = z * 0.3048

    #     # check if need to translate fruit in to get it to correct frame if vehicle is at 0 m and fruit starts...
    #     # something like 0.2 m away in the x-direction
    #     x_translate = np.amin(x) # in m
    #     # print('x smallest value, in m', x_translate)
    #     x = x - x_translate + 0.2
    #     # something like 0 m long in the y-direction
    #     y_translate = np.amin(y)
    #     y = y - y_translate
    #     # something like 0 m high in the z-direction
    #     z_translate = np.amin(z)
    #     z = z - z_translate

    #     # remove any fruit that is outside of the robot's max limits
    #     index_out_of_x_bounds = np.where(x >= self.x_lim[1])
    #     index_out_of_y_bounds = np.where(y >= self.y_lim[1])
    #     index_out_of_z_bounds = np.where(z >= self.z_lim[1])
    #     # print('out of z bounds (before):')
    #     # print(index_out_of_z_bounds[0])

    #     out_of_bounds = np.concatenate((index_out_of_x_bounds[0], index_out_of_y_bounds[0], index_out_of_z_bounds[0]), axis=None)
    #     u = np.unique(out_of_bounds) # make sure there are no dulplicates, see https://numpy.org/doc/stable/reference/generated/numpy.unique.html
    #     # print('indexes to be deleted:',u, 'with size', len(u))
    #     x = np.delete(x, u)
    #     y = np.delete(y, u)
    #     z = np.delete(z, u)

    #     numFruit = len(x) # reset numFruit because there have been two deletions, one to get the correct density and one to keep them within limits

    #     sortedFruit = self.sortNstack(x, y, z)

    #     return([numFruit, sortedFruit]) 



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


    # def uniformRandomMelon(self, density, y_seed, z_seed):
    #     '''
    #        Fruit distribution set uniform random along total y, and z given y, and z limits, seeds, 
    #        and desired density. x is asumed constant for melons.
    #     '''
    #     numFruit = int(density * (self.len_y*self.len_x*self.len_z)) 

    #     x = np.ones(numFruit)

    #     y = np.random.default_rng(y_seed).uniform(self.y_lim[0], self.y_lim[1], numFruit)
    #     z = np.random.default_rng(z_seed).uniform(self.z_lim[0], self.z_lim[1], numFruit)

    #     # # need a matrix to sort x, y, and z based on the y-axis (to know what fruit show up earlier)
    #     # fruit = np.stack([x, y, z])

    #     # axis_to_sort = np.argsort(y) # sort based on y-axis
    #     # sortedFruit = fruit[:,axis_to_sort]

    #     sortedFruit = self.sortNstack(x, y, z)

    #     # return numFruit and the sortedFruit
    #     return([numFruit, sortedFruit])


    def uniformRandomMelon_MultipleDensity(self, densities, y_seed, z_seed):
        '''
           Fruit distribution set uniform random along total y, and z given y, and z limits, seeds, 
           and desired density. x is asumed constant for melons. Row changes densities after given distance.
        '''
        offset = 2 # paper has 2 m of empty space between densities

        # length on the y-axis for each density is the same and limited by y_lim
        length_den = (self.y_lim[1] - self.y_lim[0]) / len(densities) - 2
        # print('length of the density:',length_den)

        fruit_per_density = densities * (length_den*self.len_x*self.len_z) # array with the number of fruit in each density space
        fruit_per_density = np.asarray(fruit_per_density,dtype="int") # number of fruit has to be discrete

        print('fruit per density')
        print(fruit_per_density)

        # total fruit in the row
        numFruit = int(np.sum(fruit_per_density))
        print('total number of fruit', numFruit)

        x = np.ones(numFruit)
        z = np.random.default_rng(z_seed).uniform(self.z_lim[0], self.z_lim[1], numFruit)

        y_start = self.y_lim[0]
        y_end   = y_start + length_den

        for space in range(len(densities)):
            y_part = np.random.default_rng(y_seed).uniform(y_start, y_end, fruit_per_density[space])

            y_start += length_den + offset
            y_end   += length_den + offset
            

            if space == 0:
                y = y_part
            else:
                y = np.concatenate((y,y_part))

        sortedFruit = self.sortNstack(x, y, z)

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

    

    def columnUniform_melon(self, n_fruit, d_y, z_coord):
        '''
            Fruit distribution for testing. n_fruit number of fruits are set in a line at z-height every y_dist 
            of distance. Assumes knowledge of the whole orchard row.
        '''
        x = np.ones(n_fruit)
        z = np.ones(n_fruit) * z_coord 

        y_stop = (n_fruit-1)*d_y # -1 added since the first fruit starts at 0.0

        y = np.linspace(0, y_stop, num=n_fruit, endpoint=True)
        # print('fruit y-coordinates:', y)

        sortedFruit = self.sortNstack(x, y, z)

        self.y_lim[1] = n_fruit * d_y

        # return numFruit and the sortedFruit
        return([n_fruit, sortedFruit])



    def columnRandom_melon(self, n_fruit, y_seed, z_coord):
        '''
            Fruit distribution for testing. n_fruit number of fruits are set in a line at z-height randomly 
            placed along the y-axis. Assumes knowledge of the whole orchard row.
        '''
        x = np.ones(n_fruit)
        z = np.ones(n_fruit) * z_coord 

        y = np.random.default_rng(y_seed).uniform(self.y_lim[0], self.y_lim[1], n_fruit)

        # print('fruit y-coordinates:', y)

        sortedFruit = self.sortNstack(x, y, z)

        # return numFruit and the sortedFruit
        return([n_fruit, sortedFruit])



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
        '''Use trajectory module to determine movement times for a start and end time'''

        traj_calc.adjInit(start, 0.) # start moving from zero speed
        traj_calc.noJerkProfile(traj_calc.q0, end, traj_calc.v0, v_max, a_max, d_max)

        t = traj_calc.Ta + traj_calc.Tv + traj_calc.Td

        return(t)

