import csv
import numpy as np
from numpy.random import PCG64
# from scipy.spatial import KDTree # for clusters
import sys

from fruit_distribution import *   # import module to create the various desired fruit distributions
from trajectory import *           # import module to calculate the trapezoidal/S-curve (S-curve not working yet) tajectory calculator

class fruit_handler(object):
    def __init__(self, C, R, t_grab, x_lim, y_lim, z_lim):
        
        self.C = C            # total number of columns 
        self.R = R            # total number of rows
        self.t_grab = t_grab  # in s, constant time to grab fruit once reached

        self.x_lim   = np.copy(x_lim)
        self.y_lim   = np.copy(y_lim)
        self.z_lim   = np.copy(z_lim)

        # allow calculations for the x-axis, 1.0 m long worst case
        v_max_x   = 2         # in m/s, from Motor Sizing Google sheet calculations if we want to keep triangular profile to maximize speed
        a_max_x   = 4         # in m/s^2, from Motor Sizing Google sheet calculations if we want desired linear movement time to be 1 second
        d_max_x   = a_max_x   # in m/s^2, if motors allow, keep equal to a_max
        # initialize the ability to calculate trajectory
        self.traj_calc_x = Trajectory(v_max_x, a_max_x, d_max_x) 



    def buildOrchard(self, set_distribution, seed_xyz, density=20, run_n=0):
        '''
           Set up to create the simulated environment, separated from createFruitDistribution so that MIP run/row can happen.
           Seed list is a list with three seeds to compute the random the x, y, and z-coordinates. It's obtained from a CSV but
           only one set of three values is passed to buildorchard at a time to save time. 
        '''
        # init the fruit distribution creation module with the desired limits
        fruitD = fruitDistribution(self.x_lim, self.y_lim, self.z_lim) # init fruit distribution script

        if set_distribution == 0:
            [self.N, self.sortedFruit] = self.createFruitDistribution(fruitD, set_distribution, run_n=run_n)

        elif set_distribution == 1:
            # uses the seed values to get random segments from the 2022 apple data files
            x_seed = PCG64(int(seed_xyz[0])) 
            y_seed = PCG64(int(seed_xyz[1]))
            z_seed = PCG64(int(seed_xyz[2])) 
            [self.N, self.sortedFruit] = self.createFruitDistribution(fruitD, set_distribution, x_seed=x_seed, y_seed=y_seed, z_seed=z_seed, run_n=run_n)

        elif set_distribution >= 4:
            # seed values, rather than i_run, will determine changes to any RNG-created distribution
            x_seed = PCG64(int(seed_xyz[0])) 
            y_seed = PCG64(int(seed_xyz[1]))
            z_seed = PCG64(int(seed_xyz[2])) 
            [self.N, self.sortedFruit] = self.createFruitDistribution(fruitD, set_distribution, x_seed=x_seed, y_seed=y_seed, z_seed=z_seed, density=density)

        else:
            # non-RNG distributions, so don't bother processing or sending them
            [self.N, self.sortedFruit] = self.createFruitDistribution(fruitD, set_distribution)        

        # print('Density chosen', self.density)
        print('x-lim', self.x_lim, 'y-lim', self.y_lim, 'z-lim', self.z_lim)
        print('Total fruit in the orchard row',self.N)
        # print()
        # print('length of sortedFruit', len(self.sortedFruit[0]))
        # print()
        # print('List of the x, y, and z coordinates of the sorted fruit')
        # print(sortedFruit)

        ## for now, calculate time between fruits here (only needed once per snapshot)
        self.timeBtwFruits()


    
    def calcStateTime(self, fruit_picked_by, fruit, D, V):
        '''Calculates the duration of time each arm is in each state so that plotState can plot the data'''
        total_time = D / V 
        # print('total move time:', total_time)

        # list of fruit extension times in fruit object
        N  = [i.real_index for i in fruit]    # list of fruit indexes
        TX = [i.Tx for i in fruit]            # list of precalculated fruit extension times

        ## states: idle, pick_yz, pick_x, grab, retract_x, move_z/unload
        # self.state_percent = np.zeros([self.total_arms, 6]) # save each arm's percent time in each of the six states 
        # state_time = np.zeros([total_arms, 7]) # save each arm's percent time in each of the six states plus a total

        state_time = self.calcSumPCTstates(fruit_picked_by, N, TX) 

        for r in range(self.R):
            for c in range(self.C):
                tot_arm_index = c + (r*self.C)
                # calculate idle by subtracting all numbes calculated above by total time: length_row / v
                state_time[tot_arm_index,0] = total_time - np.sum(state_time[tot_arm_index,:])
                # save the total time for this run to get total percent later
                state_time[tot_arm_index,6] = total_time
                
        return(state_time)
    


    def calcMeanTd(self, total_picked, state_time):
        '''
           Calculates the mean Td of the snapshot based on the harvested fruits. Pass the final fruit_picked_by
           list, the one that already went through queue management, etc.

           Can only be run after scheduling has finished and calcStateTime is run to produce the state_time array.

           *** Depends on no cycles (figure out cycles later) *** 
           Assumes that fruit_picked_by contains the picking order (smallest to largest) which may not be true later
           Might need to switch MIP to save order if cycles added.
        '''
        # # list of fruit extension times in fruit object
        # N  = [i.real_index for i in fruit]    # list of fruit indexes
        # TX = [i.Tx for i in fruit]            # list of precalculated fruit extension times

        # state_time = self.calcSumPCTstates(fruit_picked_by, N, TX)

        sum_pickYZ = np.sum(state_time[:,1])
        sum_pickX  = np.sum(state_time[:,2])
        sum_grab   = np.sum(state_time[:,3])
        sum_retr   = np.sum(state_time[:,4])

        # print('\nSum of pickYZ %.2f,   pickX %.2f,   grab %.2f,   retract %.2f' % (sum_pickYZ, sum_pickX, sum_grab, sum_retr))

        # get the total fruit handling time
        sum_Td_times = sum_pickYZ + sum_pickX + sum_grab + sum_retr # sum_of_ext_times + sum_of_move_times

        # divide by the number of harvested fruits to get the average
        mean_Td = sum_Td_times / total_picked
        # print('Mean Td time for the snapshot %.2f\n' % mean_Td)

        return(mean_Td)



    def calcSumPCTstates(self, fruit_picked_by, N, TX):
        '''
           Calculates the sum of each of the six PCT state (idle, pickYZ, pickX, grab, retract, unload) over
           the whole snapshot. Returns the sum over *all the arms* of the duration spent in each state.

           Can only be run after scheduling has finished.

           *** Depends on no cycles (figure out cycles later) *** 
           Assumes that fruit_picked_by contains the picking order (smallest to largest) which may not be true later
           Might need to switch MIP to save order if cycles added.
        '''
        sum_pickYZ = 0 # in s, the sum of move times
        sum_pickX  = 0 # in s, the sum of extension times
        sum_grab   = 0 # in s, the sum of grab times
        # retract will equal pickX unless proven otherwise :)
        # not using unload right now (don't have the math yet, assumes a vacuum gripper)

        state_time = np.zeros([self.R*self.C, 7]) # save each arm's percent time in each of the six states plus a total

        # iterate through fruit picked by to obtain the individual PCT/Td of every fruit
        for i_row in range(self.R):
            if self.R == 1:
                i_row = 0
            for i_col in range(self.C):
                tot_arm_index = i_col + (i_row*self.C)
                # skips not picked list
                if self.R > 1:
                    num_fruits = len(fruit_picked_by[i_row][i_col])
                    # print('the fruits in this row/column:', fruit_picked_by[i_row][i_col])
                else:
                    num_fruits = len(fruit_picked_by[i_col])
                
                if num_fruits > 1:
                    # get extension time of the first fruit in the list
                    if self.R > 1: 
                        zero_TX_i = N.index(fruit_picked_by[i_row][i_col][0])
                    else:
                        zero_TX_i = N.index(fruit_picked_by[i_col][0])
                    # print('TX[0] %.2f' % TX[fruit_picked_by[i_row][i_col][0]])

                    # obtain the extension and retraction of the first fruit (assumes it doesn't have to move to that fruit)
                    sum_pickX += TX[zero_TX_i]
                    sum_grab  += self.t_grab

                    # needs to take into account the order of harvest (move from fruit i to fruit j, figure out which is index belongs to i and which to j)
                    for i_list in range(num_fruits-1):
                        if self.R > 1:
                            curr_i = fruit_picked_by[i_row][i_col][i_list]
                            next_i = fruit_picked_by[i_row][i_col][i_list+1] 
                            # print('current i: %d and next i: %d' % (curr_i, next_i))
                        else:
                            curr_i = fruit_picked_by[i_col][i_list]
                            next_i = fruit_picked_by[i_col][i_list+1] 

                        # curr_TX_i = N.index(curr_i)
                        next_TX_i = N.index(next_i)
                        # print('which are %d and %d in N' % (curr_TX_i, next_TX_i))

                        curr_move = self.fruit_travel_matrix[curr_i, next_i]
                        # print('with move time %.2f' % curr_move)

                        # to get the indexes to work correctly, this part works with indexes (i+1), avoiding any doubling of the extension time
                        # add the extension time for the next fruit (i+1) 
                        # print('with extension time %.2f' % TX[next_TX_i])
                        sum_pickYZ += curr_move
                        sum_pickX  += TX[next_TX_i]
                        sum_grab   += self.t_grab

                elif num_fruits == 1:
                    # just worry about the one extension because only one fruit was harvested
                    if self.R > 1:
                        zero_TX_i = N.index(fruit_picked_by[i_row][i_col][0])
                    else:
                        zero_TX_i = N.index(fruit_picked_by[i_col][0])
                    sum_pickX += TX[zero_TX_i]
                    sum_grab  += self.t_grab
                    # sum_retr  += TX[zero_TX_i]

                state_time[tot_arm_index,1] = sum_pickYZ
                state_time[tot_arm_index,2] = sum_pickX
                state_time[tot_arm_index,3] = sum_grab
                state_time[tot_arm_index,4] = sum_pickX # same as pickX unless something comes up

                # reset the values
                sum_pickYZ = 0
                sum_pickX  = 0
                sum_grab   = 0

        # print('\nThe sum of extension times is %.2f s' % sum_of_ext_times)
        # print('The sum of move times is %.2f s\n' % sum_of_move_times)

        return(state_time)
    


    def calcYlimMax(self, set_distribution):
        '''
           Calculates the end of the orchard's coordinate based on the set fruit distribution. 
           
           Every data set is a different length which is currently hardcoded in this function. Function runs before 
           fruits are read in, so it cannot (at this point) be set based on the distribution. Should change this...
        '''
        # no snapshots, so the travel length is static at the orchard row's length
        if set_distribution == 0:
            # using 2022 Colombini data, full orchard rows 
            y_max  = 10.9 # in m, although they should all be around 10m, adding 0.9 to test what the correct value should be. 

        elif set_distribution == 2 or set_distribution == 4:
            # if using raj's dataset
            y_max  = 10.9 # in m, for Raj's data
            # self.density   = 48.167         # in fruit/m^2 (on avg.), constant, for Raj's data
            # n_runs    = 1

        elif set_distribution == 3 or set_distribution == 5:
            # if using juan's dataset
            y_max  = 16.5 # in m, for Juan's data
            # self.density   = 53.97            # in fruit/m^2 (on avg.), constant, for Juan's data
            # n_runs    = 1

        elif set_distribution == 6:
            # uniform random
            y_max  = 6 # in m, usually 5 m + length

        # elif set_distribution == 3:
        #     y_max  = 30 # in m
            
        # elif set_distribution == 5:
        #     v_vy_fruit_mps = v_vy_fruit_cmps / 100
        #     self.d_y  = self.Td*v_vy_fruit_mps*(n_fruit+1)/(n_fruit+2) # kind of works 3/4 or 5/8 fruit with 1 arm: (Td/2)*v_vy
        # #     d_y  = Td*v_vy_fruit*(n_fruit+1)/(n_fruit+2) # kind of works 3/4 or 5/8 fruit with 1 arm: (Td/2)*v_vy
        #     print('with Td', self.Td, 'and v_vy for fruit distribution', v_vy_fruit_cmps, 'cm/s')
        #     print('d_y for this line of fruit:', self.d_y, 'so the total distance they take up:', self.d_y*n_fruit)
        #     y_max  = self.d_y * n_fruit # in m
            
        else: 
            print('ERROR: distribution', set_distribution, 'does not exist or is not available, exiting the program')
            sys.exit(0)

        self.y_lim[1] = y_max 



    def createArms(self):
        '''Create and populate all the arms' classes then put in appropriate list'''
        ## create arm object list
        arm = list()

        starting_row_n = 0 # not used right now if solving row by row

        # check if being updated 
        # print('starting row number:', self.starting_row_n)

        if starting_row_n + 1 > self.R:
            row_n = starting_row_n + 1
        else:
            row_n = self.R

        # print('the starting row number is', self.starting_row_n, 'and the ending is', row_n)

        for r in range(starting_row_n, row_n):
            # print('row number', r)
            for c in range(self.C):
                this_arm = Arm(r, c)
                arm.append(this_arm)
        # print()
        return(arm)
    


    def createFruits(self, numFruit, sortedFruit):
        '''Create and populate all the fruits' classes then put in appropriate list. Fruit indexes are 'zeroed' so that it works with snapshots'''
        ## create fruit object list
        fruit = list()

        # only works by starting index at 0
        for index in range(numFruit):
            # if the fruit was not removed due to clustering
            x_coord      = sortedFruit[0][index]
            y_coord      = sortedFruit[1][index]
            z_coord      = sortedFruit[2][index]
            # use a 'zeroed' index so that it works with snapshots
            fruit_i      = index  
            # but make sure to save the fruit's 'real' index 
            fruit_i_real = int(sortedFruit[3][index])
            # create the object
            this_fruit   = Fruit(self.traj_calc_x, fruit_i, fruit_i_real, x_coord, y_coord, z_coord)
            # print('Fruit index', index, 'should match this index only if no snapshots', sortedFruit[3][index])
            # print('with y and z coordinates:', y_coord, z_coord)
            fruit.append(this_fruit)

        # print(fruit)
        return(fruit)


    
    def createJobs(self, arm, fruit, V, Q, d_col):
        '''Create and populate all the jobs' classes then put in appropriate list'''
        ## create job object list
        job = list()

        for i_arm in arm:
            for i_fruit in fruit:  
                this_job = Job(i_fruit, i_arm, Q, V, d_col)  # careful because makespan does not use the Jobs() TW values
                job.append(this_job)
                # print('for arm', this_job.arm_k.arm_n, 'in row', this_job.arm_k.row_n,'and fruit', this_job.fruit_i.index)
                # print('TW starts at', this_job.TW_start, 'and TW ends at', this_job.TW_end)  

        return(job) 
    


    def createFruitDistribution(self, fruitD, set_distribution, x_seed=0, y_seed=0, z_seed=0, density=20, run_n=0):
        '''
           Determines how the fruit distribution is going to be created, ether from real localization data or from RNG-created data 
           based on the given seeds. See value below for all the options.

           seed values are only provided for distributions that need them, otherwise the defualt is 0 for all seeds.
        '''
        # 0     == 2022 Digitized fruit data (Pink Ladies, Jeff Colombini orchard)
        # 1   * not available *  == random (seed-based) segments from 2022 digitized fruit
        # 2     == Raj's digitized fruits (right side)
        # 3     == Juan's digitized fruits (Stavros phone video)
        # 4     == reduced Raj's digitized fruits; can reduce the density to a desired value (density hardcoded currently to 20f/m^3)
        # 5     == reduced Juan's digitized fruits; can reduce the density to a desired value (density hardcoded currently to 20f/m^3)
        # 6     == uniform random  (if algorithm == 1, use melon version)
        # 7   * not available *  == uniform random, equal cell density
        # 8   * not available *  == multiple densities separated by some space (only melon for now)
        # 9   * not available *  == fruit in vertical columns
        # 10  * not available *  == "melon" version of columns (user inputs desired no. fruits, z height, and distance between fruit in y-coord)

        if set_distribution == 0:
            # using 2022 Colombini data, full orchard rows 
            # there are 18 CSV files, from 4 folders, each file with around 10m of fruit localization data 
            csv_folder = './TREE_FRUIT_DATA/apple_data_2022/'
            print('RUN NUMBER:', run_n)
            if run_n <= 5:
                run_files = '01_42_4_apples_' + str(run_n+1) + '.csv'
            elif run_n <= 9:
                run_files = '46_42_1_apples_' + str(run_n-5) + '.csv'
            elif run_n <= 13:
                run_files = '51_42_2_apples_' + str(run_n-9) + '.csv'
            elif run_n <= 15:
                run_files = '56_42_3_apples_' + str(run_n-13) + '.csv'
            else: 
                print('WARNING: There are no more files in the 2022 dataset to run. Defaulting to the last file.')
                run_files = '56_42_3_apples_3.csv'

            csv_file = csv_folder + run_files
            is_meter = 1
            [N, sortedFruit, real_y_max] = fruitD.csvFile(csv_file, is_meter)
            self.y_lim[1] = real_y_max

        elif set_distribution == 2:
            # using raj's dataset
            csv_file = './TREE_FRUIT_DATA/apple_tree_data_2015/Applestotheright.csv'
            is_meter = 0
            [N, sortedFruit, real_y_max] = fruitD.csvFile(csv_file, is_meter, file_delimiter=',')
            self.y_lim[1] = real_y_max

        elif set_distribution == 3:
            # use juan's dataset
            csv_file = './TREE_FRUIT_DATA/20220811_apples_Juan.csv'
            is_meter = 1
            [N, sortedFruit, real_y_max] = fruitD.csvFile(csv_file, is_meter, file_delimiter=',')
            self.y_lim[1] = real_y_max

        elif set_distribution == 4:
            # using raj's reduced dataset
            csv_file = './TREE_FRUIT_DATA/apple_tree_data_2015/Applestotheright.csv'
            [N, sortedFruit] = fruitD.csvFile_reduced(csv_file, 0, density, x_seed)

        elif set_distribution == 5:
            # using juan's reduced dataset
            csv_file = './TREE_FRUIT_DATA/20220811_apples_Juan.csv'
            [N, sortedFruit] = fruitD.csvFile_reduced(csv_file, 0, density, x_seed)

        elif set_distribution == 7:
            # uniform random in 3D 
            [N, sortedFruit] = fruitD.uniformRandom(density, x_seed, y_seed, z_seed)
            # print()
            # print('--------------------------------------------')
            # print('Number of fruit:', N)
            # print()

        # elif set_distribution == 8: 
        #     fruit_in_cell = math.ceil(density * (self.cell_h*self.cell_l*self.arm_reach)) # num of fruit in front of cell if using (equalCellDensity())
        #     print('Number of fruit in each cell:', fruit_in_cell)
        #     print()
        #     [N, sortedFruit] = fruitD.equalCellDensity(self.R, self.C, self.cell_h, self.cell_l, self.arm_reach, fruit_in_cell, x_seed, y_seed, z_seed)

        # elif set_distribution == 9: 
        #     # multiple densities with some space between them, based on Mann et. al paper
        #     densities = np.array([5, 4, 3])
        #     [N, sortedFruit] = fruitD.uniformRandomMelon_MultipleDensity(densities, y_seed, z_seed)

        # # elif set_distribution == 10: 
        #      # fruits set in columns spaced a given distance apart based on v_vy (if I remember correctly)
        # #     [N, sortedFruit] = fruitD.column(v_vy, v_max, a_max, t_grab, self.n_row, self.n_col, self.cell_h, z_seed)
        # elif set_distribution == 5:
        #     z_coord = (self.cell_h / 2) + 0.7
        #     [N, sortedFruit] = fruitD.columnUniform_melon(N, self.d_y, z_coord)
        
        else: 
            print('WARNING: Fruit distribution not available, defaulting to Raj\'s data (CHANGE THIS LATER)')
            # using raj's dataset
            csv_file = './TREE_FRUIT_DATA/apple_tree_data_2015/Applestotheright.csv'
            is_meter = 0
            [N, sortedFruit] = fruitD.csvFile(csv_file, is_meter)

        return([N, sortedFruit])

        
            
    def fruitsInView(self, Q, d_view, total_sortedFruit, d_cell, d_arm_travel):
    # def fruitsInView(q_vy, n_snapshots, l_view_m, total_sortedFruit, cell_l, pick_travel_l):
        '''Determine which fruits are unpicked and in front of the vehicle for each snapshot.'''
        
        # offset = (d_cell - d_arm_travel) / 2 # assume centered in cell
        start_y = Q                       # the y-coordinate start of the start of the view window for the snapshot ((0,0) for the vehicle at start of snapshot)

        index_this_sortedFruit = np.where((total_sortedFruit[1,:] > start_y) & (total_sortedFruit[1,:] < start_y + d_view))
        
        # print()
        # print('index of fruits in the view window', index_this_sortedFruit[0])
        # print()

        return(index_this_sortedFruit[0])



    def getHorizonIndex(self, sortedFruit, Q, d_vehicle, d_hrzn):
        '''
        Saves this snapshot's horizon fruit indexes based on the sortedFruit indexes to 
        compare and remove picked fruit.
        '''
        # edges of the horizon based on vehicle location and length
        horizon_back  = Q + d_vehicle
        horizon_front = horizon_back + d_hrzn

        H_fruit_index = np.where((sortedFruit[1,:] >= horizon_back) & (sortedFruit[1,:] < horizon_front))

        return(H_fruit_index[0])
    


    def getRNGSeedList(self, n_runs, csv_name):
        '''
        Open the random seed list rngseed_list_20200901.csv with 200 seeds for each of the 3 real fruit coordinate axis
        and 3 extra (used to be for "fake fruits" but switching to z-coordinates for row top and bottom boundaries).
        '''
        # keeps track of the row number of the csv being read (each row contains the seeds for one run)
        csv_i     = 0

        seed_list = list()

        with open(csv_name) as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
            for row in reader:  # row contains the values in the CSV's row so it cannot be used as the counter
                seed_list.append(row)
                if csv_i == n_runs-1:
                    break

                csv_i += 1

        # print(seed_list)
        return(seed_list)
    


    def calcRowZBounds(self, set_edges, z_lim, h_cell, h_g, sortedFruit, boundary_seed):
        '''
        Calculate the z-coord for each horizontal row, assuming the whole row shares these edges.

        Returns a n_row x n_col matrix for both the bottom and top edges of each cell. Takes into account the
        height of the gripper, adding dead space to remove the possiblility of collisions between arms in a
        column. If there are multiple columns, adds a random offset between columns to remove dead space.

        Returns two arrays, z_row_bot_edges[row,col] and z_row_top_edges[row,col]
        '''   
        # find what fruits are available to harvest (some may have already been picked or removed)
        index_available = np.where(sortedFruit[4,:] <= 1) 

        # compute a the random offset per column by using CSV-saved seeds so that results are reprodicible, only create C-1 offsets because column 0 is set at the boundary value
        if self.C > 1:
            boundary_offset = np.random.default_rng(PCG64(int(boundary_seed))).uniform(-h_g, h_g, [(self.R-1), self.C]) # create C number of offsets
            # zero out the c=0 offset
            boundary_offset[:,0] = 0
            # use boundary offset to create two new matrixes with zeros in the correct row to add offsets correctly to the top and bottom matrices
            zero_array      = np.zeros([self.C])
            bot_offset      = np.concatenate(([zero_array], boundary_offset), axis=0)
            top_offset      = np.concatenate((boundary_offset, [zero_array]), axis=0)
            # print('\nBoundary offset:\n', boundary_offset)
            # print('bottom offset:\n', bot_offset)
            # print('top offset:\n', top_offset)
        else:
            boundary_offset = np.zeros([self.R, self.C])

        # calculate the z-coordinates for equal height rows
        if set_edges == 0 or len(index_available[0]) <= self.R: 
            # divided equally by distance along orchard height or there are no fruits in view
            # bottom = np.mgrid
            # row bottom edge = n*self.cell_h
            bottom = np.linspace(0, (self.R*h_cell - h_cell), self.R, endpoint=True)  # h_cell is interesting because it's already calculated from h_vehicle/n_rows 
            # print('\nthe first bottom set of boundaries', bottom, '\n')

            # bot_edge = np.tile(bottom, (self.C, 1))
            bot_edge = np.transpose(np.tile(bottom, (self.C,1)))   # need to output bottom[row, col] which requires the transpose
            top_edge = np.copy(bot_edge) + h_cell
            # print('\nh_cell*c for c = 0, ...., C-1:', 0, h_cell*1, h_cell*2)
            # print('\nbottom edges:\n', bot_edge)
            # print()
            # print('top edges:\n', top_edge)
            # print()

        elif set_edges == 1:
            bot_edge = np.zeros([self.R, self.C])
            top_edge = np.zeros([self.R, self.C])
             # calculate how many fruits should be in each row
            fruit_in_row = math.floor(len(index_available[0]) / self.R)  # total fruit in each horizontal row (round down, one row could be heavier)
            # fruit_in_row = math.floor(numFruit / n_row)  # total fruit in each horizontal row (round down, one row could be heavier)
            # fruit_in_row = math.floor(self.numFruit / n_row)  # total fruit in each horizontal row (round down, one row could be heavier)
            print('\nnumber of fruit in each row, rounded down', fruit_in_row, '\n')
            print()
            
            # get the z-coord array of the remaining fruits
            # z_check = np.array(sortedFruit[2])
            z_coord = np.array(sortedFruit[2,index_available[0]])
            # print('z_check', z_check)
            # print()
            # print('z_coord', z_coord)

            # sort the array from small to large value
            z_sorted = np.sort(z_coord)
    #         print('sorted z-coord', z_sorted)
            
            for row in range(self.R-1):
                # take the average of the fruit and next fruit's coordinates that determine the z-coordinate boundary value
                # it's row+1 for the bottom because bottom[0] == 0 because it should be at the bottom frame 
                bot_edge[row+1,:] = (z_sorted[fruit_in_row*(row+1)] + z_sorted[fruit_in_row*(row+1) + 1]) / 2
                top_edge[row,:]   = (z_sorted[fruit_in_row*(row+1)] + z_sorted[fruit_in_row*(row+1) + 1]) / 2

            # the last top edge should be located at the top frame
            top_edge[self.R-1,:] = z_lim[1]
            
        else:
            print('Not an edge setting, please try again')
            sys.exit(0)

        # add the dead space caused by the gripper taking up h_g vertical space, avoid adding or subtracting from the top and bottom physical frame location values
        # print('\nBEFORE h_g and the random offset')
        # print('\nbottom edges:\n', bot_edge)
        # print()
        # print('top edges:\n', top_edge)
        bot_edge[1:,:]          += 1/2 * h_g
        top_edge[:(self.C-2),:] -= 1/2 * h_g

        # add the random offset
        self.z_row_bot_edges = bot_edge + bot_offset
        self.z_row_top_edges = top_edge + top_offset
        # print('\nAFTER h_g')
        # print('bottom z-axis edges:\n', self.z_row_bot_edges)
        # print()
        # print('top z-axis edges:\n', self.z_row_top_edges)
        # print()
        # sys.exit(0)  # testing only

        return([bot_edge, top_edge])  
    


    def timeBtwFruits(self):
        '''
           Create a matrix that saves the time it takes to move between every two fruits in the snapshot. Time calculated as max(Ty, Tz) using the trajectory.py
           module. One side of matrix is calculated and then mirrored over since the distance should be the same between i and j vs j and i. 

           Requires that buildOrchard() has already been run.
        '''
        # allow calculations for the y-axis, 0.7 m long worst case
        v_max_y   = 1.4 # m/s from Motor Sizing Google sheet calculations if we want to keep triangular profile to maximize speed
        a_max_y   = 2.8 # m/s^2 from Motor Sizing Google sheet calculations if we want desired linear movement time to be 1 second
        d_max_y   = a_max_y # if motors allow, keep equal to a_max
        # initialize the ability to calculate trajectory
        traj_calc_y = Trajectory(v_max_y, a_max_y, d_max_y) 

        # allow calculations for the z-axis, 0.61? m worst case
        v_max_z   = 1.3 # m/s from Motor Sizing Google sheet calculations if we want to keep triangular profile to maximize speed
        a_max_z   = 2.8 # m/s^2 from Motor Sizing Google sheet calculations if we want desired linear movement time to be 1 second
        d_max_z   = a_max_z # if motors allow, keep equal to a_max
        # initialize the ability to calculate trajectory
        traj_calc_z = Trajectory(v_max_z, a_max_z, d_max_z) 

        self.fruit_travel_matrix = np.zeros((self.N, self.N), dtype=float )  
        # fill the diagonal with np.inf (so it can never be used)
        np.fill_diagonal(self.fruit_travel_matrix, np.inf)

        for i in range(1,self.N):
            for j in range(i):
                if i != j:
                    start_y = self.sortedFruit[1,i]  # in m, fruit i y_coordinate
                    start_z = self.sortedFruit[2,i]  # in m, fruit i z_coordinate

                    end_y   = self.sortedFruit[1,j]  # in m, fruit j y_coordinate
                    end_z   = self.sortedFruit[2,j]  # in m, fruit j z_coordinate

                    # calculate extension to fruit in y-axis
                    traj_calc_y.adjInit(start_y, 0.) # start moving from zero speed
                    traj_calc_y.noJerkProfile(traj_calc_y.q0, end_y, traj_calc_y.v0, v_max_y, a_max_y, d_max_y)
                    T_y = traj_calc_y.Ta + traj_calc_y.Tv + traj_calc_y.Td 
                    
                    # calculate extension to fruit in z-axis
                    traj_calc_z.adjInit(start_z, 0.) # start moving from zero speed
                    traj_calc_z.noJerkProfile(traj_calc_z.q0, end_z, traj_calc_z.v0, v_max_z, a_max_z, d_max_z)
                    T_z = traj_calc_z.Ta + traj_calc_z.Tv + traj_calc_z.Td 

                    # add to matrix and mirror it
                    self.fruit_travel_matrix[i,j] = max(T_y, T_z)
                    self.fruit_travel_matrix[j,i] = max(T_y, T_z)

        # print('travel times between fruits:')
        # print(self.fruit_travel_matrix)



    def calcDensity(self, Q, C, R, d_col, arm_reach, sortedFruit):
        '''Get the fruit density, d, of each cell'''
        ## should the columns be based on cell length? number of arms? 
        #  should the columns be the same width? increase/decrease the closer to the front of vehicle?
        #  should I calculate R per horizontal row of arms?

        density = np.zeros([R, C])  # total number of cells
        # starting position on the z-axis (up-down on robot)
        row_z = 0.

        for i_row in range(R):
            # starting position in the y_axis (front-back on robot)
            y_col = Q
            h_row = self.z_row_top_edges[0,i_row] - self.z_row_bot_edges[0,i_row]
            # print('Cell height for this', n, 'loop', h_row)
            # print('bottom', self.z_row_bot_edges[0,n], 'top', self.z_row_top_edges[0,n], '\n')

            for k in range(C):
                # print('col', n, 'row', k)
                # print('back', y_col, 'front', y_col + d_col)
                # print('bottom', row_z, 'top', row_z + h_row)
                index = np.where((sortedFruit[1,:] >= y_col) & (sortedFruit[1,:] < y_col + d_col) & 
                            (sortedFruit[2,:] >= row_z) & (sortedFruit[2,:] < row_z + h_row) & 
                            (sortedFruit[4,:] < 1))
                # save the number of fruit in this cell and divide all the values by the volume of space in front of each cell 
                density[i_row,k] = len(index[0]) / (arm_reach * d_col * h_row)

                # print(d)
                # move to the next column of cells
                y_col += d_col

            # move up to the next cell on this column
            row_z += h_row

        # before calculating the true density, check total number of fruit
        # print('which sums to', np.sum(d))   # has to be equal to numer of fruit
        # divide all the values by the volume of space in front of each cell 
        # d = d / (arm_reach * cell_l * h_row)

        # print('fruit density in each cell [fruit/m^3]:')
        # print(d)

        return(density)



    def calcR(self, V, fruit_in_horizon, d_h, h_v, arm_reach):
        '''Calculate the R value given a speed and horizon volume and density'''
        try:
            rho_hor = fruit_in_horizon / (d_h * h_v * arm_reach)
            t_hor    = d_h / V

            R         = rho_hor / t_hor # in fruit / (m^3 * s)

        except ZeroDivisionError:
            R         = 0 

        # print('Fruit incoming rate based on the horizon [fruit/(m^3 s)]:')
        # print(R)
        return(R) 
    


## Required classes for the arms, fruits, and jobs
class Arm():
    def __init__(self, arm_r, arm_c):
        # print(f'creating arm with r = %d and c = %d' %(arm_r, arm_c))
        self.arm_r = arm_r
        self.arm_c = arm_c

    # def __str__(self):
    #     return f"Arm: {self.arm_n}\n Horizontal Row Number: {self.row_n}"
    


class Fruit():
    def __init__(self, traj_calc, index, real_index, x_coord, y_coord, z_coord):#, job, tStart, tEnd, tDue):
        self.index = index            # fruit's index when ordered by y-coordinate
        self.real_index = real_index  # fruit's real index before it's zeroed if using snapshots
        self.x_coord = x_coord        # x-coordinate of the fruit
        self.y_coord = y_coord        # y-coordinate of the fruit
        self.z_coord = z_coord        # z-coordiante of the fruit
        # calculate the extension + grab + retract of the arm (use trajectory.py module)
        start_x = 0
        v_max   = 2 # m/s from Motor Sizing Google sheet calculations if we want to keep triangular profile to maximize speed
        a_max   = 4 # m/s^2 from Motor Sizing Google sheet calculations if we want desired linear movement time to be 1 second
        d_max   = a_max # if motors allow, keep equal to a_max

        # calculate extension to fruit in x-axis
        traj_calc.adjInit(start_x, 0.) # start moving from zero speed
        traj_calc.noJerkProfile(traj_calc.q0, self.x_coord, traj_calc.v0, v_max, a_max, d_max)
        
        self.Tx = traj_calc.Ta + traj_calc.Tv + traj_calc.Td  # in s, the total extensiontime for this fruit
        
        # self.Tx = t_ext + t_grab + t_ext 
        # print('the extension time of this fruit takes {:.3f} sec\n'.format(self.Tx))
        
    # def __str__(self):
    #     return f"Fruit Index: {self.index}\n  Y-axis location: {self.y_coord}\n"



class Job():
    def __init__(self, fruit_i, arm_cr, Q, V, d_col, pick_travel_l=0):
        self.fruit_i  = fruit_i
        self.arm_cr   = arm_cr
        self.V        = V
        offset        = (d_col - pick_travel_l) / 2  # in m, assume centered
        # k+1 was added because the MIP model in paper assumes k starts at 1
        # self.TW_start = (self.fruit_i.y_coord + (self.arm_cr.arm_c)*d_col + 1/2 * offset) / (v_vy/100)
        # self.TW_end   = (self.fruit_i.y_coord + (self.arm_cr.arm_c + 1)*d_col - 1/2 * offset) / (v_vy/100)

        # fix to start the back of the vehicle as (0,0)
        self.TW_start = (self.fruit_i.y_coord - (Q + (self.arm_cr.arm_c + 1)* d_col) ) / (V/100) 
        if self.TW_start < 0:
            # correction since it cannot be harvested at negative values, so make the window smaller by making it able to start at 0? -> might require the addition of q_vy_start
            self.TW_start = 0

        self.TW_end   = (self.fruit_i.y_coord - (Q + self.arm_cr.arm_c * d_col) ) / (V/100)
        if self.TW_end <= 0:
            # if TW_end is less than zero, the fruit cannot be harvested so just set it all to zero
            # was causing problems for the TW constraints because the max and min were not always the start and end. 
            self.TW_start = 0
            self.TW_end = 0


## create snapshot object for data analysis
class Snapshot(object):
    def __init__(self, n_col, n_row, horizon_l, vehicle_l, cell_l, Td, v_vy, FPE_g, FPEavg, FPT_g, FPTavg, y_lim, numFruit, curr_j, sortedFruit, fruit_picked_by, state_time):
        # constants for the whole run
        self.n_col           = n_col
        self.n_row           = n_row
        self.horizon_l       = horizon_l
        self.vehicle_l       = vehicle_l
        self.cell_l          = cell_l
            
        # constants and results for each snapshot in the run
        self.v_vy            = v_vy
        self.FPE             = FPE_g    # global value
        self.FPEavg          = FPEavg   # average value
        self.FPT             = FPT_g    # global value
        self.FPTavg          = FPTavg   # average value
        self.y_lim           = y_lim
        self.Td              = Td       # in s, handling time
        self.N_snap          = numFruit
        self.curr_j          = curr_j
        self.avg_PCT         = 0.
        self.state_time      = state_time
        self.fruit_picked_by = fruit_picked_by
        self.sortedFruit     = sortedFruit




# # def distCenterline(n_row, z_row_bot_edges, z_row_top_edges, sortedFruit):
# #     '''Calculate mean and variance of fruits to centerline of their respective row'''
# #     centerline   = np.zeros(n_row)
# #     sum_distance = np.zeros(n_row)
# #     # where_array_list = list()

# #     fruit_z = np.copy(sortedFruit[2,:])

# #     for row in range(n_row):
# #         # calculate centerline of row
# #         centerline[row] = (z_row_top_edges[0,row] - z_row_bot_edges[0,row])/2 + z_row_bot_edges[0,row]
# #         print('row', row, 'centerline z-coordinate', centerline[row])

# #         # check which fruits are in this rows
# #         this_row = np.where((sortedFruit[2,:] > z_row_bot_edges[0,row]) & (sortedFruit[2,:] < z_row_top_edges[0,row]))
# #         # where_array_list.append(this_row[0])

# #         for fruit_i in this_row[0]:
# #             fruit_z[fruit_i] = np.absolute(fruit_z[fruit_i] - centerline[row])

# #         # calculate row's mean and variance for distance of fruit from centerline 
# #         row_mean = np.mean(fruit_z[this_row[0]]) 
# #         row_var  = np.var(fruit_z[this_row[0]])
# #         print('row', row, 'mean', row_mean)
# #         print('row', row, 'variance', row_var)

# #     # print('sortedFruit', sortedFruit[2,:])
# #     # print()
# #     # print('distance from the centerline', fruit_z)
# #     # print()


# # def findClustersTotal(sortedFruit, v_vy, Td, n_col):
# #     '''
# #        Use k-d tree (scipy) to find clusters of fruits made up of n_col fruits at a v_vy*Td distance from each other
# #        in the y and z axis (add x when it actually matters)

# #        see https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.html#scipy.spatial.KDTree
# #        and https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.query_ball_tree.html#scipy.spatial.KDTree.query_ball_tree
# #     '''

# #     d_cluster = (v_vy * Td) /2 # find all the neighbors that can be picked, half cause radius(?)
# #     print()
# #     print('Distance used to find clusters:', d_cluster)
# #     print()

# #     problem_cluster_num = 0

# #     coordinates1 = np.copy(sortedFruit[0:2,:]).T
# #     coordinates2 = np.copy(sortedFruit[0:2,:]).T

# #     kd_tree1 = KDTree(coordinates1)   
# #     kd_tree2 = KDTree(coordinates2)

# #     indexes = kd_tree1.query_ball_tree(kd_tree2, r=d_cluster)

# #     print()
# #     # print('Number of pairs within the problem distance', len(indexes)) # this will always be = all because we're comparing
# #     # the fruit against itself? => yup, this is what happens
# #     print('Fruit index lists showing all neighbors within d distance from fruit index i', indexes)

# #     plt.figure(figsize=(6, 6))
# #     plt.plot(sortedFruit[1,:], sortedFruit[2,:], "ok", markersize=5)
# #     plt.xlabel('Distance along orchard row (m)')
# #     plt.ylabel('Height from ground (m)')
# #     color = ['blue', 'red', 'purple', 'chartreuse', 'black', 'aqua', 'pink', 'sienna', 'deepskyblue', 'teal', 'tomato', 'slategrey']
# #     color_index = 0

# #     for i in range(len(indexes)):
# #         if len(indexes[i]) > n_col+1: # because the list includes the fruit's index (tree compared to itself), so one extra
# #             problem_cluster_num += 1

# #             for j in indexes[i]:
# #                 # plot only the problem clusters
# #                 line_color = str(color[color_index])
# #                 plt.plot([sortedFruit[1,i], sortedFruit[1,j]], [sortedFruit[2,i], sortedFruit[2,j]], linestyle='-', color='r')

# #             color_index +=1 
# #             if color_index == 12:
# #                 color_index = 0

# #     print('Number of problem clusters', problem_cluster_num)

# #     plt.show()


# # def findClustersByRow(sortedFruit, v_vy, Td, n_col, n_row, z_row_bot_edges, z_row_top_edges):
# #     '''
# #        Use k-d tree (scipy) to find clusters of fruits made up of n_col fruits at a v_vy*Td distance from each other
# #        in the y and z axis (add x when it actually matters). Done individually for each row however the row was separated

# #        see https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.html#scipy.spatial.KDTree
# #        and https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.query_ball_tree.html#scipy.spatial.KDTree.query_ball_tree
# #     '''
# #     d_cluster = (v_vy * Td)  # find all the neighbors that can be picked, half cause radius(?)

# #     all_cluster_list     = list()
# #     fruits2remove_list   = list()
# #     # fruits2remove_list = [134,135,136,137,140,143]

# #     problem_cluster_num = 0

# #     coordinates1 = np.copy(sortedFruit[1:3,:]).T
# #     coordinates2 = np.copy(sortedFruit[1:3,:]).T

# #     for row in range(n_row):
# #         # check which fruits are in this rows
# #         this_row = np.where((sortedFruit[2,:] > z_row_bot_edges[0,row]) & (sortedFruit[2,:] < z_row_top_edges[0,row]))
# #         # print('this row indexes', this_row[0])

# #         # the indexes in the cut down coordinates array do not match the ones in coordinates, need to 'transform' them back
# #         kd_tree1 = KDTree(coordinates1[this_row[0],:])   
# #         kd_tree2 = KDTree(coordinates2[this_row[0],:])
# #         indexes = kd_tree1.query_ball_tree(kd_tree2, r=d_cluster)

# #         # transform the indexes back
# #         # print()
# #         for i in range(len(indexes)):
# #             all_cluster_list.append([this_row[0][i],this_row[0][indexes[i]]])   

# #     # print(all_cluster_list)

# #     plt.figure(figsize=(6, 6))
# #     plt.plot(sortedFruit[1,:], sortedFruit[2,:], "ok", markersize=5)
# #     plt.xlabel('Distance along orchard row (m)')
# #     plt.ylabel('Height from ground (m)')
# #     color = ['blue', 'red', 'purple', 'chartreuse', 'black', 'aqua', 'pink', 'sienna', 'deepskyblue', 'teal', 'tomato', 'slategrey']
# #     color_index = 0

# #     # print()
# #     # print('sorted fruits list')
# #     # print(sortedFruit)

# #     print()  # helps make the following prints cleaner 
# #     for cluster in all_cluster_list:
# #         if len(cluster[1]) > n_col:
# #             # if there are more fruits in the cluster than the number of arms+1
# #             problem_cluster_num += 1
# #             # get the correct index connected to the list of neighbors
# #             i = cluster[0]
# #             print('Fruits within d distance from fruit index',i, 'are', cluster[1])

# #             # tag these fruits as removed, going to remove the index fruit which is the one close to all the other ones
# #             # fruits2remove_list.append(i)

# #             for j in cluster[1]:
# #                 # plot only the problem clusters
# #                 line_color = str(color[color_index])
# #                 plt.plot([sortedFruit[1,i], sortedFruit[1,j]], [sortedFruit[2,i], sortedFruit[2,j]], linestyle='-', color=line_color)

# #             color_index +=1 
# #             if color_index == 12:
# #                 color_index = 0
                
# #     plt.show()

# #     return(fruits2remove_list)



        