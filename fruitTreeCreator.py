import numpy as np
from numpy.random import PCG64

class fruitTreeCreator(object):
    def __init__(self, y_lim, seeds):
        '''
           Class that can create synthetic distributions to populate a 6D matrix of coordinates for the
           simulator. Uses inputted densities to create desired fruit rows/robot arm rows .

           INPUT:  list made up of the starting and ending y-coord of fruit row

           OUTPUT: 6D matrix with fruit x, y, z-coordinates, state of the fruit (doesn't exist,
                   picked, scheduled, etc.), the arm scheduled to pick the fruit, and if it's a real
                   fruit
        '''
        ######################### FLAGS #########################
        # Flag values that set if the fruit is real or not
        self.fruit_FAKE     = 0
        self.fruit_REAL     = 1

        # Flag values for the type of distribution
        self.dist_UNIFORM   = 0
        self.dist_LINE      = 1

        #################### CLASS VARIABLES ####################
        # fruit coordinate lists (for all the rows)
        self.x_fr  = []
        self.y_fr  = []
        self.z_fr  = []
        self.exist = []  # will populate with 0 or 1 depending if the fruit is "real" or not

        # fruit statistics
        self.tot_fruit    = 0  # total number of real fruit created
        self.fruit_in_row = [] # list of total number of real fruit in each row

        # y-limits for where the fruit can be added (x and z will depend on the robot configuration)
        self.y_lim = y_lim

        # rng seeds created through numpy SeedSequence()
        self.x_seed = PCG64(int(seeds[0]))
        self.y_seed = PCG64(int(seeds[1]))
        self.z_seed = PCG64(int(seeds[2]))
        # seeds for the fake fruit
        self.xf_seed = PCG64(int(seeds[3]))
        self.yf_seed = PCG64(int(seeds[4]))
        self.zf_seed = PCG64(int(seeds[5]))

        # # rng seeds created through numpy SeedSequence()
        # self.x_seed = PCG64(37428395352013185889194479428694397783)
        # self.y_seed = PCG64(13250124924871709375127216220749555998)
        # self.z_seed = PCG64(165440185943501291848242755689690423219)
        # # seeds for the fake fruit
        # self.xf_seed = PCG64(264090507119924891834746017829286837587)
        # self.yf_seed = PCG64(307175982666302731017951161793326853810)
        # self.zf_seed = PCG64(202459549346992037879433717317760015805)


    def fruitLine(self, num_rows, row_fruit_density, x_lim, z, fake_density):
        '''
           Function to place fruits in random locations on the x and y coordinates, based on the
           uniform distribution but a specific z-coordinate to test reaction when dealing with a
           "ribbon" of fruit (easy calculations)

           INPUT:  total number of rows of arms, list of desired fruit densities, length arm x-axis
                   range, z height of line

           OUTPUT: three lists with x, y, z-coordinates of the fruit
        '''

        # run the create fruit function to create all the real fruit
        self.createFruit(num_rows, row_fruit_density, x_lim, [z], 1)
        # run the create fruit function to create all the fake fruit
        self.createFruit(num_rows, fake_density, x_lim, [z], 0)


    def fruitUniform(self, num_rows, row_fruit_density, x_lim, z_lim, fake_density):
        '''
           Function to place fruits in random locations in all three coordiantes based on the uniform
           distribution. Default distribution for statistical tests, even though it is not a realistic
           fruit distribution.

           INPUT: total number of rows of arms, list of desired fruit densities, length x, z to get volume,
                  density of fake fruit to be added
        '''
        # run the create fruit function to create all the real fruit
        self.createFruit(num_rows, row_fruit_density, x_lim, z_lim, 1)
        # run the create fruit function to create all the fake fruit
        self.createFruit(num_rows, fake_density, x_lim, z_lim, 0)


    def createFruit(self, num_rows, density, x_lim, z_lim, isReal):
        '''
           Function to create the base uniform distribution coordinates in x, y, z for real or fake fruit
           according to a given density.

           INPUT:  the total number of rows, the density desired, the x and z limits to where fruit can go,
                   if the fruits created are real or not

           OUTPUT: four shared lists, all the x-coord, y-coord, and z-coord as well as if the fruit are
                   real or not
        '''
        # https://numpy.org/devdocs/reference/random/generated/numpy.random.Generator.uniform.html#numpy.random.Generator.uniform

        # convert from fruit density to number of fruit
        num_densities = len(density) # number of desired fruit densities in the fruit row

        len_x = x_lim[1] - x_lim[0]            # calculate length of the arm's reach in the row
        len_y = self.y_lim[1] - self.y_lim[0]  # calculate the total length of the fruit row
        # calculate length of the arm's reach in the row
        if len(z_lim) == 2:
            len_z  = z_lim[1] - z_lim[0]
            dist   = self.dist_UNIFORM

        elif len(z_lim) == 1:
            height = z_lim[0]
            len_z  = 1.
            dist   = self.dist_LINE

        len_sections = len_y / num_densities   # calculate how long each row section is (equal length for all)
        # y-limit start for the first section (updated in loop)
        y_sec_lim = [self.y_lim[0], self.y_lim[0]+len_sections]

        for section_n in range(num_densities):
            # num_fruit = density * volume
            numFruit = int(density[section_n] * (len_sections*len_x*len_z))

            # different seeds for real vs fake
            if isReal == self.fruit_REAL:
                x = np.random.default_rng(self.x_seed).uniform(x_lim[0], x_lim[1], numFruit)
                y = np.random.default_rng(self.y_seed).uniform(y_sec_lim[0], y_sec_lim[1], numFruit)
                # append number of real fruit into the lists
                self.fruit_in_row.append(numFruit)
                # add this row's fruit to the total number of created fruit
                self.tot_fruit += numFruit

                if dist == self.dist_UNIFORM:
                    z = np.random.default_rng(self.z_seed).uniform(z_lim[0], z_lim[1], numFruit)

                elif dist == self.dist_LINE:
                    z = np.full(len(x), height)

                # add an existance flag to the 6th array set at 1 for real
                add = np.ones(len(x))
                self.exist = np.concatenate((self.exist, add))

            elif isReal == self.fruit_FAKE:
                x = np.random.default_rng(self.xf_seed).uniform(x_lim[0], x_lim[1], numFruit)
                y = np.random.default_rng(self.yf_seed).uniform(y_sec_lim[0], y_sec_lim[1], numFruit)

                if dist == self.dist_UNIFORM:
                    z = np.random.default_rng(self.zf_seed).uniform(z_lim[0], z_lim[1], numFruit)

                elif dist == self.dist_LINE:
                    z = np.full(len(x), height)

                # add an existance flag to the 6th array set at 0 for !real
                add = np.zeros(len(x))
                self.exist = np.concatenate((self.exist, add))

            # put it all together
            self.x_fr = np.concatenate((self.x_fr, x))
            self.y_fr = np.concatenate((self.y_fr, y))
            self.z_fr = np.concatenate((self.z_fr, z))

            # calculate the next sections y-limits
            y_sec_lim = [y_sec_lim[0]+len_sections,y_sec_lim[1]+len_sections]


    def fruitTreeOut(self):
        '''
           Converts list of fruit into arrays to build a matrix used in simulator of the fruit coordinates.

           OUTPUT: 6D array with the three fruit coordinates, fruit picking state, arm scheduled to the fruit,
                   if the fruit is real
        '''
        # get total amount of fruits in distribution
        total_fruits = len(self.x_fr)

        picked  = np.zeros(total_fruits)
        arm     = np.full(total_fruits, np.inf) # will indicate which arm is set to this goal

        # need a matrix to sort x, y, and z based on the y-axis (to know what fruit show up earlier)
        fruit = np.stack([self.x_fr, self.y_fr, self.z_fr, picked, arm, self.exist])

        rowIndex = 1 # sort based on y-axis
        self.sortedFruit = fruit[ :, fruit[rowIndex].argsort()]
