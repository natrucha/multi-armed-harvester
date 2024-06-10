import csv

import numpy as np
from numpy.random import PCG64, SeedSequence  # random number seed generator based on PCG64
import time                  # use OS clock


'''
   Code to get a CSV file with seeds for PCG64 RNG of fruit distributions.

   File should not be overwritten because testing should always be done using
   the same seeds. Just needed many to get multiple distributions.
'''

N             = 200
per_run       = 6
new_seed_flag = 0      # set = 1 to generate a new random seed value to generate the random seed values...

if new_seed_flag == 1:
    # only use when a new seed to create seeds is needed. Save this value if the need to create the same seed list is needed.
    # https://numpy.org/devdocs/reference/random/index.html
    sq1 = np.random.SeedSequence()
    seed_for_seeds = sq1.entropy
    print(seed_for_seeds)   # this only prints it, if you want to use it, add to random.SeedSequence()

    ## How to use seeds
    # ss = SeedSequence(seed)
    # print('seed = {}'.format(ss.entropy))
    # bg = PCG64(ss)

    ######## GENERATED SEEDS ##########
    # FRUIT TREE CREATOR SEEDS ##
    # self.x_fr - 37428395352013185889194479428694397783
    # self.y_fr - 13250124924871709375127216220749555998
    # self.z_fr - 165440185943501291848242755689690423219
    # self.x_fake - 264090507119924891834746017829286837587
    # self.y_fake - 307175982666302731017951161793326853810
    # self.z_fake - 202459549346992037879433717317760015805

seeds = np.zeros((N, per_run))

for s in range(per_run):
    for runs in range(N):
        seed = np.random.SeedSequence().entropy
        seeds[runs,s] = seed

print(seeds)

np.savetxt('rngseed_list.csv', seeds, delimiter=",")


