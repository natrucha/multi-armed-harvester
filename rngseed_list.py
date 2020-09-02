import csv

import numpy as np
from numpy.random import PCG64, SeedSequence  # random number seed generator based on PCG64
import time                  # use OS clock


'''
   Code to get a CSV file with seeds for RNG of fruit distributions.

   File should not be overwritten because testing should always be done using
   the same seeds. Just needed many to get multiple distributions.
'''

N       = 200
per_run = 6

seeds = np.zeros((N, per_run))

for s in range(per_run):
    for runs in range(N):
        seed = np.random.SeedSequence().entropy
        seeds[runs,s] = seed

print(seeds)

np.savetxt('rngseed_list.csv', seeds, delimiter=",")
