######## RANDOM SEED GENERATOR ########
'''Use only when a new seed is needed'''
# https://numpy.org/devdocs/reference/random/index.html

from numpy.random import PCG64, SeedSequence

sq1 = np.random.SeedSequence()

seed = sq1.entropy

print(seed)

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
