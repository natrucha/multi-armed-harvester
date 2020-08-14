from enum import Enum
import json # configuration file encode and decode
# see https://realpython.com/python-json/#decoding-custom-types
# https://docs.python.org/3/library/json.html
import os   # to save files in a different directory
import time # to save the file with the current date

## Set Flag values
class spaceConf(Enum):
    '''Flag values for if the workspace in each row is individual or shared'''
    INDIVIDUAL = 0
    SHARED     = 1

class calendar(Enum):
    '''Flag values for the type of scheduler to use, name seems like a bad idea :P'''
    SINGLE_FRUIT = 0
    EDF          = 1 # Earliest Deadline First, batch

class treeCreation(Enum):
    '''Flags values for the type of data being used as fruit coordinates'''
    CSV_RAJ  = 0
    SYN_LINE = 1
    UNIFORM  = 2

class noiseMaker(Enum):
    TURN_OFF = 0
    TURN_ON  = 1

class reality(Enum):
    '''Flag values determining how many fruit are real and how many are fake in the simulation'''
    TURN_OFF  = 0
    DENSITY   = 1

class simulation_config(object):
    def __init__(self):
        '''
           Function that takes initial parameters for the harvester simulator and converts
           it to JSON. Useful when working with Monte Carlo because it can automate changing
           and saving the parameters.
        '''
        ###### Parameter settings, will later become a configuration file ######
        # fruit row depth, in ft  -- x-axis
        self.fruit_row_ed   = 0.3 # how far the vehicle will be from the edges of the tree
        self.fruit_row_tk   = 1.3 # how far the arms can reach into the canopy
        # fruit row length, in ft -- y-axis
        self.fruit_row_st   = 0.
        self.fruit_row_end  = 25.
        # fruit row height, in ft -- z-axis
        self.fruit_row_bt   = 0.
        self.fruit_row_tp   = 9.
        # fruit density
        self.rho_real       = 1.
        # fake fruit density
        self.rho_fake       = 0.3

        # decide on the number of arms and rows
        self.num_arms     = 3           # set number of arms on robot, will determine the length of the robot (for now)
        self.num_row      = 3           # set the number of rows of arms

        # arm's max velocity and acceleration values apparently in ft/s
        self.max_v = 1.
        self.max_a = 10.

        # vehicle velocity, in ft/s
        self.v_vx = 0.0
        self.v_vy = 0.05

        # if semionline:
        self.n_goals = 20       # number of goals the semionline scheduler will look for

        # when working with the fruit ribbon, how high above the conveyors will the ribbon be
        self.ribbon_z = 1.

        # decide if the arms are in individual or shared spaces
        self.space_config = spaceConf.SHARED
        # decide on the type of scheduler
        self.appointment = calendar.EDF
        # decide what fruit distribution to use to create fruit coordinates
        self.data_config  = treeCreation.UNIFORM
        # decide if fake fruit will be added
        self.existance = reality.TURN_OFF
        # decide if noise should be added and the distribution
        self.noise_level = noiseMaker.TURN_OFF

        ## create the initial default JSON file (good as a check)
        self.convertJSON()


    def changeV_v(self, newV_v):
        '''
           Placeholder function used to test changes to the vehicle velocity.

           INPUT: new y-coordinate vehicle velocity
        '''
        self.v_vy = newV_v
        self.convertJSON()


    def monteCarlo(self):
        '''
           Function to run Monte Carlo simulation where a value is changes per parameter and a
           JSON file exported to be used by the simulation code
        '''
        print("Functionality not added yet")


    def convertJSON(self):
        '''
           Convert data into JSON format that is then sent to createFiles to turn it into a file.
        '''
        data = {'orchard': {'x': {'start': self.fruit_row_ed, 'end': self.fruit_row_tk},
                            'y': {'start': self.fruit_row_st, 'end': self.fruit_row_end},
                            'z': {'start': self.fruit_row_bt, 'end': self.fruit_row_tp},
                            'rho_real': self.rho_real,
                            'rho_fake': self.rho_fake   },
                'vehicle': {'num_arms': self.num_arms,
                            'num_rows': self.num_row,
                            'v_vx':     self.v_vx,
                            'v_vy':     self.v_vy,},
                'arms':    {'max_v': self.max_v,
                            'max_a': self.max_a},
                'num_goals':    self.n_goals,
                'ribbon_z':     self.ribbon_z,
                'space_config': self.space_config.value,
                'appointment':  self.appointment.value,
                'data_config':  self.data_config.value,
                'existance':    self.existance.value,
                'noise_level':  self.noise_level.value
                }

        # Use the data dictionary to create a JSON file, and save the used parameters in
        # a new folder
        self.createFiles(data)


    def createFiles(self, data):
        '''
           Create the same json file that will be instantly opened by simulator
           code.

           INPUT: dict  with all the parameters that will be encoded into JSON
        '''
        with open('data.json', 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=4, sort_keys=True)

        ##### ALLOWS SAVING THE CONFIGURATION FILE (add later)
        # save = input('Would you like to save the JSON configurations in seperate file? (y or n)')      # If you use Python 3
        #
        # if save == "y":
        #     timestr   = time.strftime("%Y%m%d")
        #     timestr_f = time.strftime("%Y%m%d-%H%M%S")
        #
        #     # create file path to document the parameters used
        #     path = 'json_files' + timestr
        #
        #     if not os.path.exists(path):
        #         os.makedirs(path)
        #
        #     filename = 'data-in-'+timestr_f+'.json'
        #
        #     # create a json file that gets saved
        #     with open(os.path.join(path, filename), 'w', encoding='utf-8') as f:
        #         json.dump(data, f, ensure_ascii=False, indent=4, sort_keys=True)
        #
        # elif save == "n":
        #     print("File was not saved. Thank you for using our services!")
        #
        # else:
        #     print("Invalid answer, default set to not save the file.")
