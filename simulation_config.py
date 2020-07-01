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

        # decide on the number of arms and rows
        self.num_arms     = 3           # set number of arms on robot, will determine the length of the robot (for now)
        num_row      = 3           # set the number of rows of arms
        frame_height = 3.          # set the height of the arm's frame
        # frame height will be calculated from tree height and number of rows?

        # arm's max velocity and acceleration values apparently in ft/s
        max_v = 1.
        max_a = 10.
        # if semionline:
        n_goals = 20       # number of goals the semionline scheduler will look for

        # decide if the arms are in individual or shared spaces
        space_config = spaceConf.SHARED
        # decide on the type of scheduler
        appointment = calendar.EDF
        # decide what fruit distribution to use to create fruit coordinates
        data_config  = treeCreation.SYN_LINE
        # decide if fake fruit will be added
        existance = reality.TURN_OFF
        # decide if noise should be added and the distribution
        noise_level = noiseMaker.TURN_OFF

        ## Convert to JSON
        data = {'orchard': {'x': {'start': fruit_row_ed, 'end': fruit_row_tk},
                            'y': {'start': fruit_row_st, 'end': fruit_row_end},
                            'z': {'start': fruit_row_bt, 'end': fruit_row_tp}},
                'vehicle': {'num_arms': num_arms,
                            'num_rows': num_row,
                            'frame_height': frame_height},
                'arms':    {'max_v': max_v,
                            'max_a': max_a},
                'num_goals':    n_goals,
                'space_config': space_config.value,
                'appointment':  appointment.value,
                'data_config':  data_config.value,
                'existance':    existance.value,
                'noise_level':  noise_level.value
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

        save = input('Would you like to save in seperate file? (y or n)')      # If you use Python 3

        if save == "y":
            timestr   = time.strftime("%Y%m%d")
            timestr_f = time.strftime("%Y%m%d-%H%M%S")

            # create file path to document the parameters used
            path = 'json_files' + timestr

            if not os.path.exists(path):
                os.makedirs(path)

            filename = 'data-in-'+timestr_f+'.json'

            # create a json file that gets saved
            with open(os.path.join(path, filename), 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=4, sort_keys=True)

        elif save == "n":
            print("File was not saved. Thank you for using our services!")

        else:
            print("Invalid answer, default set to not save the file.")
