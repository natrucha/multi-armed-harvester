# Scheduling research for a multi-armed, apple harvester

Code used in my PhD dissertation research, [Improving Fruit Harvesting Speed with Multi-Armed Robots](https://escholarship.org/uc/item/2sm6z5mq). multi_run.sh is a simple bash script showing how to run the main code, main_scheduling.py, and the arguments that determine what is run. For example, the fifth argument determines which scheduling algorithm should be used. Note that any MIP scheduling is performed using Gurobi as the solver, and it requires a valid license to run.

No fruit localization datasets are currently included in the source code. Please send me a message to request access to the dataset of digitized orchard fruits. 