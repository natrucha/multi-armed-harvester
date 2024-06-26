#!/bin/bash

# Bash code to run the scheduler with set options
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


##################### RUN MULTIPLE MIP PYTHON SCRIPT #####################
## command line arguments
    # args[0] == n_col       number of columns of arms
    # args[1] == n_row       number of arms within each column
    # args[2] == n_runs      how many time a setting is run (e.g. for Monte Carlo runs)
    # args[3] == v_vy_lb     lower bound towing/vehicle speed or the single speed being tested in cmps
    # args[4] == v_vy_ub     upper bound towing/vehicle speed (set equal to v_vy_lb if only testing one velocity) in cmps
    # args[5] == algorithm
        # 0     == extended TOPTW MIP model with objective to maximize the number of harvested fruit, takes one velocity or a range of velocities to determine best FPE vs. FPT
        # 1     == "goal programming" TOPTW MIP model with the objective to maximize FPE*FPT, includes slack variables, minFPE, and minFPT
        # 2     == "epsilon-constraint" makespan TOPTW MIP model with the objective to minimize makespan, includes slack variable and minFPE
        # 3     == FCFS
        # 4     == FCFS to find speed lower bound, FPE*FPT to find schedule
        # 5     == FCFS to find speed lower bound, makespan to find schedule
        # 6     == SPT
    # args[6] == distribution
        # 0     == 2022 Digitized fruit data (Pink Ladies, Jeff Colombini orchard)
        # 1     == segments of 2022 digitized fruit data between a given ymin and ymax determined in fruit_handler.py createFruitDistribution(), runs through all 2022 Colombini files 
        # 2     == Raj's digitized fruits (right side)
        # 3     == Juan's digitized fruits (Stavros phone video)
        # 4     == reduced Raj's digitized fruits; can reduce the density to a desired value (density hardcoded currently to 20f/m^3)
        # 5     == reduced Juan's digitized fruits; can reduce the density to a desired value (density hardcoded currently to 20f/m^3)
        # 6     == uniform random  (if algorithm == 1, use IG melon paper version)
        # 7   * not available *  == uniform random, equal cell density
        # 8   * not available *  == multiple densities separated by some space (only melon for now)
        # 9   * not available *  == fruit in vertical columns
        # 10  * not available *  == "melon" version of columns (user inputs desired no. fruits, z height, and distance between fruit in y-coord)
    # args[7] == set_MPC
        # 0     == do not use sliding planning window method (for now, FPEmin = 0.35)
        # 1     == use sliding planning window method (for now, FPEmin = 0.95)
    # args[8] == set_edges
        # 0     == z-edges are divided equally along orchard height
        # 1     == z-edges are divided so each row has equal number of fruit (or close to equal)
    # args[9] == set_view_field 
        # 0     == robot sees the whole dataset
        # 1     == robot only sees what's in front of it
        # 2     == robot only sees what's in front of it plus a horizon
    # args[10] == segment_n (assuming d_w = 2.7 m) if set_distribution == 1
        # 0     == y-coordinates between 0    - 2.7 m
        # 1     == y-coordinates between 2.7  - 5.4 m
        # 2     == y-coordinates between 5.4  - 8.1 m
        # 3     == y-coordinates between 8.1  - 10.8 m
        # 4     == y-coordinates between 10.8 - 13.5 m
    # args[11] == fraction of d_vw/d_plan that will be used for D if set_distribution == 0, only ints for now

n_datasets=32 # the number of different fruit distributions/orchard rows being tested. Currently 32 total digitized fruit distributions (not included in github), but can also be used to make RNG distributions
n_segments=1  # the number of segments an orchard row is divided into. Each segment is solved individually. Currently, the max is 5 but index starts at 0; calc with d_w = d (13.95 / d_cell = n_segments)

set_algorithm=3
set_distribution=0
set_mpc=1
set_view_field=2

V_lb=1
V_hb=80

set_edges=1

# single runs
# python3 ./main_scheduling.py 3 3 $n_datasets $V_lb $V_hb 3 $set_distribution $set_mpc $set_edges $set_view_field 0
python3 ./main_scheduling.py 1 1 $n_datasets $V_lb $V_hb $set_algorithm $set_distribution $set_mpc $set_edges $set_view_field 0 1
python3 ./main_scheduling.py 1 1 $n_datasets $V_lb $V_hb $set_algorithm $set_distribution $set_mpc $set_edges $set_view_field 0 2
python3 ./main_scheduling.py 1 1 $n_datasets $V_lb $V_hb $set_algorithm $set_distribution $set_mpc $set_edges $set_view_field 0 3

for r_i in $(seq 1 3);
do
    # echo $c $r $segment_i
    python3 ./main_scheduling.py 3 $r_i $n_datasets $V_lb $V_hb $set_algorithm $set_distribution $set_mpc $set_edges $set_view_field 0 1
    python3 ./main_scheduling.py 3 $r_i $n_datasets $V_lb $V_hb $set_algorithm $set_distribution $set_mpc $set_edges $set_view_field 0 2
    python3 ./main_scheduling.py 3 $r_i $n_datasets $V_lb $V_hb $set_algorithm $set_distribution $set_mpc $set_edges $set_view_field 0 3
done

# for segment_i in $(seq 0 $n_segments);
# do
    # fcfs
    # python3 ./main_scheduling.py 1 1 $n_datasets $V_lb $V_hb 3 $set_distribution $set_mpc $set_edges $set_view_field $segment_i
    # python3 ./main_scheduling.py 1 3 $n_datasets $V_lb $V_hb 3 $set_distribution $set_mpc $set_edges $set_view_field $segment_i
    # python3 ./main_scheduling.py 2 3 $n_datasets $V_lb $V_hb 3 $set_distribution $set_mpc $set_edges $set_view_field $segment_i
    # python3 ./main_scheduling.py 3 3 $n_datasets $V_lb $V_hb 3 $set_distribution $set_mpc $set_edges $set_view_field $segment_i
# done


# for segment_i in $(seq 0 $n_segments);
# do
#     python3 ./main_scheduling.py 3 3 $n_datasets $V_lb $V_hb $set_algorithm $set_distribution $set_mpc $set_edges $set_view_field $segment_i
    # python3 ./main_scheduling.py 1 1 $n_datasets $V_lb $V_hb $set_algorithm $set_distribution $set_mpc $set_edges $set_view_field $segment_i
    # python3 ./main_scheduling.py 1 3 $n_datasets $V_lb $V_hb $set_algorithm $set_distribution $set_mpc $set_edges $set_view_field $segment_i
    # python3 ./main_scheduling.py 2 3 $n_datasets $V_lb $V_hb $set_algorithm $set_distribution $set_mpc $set_edges $set_view_field $segment_i
# done

# for c_i in $(seq 1 3);
# do
# 	for r_i in $(seq 1 3);
# 	do
# 		for segment_i in $(seq 0 $n_segments);
# 		do
# 			# echo $c $r $segment_i
# 			python3 ./main_scheduling.py $c_i $r_i $n_datasets $V_lb $V_hb $set_algorithm $set_distribution $set_mpc $set_edges $set_view_field $segment_i
# 		done
# 	done
# done



