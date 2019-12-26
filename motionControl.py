#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass

## [1] http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf
# https://pdfs.semanticscholar.org/29c1/c0f2cb6277d8fc120f29e4efc50b026289ec.pdf
# https://journals.sagepub.com/doi/full/10.5772/5652
## [2] http://www.abbmotion.com/support/SupportMe/Downloads/DocsLib/AN00115-003%20Trapezoidal%20Move%20Calculations.pdf
## [3] https://www.mdpi.com/2079-9292/8/6/652/pdf
## [4] https://pdfs.semanticscholar.org/b090/8986c0d43573a90f790c0480f6ec5c834262.pdf

# example motor: https://www.digikey.com/product-detail/en/trinamic-motion-control-gmbh/QBL4208-61-04-013/1460-1087-ND/4843438
# brushless motors: http://electrathonoftampabay.org/www/Documents/Motors/Brushless%20DC%20(BLDC)%20Motor%20Fundamentals.pdf

################## DATA CLASSES ########################
@dataclass
class axisValues:
    '''Class for keeping track of important values for each axis's motion.'''
    maxVel: float
    maxAccel: float
    maxJerk: float

@dataclass
class periodValues:
    x: float
    x_bar: float
    x_hat: float

##################### FUNCTIONS ##########################

def returnAxisMaxValues( ):
    # Specific to the motor being used
    # see "example motor"
    rpm = 4000      # rpm -> 1 RPM = 0.10472 rad/sec
    inertia = 48e-6 # kgm2
    torque = 0.125  # Nm

    maxVel   = rpm * 0.10472 /1000 # in rad/sec -> rad/msec
    maxAccel = torque / inertia /(1000*1000)# in rad/msec^2
    maxJerk  = maxAccel**2 / (maxVel - (v0/1000))# /(1000*1000*1000)# X.maxJerk >= Time / (2*X.maxAccel)
    return axisValues(maxVel, maxAccel, maxJerk)

def calculatePeriods( maxVel, maxAccel, maxJerk ):
    # calculate x and x_bar depending on the profile type
    # based on [4]
    x_hat = 0
    if profileType == 0:
        x = 2*maxAccel / maxJerk
        x_bar = 2*maxAccel / maxJerk
    elif profileType == 1:
        x = maxAccel / (2*maxJerk)
        x_bar = maxAccel / (2*maxJerk)
    elif profileType == 2:
        x = (maxVel - v0) / maxAccel + maxAccel / maxJerk
        x_bar = (maxVel - vf) / maxAccel + maxAccel / maxJerk
        x_hat = ( xDist - 0.5*(v0 + maxVel)*x - 0.5*(maxVel-vf)*x_bar ) / maxVel # from L = 0.5*(v0 + vp)*x + 0.5*(vp-vf)*x_bar + vp*x_hat
    return periodValues(x, x_bar, x_hat)

##################### MAIN FUNCTION #########################

if __name__ == '__main__':
    # import sys
    # if len(sys.argv) > 1:
    #     file_location = sys.argv[1].strip()
    #     with open(file_location, 'r') as input_data_file:
    #         input_data = input_data_file.read()
    #     print(solve_it(input_data)) # will need to be the fruit tree data?
    # else:
    #     print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/tsp_51_1)')
    # initial velocity
    v0 = 0 # will have to get from simulation
    vf = 0 # will probably always want to stop at the end

    # starting point in x, y for now
    start = [2, 0] # will need to get this from the simulation
    end   = [100, 6] # will need to get this from the simulation

    X = returnAxisMaxValues()
    Y = returnAxisMaxValues()

    ## calculate total time to travel using the trapezoidal curve
    # distance for x-coordinates
    xDist = np.sqrt( end[0]**2 - start[0]**2 )
    # distance for y-coordinates
    yDist = np.sqrt( end[1]**2 - start[1]**2 )

    # Calculate max trapezoidal time for x
    timeAccel_x   = X.maxVel / X.maxAccel
    timeDeaccel_x = X.maxVel / X.maxAccel
    # print("One-way time during acceleration for max velocity")
    # print(timeAccel_x)

    # Calculate distance travel during max accel/Deaccel to max velocity
    distAccel_x = 0.5 * timeAccel_x * X.maxVel
    distDeaccel_x = 0.5 * timeDeaccel_x * X.maxVel # if deaccel != accel
    # print("One way distance travelled during acceleration for max velocity")
    # print(distAccel_x)

    # Set default profile type (if no const velocity section but it does reach max velocity)
    profileType = 0

    # Set default variables for profile type 0
    finalVel_x = X.maxVel
    distConstVel_x = 0
    timeConstVel_x = 0

    if (distAccel_x + distDeaccel_x) > xDist:
        # calculate the actual "final velocity" at which deaccel needed
        distAccel_x = xDist / 2
        distDeaccel_x = xDist / 2
        finalVel_x  = X.maxAccel / distAccel_x
        # Calulate total travel time
        timeAccel_x = finalVel_x / X.maxAccel
        timeDeaccel_x = finalVel_x / X.maxAccel
        total_time_x = timeAccel_x + timeDeaccel_x
        profileType = 1

    elif (distAccel_x + distDeaccel_x) < xDist:
        # calculate the constant velocity section time and distance
        distConstVel_x = xDist - distAccel_x - distDeaccel_x
        timeConstVel_x = distConstVel_x / X.maxVel
        profileType = 2

    total_time_x = timeAccel_x + timeDeaccel_x + timeConstVel_x
    # total_dist_x = distAccel_x + distDeaccel_x + distConstVel_x # checks that final value equal to distance

    X_time = calculatePeriods( X.maxVel, X.maxAccel, X.maxJerk)

    print("x value: ", X_time.x, "x_bar value: ", X_time.x_bar, "x_hat value: ", X_time.x_hat )

    change = 0
    # Calculate when max acceleration has to be changed. Not quite correct for smaller values   
    if X.maxVel - v0 > 0.25*X.maxJerk*X_time.x**2 :
        X.maxAccel = 0.5*X.maxJerk*X_time.x*2
        change = 1

    if X.maxVel - vf > 0.25*X.maxJerk*X_time.x_bar**2 :
        X.maxAccel = 0.5*X.maxJerk*X_time.x_bar*2
        change = 1

    if change == 1:
        X_time = calculatePeriods( X.maxVel, X.maxAccel, X.maxJerk)

    print("x value: ", X_time.x, "x_bar value: ", X_time.x_bar, "x_hat value: ", X_time.x_hat )

    # Set the various timepoints
    T1 = X.maxAccel / X.maxJerk
    T2 = X_time.x - 2*T1
    if T2 <= 0:
        T2 = 0
    T3 = T1
    T4 = X_time.x_hat
    if T4 < 0:
        T4 = 0
    T5 = T1       # when symmetric accel and deaccel
    T6 = X_time.x_bar - 2*T5
    if T6 < 0:
        T6 = 0
    T7 = 2*T1 + T2 + T3 + T4 + T5 + T6  # only works when vo==vf and Accel==Deaccel
    print("time_1: ",T1,"time_2: ",T2,"time_3: ",T3)
    print("time_4: ",T4,"time_5: ",T5,"time_6: ",T6)

    Ts1 = T1 + T2
    Ts2 = Ts1 + T3
    Ts3 = Ts2 + T4
    Ts4 = Ts3 + T5
    Ts5 = Ts4 + T6

    print("final time smooth: ",T7, "final time trapezoid: ", total_time_x)

    ## plot distance versus time to check results
    time_x = [0, timeAccel_x, (timeConstVel_x + timeAccel_x), (timeConstVel_x + timeAccel_x + timeDeaccel_x)]
    dist_x = [0, distAccel_x, (distAccel_x + distConstVel_x), (distAccel_x + distConstVel_x + distDeaccel_x)]
    vel_x  = [v0, finalVel_x, finalVel_x, v0]
    # plt.plot(time_x, vel_x)
    plt.plot(time_x, dist_x)
    # plt.xlabel('Time [s]', fontsize=18)
    # plt.ylabel('Distance [u]', fontsize=16)
    # plt.show()

    ## S-curve setup
    # calculate the number of points for the s, v, and a arrays
    # numPoints = int(total_time_x / 5)#* 1000 / 5) # total time divided into 5 msec points
    numPoints = int(T7 / 5)
    print("num points:", numPoints)
    # print(numPoints)
    # set up the arrays
    s = np.zeros(numPoints)
    v = np.zeros(numPoints)
    a = np.zeros(numPoints)
    j = np.zeros(numPoints)

    t = np.zeros(numPoints)

    curveSection = 0

    for i in range(numPoints):
        t[i] = i * 5 # in msec
        if curveSection == 0:
            # Concave accel section
            if t[i] >= T1 and T2 > 0:
                print("time_1: ", t[i], "change: ", T1, "dist moved: ", s[i-1])
                # curveSection = 1
                curveSection = 10
                v1 = v[i-1]
                s1 = s[i-1]
            elif t[i] >= T1 and T2 <= 0:
                print("time_1: ", t[i], "change: ", T1, "dist moved: ", s[i-1])
                # curveSection = 1
                curveSection = 1
                v1 = v[i-1]
                v2 = v1
                s1 = s[i-1]
                s2 = s1
            else:
                j[i] = X.maxJerk
                a[i] = X.maxJerk*t[i]                   ##
                v[i] = v0 + X.maxJerk*t[i]**2 / 2       ##
                s[i] = v0*t[i] + X.maxJerk*t[i]**3 / 6  ##

        if curveSection == 10:
            if t[i] >= Ts1:
                print("time_2: ", t[i], "change: ", Ts1, "dist moved: ", s[i-1])
                curveSection = 1
                v2 = v[i-1]
                s2 = s[i-1]
            else:
                j[i] = 0                                                ##
                a[i] = X.maxAccel
                v[i] = v1 + X.maxAccel*(t[i]-T1)                        ##
                s[i] = s1 + v1*(t[i]-T1) + X.maxAccel*(t[i]-T1)**2 / 2  ##

        if curveSection == 1:
            if t[i] >= Ts2 and T4 > 0:
                print("time_3: ", t[i], "change: ", Ts2, "dist moved: ", s[i-1])
                curveSection = 2
                v3 = v[i-1]
                s3 = s[i-1]
            elif t[i] >= Ts2 and T4 <= 0:
                print("time_3: ", t[i], "change: ", Ts2, "dist moved: ", s[i-1])
                curveSection = 3
                v3 = v[i-1]
                v4 = v3
                s3 = s[i-1]
                s4 = s3
            else:
                # Convex accel period
                j[i] = -X.maxJerk
                a[i] = X.maxAccel - X.maxJerk*(t[i]-Ts1)                                              ##
                v[i] = v2 + X.maxAccel*(t[i]-Ts1) - X.maxJerk*(t[i]-Ts1)**2 / 2                 ##
                s[i] = s2 + v2*(t[i]-Ts1) + X.maxAccel*(t[i]-Ts1)**2 / 2 - X.maxJerk*(t[i]-Ts1)**3 / 6  ##

        if curveSection == 2:
            if t[i] >= Ts3:
                print("time_4: ", t[i], "change: ", Ts3, "dist moved: ", s[i-1])
                curveSection = 3
                v4 = v[i-1]
                s4 = s[i-1]
            else:
                # Constant velocity section
                j[i] = 0
                a[i] = 0                       ##
                v[i] = v3 #X.maxVel             ##
                s[i] = s3 + X.maxVel*(t[i]-Ts2) ##

        if curveSection == 3:
            # Convex deaccel period
            if t[i] >= Ts4 and T6 > 0:
                print("time_5: ", t[i], "change: ", Ts4, "dist moved: ", s[i-1])
                # curveSection = 4
                curveSection = 20
                v5 = v[i-1]
                s5 = s[i-1]
            elif t[i] >= Ts4 and T6 <= 0:
                print("time_5: ", t[i], "change: ", Ts4, "dist moved: ", s[i-1])
                # curveSection = 4
                curveSection = 4
                v5 = v[i-1]
                v6 = v5
                s5 = s[i-1]
                s6 = s5
            else:
                j[i] = -X.maxJerk
                a[i] = -X.maxJerk*(t[i]-Ts3)                           ##
                v[i] = v4 - X.maxJerk*(t[i]-Ts3)**2 / 2                ##
                s[i] = s4 + v4*(t[i]-Ts3) - X.maxJerk*(t[i]-Ts3)**3 / 6 ##

        if curveSection == 20:
            if t[i] >= Ts5:
                print("time_6: ", t[i], "change: ", Ts5, "dist moved: ", s[i-1])
                curveSection = 4
                v6 = v[i-1]
                s6 = s[i-1]
            else:
                j[i] = 0
                a[i] = -X.maxAccel                                              ##
                v[i] = v5 - X.maxAccel*(t[i]-Ts4)                   ##
                s[i] = s5 + v5*(t[i]-Ts4) - X.maxAccel*(t[i]-Ts4)**2 / 2      ##

        if curveSection == 4:
            # Concave deaccel period
            j[i] = X.maxJerk
            a[i] = -X.maxAccel + X.maxJerk*(t[i]-Ts5)                                                                  ##
            v[i] = v6 - X.maxAccel*(t[i]-Ts5) + X.maxJerk*(t[i]-Ts5)**2 / 2 ##
            s[i] = s6 + v6*(t[i]-Ts5) - X.maxAccel*(t[i]-Ts5)**2 / 2 + X.maxJerk*(t[i]-Ts5)**3 / 6

    print("calculated distance:", xDist,"distance moved:", s[numPoints - 1])

    # Plot values
    plt.plot(t, s)
    # plt.plot(t, v)
    # plt.plot(t, a)
    plt.xlabel('Time [msec]', fontsize=18)
    # plt.ylabel('Distance [u]', fontsize=16)
    plt.show()
