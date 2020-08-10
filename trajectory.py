import numpy as np
import math

class Trajectory(object):
    def __init__(self, vm, am, dm):
        ####### CLASS VARIABLES ########
        # kinematic bounds
        self.v_max = vm  # Shared instance member :D
        self.a_max = am
        self.d_max = dm
        self.j_max = 250

#         self.v_max = 1  # Shared instance member :D
#         self.a_max = 16
#         self.d_max = 16
#         self.j_max = 250

        # initial values
        self.q0 = 0
        self.v0 = 0
        self.qe = 0

        # jerk limited profile initial conditions
        self.q0f = 0     # initial location
        self.v0f = 0     # initial velocity
        self.a0f = 0     # initial acceleration

        self.delta_v = 0
        self.delta_q = 0

        # reached values of velocity and acceleration
        self.vr = 0
        self.ar = 0
        self.dr = 0

        # duration time of each of the three stages for acceleration limited profile
        self.Ta = 0
        self.Tv = 0
        self.Td = 0
        self.Tt = 0 # Total AL time

        # duration time of each of the added stages for jerk limited profile
        self.Tja = 0
        self.Tjv = 0
        self.Tjd = 0
        self.Tjt = 0 # Total JL time

        # total distance needed to travel
        self.Dq = 0
        # min distance for which max velocity is reached
        self.Dq_v_max = 0

        self.s = 1


    ####### FUNCTIONS ########
    #/* ***************************** */#
    #/* Determine the sign of a value */#
    #/* ***************************** */#
    def checkSign(self, value):
        sign = 1
        if value < 0:
            sign = -1

        return sign


    #/* ********************************************************** */#
    #/* Calculate the trapezoidal profile times and reached values */#
    #/* ********************************************************** */#
    def trapProfile(self, v, a, d, s):
#         print("trap function inside, a:", a, "v:", v, "d:", d)
        self.vr = self.s * v
        self.ar = self.s * a
        self.dr = self.s * d

        self.Ta = (self.vr - self.v0) / self.ar
        self.Td = self.vr / self.dr
        self.Tv = (self.Dq - self.s*self.Dq_v_max) / self.vr


    #/* ********************************************************* */
    #/* Calculate the triangular profile times and reached values */
    #/* ********************************************************* */
    def triangProfile(self, v, a, d, s):
        self.vr = self.s * math.sqrt( (2*abs(a)*abs(d)*abs(self.Dq) - abs(d)*self.v0**2) / (abs(a) + abs(d)) )
        self.ar = self.s * a
        self.dr = self.s * d

        self.Ta = (self.vr - self.v0) / self.ar
        self.Td = self.vr / self.dr
        self.Tv = 0


    #/* ********************************************************************** */
    #/* Calculate the acceleration limited profile (triangular or trapezoidal) */
    #/* ********************************************************************** */
    def noJerkProfile(self, q0, qe, v0, vm, am, dm):
        # print("In the function!")
        self.q0 = q0
        self.v0 = v0
        self.qe = qe

        # print("start, end, start velocity:", self.q0, self.qe, self.v0)

        sign = 1
#         Dq_stop = self.v0**2 / (2 * self.d_max) # minimum stopping distance
        Dq_stop = self.v0**2 / (2 * dm) # minimum stopping distance
        self.Dq = self.qe - self.q0

        # check if a full stop trajectory is needed
        if (abs(self.Dq) <= Dq_stop):
            sign = self.checkSign(self.v0)
            self.v0 = 0
            self.q0 = self.q0 + sign*Dq_stop

        # special case found when v_max is changed on the fly
        if (abs(self.v0) > vm):
            sign = self.checkSign(self.v0)
            self.v0 = sign*vm
            Dq_v0 = (abs(self.v0) - vm)**2 / (2*dm) # minimum stopping velocity
            self.q0 = self.q0 + sign*Dq_v0

        self.Dq_v_max = ( (self.v_max**2-self.v0**2) / (2*self.a_max) ) + (self.v_max**2 / (2*self.d_max))

        self.s = self.checkSign(self.Dq) # direction (sign) of trajectory

        # check if the profile is trapezoidal or triangular
        if (abs(self.Dq) >= self.Dq_v_max):
            self.trapProfile(vm, am, dm, self.s)
        else:
            self.triangProfile(vm, am, dm, self.s)

        self.Tt = abs(self.Tv) + abs(self.Ta) + abs(self.Td); # calculate the total time for displacement


    #/* ********************************************************************************* */
    #/* Adjust the initial velocity and displacement to fit FIR filter current conditions */
    #/* ********************************************************************************* */
    def adjInit(self, q0_curr, v0_curr):
        self.v0f = v0_curr
        self.q0f = q0_curr

        self.delta_v = self.s*self.a0f*( (2*self.ar-self.a0f) / self.j_max )

        self.delta_q = self.s*(self.ar*self.v0f - (self.ar+self.dr)*self.vr) / (2*self.j_max) + (self.a0f**2)*( (3*self.ar-2*self.a0f) / (12*self.j_max**2) )

#         print("delta_q:", self.delta_q, "delta_v:", self.delta_v)

        self.q0 = self.q0f
        self.v0 = self.v0f

#         self.q0 = self.q0f + self.delta_q
#         self.v0 = self.v0f + self.delta_v

#         print("q0f:", self.q0f, "v0f:", self.v0f)


    #/* *************************************************** */
    #/* Calculate the jerk Time modifications Tja, Tjv, Tjd */
    #/* *************************************************** */
    def TjCalc(self):
        self.Tjv = abs(self.ar) / self.j_max
        self.Tja = abs(self.ar-self.a0f) / self.j_max
        self.Tjd = abs(self.dr) / self.j_max


    #/* ***************************************************************** */
    #/* Check if adaptations are needed for any of the trapezoidal values */
    #/* ***************************************************************** */
    def adaptation(self):
        # check velocity adaptations (type II adaptations)
        self.adaptationVelocity()
        self.TjCalc()

        vr_v = self.vr
        Tjv_v = self.Tjv

        # clear results
#         self.noJerkProfile(self.q0, self.qe, self.v0, self.v_max, self.a_max, self.d_max)
#         self.TjCalc()

        # check acceleration adaptations (type III adaptations)
        self.adaptationAccel()

        # combine the two
#         self.vr = vr_v
#         self.Tjv = Tjv_v


    #/* ****************************************************************** */
    #/* Check if adaptations are needed for the trapezoidal velocity value */
    #/* ****************************************************************** */
    def adaptationVelocity(self):
        if (self.Tv < self.Tjv):
            # Type II/IV adaptation
            print("Type II adaptation!")
            self.vr = self.j_max * (self.Dq - self.s*self.Dq_v_max) / abs(self.ar)
            # iterate over noJerkProfile
            self.noJerkProfile(self.q0, self.qe, self.v0, self.vr, self.ar, self.dr)


    #/* ********************************************************************************* */
    #/* Check if adaptations are needed for either of the trapezoidal acceleration values */
    #/* ********************************************************************************* */
    def adaptationAccel(self):

        # check if we need a Type III adaptation
        if (self.Ta < self.Tja and self.Td < self.Tjd):
            # Type c adaptation
            print("Type c adaptation!")
            self.ar = (self.a0f + self.s*math.sqrt( (self.a0f**2)+4*self.j_max*(abs(self.vr)-self.v0) ))/2
            self.dr = self.s*math.sqrt(self.j_max*abs(self.vr))
            # iterate over noJerkProfile
            self.noJerkProfile(self.q0, self.qe, self.v0, self.vr, self.ar, self.dr)

        elif (self.Ta < self.Tja):
            # Type a adaptation
            print("Type a adaptation!")
            self.ar = (self.a0f + self.s*math.sqrt( (self.a0f**2)+4*self.j_max*(abs(self.vr)-self.v0) ))/2
            # iterate over noJerkProfile
            self.noJerkProfile(self.q0, self.qe, self.v0, self.vr, self.ar, self.dr)

        elif (self.Td < self.Tjd):
            print("Type b adaptation!")
            # Type b adaptation
            self.dr = self.s*math.sqrt(self.j_max*abs(self.vr))
            # iterate over noJerkProfile
            self.noJerkProfile(self.q0, self.qe, self.v0, self.vr, self.ar, self.dr)


    #/* ************************************************************************************** */
    #/* Derive the difference between non-filtered and filtered velocity and position profiles */
    #/* ************************************************************************************** */
    def jerkProfile(self):

        self.TjCalc()

        self.adaptation()

        self.TjCalc()

#         self.adjInit()

        self.Tt = abs(self.Tv) + abs(self.Ta) + abs(self.Td)
        self.Tjt = self.Tt + self.Tjd
