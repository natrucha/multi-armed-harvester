#!/usr/bin/python
import csv
import numpy as np
import matplotlib.pyplot as plt
import array as arr

n = 150

#ar = 16
#vr = 1
#dr = 16

Ta = 0.0532
Tv = 0.0640
Td = 0.0532

Tt = Ta + Tv + Td
#dT = Tt/n

t = np.zeros(n)
v = np.zeros(n)
q = np.zeros(n)
a = np.zeros(n)

t[0] = 0
v[0] = 0.5
q[0] = 0
a[0] = 0

d = 0

# for i in range(1,n):
#     t[i] = t[i-1] + dT
#     if ( t[i] <= abs(Ta) ):
#       v[i] = v[0] + ar*t[i]
#
#     elif ( t[i] <= (abs(Ta) + abs(Tv)) ):
#       v[i] = vr
#
#     elif ( t[i] <= (abs(Ta) + abs(Tv) + abs(Td)) ):
#       v[i] = vr - dr*(t[i] - (abs(Ta)+abs(Tv)))
#
#
#     # calculate the acceleration at each time point
#     # a[i] = (v[i] - v[i-1]) / (t[i] - t[i-1])
#
#     # calculate the displacement at each time point
#     q[i] = v[i-1]*(t[i] - t[i-1]) + d
#     d = q[i]
#
#
# plt.subplot(2, 1, 1)
#
# plt.plot(t,v)
# plt.title('Velocity vs time trapezoidal profile')
# plt.xlabel('time (sec)')
# plt.ylabel('velocity (cm/s)')
# plt.grid(True)
#
# plt.subplot(2, 1, 2)
# plt.plot(t,q)
# plt.title('Displacement vs time trapezoidal profile')
# plt.xlabel('time (sec)')
# plt.ylabel('displacement (cm)')
# plt.grid(True)
#
# plt.show()

Tja = 0.0532
Tjv = 0.0532
Tjd = 0.0532

Tj = Tt + Tjd

dT = Tj/n

ar = 13.3042
vr = 0.7080
dr = 13.3042
jm = 250

for i in range(1,n):
    t[i] = t[i-1] + dT
    if t[i] <= Tja:
        a[i] = a[0] + jm*t[i]

    elif t[i] <= Ta:
        a[i] = ar

    elif t[i] <= Tjv + Ta:
        a[i] = ar - jm*(t[i]-Ta)

    elif t[i] <= Tv + Ta:
        a[i] = 0

    elif t[i] <= Tjd + Tv + Ta:
        a[i] = -jm*(t[i]-(Tv + Ta))

    elif t[i] <= Td + Tv + Ta:
        a[i] = -dr

    elif t[i] <= Tjd + Td + Tv + Ta:
        a[i] = -dr + jm*(t[i]-(Tv + Ta + Td))

    # calculate the velocity at each time point
    v[i] = a[i-1]*(t[i] - t[i-1]) + v[i-1]

    # calculate the displacement at each time point
    q[i] = v[i-1]*(t[i] - t[i-1]) + d
    d = q[i]


plt.subplot(3, 1, 1)
plt.plot(t,a)
plt.xlabel('time (sec)')
plt.ylabel('acceleration (cm/s^2)')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(t,v)
plt.xlabel('time (sec)')
plt.ylabel('velocity (cm/s)')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(t,q)
plt.xlabel('time (sec)')
plt.ylabel('displacement (cm)')
plt.grid(True)

plt.show()