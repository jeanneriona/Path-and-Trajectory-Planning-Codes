import roboticstoolbox as rtb
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import spatialmath
from spatialmath import SE3
import matplotlib.pyplot as plt

### create model
a1 = float(input("a1 = ")) # FT 150
a2 = float(input("a2 = ")) #FT 80
a3 = float(input("a3 = ")) #FT 70


def mm_to_meter(a):
    m = 1000
    return a/m

a1 = mm_to_meter (a1)
a2 = mm_to_meter (a2)
a3 = mm_to_meter (a3)


lm1 = float(input("lm1 = "))
lm1 = mm_to_meter(lm1)


# create Links
# [Robot variable] = DHRevolute(d,r/a,alpha,offset)
Arti_Elbow = DHRobot([
    RevoluteDH(a1,0,(90.0/180.0)*np.pi,0,qlim=[(-90.0/180.0)*np.pi,(90.0/180.0)*np.pi]),
    RevoluteDH(0,a2,0,0,qlim=[(-20.0/180.0)*np.pi,(90.0/180.0)*np.pi]),
    RevoluteDH(0,a3,0,0,qlim=[(-90.0/180.0)*np.pi,(90.0/180.0)*np.pi]),
    ], name='Articulated-Elbow')

def deg_to_rad(T):
    return (T/180)*np.pi

q0  = np.array([0,0,0])

q1  = np.array(([deg_to_rad(float(input("T1 = "))),
                 deg_to_rad(float(input("T2 = "))),
                 deg_to_rad(float(input("T3 = ")))]))

q2  = np.array(([deg_to_rad(float(input("T1 = "))),
                 deg_to_rad(float(input("T2 = "))),
                 deg_to_rad(float(input("T3 = ")))]))

q3  = np.array(([deg_to_rad(float(input("T1 = "))),
                 deg_to_rad(float(input("T2 = "))),
                 deg_to_rad(float(input("T3 = ")))]))

traj1 = rtb.jtraj (q0,q1,10)
traj2 = rtb.jtraj (q1,q2,10)
traj3 = rtb.jtraj (q2,q3,10)

# plot scale limits
x1 = -0.1
x2 = 0.1
y1 = -0.1
y2 = 0.1
z1 = -0.1
z2 = 0.1

# for Joint Variable vs Time(s) table
rtb.qplot(traj1.q)
rtb.qplot(traj2.q)
rtb.qplot(traj3.q)

#plot for trajectory
Arti_Elbow.plot(traj1.q, limits=[x1, x2, y1, y2, z1, z2])
Arti_Elbow.plot(traj2.q, limits=[x1, x2, y1, y2, z1, z2])
Arti_Elbow.plot(traj3.q, limits=[x1, x2, y1, y2, z1, z2], block=True)

print(Arti_Elbow)
#Arti_Elbow.teach(jointlabels=1)
