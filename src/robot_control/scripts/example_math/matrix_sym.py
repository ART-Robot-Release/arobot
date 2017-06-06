from math import sin, cos
from sympy import *
import numpy as np

def getTheTranMatrix(orgCoor, roll, pitch, yaw, d):
    
    TM = Matrix([[1,0,0,d[0]], [0,1,0,d[1]],[0,0, 1, d[2]],[0,0,0,1]])
    Tx = Matrix([[1,0,0,0], [0, cos(roll), -1*sin(roll),0],[0, sin(roll), cos(roll), 0], [0,0,0,1]]);
    Ty = Matrix([[cos(pitch), 0, sin(pitch), 0], [0, 1, 0, 0],[-1*sin(pitch), 0, cos(pitch), 0], [0,0,0,1]]);
    Tz = Matrix([[cos(yaw), -1*sin(yaw),0, 0], [sin(yaw), cos(yaw), 0, 0],[0, 0, 1, 0], [0,0,0,1]]);

    return orgCoor*TM*Tx*Ty*Tz


rarr  = np.array([0, 0, np.pi/2, 0, -1*np.pi/2, 0]) # radian 
darr  = np.array([5,20,10,5]) # len
p = Matrix([[0],[0],[0],[1]]) # start point

th0 = Symbol('th0')
th1 = Symbol('th1')
d0 = Symbol('d0')

org = Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
d0_ = Matrix([0,0,d0])

A = getTheTranMatrix(org, th0,th1,0,d0_)
print (A)

th2 = Symbol('th2')
d1 = Symbol('d1')
d1_ = Matrix([0,0,d1])

B = getTheTranMatrix(A, 0,  th2, 0, d1_)
print (B)

th3 = Symbol('th3')
th4 = Symbol('th4')
th5 = Symbol('th5')
d2 = Symbol('d2')
d2_ = Matrix([0,0,d2])

C = getTheTranMatrix(B, th3,  th4, th5, d2_)
print
# print(simplify(C))


print p

p1 = A.subs([(th0,rarr[0]),(th1,rarr[1]),(d0, darr[0])]) * p # ankle
print (p1)

p2 = B.subs([(th0,rarr[0]),(th1, rarr[1]), (th2,rarr[2]),(d0, darr[0]), (d1, darr[1])]) * p # knee
print(p2)

p3 = C.subs([(th0,rarr[0]),(th1, rarr[1]), (th2,rarr[2]), (th3,rarr[3]), (th4,rarr[4]), (th5,rarr[5]),(d0, darr[0]), (d1, darr[1]), (d2, darr[2])]) * p # hip - 1
print(p3)

d3 = Symbol('d3')
hip2 = Matrix([0,d3,0,1]) # start point
p4 =  C.subs([(th0,rarr[0]),(th1, rarr[1]), (th2,rarr[2]), (th3,rarr[3]), (th4,rarr[4]), (th5,rarr[5]),(d0, darr[0]), (d1, darr[1]), (d2, darr[2])]) * hip2.subs(d3, darr[3])

print(p4)

# verify the inverse kinematics

def inverseKniematics(pfoot, phip1, phip2):
    d1 = darr[1]
    d2 = darr[2]
    rlo = phip1 - pfoot
    print rlo
    th0 = atan(-1*rlo[1]/rlo[2])
    th3 = -1*th0
    r2 = rlo[0]**2 + (rlo[2]/cos(th0))**2
    r = sqrt(r2)
    afa = acos((d1**2+d2**2-r**2)/2*d1*d2)
    th2 = np.pi - afa
    beta = acos((r2+d1**2-d2**2)/(2*d1*r))
    print r2+d1**2-d2**2
    print 2*d1*r
    print beta
    fi = atan(rlo[2]/cos(th0)/rlo[0])
    th1 = np.pi/2 - fi - beta
    th4 = -1*(th1 + th2)
    return [th0, th1, th2, th3, th4]

retr = inverseKniematics(p1, p3, p4)

print (retr)
print (rarr)


