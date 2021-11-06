from __future__ import division
import pinocchio as pin, math
import numpy as np
from matplotlib import pyplot as plt
# load the urdf file of your robot model
model = pin.buildModelFromUrdf('iiwa7_description.urdf')
print('model name: ' + model.name)
# Create data required by the algorithms
data = model.createData()
NQ = model.nq
NV = model.nv
print('Dimension of the configuration vector representation: ' + str(NQ))
print('Dimension of the velocity: ' + str(NV))
# calculate the total mass of the model, put it in data.mass[0] and return it
total_Mass = pin.computeTotalMass(model,data)
print('Total mass of the model: ', data.mass[0])
# Generating joint position, angular velocity and angular acceleration using quintic polynomials
# 1. Generating position q, initial: 0 deg, end: 60 deg, ouput:rad
def mypoly_p(t):
  p = np.poly1d([12*60*math.pi/180/(2*3**5),-30*60*math.pi/180/(2*3**4),20*60*math.pi/180/(2*3**3),0,0,0])
  return p(t)

q = np.zeros((61, 7))
for i in range(0, 61):
    for j in range(7):
        q[i, j] = mypoly_p(0 + 3 / 60 * i) # 61*7 matrix, each column represents a sequence of joint


# 2. Generating angular velocity qdot, initial: 0 rad/s, end: rad/s, ouput:rad/s
def mypoly_v(t):
  p = np.poly1d([5*12*60*math.pi/180/(2*3**5),-4*30*60*math.pi/180/(2*3**4),3*20*60*math.pi/180/(2*3**3),0,0])
  return p(t)

qdot = np.zeros((61, 7))
for i in range(0, 61):
    for j in range(7):
        qdot[i, j] = mypoly_v(0 + 3 / 60 * i) # 61*7 matrix, each column represents a sequence of joint

# 3. Generating angular acceleration qddot, initial: 0 rad/s^2, end: rad/s^2, ouput:rad/s^2
def mypoly_a(t):
  p = np.poly1d([4*5*12*60*math.pi/180/(2*3**5),-3*4*30*60*math.pi/180/(2*3**4),2*3*20*60*math.pi/180/(2*3**3),0])
  return p(t)

qddot = np.zeros((61, 7))
for i in range(0, 61):
    for j in range(7):
        qddot[i, j] = mypoly_a(0 + 3 / 60 * i) # 61*7 matrix, each column represents a sequence of joint

# Calculates the torque of each joint, return 1*7 vector
Torque = np.zeros((61, 7))
for i in range(0,61):
    tau = pin.rnea(model,data,q[i],qdot[i],qddot[i])   # 1*7 vector
    Torque[i][:] = tau.T
    print('The ' + str(i) + 'th Torque is: ',i,tau.T)

# Computes the generalized gravity contribution G(q), stored in data.g
G_Torque = np.zeros((61, 7))
for i in range(0,61):
    G_Tau = pin.computeGeneralizedGravity(model,data,q[i])
    G_Torque[i][:] = G_Tau
    print('The ' + str(i) + 'th G_Tau is: ', G_Tau)


# Computes the upper triangular part of the joint space inertia matrix M, stored in data.M
M_Matrix = np.zeros((61,7,7))
for i in range(0,61):
    M_Temp = pin.crba(model,data,q[i])
    M_Matrix[i,:,:] = M_Temp
    print('The ' + str(i) + 'th M_Matrix is: ', M_Temp)

#Computes the Coriolis Matrix C
C_Matrix = np.zeros((61,7,7))
for i in range(0,61):
    C_Temp = pin.computeCoriolisMatrix(model,data,q[i], qdot[i])
    C_Matrix[i,:,:] = C_Temp
    print('The ' + str(i) + 'th C_Matrix is: ', C_Temp)

# Verify the anti-symmetric property of dM/dt - 2* C, take the fifth sequence as example
M = pin.crba(model,data,q[5])
dt = 1e-8
q_plus = pin.integrate(model,q[5],qdot[5]*dt)
M_plus = pin.crba(model,data,q_plus)
dM = (M_plus - M)/dt
C = pin.computeCoriolisMatrix(model,data,q[5],qdot[5])
print('The ' + str(5) + 'th C_Matrix is: ', C)
A = dM - 2*C
print('A is: ', A)
res = A + A.T
print('res is: ', res)
