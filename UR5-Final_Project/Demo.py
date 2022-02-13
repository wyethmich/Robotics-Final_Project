import numpy as np

import matplotlib.pyplot as plt

import modern_robotics as mr
from serialRobot import *
from UR5 import UR5

# E399 Robotics I Final Project
# By Ethan Nguyen and Wyeth Michaelsen

def inverseKin(se3Res, prevTheta = np.zeros(6), eomg = 0.1, ev = 0.1):
  thetaResCir = []

  for i in range(len(se3Res)): 
      thetalist, success = mr.IKinBody(UR5.BList, UR5.Mb, se3Res[i], prevTheta, eomg, ev)
      thetaResCir.append(thetalist)
      prevTheta = thetalist.copy()
  return thetaResCir

def jointTrag(targetJointAngles, currJointAngles, frameInst = 100):
  # Mod the Current Angles to find the relative targets for trajectory planning
  curr_pos_mod = np.mod(currJointAngles+np.pi, 2*np.pi)-np.pi
  targ_pos_mod = np.mod(targetJointAngles+np.pi, 2*np.pi)-np.pi

  joinTraj = mr.JointTrajectory(curr_pos_mod, targ_pos_mod, 2.5, frameInst,5)
  return joinTraj

def movePositions(Xstart, Xend, currAngles, frameInst = 100, quarterInt = 2.5):
  se3Res = mr.ScrewTrajectory(Xstart,Xend,quarterInt,frameInst,5)
  screwAngleResult = inverseKin(se3Res,currAngles)
  joinTrag = jointTrag(screwAngleResult[-1], currAngles)
  return joinTrag, screwAngleResult[-1]

def plotPose():
  fig, ax = plt.subplots(1, 1, figsize=(7.5, 7.5), subplot_kw={'projection': '3d'})
  ax.view_init(azim=45)
  plt.sca(ax)
  ax.set_xlim3d(-0.7, 0.7)
  ax.set_ylim3d(-0.7, 0.7)
  ax.set_zlim3d(0, 1)

  ax.set_xlabel('x')
  ax.set_ylabel('y')
  ax.set_zlabel('z')

  UR5.plot(ax=ax)
  plt.show()

homeAngles = np.array([0.0 , -2.3, 2, 0.25, 1.55, 0])
homeFrame = np.array([[-0.02076884, -0.04997917,  0.99853429,  0.17847333],\
        [ 0.99978376,  0.         , 0.02079483 , 0.11086141],\
        [-0.00103931 , 0.99875026 , 0.04996836 , 0.4315822 ],\
        [ 0.    ,      0.       ,   0.      ,    1.        ]])

'''UR5.pose(homeAngles)
print(UR5.Tb)
print(UR5.TList)
#plotPose()
'''
# [0,6,3,7]

posSequence = np.random.randint(0,11,20) #[0,1,2,6,7,8,9,10,11,4,7,5,6,2,10,5,7,2,0,3]


# Generate Main Position Frames
objPos = [(-0.4,-0.4, 0.7),  (0,  -0.4, 0.7),  (0.4,  -0.4, 0.7),  (-0.4,  0.4, 0.7),  (0,  0.4, 0.7),  (0.4,  0.4, 0.7),  (-0.4,  -0.4,  0.4), (0, -0.4, 0.4), (0.4, -0.4, 0.4), (-0.4, 0.4, 0.4), (0, 0.4, 0.4), (0.4, 0.4, 0.4)]

leftOrient = np.array([[-1.  , 0.  , 0. ],\
       [0.  , 0.  , 1.  ],\
       [0.  , 1.  , 0.  ],\
       [0.  , 0.  , 0.  ]])

rightOrient = np.array([[1.  , 0.  , 0. ],\
       [0.  , 0.  , -1.  ],\
       [0.  , 1.  , 0.  ],\
       [0.  , 0.  , 0.  ]])

# Array to hold the frames
posFrames = []
iPosFrames = []

for i in range(12):
  pos = np.append(objPos[i], 1)
  if pos[1] < 0:
    # Right Side positions    
    frame = np.insert(rightOrient, 3, pos, 1)
  else: 
    # Left Side positions
    frame = np.insert(leftOrient, 3, pos, 1)
  posFrames.append(frame)

# Generate Intermediary Frames
for i in range(12):
  pos = np.append(objPos[i], 1)
  # Adjust Y position
  if pos[1] < 0:
    # Right Side positions        
    pos[1] = pos[1] + 0.2
    frame = np.insert(rightOrient, 3, pos, 1)
  else:
    # Left Side positions
    pos[1] = pos[1] - 0.2
    frame = np.insert(leftOrient, 3, pos, 1)
  iPosFrames.append(frame)

'''
for i in range(len(posFrames)):
  print("Frame ",i)
  print(posFrames[i])
'''

# Check Have Input
if len(posSequence) < 1:
  print("Error Input has length 0")
  quit()


results = []
currAngles = homeAngles

print("Generating Move 0")
sectRes, currAngles = movePositions(homeFrame, posFrames[posSequence[0]], homeAngles)
for j in range(sectRes.shape[0]):
  results.append(sectRes[j,:].copy())

print(currAngles)

if len(posSequence) != 1:
  for i in range(1, len(posSequence)):
    print("Generating Move", i)
    # Move to Actual Position
    sectRes, currAngles = movePositions(posFrames[posSequence[i-1]], posFrames[posSequence[i]], currAngles)
    for j in range(sectRes.shape[0]):
      results.append(sectRes[j,:].copy())
    print(currAngles)

print("Generating Return to Home")
sectRes = jointTrag(homeAngles, currAngles)
for j in range(sectRes.shape[0]):
  results.append(sectRes[j,:].copy())

print("Compiling Results")
# Prep the Results For animation
aniThetas = np.transpose(results)
aniThetas = np.mod(aniThetas+np.pi, 2*np.pi)-np.pi
np.savetxt("Final.csv", aniThetas, delimiter=",")
np.savetxt("coppelia_final.csv", np.transpose(aniThetas), delimiter=",")

# Animate the Result
'''
fig, ax = plt.subplots(1, 1, figsize=(7.5, 7.5), subplot_kw={'projection': '3d'})
ax.view_init(azim=45)
plt.sca(ax)
ax.set_xlim3d(-0.7, 0.7)
ax.set_ylim3d(-0.7, 0.7)
ax.set_zlim3d(0, 1)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

UR5.animate(aniThetas, fps=40, ax=ax)
UR5.plot(ax=ax)
plt.show()
'''