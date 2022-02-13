import numpy as np
from serialRobot import *

''' Universal Robot 5 (UR5) dimension'''
H1 = 0.089159
W1 = 0.13585
L1 = 0.425
W2 = 0.1197
L2 = 0.39225
W3 = 0.093
H2 = 0.09465
W4 = 0.0823

GRAY = [0.6, 0.6, 0.6]
BLACK = [0, 0, 0]

'''Link 1'''
R1 = np.identity(3)
p1 = np.array([0, 0, H1])
T1 = mr.RpToTrans(R1, p1)

S1 = np.array([0,0,1,0,0,0]) # Rot about z
lines1 = np.array([[0,0,0],[0,0,-H1], [0,0,0], [0, W1, 0]]).T

Mc1 = np.array([0,0,0])
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])

link1 = Link(screw = S1, linkLines = lines1, M = T1, Mcom = Mc1, G = G1, color = BLACK)

'''Link 2'''
R2 = np.array([[0,0,-1], [0, 1, 0], [1, 0, 0]]).T
p2 = np.array([0, W1, H1])

T2 = mr.RpToTrans(R2, p2)

S2 = np.array([0,1,0,0,0,0]) # Rot about y
lines2 = np.array([[0,0,0],[0,0,L1], [0,-W2,L1]]).T
Mc2 = np.array([0, 0, 0.28])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])

link2 = Link(screw = S2, linkLines = lines2, M = T2, Mcom = Mc2, G = G2, color = GRAY)

'''Link 3'''
R3 = np.array([[0,0,-1], [0, 1, 0], [1, 0, 0]]).T
p3 = np.array([L1, W1-W2, H1])
T3 = mr.RpToTrans(R3, p3)

S3 = np.array([0,1,0,0,0,0]) # Rot about y
lines3 = np.array([[0,0,0],[0,0,L2]]).T

Mc3 = np.array([0, 0, 0.25])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])

link3 = Link(screw = S3, linkLines = lines3, M = T3, Mcom = Mc3, G = G3, color = BLACK)

'''Link 4'''
R4 = np.array([[-1,0,0], [0, 1, 0], [0, 0, -1]]).T
p4 = np.array([L1+L2, W1-W2, H1])
T4 = mr.RpToTrans(R4, p4)

S4 = np.array([0,1,0,0,0,0]) # Rot about y
lines4 = np.array([[0,0,0],[0,W3,0]]).T

Mc4 = np.array([0, 0, 0])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])

link4 = Link(screw = S4, linkLines = lines4, M = T4, Mcom = Mc4, G = G4, color = GRAY)

'''Link 5'''
R5 = np.array([[-1,0,0], [0, 1, 0], [0, 0, -1]]).T
p5 = np.array([L1+L2, W1-W2+W3, H1])
T5 = mr.RpToTrans(R5, p5)

S5 = np.array([0,0,1,0,0,0]) # Rot about z
lines5 = np.array([[0,0,0],[0,0,H2]]).T

Mc5 = np.array([0, 0, 0])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])

link5 = Link(screw = S5, linkLines = lines5, M = T5, Mcom = Mc5,G = G5, color = BLACK)

'''Link 6'''
R6 = np.array([[-1,0,0], [0, 1, 0], [0, 0, -1]]).T
p6 = np.array([L1+L2, W1-W2+W3, H1-H2])
T6 = mr.RpToTrans(R6, p6)

S6 = np.array([0,1,0,0,0,0]) # Rot about y
lines6 = np.array([[0,0,0],[0,W4,0]]).T

Mc6 = np.array([0, 0, 0])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])

link6 = Link(screw = S6, linkLines = lines6, M = T6, Mcom = Mc6, G = G6, color = GRAY)

'''End effector (body frame)'''
Rb = np.array([[-1,0,0], [0, 0, 1], [0, 1, 0]]).T
pb = np.array([L1+L2, W1-W2+W3+W4, H1-H2])
Tb = mr.RpToTrans(Rb, pb)

linkList = [link1, link2, link3, link4, link5, link6]
UR5 = SerialRobot(linkList = linkList, Mb = Tb)