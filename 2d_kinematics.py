# Importing Libraries
from roboticstoolbox import ET2
import roboticstoolbox as rtb
import math

# Joint angles (rad)
pi = math.pi
q = [pi/6, 0.8]

# Links
a1 = 1
a2 = 1

# 2-DOF Robot using Homogeneous Transformation Matrices
T_01 = ET2.R(q[0], 'rad') * ET2.tx(a1)
T_12 = ET2.R(q[1], 'rad') * ET2.tx(a2)
T_02 = T_01 * T_12
K_2d= T_02.fkine([])
print("2 DOF planer robot (2D space = FK)")
print (K_2d)