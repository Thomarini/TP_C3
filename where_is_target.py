from math import atan2, cos, sin, sqrt, pi
from config import *

def getNewRef(target_x, target_z, state):
    # Target rotation
    angle = -state[2] # angle négatif pour avoir l'équivalence avec la simulation
    new_target_z = cos(angle)*target_z + sin(angle)*target_x
    new_target_x = -sin(angle)*target_z + cos(angle)*target_x

    #Target translation
    
    if ((target_x >0 and state[1] >0) and (target_z >0 and state[0] <0)):
        new_target_x += state[1]
        print("aled")
    else :        
        new_target_x -= state[1]    
    new_target_z -= state[0]  
    
      
    return new_target_x, new_target_z
    
    
def getAngleToTarget(new_target_x, new_target_z):
    angleToTarget = atan2(new_target_z, new_target_x)
    
    return angleToTarget
    
    
    
def getNextTarget(target_x, target_z, state):

    new_target_x, new_target_z = getNewRef(target_x, target_z, state)
    angleToTarget = getAngleToTarget(new_target_x, new_target_z)-pi/2
    distanceToTarget = sqrt(new_target_z**2+new_target_x**2)

    return angleToTarget, distanceToTarget, new_target_x, new_target_z
