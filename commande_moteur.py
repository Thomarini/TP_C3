""" Fonctions de controle du robot """

from config import motor_left, motor_right, epsi_rotation, epsi_rotation_max, epsi_translation
import config
from math import sqrt

velocity = config.velocity

def up(velocity):
    motor_left.setVelocity(velocity)
    motor_right.setVelocity(velocity)

def down(velocity):
    motor_left.setVelocity(-velocity)
    motor_right.setVelocity(-velocity)

def left(velocity):
    motor_left.setVelocity(-velocity)
    motor_right.setVelocity(velocity)

def right(velocity):
    motor_left.setVelocity(velocity)
    motor_right.setVelocity(-velocity)

def stop():
    motor_left.setVelocity(0)
    motor_right.setVelocity(0)
    
    
    
    
def rotateToTarget(angleToTarget, state):
    #TODO: faire mieux - epsi trouvé expérimentalement 
    tmp = (sqrt(max(state[2]**2, angleToTarget**2) - min(state[2]**2, angleToTarget**2)))
    if (tmp < epsi_rotation_max):
        return True
    elif (angleToTarget < state[2] + epsi_rotation):
        left(velocity)
    elif (angleToTarget > state[2] - epsi_rotation):
        right(velocity)        
        
    return False


def goToTarget(distanceToTarget, distanceTraveled, ds): 
    if (distanceToTarget  > distanceTraveled -epsi_translation):
        distanceTraveled += ds
        up(velocity)
    return distanceTraveled
    # TODO: aller en arrière
    # elif (distanceToTarget - (stateToTarget[0]- state[0]) > epsi_translation):
    #    distanceToTarget -= ds
    