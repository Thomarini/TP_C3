from config import velocity, motor_left, motor_right, e, r, robot_node
import config
import numpy as np
import copy
from math import atan2, pi, cos , sin

def updateEstimatedState(dt):
    """ Update la position éstimée du robot du robot 
        Renvoie la distance et l'angle parcouru """   
    # TODO: séparer les deux taches en deux fonctions
    config.state[3], config.state[4] = motor_left.getVelocity()*r, motor_right.getVelocity()*r
    
    ds = (config.state[3]*dt + config.state[4]*dt)/2
    dtheta = (config.state[3]*dt - config.state[4]*dt)/e    

    # Soit - sur le sinus soit - sur théta (state[2][0]).
    # à cause de l'orientation négative de théta dans le repère ?
        
    config.state[0] = config.state[0] + ds*cos(config.state[2] + dtheta/2)
    config.state[1] = config.state[1] - ds*sin(config.state[2] + dtheta/2) 
    # Modulo utilisé pour l'affichage dans la console 
    # à voir si l'utilisation en embarqué profiterait de son absence (mémoire vs temps de calcul)
    config.state[2]  = (config.state[2] + dtheta + pi)%(2*pi)-pi

    return ds, dtheta

def updateRealState():
    """ Update la position réelle du robot """    
    # Réalité de terrain
    robot_position = robot_node.getPosition()
    robot_rotation = robot_node.getOrientation()
    
    # Update
    config.state[5], config.state[6] = robot_position[2]*100, robot_position[0]*100
    config.state[7] = atan2(robot_rotation[6], robot_rotation[0])

def updateTrajectory():
    config.pos_z.append(config.state[0][0])
    config.pos_x.append(-config.state[1][0])
    config.pos_z_true.append(config.state[5][0])
    config.pos_x_true.append(-config.state[6][0])

def updateTrajectoryLidar():
    config.pos_z_lidar.append(config.state[8][0])
    config.pos_x_lidar.append(-config.state[9][0])

def updateLidarState(translation_x, translation_z, theta):
    """ Update la position supposé du robot corrigé avec le ICP"""   

    config.state[8] = config.state[0] + translation_z
    config.state[9] = config.state[1] + translation_x
    config.state[10] = (config.state[2] - theta -pi/2 + pi)%(2*pi)-pi
    
def updateRealStateWithLidar():
    config.state[0], config.state[1] = config.state[8], config.state[9]
    # config.state[2] = config.state[10]
