import sys,os
sys.path.append('C:\Program Files\Webots\lib\controller\python39')
import math
import time;
from controller import *
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
from matplotlib import collections  as mc
import random

import copy

#============================================================================================================
""" Import des modules customs """

from config import *
import config

init_variable()

from utilisation_keyboard import control
from update_state import updateEstimatedState, updateRealState, updateLidarState, updateTrajectory, updateRealStateWithLidar, updateTrajectoryLidar
from where_is_target import getNextTarget, getNewRef
from commande_moteur import rotateToTarget, goToTarget, stop
from lidar_processing import getLidarData, getRotationTranslationICP, updateCorrectedLidarData
from affichage import display


#============================================================================================================
""" Initialisation des variables """

# Vitesse et position des moteurs
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

motor_left.setVelocity(0)
motor_right.setVelocity(0)

# Réalité de terrain
robot_position = robot_node.getPosition()
robot_rotation = robot_node.getOrientation()

# Initialisation de l'état initial
config.state[0], config.state[1], config.state[2] = robot_position[2]*100, robot_position[0]*100, math.atan2(robot_rotation[6], robot_rotation[0])
config.state[5], config.state[6], config.state[7] = robot_position[2]*100, robot_position[0]*100, math.atan2(robot_rotation[6], robot_rotation[0])


# Initialisation de la fenêtre d'affichage des erreurs et des listes associées
if (display_error == True):
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(11, 3))
    list_err_x, list_err_z, list_err_theta = [], [], [] # liste des erreurs
    list_z_real, list_x_real, list_theta_real = [], [], [] # listes des paramètres pour servir d'abscisse
    list_z, list_x, list_theta = [], [], []



# Distances utilisées pour le repérage par rapport à la cible
distanceTraveled = 0.0
distanceToTarget = 0.0

index_target = 0

rotatedToTarget = True # initialisation de la rotation
arrivedAtTarget = True # initialisation de la translation

secu_init = True # Gestion de cas particulier en cas d'initialisation à vitesse nulle


#============================================================================================================
""" Simulation """
time_old = robot.getTime()
time_old_ICP = robot.getTime()
cpt_lidar = 0

# Utilisation du robot pour l'étude de situations particulières
if (display_error == True):
    motor_left.setVelocity(3.8)
    motor_right.setVelocity(4)

while (robot.step(timestep) != -1):
    # Gestion de l'intervalle de temps (=! de timestep)
    time_new = robot.getTime()
    dt = time_new - time_old
    time_old = time_new

    # Controle au clavier
    if (controle_keyboard == True):
        control()  

    # Odométrie
    ds, dtheta = updateEstimatedState(dt)    
    updateRealState()   

    # lidar
    if (use_tempo_ICP == True):
        if (robot.getTime() - time_old_ICP >= tempo_ICP):
            time_old_ICP = robot.getTime()
            
            if (lidar_enable == True):
                reqR, reqT, validity_check = getLidarData()
                # print(reqR, reqT)
                # Gestion de l'erreur de manière brutale
                if (validity_check == True):
                    translation_x, translation_z, theta = getRotationTranslationICP(reqR, reqT)
                    theta = (theta + np.pi)%(2*np.pi)-np.pi
                    updateCorrectedLidarData(reqR, reqT)
                    updateLidarState(translation_x, translation_z, theta)
                    updateTrajectoryLidar()
                    updateRealStateWithLidar()
                    
                    
                else:
                    print("olala erreur")
                    
            # TODO: séparer la latence d'affichage du lidar        
            display()
                    
    if (display_trajectory == True):
        # Doit être placé après toutes les updates du vecteur d'état
        updateTrajectory()

#============================================================================================================       
