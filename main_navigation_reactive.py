import sys,os
sys.path.append('C:\Program Files\Webots\lib\controller\python39')
import math
from math import pi, sin, cos, inf, sqrt, atan2
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

lidar = lidar2

init_variable()

from utilisation_keyboard import control
from update_state import updateEstimatedState, updateRealState, updateLidarState, updateTrajectory, updateRealStateWithLidar, updateTrajectoryLidar
from where_is_target import getNextTarget, getNewRef
from commande_moteur import rotateToTarget, goToTarget, stop, left, right, up
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


# TMP
threshold_distance_lidar = 0.2

threshold_dist = 0.04    #TODO: figure that shit out

epsi_angle = 0.05




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

    # Navigation réactive
    if (controle_reactif == True):
        # Récupération et traitement des données lidars
        point_cloud = lidar.getRangeImage()
        
        x_lidar = []
        y_lidar = []

        x_left = []
        y_left = []
        
        x_right = []
        y_right = []
        
        # tableau de stockage des données
        angle = 0    
        for idx, i in enumerate(point_cloud):
            # Distance à l'obstacle grande => infini
            if (i > threshold_distance_lidar): i = inf                
            tmp = [i*sin(angle), i*cos(angle)] 
            if math.isnan(tmp[0]): tmp[0] = inf

            if ((pi/2 + pi/4 > angle) or (angle > 3*pi/2 - pi/4)): 
                x_lidar.append(tmp[0])
                y_lidar.append(tmp[1])
                if ((threshold_dist < i) and (i < inf)):
                    
                    if (tmp[0] > 0) : 
                        x_right.append(tmp[0])
                        y_right.append(tmp[1])
                                                
                    elif (tmp[0] < 0) : 
                        x_left.append(tmp[0])
                        y_left.append(tmp[1])
                    
            angle += 2*pi/lidar.getHorizontalResolution()
        
        
        # Du fait de l'ordre de remplissage des listes qui suit le sens trigo, on inverse la sous liste récupérée 
        # pour garder la cohérence sur la distance entre le robot et les points représentatif des murs
        tmp = int(1*len(x_right)/4)        
        cpt_right = len(x_right[:tmp])               

        x_right = sum(x_right[:tmp])/cpt_right
        y_right = sum(y_right[:tmp])/cpt_right
        
        
        tmp = int(3*len(x_left)/4)
        cpt_left = len(x_left[tmp:])
        
        x_left = sum(x_left[tmp:])/cpt_left
        y_left = sum(y_left[tmp:])/cpt_left
        
        
        x_middle = (x_right + x_left)/2
        y_middle = (y_right + y_left)/2
        
        angleToTarget = atan2(x_middle, y_middle)
        
        if (angleToTarget < -epsi_angle): 
            left(config.velocity)
        elif (angleToTarget > epsi_angle): 
            right(config.velocity)
        elif (-epsi_angle < angleToTarget < epsi_angle ):
            up(config.velocity)
            
        else:
            stop()
            print("error for : ", angleToTarget)
        
        if (cpt_lidar == 0):            
            cpt_lidar = 25
            plt.ion() 
            plt.plot(x_lidar, y_lidar, "b+")
            
            plt.plot(x_right, y_right, "r*")
            plt.plot(x_left, y_left, "r*")
            plt.plot(0, 0, "m+")
            plt.plot(x_middle, y_middle, "g*")
        
            legend = ['lidar', 'centre_droit', 'centre_gauche','robot', 'target']
            legend_color = ["b", "r", "r", "m", "g"]
            plt.legend(legend, loc = 'lower left', labelcolor = legend_color)
            
            plt.draw()        
            plt.pause(0.001)    
            plt.clf()    
            
        cpt_lidar -= 1 

        """ updateRealState()
        if (cpt_lidar == 0):
            cpt_lidar = 500
            display()
        cpt_lidar -= 1 """
    """ # Odométrie
    ds, dtheta = updateEstimatedState(dt)    
    updateRealState()   
    """

    """ # lidar
    if (use_tempo_ICP == True):
        if (robot.getTime() - time_old_ICP >= tempo_ICP):
            time_old_ICP = robot.getTime()
            
            if (lidar_enable == True):
                reqR, reqT, validity_check = getLidarData()
                # Gestion de l'erreur de manière brutale
                if (validity_check == True):
                    translation_x, translation_z, theta = getRotationTranslationICP(reqR, reqT)
                    theta = (theta + np.pi)%(2*np.pi)-np.pi
                    updateCorrectedLidarData(reqR, reqT)
                    updateLidarState(translation_x, translation_z, theta)
                    updateTrajectoryLidar()
                    updateRealStateWithLidar()
                    
            # TODO: séparer la latence d'affichage du lidar        
            display() """
                    
    if (display_trajectory == True):
        # Doit être placé après toutes les updates du vecteur d'état
        updateTrajectory()

#============================================================================================================       
