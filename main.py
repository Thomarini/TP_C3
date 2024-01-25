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
from scipy.spatial import cKDTree as KDTree
import copy

#============================================================================================================
""" Import des modules customs """

from config import *
from utilisation_keyboard import control
from update_current_state import updateState
from where_is_target import getNextTarget
from commande_moteur import rotateToTarget, goToTarget, stop

#============================================================================================================
""" Initialisation des librairies et objets associés utiles à la simulations """

# Initialisation des capteurs de proximités
prox0 = robot.getDevice("prox.horizontal.0")
prox0.enable(timestep)
prox1 = robot.getDevice("prox.horizontal.1")
prox1.enable(timestep)
prox2 = robot.getDevice("prox.horizontal.2")
prox2.enable(timestep)
prox3 = robot.getDevice("prox.horizontal.3")
prox3.enable(timestep)
prox4 = robot.getDevice("prox.horizontal.4")
prox4.enable(timestep)

#============================================================================================================
""" Grandeurs statiques """

# Vitesse
velocity = 1 

# Visualisation des murs
mur_ext_z = np.array([75, -25, -25, -75, -75, -25, -25, 25, 25, 75, 75])
mur_ext_x = np.array([-75, -75, -25, -25, 25, 25, 75, 75, -25, -25, -75])

mur_int_z = np.array([50, 0, 0, -50, 0, 0])
mur_int_x = np.array([-50, -50, 0, 0, 0, 50])

# Targets dans le référentiel de la simulation (point de passage pour parcourir le circuit
list_target_x = [ 12.5,  62.5, 62.5, 37.5, 37.5, -62, -62.5, -12.5, -12.5, 12.5]
list_target_z = [-12.5, -12.5, 62.5, 62.5, 12.5,12.5, -12.5, -12.5, -62.5, -62.5]

if (reversePathing == True):
    list_target_x.reverse()
    list_target_z.reverse()

#============================================================================================================
""" Initialisation des variables """

# Vitesse et position des moteurs
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

motor_left.setVelocity(0)
motor_right.setVelocity(0)

# Etats du système 
# z, x, theta, vitesse mot gauche, vitesse mot droit, capteurs proximités
state = np.zeros([8, 1]) # On anticipe le filtre de Kalman...
old_state = np.zeros([8, 1])


# Réalité de terrain
robot_position = robot_node.getPosition()
robot_rotation = robot_node.getOrientation()

# Initialisation de l'état initial
init_state = np.zeros([8,1])
init_state[0], init_state[1] = robot_position[2]*100, robot_position[0]*100
init_state[2] = math.atan2(robot_rotation[6], robot_rotation[0])

state = copy.deepcopy(init_state)

# Les vitesses et detection d'ostacles sont considérés initialement nuls.


# Initialisation de la fenêtre d'affichage des erreurs et des listes associées
if (display_error == True):
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(11, 3))
    list_err_x, list_err_z, list_err_theta = [], [], [] # liste des erreurs
    list_z_real, list_x_real, list_theta_real = [], [], [] # listes des paramètres pour servir d'abscisse
    list_z, list_x, list_theta = [], [], []

# Compteur utilisé pour tempo l'affichage
cpt_display = 0 # initialisé à 0 pour avoir la situation initiale

# Compteur utilisé pour le debug
cpt_debug = 0 # initialisé à 0 pour avoir la situation initiale

# Listes pour l'affichage de la trajectoire
pos_z, pos_x, pos_z_true, pos_x_true = [], [], [], []
 
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

# Utilisation du robot pour l'étude de situations particulières
if (display_error == True):
    motor_left.setVelocity(3.8)
    motor_right.setVelocity(4)
    

while (robot.step(timestep) != -1):
    # Gestion de l'intervalle de temps (=! de timestep)
    time_new = robot.getTime()
    dt = time_new - time_old
    time_old = time_new
    
    # Actualisation de la vérité de terrain
    robot_position = robot_node.getPosition() 
    robot_rotation = robot_node.getOrientation()    
    robot_orientation = math.atan2(robot_rotation[6], robot_rotation[0])


    ds, dtheta = updateState(state, dt)
    
    # Si controle au clavier
    if (controle_keyboard == True):
        control()  
        
    if (controle_target == True):     
               
        if ((arrivedAtTarget == True) and (rotatedToTarget == True)):        
            arrivedAtTarget = False  
            rotatedToTarget = False  
            distanceTraveled = 0
            distanceToTarget = 0
            
            target_x = list_target_x[index_target]
            target_z = list_target_z[index_target]  
                                  
            index_target += 1            

                
            angleToTarget, distanceToTarget,new_target_x, new_target_z = getNextTarget(target_x, target_z, state)
            stateToTarget = copy.deepcopy(state)
            
            print("==============================================================")
            print("distanceToTarget : {}, distanceTraveled : {}".format(distanceToTarget, distanceTraveled)) 
            print("rotated : {}, translated : {}".format(rotatedToTarget, arrivedAtTarget))
            print("angleToTarget : ", angleToTarget*180/np.pi)
            print("state :      ", state[1][0], state[0][0], state[2][0]*180/np.pi)                        
            print("target_x : {}, target_z : {}".format(target_x, target_z))
            print("new_target_x : {}, new_target_z : {}\n".format(new_target_x, new_target_z))
            print("==============================================================")
            
        if ((arrivedAtTarget == False) and (rotatedToTarget == False)):
            rotatedToTarget = rotateToTarget(angleToTarget, state)
            
        if ((arrivedAtTarget == False) and (rotatedToTarget == True)): 
            distanceTraveled = goToTarget(distanceToTarget, distanceTraveled, ds)
            if (distanceToTarget  < distanceTraveled -epsi_translation):
                arrivedAtTarget = True
       
        
        if ( display_debug == True) :
            if (cpt_debug == 0):
                cpt_debug = 50
                """print("distanceToTarget : {}, distanceTraveled : {}".format(distanceToTarget, distanceTraveled)) 
                print("rotated : {}, translated : {}".format(rotatedToTarget, arrivedAtTarget))
                print("angleToTarget : ", angleToTarget*180/np.pi)                        
                print("target_x : {}, target_z : {}".format(target_x, target_z))
                print("new_target_x : {}, new_target_z : {}\n".format(new_target_x, new_target_z))
                print("----------------------------------------------------------")"""
            cpt_debug -= 1

    if (display_error == True):
        # update des grandeurs à chaque pas de simulation
        list_err_x.append(state[1][0]-robot_position[0]*100)
        list_err_z.append(state[0][0]-robot_position[2]*100)
        list_err_theta.append((state[2][0] - robot_orientation)*180/np.pi)
        list_x.append(state[1][0])
        list_z.append(state[0][0])
        list_theta.append(state[2][0]*180/np.pi)
        list_x_real.append(state[1][0])
        list_z_real.append(state[0][0])        
        list_theta_real.append(robot_orientation*180/np.pi)

        # Utilisé pour voir l'influence d'un arret et redémarrage sur l'erreur
        if (cpt_display == 99):
            """motor_left.setVelocity(3)
            motor_right.setVelocity(3)"""       
            """
            # Suppression du point d'arret
            list_err_x.pop(-1)
            list_err_z.pop(-1)
            list_err_theta.pop(-1)
            list_x.pop(-1)
            list_z.pop(-1)
            list_x_real.pop(-1)
            list_z_real.pop(-1)
            list_theta_real.pop(-1)
            """
            
            """tmp1, tmp2 = motor_right.getVelocity(), motor_left.getVelocity()
            motor_left.setVelocity(tmp1)
            motor_right.setVelocity(tmp2)"""
        
        # affichage périodique
        if (cpt_display == 0): 
            """motor_left.setVelocity(0)
            motor_right.setVelocity(0)"""
            
            iteration = np.arange(len(list_err_x))             
            plt.ion()       
            if  (len(list_err_x) >= 1000):
                motor_left.setVelocity(3)
                motor_right.setVelocity(3)
            
            print(motor_left.getVelocity())
            ax1.plot(iteration, list_err_z, "b")
            ax1.set_ylabel("erreur longitudinale")
            ax1.set_xlabel("itération")
            
            """ax1.plot(list_x, list_z, "b")
            ax1.set_ylabel("distance réelle z parcourue [m] ")
            ax1.set_xlabel("distance réelle x parcourue [m]")"""
            
            ax2.plot(iteration, list_err_x, "g")
            ax2.set_ylabel("erreur radiale ")
            ax2.set_xlabel("itération")
            
            """ax2.plot(list_x_real, list_z_real, "g")
            ax2.set_ylabel("distance z parcourue [m] ")
            ax2.set_xlabel("distance x parcourue [m]")"""
            
            """ax3.plot(iteration, list_err_theta, "b")
            ax3.set_ylabel("erreur du cap [°]")
            ax3.set_xlabel("itération")"""
            
            ax3.plot(iteration, list_theta, "r")
            ax3.plot(iteration, list_theta_real, "y")
            ax3.legend(["commande cap [°]", "commande réel [°]"], loc='upper left')
            ax3.set_xlabel("itération")
        
            plt.tight_layout()            
            plt.draw()        
            plt.pause(0.001)  
            ax1.clear()
            ax2.clear()
            ax3.clear()
            cpt_display = 300     
        cpt_display -= 1
    
    
    # Soit on s'interesse à l'erreur soit on s'interesse au monde de la simu
    # TODO : synchroniser les param de la simu
    else :
    
        if (display_trajectory == True):  
            # Actualisation des listes de position     
            # Les positions x sont ajouté en négatif car l'axe des abscisses
            # est représenté par son opposé 
            pos_z.append(state[0][0])
            pos_x.append(-state[1][0])
            pos_z_true.append(robot_position[2]*100)
            pos_x_true.append(-robot_position[0]*100)    
            plt.legend(["estimation ", "réalité" ], loc='upper right', labelcolor= ["r", "y"])
    
    
        # Affichage de la situation
        if (cpt_display == 0):
            cpt_display = 100
            
            plt.ion()
            # Etat supposé            
            plt.plot(-state[1], state[0], "b+") 
            
            if ( display_debug == True) :
                print("state :      ", state[0][0], state[1][0], state[2][0]*180/np.pi)
                print("true state : ", robot_position[2]*100, robot_position[0]*100, robot_orientation*180/np.pi)
                print("ds : {}, dtheta : {}\n".format(ds, dtheta))
                print("===============================================================")
    
    
            if (display_trajectory == True):
                plt.plot(pos_x, pos_z, "r")            
                plt.plot(pos_x_true, pos_z_true, "y")
                plt.legend(["estimation ", "réalité" ], loc='upper right', labelcolor= ["r", "g"])
                
            if (display_target == True):
                tmp_list_target_x = [-x for i,x in enumerate(list_target_x) if i != index_target-1]
                tmp_list_target_z = [z for i,z in enumerate(list_target_z) if i != index_target-1]
                
                plt.plot(tmp_list_target_x, tmp_list_target_z, "r+")
                plt.plot(-list_target_x[index_target-1], list_target_z[index_target-1], "g*")
                
            if ((display_target_debug == True) and (controle_target == True)):
                # Target dans le référentiel de la simu (pour débug)
                plt.plot(-(new_target_x+state[1]), new_target_z+state[0], 'y+')
                # Target dans le référentiel du robot (pour débug)
                plt.plot(-new_target_x, new_target_z, 'y*')  
                plt.legend(["estimation de position", "cibles", "cible actuelle", "cible dans repère robot", "cible dans repère simu"], loc='upper right', labelcolor= [ "b", "r", "g", "y", "y"])          
              
            # Murs   
            plt.plot(mur_ext_x, mur_ext_z, '0.4')
            plt.plot(mur_int_x, mur_int_z, '0.4')
            
            plt.draw()        
            plt.pause(0.001)    
            plt.clf()        
        cpt_display = cpt_display-1
        
