"""
    Fichier de configuration associé à l'application souhaitée
"""

from controller import *

""" Paramètres d'utilisation du main """
controle_keyboard = False # If true, autorise le controle du robot par l'utilisation du clavier

controle_target = True # If true, robot goes from one target to the next on his own

display_target = True # If true, affiche les targets visées successivement par le robot

display_trajectory = False  # If true, affiche la trajectoire parcourue et 
                            # la trajectoire supposément parcourue
     
display_error = False # If true, affiche la différence entre l'état supposé et la réalité
                     # de terrain 
                            
reversePathing = False # If true, traverser le parcours dans le sens horaire



""" Paramètres de débuggage du main """ 

display_debug = False # If true, affiche toutes les informations utiles


display_target_debug = True # If true, affiche la position de la prochaine cible 
                            # dans le référentiel du robot et de la simu 
                            

                            
 
""" Initialisation des objets associés utiles à la simulations """

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

key = -1
keyboard = Keyboard()
keyboard.enable(timestep)

robot_node = robot.getFromDef("Thymio")

motor_left = robot.getDevice("motor.left");
motor_right = robot.getDevice("motor.right");

global velocity # En théorie ca rend vélocité modifiable depuis main. En pratique je n'y arrive pas.
velocity = 1


# TODO trouver mieux
epsi_rotation = 0.001  # variation autorisé autour de l'angle cible en rad
epsi_rotation_max = 0.05 # ecart max entre l'angle de la target et le cap du robot

epsi_translation = 0.001 # variation autorisé autour de l'angle cible en rad


# Dimensions du robot
e = 10.8 # entre-axe
r = 2.105 # rayon roue


