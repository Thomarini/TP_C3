from config import *
import matplotlib.pyplot as plt
import config
from math import pi


def walls():
    plt.plot(mur_int_x, mur_int_z, '0.4')
    plt.plot(mur_ext_x, mur_ext_z, '0.4')


def lidar_raw(legend, legend_color):
    # Visualisation des données brutes du lidar
    plt.plot(config.list_lidar_x, config.list_lidar_z, "m+")
    legend += ['lidar in sim']
    legend_color += ['m']
    return legend, legend_color
    
def lidar_corr(legend, legend_color):
    # Visualisation des données fit sur la map du lidar    
    plt.plot(config.list_lidar_x_corr, config.list_lidar_z_corr, "c+")
    legend += ['lidar corrected']
    legend_color += ['c']
    return legend, legend_color
    
def lidar_polar(legend, legend_color):
    # Visualisation des données fit sur la map du lidar    
    plt.plot(config.list_lidar_polar_x*180/pi, config.list_lidar_polar_z*180/pi, "g+")
    print(config.list_lidar_polar_x*180/pi)
    print(config.list_lidar_polar_z*180/pi)
    legend += ['polar lidar']
    legend_color += ['g']
    return legend, legend_color
    
def lidar_true(legend, legend_color):
    # Visualisation des données fit sur la map du lidar    
    plt.plot(config.list_lidar_truth_x, config.list_lidar_truth_z, "g+")
    legend += ['lidar in sim truth']
    legend_color += ['g']
    return legend, legend_color




def pos_odo(legend, legend_color):
    plt.plot(-state[1], state[0], "b+")
    legend += ['position odométrie']
    legend_color += ['b']
    return legend, legend_color

def pos_true(legend, legend_color):
    plt.plot(-state[6], state[5], "r+")
    legend += ['position réelle']
    legend_color += ['r']
    return legend, legend_color



def trajectory(legend, legend_color):
    plt.plot(config.pos_x, config.pos_z, "b")    
    legend += ['trajectoire odométrie']
    legend_color += ['b']    
        
    plt.plot(config.pos_x_true, config.pos_z_true, "r")
    legend += ['trajectoire réelle']
    legend_color += ['r']  
        
    plt.plot(config.pos_x_lidar, config.pos_z_lidar, "g.")
    legend += ['point(s) ICP']
    legend_color += ['g']  
    
    return legend, legend_color




def target(legend, legend_color):
    """ plt.plot(tmp_list_target_x, tmp_list_target_z, "r+")
    plt.plot(-list_target_x[index_target-1], list_target_z[index_target-1], "g*") """
    return legend, legend_color

def target_debug(legend, legend_color):
    """ # Target dans le référentiel de la simu (pour débug)
    plt.plot(-(new_target_x+state[1]), new_target_z+state[0], 'y+')
    # Target dans le référentiel du robot (pour débug)
    plt.plot(-new_target_x, new_target_z, 'y*')   """
    return legend, legend_color
    
    
def display():
    
    plt.ion()     
    legend, legend_color = [], []
    if (display_pos_odo == True): legend, legend_color = pos_odo(legend, legend_color)
    if (display_true_pos == True): legend, legend_color = pos_true(legend, legend_color)
    if (display_trajectory == True): legend, legend_color = trajectory(legend, legend_color)
    if (display_lidar_polar == True): legend, legend_color = lidar_polar(legend, legend_color)
    if (display_lidar_true == True): legend, legend_color = lidar_true(legend, legend_color)
    if (display_lidar == True): legend, legend_color = lidar_raw(legend, legend_color)
    if (display_lidar_corr == True): legend, legend_color = lidar_corr(legend, legend_color)
    if (display_target == True): pass
    if (display_error == True): pass
    if (display_debug == True): pass
    if (display_target_debug == True): pass
    
    plt.legend(legend, loc='upper right', labelcolor = legend_color)
    
    # Has to be last for the legend to be coherent
    if (display_wall == True): walls()
        
    plt.draw()        
    plt.pause(0.001)    
    plt.clf()  
