from math import cos, sin, pi, atan2
import numpy as np
from scipy.spatial import cKDTree as KDTree
from scipy.spatial.transform import Rotation

from config import mur_x, mur_z, lidar2, robot_node
import config

lidar = lidar2
""" Etrangement, importer "lidar" ne fonctionne pas mais importer lidar2 fonctionne. Peut être un problème d'import cyclique
ou un nom déjà utilisé dans une autre librairie  
"""

#============================================================================================================
""" Opération mathématiques """

def multMatrix(X, Y, T):
    """ Changement de repère avec matrice de rotation """
    
    res = []
    res.append(X[0] * Y[0] + X[3] * Y[1] + X[6] * Y[2] - T[0])
    res.append(X[1] * Y[0] + X[4] * Y[1] + X[7] * Y[2] + T[1])
    res.append(X[2] * Y[0] + X[5] * Y[1] + X[8] * Y[2] + T[2])
    
    return res

def robotToSim(x_in_robot, z_in_robot):
    """ Changement de repère robot -> simu"""  
    
    angle = config.state[2]   # angle négatif pour avoir l'équivalence avec la simulation    
    #angle = config.state[7]    # vérité de terrain
    z_in_simu = cos(angle)*z_in_robot - sin(angle)*x_in_robot + config.state[0]
    x_in_simu = sin(angle)*z_in_robot + cos(angle)*x_in_robot - config.state[1]

    return x_in_simu, z_in_simu

def simToRobot(x_in_simu, z_in_simu):
    """ Changement de repère simu -> robot"""
    
    angle = -config.state[2]   # angle négatif pour avoir l'équivalence avec la simulation
    #angle = config.state[7]    # vérité de terrain
    z_in_robot = cos(angle)*z_in_simu - sin(angle)*x_in_simu - config.state[0]
    x_in_robot = sin(angle)*z_in_simu + cos(angle)*x_in_simu + config.state[1]

    return x_in_robot, z_in_robot

#============================================================================================================
""" Iterated Closest Points """

def indextMean(index, arrays):
    """ Returns center of mass """
    
    indexSum = np.array([0.0, 0.0, 0.0])
    for i in range(np.size(index, 0)): 
        indexSum = np.add(indexSum, np.array(arrays[index[i]]), out = indexSum, casting='unsafe')
    
    return indexSum/np.size(index, 0)

def indextfixed(index, arrays):
    """ cf rapport """
    T = []
    for i in index:
        T.append(arrays[i])
        
    return np.asanyarray(T)

def ICPSVD(fixedX, fixedY, movingX, movingY):
    """ Renvoie la matrice de rotation 3D et le vecteur de translation 3D
        de la différence entre les ""cartes"" (fixedX, fixedY) et
        movingX, movingT) """

    reqR = np.identity(3) # vecteur de translation
    reqT = [0.0, 0.0, 0.0] # vecteur de rotation
    
    fixedt = [[float(x), float(y), 0.0] for x, y in zip(fixedX, fixedY)]
    movingt = [[float(x), float(y), 0.0] for x, y in zip(movingX, movingY)]
    
    """ Erreur de typage insuportable : si on prend fixedX,.. en tant que np.array, les scalaires récupérés sont des
        np.float. Si on rajoute des 0 comme fait ci-dessus, on a une liste de np.float et de float classiques ce qui
        génère des erreurs lors de l'utilisation de np.linalg.svd """

    # conversion des listes en array
    moving = np.asarray(movingt)
    fixed = np.asarray(fixedt)
    
    n = np.size(moving, 0)

    tmp = ""
    TREE = KDTree(fixed)
    for i in range(10):
        # Récupération des distances et indices des points les plus proches
        distance, index = TREE.query(moving)
        err = np.mean(distance**2)
        
        # Utilisé pour génerer des données
        #tmp += ", [{:.2f}, {}, {:.3f}]".format(-config.state[2][0]*180/pi, i, err)
        
        # TODO: faire mieux qu'un threshold arbitraire
        if err < 50:
            com = np.mean(moving, 0)
            cof = indextMean(index, fixed)

            # On place les points du murs repérés dans le référentiel de la carte 
            W = np.dot(np.transpose(moving), indextfixed(index, fixed)) - n*np.outer(com, cof)

            # On décompose W en "matrices de valeurs propre"
            U, _, V = np.linalg.svd(W, full_matrices = False)
            
            # On déplace les points d'une quantités liés "à leur valeur propre"
            tempR = np.dot(V.T, U.T)
            tempT = cof - np.dot(tempR, com)
            
            # On revient dans le repère initial
            moving = (tempR.dot(moving.T)).T
            moving = np.add(moving, tempT)
            reqR = np.dot(tempR, reqR)
            reqT = np.add(np.dot(tempR, reqT), tempT)
            
            validity_check = True
        else :
            # gestion du cas où l'erreur est trop grande
            validity_check = False
            reqR, reqT = np.zeros([3, 3]), np.zeros([3, 1])
    
    return reqR, reqT, validity_check

#============================================================================================================
""" Link b/w ICP and lidar """

def getLidarData():
    """ Returns matrice de rotation et le vecteur de translation nécessaire pour avoir correspondance entre la carte et le lidar"""

    # Récupération et traitement des données lidars
    point_cloud = lidar.getRangeImage()
    
    # tableau de stockage des données
    config.data_length = len(point_cloud)   # Pas nécessaire mais il faut garder à l'esprit qu'un lidar peut sauter des données
    config.list_lidar_x, config.list_lidar_z = np.zeros([config.data_length, 1]),np.zeros([config.data_length, 1])  
    config.list_lidar_polar_x, config.list_lidar_polar_z = np.zeros([config.data_length, 1]),np.zeros([config.data_length, 1]) 
    
    angle = 0    
    for idx, i in enumerate(point_cloud):
        xy = [i*sin(angle), 0, i*cos(angle)] 
        # Placement des données dans le repère de la simulation avec la position réelle et estimée
        x_in_sim, z_in_sim = robotToSim(xy[0]*100, xy[2]*100)   # dépend de l'état du robot => source d'erreur      
        x_in_sim_true, _, z_in_sim_true = multMatrix(robot_node.getOrientation(), xy, robot_node.getPosition()) # absolue
        angle += 2*pi/lidar.getHorizontalResolution()
        
        # Stockage pour l'affichage
        config.list_lidar_x[idx] = x_in_sim[0]
        config.list_lidar_z[idx] = z_in_sim[0]        
        config.list_lidar_polar_x[idx] = xy[0]
        config.list_lidar_polar_z[idx] = xy[2]
        config.list_lidar_truth_x[idx] = x_in_sim_true*100
        config.list_lidar_truth_z[idx] = z_in_sim_true*100

    # Angle et translation des données lidar par rapport à la carte
    reqR, reqT, validity_check = ICPSVD(mur_x, mur_z, config.list_lidar_x, config.list_lidar_z)
    
    return reqR, reqT, validity_check

#============================================================================================================
""" Processing ICP data """

def correctionLidar(reqR, reqT, x_in_sim, z_in_sim):
    """ Returns one lidar point after correcting its position relative to the map"""
    
    data = np.array([float(x_in_sim), float(z_in_sim), 0.0])    
    data = np.matmul(reqR, data.transpose())
    data = data + reqT
    
    return data[0], data[1]

def updateCorrectedLidarData(reqR, reqT):    
    """ Updates all lidar points after correcting its position relative to the map"""
    
    # Tableau de stockage des données
    config.list_lidar_x_corr, config.list_lidar_z_corr = np.zeros([config.data_length, 1]), np.zeros([config.data_length, 1])     
    for i in range(config.data_length): 
        config.list_lidar_x_corr[i], config.list_lidar_z_corr[i] = correctionLidar(reqR, reqT, config.list_lidar_x[i], config.list_lidar_z[i])

def getRotationTranslationICP(reqR, reqT):
    """ Returns l'ange et la translation entre la map et les données lidar"""
    
    angle_R = atan2(reqR[1][0], reqR[0][0])
    
    return reqT[0], reqT[1], angle_R