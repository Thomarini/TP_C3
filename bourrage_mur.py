import numpy as np
import matplotlib.pyplot as plt
from math import copysign


sign = lambda x: copysign(1, x)

#============================================================================================================
""" Liste des points d'interets"""

mur_int_z = np.array([50, 0, 0, -50, 0, 0]).tolist()
mur_int_x = np.array([-50, -50, 0, 0, 0, 50]).tolist()

mur_ext_z = np.array([75, -25, -25, -75, -75, -25, -25, 25, 25, 75, 75]).tolist()
mur_ext_x = np.array([-75, -75, -25, -25, 25, 25, 75, 75, -25, -25, -75]).tolist()

def link_scalar(origin, destination):
    # Utilisation de l'équation de la droite pour générer les points intermédiaires
    tmp = [origin]
    cpt = 0
    coef = sign(destination - origin)
    while (tmp[-1] != destination):
        cpt += 1    # Nombre de points intermédiaires nécessaire
        tmp.append(origin + cpt*coef)     
    return tmp, cpt

def bourrageList(liste1, liste2):
    out1, out2 = [], []
    for i in range(len(liste1)-1):
        tmp1, cpt1 = link_scalar(liste1[i], liste1[i+1])
        tmp2, cpt2 = link_scalar(liste2[i], liste2[i+1])

        # Si l'on n'a pas de point intermédiaire, on recopie la valeur actuelle
        # afin de garantir l'égalité des dimensions et de garder un sens géométrique
        if (cpt1 == 0):
            out1+=tmp1*(cpt2+1)
            out2+= tmp2            

        elif (cpt2 == 0):
            out1+=tmp1
            out2+=tmp2*(cpt1+1)

        else:
            out1+=tmp1
            out2+=tmp2

    return out1, out2

tmp1, tmp2 = bourrageList(mur_int_x, mur_int_z)
tmp3, tmp4 = bourrageList(mur_ext_x, mur_ext_z)

print("mur_int_x = np.array({}) \nmur_int_z = np.array({}) \nmur_ext_x = np.array({})\nmur_ext_z = np.array({})".format(tmp1, tmp2, tmp3, tmp4))