from config import velocity, motor_left, motor_right, e, r
import numpy as np
import copy

def updateState(state, dt):
    
    state[3], state[4] = motor_left.getVelocity()*r, motor_right.getVelocity()*r
    
    ds = (state[3]*dt + state[4]*dt)/2
    dtheta = (state[3]*dt - state[4]*dt)/e    

    # Soit - sur le sinus soit - sur théta (state[2][0]).
    # à cause de l'orientation négative de théta dans le repère ?
        
    state[0] = state[0] + ds*np.cos(state[2] + dtheta/2)
    state[1] = state[1] - ds*np.sin(state[2] + dtheta/2) 
    # Modulo utilisé pour l'affichage dans la console 
    # à voir si l'utilisation en embarqué profiterait de son absence (mémoire vs temps de calcul)
    state[2]  = (state[2] + dtheta + np.pi)%(2*np.pi)-np.pi
        
    #TODO : update l'état des capteurs
    
    return ds, dtheta
