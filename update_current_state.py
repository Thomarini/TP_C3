from config import velocity, motor_left, motor_right, e, r
import numpy as np
import copy

def updateState(state, dt):
    
    state[3], state[4] = motor_left.getVelocity()*r, motor_right.getVelocity()*r
    
    ds = (state[3]*dt + state[4]*dt)/2
    dtheta = (state[3]*dt - state[4]*dt)/e    

    state[0] = state[0] + ds*np.cos(state[2] + dtheta/2)
    state[1] = state[1] - ds*np.sin(state[2] + dtheta/2) # jsp pourquoi il faut un -
    state[2]  = (state[2] + dtheta + np.pi)%(2*np.pi)-np.pi
        
    #TODO : update l'état des capteurs
    
    return ds, dtheta