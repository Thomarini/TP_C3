""" Fonctions de controle par clavier du robot"""

from config import keyboard, Keyboard
import config
velocity = config.velocity
from commande_moteur import up, left, down, right, stop

def control():
    key = keyboard.getKey()
    if (key == Keyboard.UP):
        up(velocity)
    elif (key == Keyboard.DOWN):
        down(velocity)
    elif (key == Keyboard.LEFT):
        left(velocity)
    elif (key == Keyboard.RIGHT):
        right(velocity)
    elif (key == Keyboard.SHIFT+ord('C')):
        stop()
    else:
        pass