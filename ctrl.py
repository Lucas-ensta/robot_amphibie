from __future__ import print_function
from matplotlib.pyplot import *
from numpy import *
from math import cos, sin, sqrt, pi, atan2


class Control:
    def __init__(self, kp_angle=2.0):
        self.kp_angle = kp_angle
        self.contourning = False 


    def calcul_commande(self, x, target):
            """
            Calcul la commande u 
            param: x le vecteur d'etat du robot
            param: target la position de la cible 
            param: distance du sonar front 
            param: distance du sonar gauch et droite
            """
            
            # Mode normal : guidage proportionnel vers la cible
           
            # Calcul de l’angle désiré vers la cible
            dx = target[0] - x[0]
            dy = target[1] - x[1]
            theta_desired = atan2(dy, dx)
            erreur_theta = (theta_desired - x[2]) % (2*pi)
            if erreur_theta > pi:
                erreur_theta -= 2*pi
            
            omega = float(self.kp_angle * erreur_theta)
            a = 0.0
            return array([omega, a]).reshape(2,1)
    
  
    

