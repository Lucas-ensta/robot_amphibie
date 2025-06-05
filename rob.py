from __future__ import print_function
from matplotlib.pyplot import *
from numpy import *
from math import cos, sin, sqrt, pi, atan2


# ---------------- Classe Robot ---------------- #
class Robot:
    def __init__(self, x0):
        self.x = x0.reshape(4, 1)  # x, y, theta, v
        self.obstacle_radius = 2.0

    def update(self, u, dt):
        dx = array([self.x[3] * cos(self.x[2]),
                    self.x[3] * sin(self.x[2]),
                    u[0],
                    u[1]]).reshape(4, 1)
        self.x += dx * dt

    def draw(self):
        M = array([[1, -1, 0, 0, -2, -2, 0, 0, -1, 1, 0, 0, 3, 3, 0],
                   [-2, -2, -2, -1, -1, 1, 1, 2, 2, 2, 2, 1, 1, -1, -1],
                   ones(15)])
        M = dot(array([[cos(self.x[2][0]), -sin(self.x[2][0]), float(self.x[0])],
                       [sin(self.x[2][0]), cos(self.x[2][0]), float(self.x[1])],
                       [0, 0, 1]]), M)
        plot(M[0], M[1], 'b')

    def sonar_front(self, obs, sonar_range=40.0, fov=pi/3):
        """
        Simule ce que renverrai un sonar frontal du robot 
        """
        min_dist = None
        x_pos, y_pos, theta = float(self.x[0]), float(self.x[1]), float(self.x[2])
        ox, oy = float(obs[0]), float(obs[1])
        dx, dy = ox - x_pos, oy - y_pos
        dist = sqrt(dx**2 + dy**2) # on calcul la distance entre le robot et l'obstacle

        if dist > sonar_range:
            return 

        angle_to_obs = atan2(dy, dx)
        angle_diff = (angle_to_obs - theta + pi) % (2 * pi) - pi # différence entre orientation du robot et obtable, entre -pi et pi

        if abs(angle_diff) <= fov / 2:       # On vérifie que l'obstacle soit en face (sonar frontal)
            if min_dist is None or dist < min_dist:
                min_dist = dist

        return min_dist  # ou None si rien vu attention 
    
    def sonar_left(self, obs, sonar_range=40.0, fov=pi/3):
        """
        Simule ce que renverrai un sonar latéral gauche
        """
        min_dist = None
        x_pos, y_pos, theta = float(self.x[0]), float(self.x[1]), float(self.x[2])
        ox, oy = float(obs[0]), float(obs[1])
        dx, dy = ox - x_pos, oy - y_pos
        dist = sqrt(dx**2 + dy**2) #  distance entre le robot et l'obstacle

        if dist > sonar_range:
            return 
        
        angle_to_obs = atan2(dy, dx) # angle le robot et l'obstacle
       
        left_theta = theta + pi / 2  # Décalage de 90° vers la gauche (sens trigonométrique) de l'orientation du robot

        angle_diff = (angle_to_obs - left_theta + pi) % (2 * pi) - pi   # Normalisation de l'angle pour qu'il soit dans [-pi, pi]
        
        # On vérifie si l'obstacle est dans le champ de vision latéral gauche
        if abs(angle_diff) <= fov / 2:
            if min_dist is None or dist < min_dist:
                min_dist = dist

        return min_dist
    

    def sonar_right(self, obs, sonar_range=40.0, fov=pi/3):
        """
        Simule ce que renverrai un sonar latéral droit
        """
        min_dist = None
        x_pos, y_pos, theta = float(self.x[0]), float(self.x[1]), float(self.x[2])
        ox, oy = float(obs[0]), float(obs[1])

        # Vecteur entre le robot et l'obstacle
        dx, dy = ox - x_pos, oy - y_pos
        dist = sqrt(dx**2 + dy**2)

        # Si l'obstacle est hors de portée, on ignore
        if dist > sonar_range:
            return

        # Angle vers l'obstacle
        angle_to_obs = atan2(dy, dx)

        # Direction vers la droite : -90° = theta - pi/2
        right_theta = theta - pi / 2

        # Normalisation de l'angle dans l'intervalle [-pi, pi]
        angle_diff = (angle_to_obs - right_theta + pi) % (2 * pi) - pi

        # Vérifie si l'obstacle est dans le cône de vision du sonar droit
        if abs(angle_diff) <= fov / 2:
            if min_dist is None or dist < min_dist:
                min_dist = dist

        return min_dist
    


    
    def contourner_obstacle(self, obstacles, dt, fig, scale, target):
        print("Début contournement...")
        # on ralenti
        self.x[3] = 7.0
        # Trouver l’obstacle frontal le plus proche
        f_obs = None
        min_dist = float('inf')
        for obs in obstacles:
            dist = self.sonar_front(obs)
            if dist is not None and dist < min_dist:
                min_dist = dist
                f_obs = obs

        if f_obs is None:
            print("Erreur : aucun obstacle détecté frontalement.")
            return

        # Calcul de l’angle relatif (exactement comme dans sonar_front)
        x_pos, y_pos, theta = float(self.x[0]), float(self.x[1]), float(self.x[2])
        ox, oy = float(f_obs[0]), float(f_obs[1])
        dx, dy = ox - x_pos, oy - y_pos
        angle_to_obs = atan2(dy, dx)
        angle_diff = (angle_to_obs - theta + pi) % (2 * pi) - pi

        # Choisir le côté le plus direct : droite si angle_diff < 0
        if angle_diff > 0:
            cote = 'droite'
            direction = -1.0
            sonar_check = self.sonar_left  # obstacle à gauche du robot après pivot
            angle_vise = theta - pi / 2
        else:
            cote = 'gauche'
            direction = 1.0
            sonar_check = self.sonar_right  # obstacle à droite après pivot
            angle_vise = theta + pi / 2

        print(f"Contournement par la {cote}")

        # Étape 1 : pivot de 90° dans la direction choisie
        while abs((self.x[2] - angle_vise + pi) % (2 * pi) - pi) > 0.05:
            self.update(array([[direction], [0.0]]), dt)
            self._dessine_et_pause(obstacles, fig, scale, target)

        print("Avance en longeant le mur...")

        # Étape 2 : avancer jusqu’à ce que le sonar latéral ne détecte plus rien
        while True:
            closest = None
            for obs in obstacles:
                dist = sonar_check(obs)
                if dist is not None and (closest is None or dist < closest):
                    closest = dist

            if closest is None or closest > 10.0:
                print("Obstacle contourné.")
                break

            self.update(array([[0.0], [0.0]]), dt)
            self._dessine_et_pause(obstacles, fig, scale, target)

        

    def _dessine_et_pause(self, obstacles, fig, scale, target):
        clf()
        axis('square')
        axis([-scale, scale, -scale, scale])
        self.draw()

        for obs in obstacles:
            circle = Circle((obs[0][0], obs[1][0]), self.obstacle_radius, color='gray')
            gca().add_patch(circle)
        
        if target is not None:
            plot(target[0], target[1], 'rs', ms=10)  # cible

        fig.canvas.draw()
        pause(0.015)


    def choisir_cote_contournement(self, obstacle):
        """
        Return gauche ou droite en fonction de la direction qui demande le moins de rotation pour contourner l'obstacle 
        """
        x_pos, y_pos, theta = float(self.x[0]), float(self.x[1]), float(self.x[2])
        ox, oy = float(obstacle[0]), float(obstacle[1])

        angle_to_obs = atan2(oy - y_pos, ox - x_pos)

        angle_diff_right = abs((angle_to_obs - (theta - pi / 2) + pi) % (2 * pi) - pi)
        angle_diff_left = abs((angle_to_obs - (theta + pi / 2) + pi) % (2 * pi) - pi)

        return 'droite' if angle_diff_right < angle_diff_left else 'gauche'




