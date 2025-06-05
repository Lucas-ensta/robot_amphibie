from __future__ import print_function
from matplotlib.pyplot import *
from matplotlib.patches import Circle
from numpy import *
from numpy.random import *
from math import cos, sin, sqrt, pi, atan2
from rob import Robot 
from ctrl import Control

class Simulation:
    def __init__(self):
        # Paramètres de simulation
        self.scale = 50.0           # Échelle de la scène graphique
        self.dt = 0.015             # Pas de temps (en secondes)
        self.t = 0.0                # Temps écoulé
        self.targets_caught = 0     # Nombre de cibles atteintes

        # Initialisation du robot (position x, y, angle theta, vitesse)
        v = 20.0
        self.robot = Robot(array([0.0, 0.0, 0.0, v]))

        # Contrôleur pour générer les commandes de mouvement du robot
        self.control = Control()

        # Génération d'une cible aléatoire dans l'environnement
        self.target = self.scale * (rand(2, 1) - 0.5) * 2

        # Génération de 10 obstacles à des positions aléatoires
        self.obstacles = [(rand(2, 1) - 0.5) * 2 * self.scale for _ in range(10)]

        # Activation du mode interactif de matplotlib
        ion()

        # Création de la fenêtre graphique
        self.fig = figure("Autonomous Robot")

        # Pour certaines interfaces (comme Tkinter), on donne le focus à la fenêtre
        if "get_tk_widget" in dir(self.fig.canvas):
            self.fig.canvas.get_tk_widget().focus_set()

    def run(self):
        while True:
            # Efface le contenu précédent et redéfinit les axes
            clf()
            axis('square')
            axis([-self.scale, self.scale, -self.scale, self.scale])

            # Dessine le robot à sa position actuelle
            self.robot.draw()

            # Affiche la cible (waypoint) sous forme de carré rouge
            plot(self.target[0], self.target[1], 'rs', ms=10)

            # Dessine les obstacles sous forme de cercles gris
            for obs in self.obstacles:
                circle = Circle((obs[0][0], obs[1][0]), self.robot.obstacle_radius, color='gray')
                gca().add_patch(circle)

            # Utilise le sonar pour détecter les obstacles à proximité
            f_dist = None
            for obs in self.obstacles:
                dist_front = self.robot.sonar_front(obs)
                # On garde la plus petite distance frontale détectée
                if dist_front is not None and (f_dist is None or dist_front < f_dist):
                    f_dist = dist_front
            
            if f_dist is not None and f_dist < 10:
                self.robot.contourner_obstacle(self.obstacles, self.dt, self.fig, self.scale, self.target)
                continue  # on saute cette itération pour ne pas exécuter le contrôle normal

              


                

            # Calcul de la commande de contrôle (vitesse angulaire, accélération)

            self.robot.x[3] = 20
            u = self.control.calcul_commande(self.robot.x, self.target)

            # Mise à jour de l'état du robot selon la commande et le pas de temps
            self.robot.update(u, self.dt)

            # Vérifie si la cible est atteinte (distance < seuil)
            dx = self.target[0] - self.robot.x[0]
            dy = self.target[1] - self.robot.x[1]
            dist = sqrt(dx**2 + dy**2)
            if dist < 2.5:
                self.targets_caught += 1
                # Génère une nouvelle cible aléatoire
                self.target = self.scale * (rand(2, 1) - 0.5) * 2

            # Pause pour respecter le pas de temps visuel
            pause(self.dt)
            self.t += self.dt  # Incrémente le temps écoulé

            # Condition d'arrêt : si la fenêtre est fermée
            if gcf().canvas.manager.key_press_handler_id is not None and not gcf().canvas.manager.window.isVisible():
                break

        # Affiche le score final
        self.print_score()

        # Fonction interne d'affichage du score
        def print_score(self):
            score = 100 * self.targets_caught / self.t
            print(f"\n\nTargets caught: {self.targets_caught}, Time: {self.t:.2f} s, Score: {score:.2f}\n")


# Lancement de la simulation si ce fichier est exécuté en tant que programme principal
if __name__ == "__main__":
    print("Simulation en cours...")
    sim = Simulation()
    sim.run()
