"""
Stratégie utilisation du Lidar 3D (LiDAR Robosense Helios-16P)

==================== Principe de base =========================: 

ETAT = "AVANCER"
boucle:
    lidar_data = get_lidar_cloud()

    obstacles = detecter_obstacles(lidar_data)

    si ÉTAT == "AVANCER":
        si obstacle_devant(obstacles):
            ÉTAT = "EVITER"
        sinon:
            mode_guidage_waypoint_simple()

    sinon si ÉTAT == "EVITER":
        mode_contournement(obstacles)
        si plus_d_obstacle(obstacles):
            ÉTAT = "AVANCER"

================== Comment récupérer les données du capteur =====================

    1) Comprendre le format des données bruts du lidar : nuage point 3D [(x,y,z), ... ]
        Chaque point contient les information de coordonée spatiale, mais aussi "l'intensité, l'horodatage" utile pour des anlyse plus fines,
        mais que nous n'utilisons pas dans notre stratégie 

    2) Convertir le nuange de point en une représentation 2D

        exemple de base : 

        # Conserver uniquement les points proches du sol
            ground_points = [p for p in cloud if abs(p.z) < z_max]

        # Projeter les points (x, y)
            obstacles_2d = [(p.x, p.y) for p in ground_points]


    3) Detection d'obstacle dans un cône frontale (fov = pi/6) devant le robot à moins de (distance_lim = 5m) (comme le sonar utlisé dans la simulation)

        obstacles_front = [p for p in obstacles_2d if abs(atan2(p.y, p.x)) < fov and sqrt(p.x**2 + p.y**2) < distance_lim]
    
    
    (facultatif si on veut aller plus loin que simplement "obstacle devant") -> 
    4) Regrouper les points qui représentent le même obstacle pour éviter les problème de détection double du même obstacle 

        Utiliser DBSCAN, un algorithme de clustering basé sur la densité (inclus dans le mudule python scikit-learn), très adapté aux nuages de points :


    =============== Améliroation du principe de détection de base =========================
    
    1) Pistes possibles pour ne pas considerer les points du sol et filtrer éventuellemnt la présence de hautes herbes 
        a) Utiliser un filtrage simple de l'altitude : 

            ground = [p for p in cloud if p.z < 0.05]
            grass = [p for p in cloud if 0.05 <= p.z <= 0.4]
            obstacle = [p for p in cloud if p.z > 0.4]

        b) Utiliser une analyse temporelle (detection du mouvement des feuille / herbes pour diférencier d'un vrai obstacle) 
            Comparer plusieurs scans successifs, et écarter les points fluctuants.

    ============= structure Pseudo-code intégrant chaque type de filtrage possible =================
    
    # Initialisation
        previous_scan_points = []

        loop every scan:
            lidar_points = get_lidar_scan()  # Liste de (x, y, z)
            
            # 1. Filtrage spatial
            filtered_points = []
            for point in lidar_points:
                if 0.3 < norm(point.x, point.y) < 10 and 0.1 < point.z < 1.5:
                    filtered_points.append(point)

            # 2. Filtrage temporel (fluctuations)
            stable_points = []
            for p in filtered_points:
                if point_exists_nearby(p, previous_scan_points, radius=0.2):
                    stable_points.append(p)

            # 3. Clustering spatial (DBSCAN-like)
            clusters = []
            visited = set()

            for point in stable_points:
                if point in visited:
                    continue
                cluster = find_neighbors(point, stable_points, eps=0.5)
                if size(cluster) >= 5:
                    clusters.append(cluster)
                    visited.update(cluster)

            # 4. Extraire position des obstacles (moyenne spatiale des position des clusters)
            obstacles = []
            for cluster in clusters:
                center = average_position(cluster)
                obstacles.append(center)

            # 5. Mettre à jour le scan précédent
            previous_scan_points = stable_points.copy()

            # 6. Envoyer obstacles à l'algorithme d'évitement
            update_navigation_with_obstacles(obstacles)






"""