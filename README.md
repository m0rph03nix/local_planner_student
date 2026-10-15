
# local_planner_student

## Binômes  

Vous conserverez les mêmes goupes (monomes/binômes) dans tous les TPs de ce module, quelque soit l'intervenant.

## Introduction au TP

Le but de ce TP est de coder un "local planner" sur la base du template fourni : [local_planner.py](local_planner_student/local_planner_student/local_planner.py)
Ce fichier se lance en l'état MAIS NE FONCTIONNE DONC PAS EN L'ETAT. Il faut d'abord compléter TOUTES les balises `#TODO` après avoir lu tout ce README d'explication.    

Voici le schéma de principe du fonctionnement de ce "local planner" : 

| ![local_planner.jpg](img/local_planner.jpg) |
|:-------:|
| *local_planner_student basic principle for a differential drive robot like turtlebot* |


Ce "local planner" fonctionne (une fois complété :smile:) avec un robot à base différentielle.
- Entrées : 
  - Capteurs : topics `/odom` et `/scan`
  - Ordre : services `/goalService` ou `/pathService` 
- Sorties : commande en velocité envoyée sur le topic `/cmd_vel` :


```mermaid 
graph LR
    T1[Odometry source] -- /odom --> Node((local_planner_student))
    T2[Laser source] -- /scan --> Node((local_planner_student))

    S1[ ] -. /goalService .-> Node
    S2[ ] -. /pathService .-> Node

    Node -- /cmd_vel -->D[base controller]
```

## Installation et rendu

> :warning: **Ne clonez pas ce repo.** Suivez le guide **[SETUP_TP.md](https://gitlab.com/cpelyon/rob_ressources/ros2_cuda_gazebo.docker/-/blob/main/SETUP_TP.md)**, étapes 0 à 4, dans l'ordre. Il est commun à tous les TPs de navigation : si vous l'avez déjà suivi pour un autre TP, faites seulement l'ajout de ce sujet (étape 2, `git subtree add` avec `tp_local`).

**Lancez toujours le conteneur avec `./run_tp.sh`, depuis la racine de votre workspace** (guide, étape 3) : le script vérifie votre installation avant de démarrer.
- Corrigez tout point rouge ✖ avant de continuer.
- Pour ce TP, `local_planner_student` et `global_planner_short_path_student` doivent être en vert ✔.

Le rendu se fait en poussant sur votre repo de rendu (guide, étape 5).

### Simulateur

Dans le conteneur, cette commande ouvre Gazebo **et RViz** (déjà configuré pour Nav2) :

```bash
export TRAINING_GLOBAL=/root/student_ws/src/global_planner_short_path_student
ros2 launch nav2_bringup tb4_simulation_launch.py headless:=False world:=$TRAINING_GLOBAL/world/baseline.world x_pose:=1.0 y_pose:=1.0 z_pose:=0 map:=$TRAINING_GLOBAL/map/baseline.yaml
```

Puis, dans un autre terminal (`./run_tp.sh`), initialisez la position du robot sur la carte (nécessaire pour `/pathService`, qui utilise la TF `map`) :

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}}"
```

(ou, dans RViz, outil **2D Pose Estimate** : cliquez sur la position du robot et orientez la flèche vers l'avant du robot.)

## Lancement des codes

### Local Planner
  ```bash
  ros2 run local_planner_student local_planner # Valeurs par défaut dans code python
  ```
ou
  ```bash
  ros2 launch local_planner_student local_planner_launch.py # Valeurs par défaut dans config/config.yaml
  ```

### Client au service pathService qui publie des waypoints pour une trajectoire de test

  ```bash
  ros2 run local_planner_student test_path_generator
  ```


## Développement

Pour piloter le robot, votre noeud doit pouvoir traiter les 2 services suivants :
+ un Goal sur `goalService`
  - Ce service recoit un message `goalPose2D` de type `geometry_msgs/Pose2D` et répond le message `possible` de type `std_msgs/Bool` donnant la faisabilité de la consigne demandée (on ne fera que vérifier de manière simpliste qu'il n'y a pas d'obstable dans la zone OBSTACLE_RANGE). Les messages du service sont dans le fichier [localGoal.srv](local_planner_srvs/srv/LocalGoal.srv) dans le paquet local_planner_srvs.
  - La consigne `goal_pose2d` est exprimée en absolue dans le repère de la TF odom
  - Le plus simple pour appeler ce service est d'utiliser la commande suivante :
    ```{r, engine='bash', count_lines} 
    ros2 service call /goalService local_planner_srvs/srv/LocalGoal "{
      'goal_pose2d': {
        'x': 1.0,
        'y': 1.0,
        'theta': 0.0
      }
    }"
    ```
+ un Path sur `/pathService`
  - Ce service reçoit un message `path_to_goal` de type `nav_msgs/Path` et répond le message `success` de type `std_msgs/Bool` renseignant le succès ou l'échec de prise en compte de la liste de waypoints (ne garantis pas qu'elle sera bien effectuée sans obstacles) Les messages du service sont dans le fichier [PathToGoal.srv](local_planner_srvs/srv/PathToGoal.srv) dans le paquet local_planner_srvs.
  - Le message étant long et difficile à écrire en console, utilisez le noeud path_generator fourni avec le template. 
      ```{r, engine='bash', count_lines} 
      ros2 run local_planner_student test_path_generator
      ```
      Le code source du générateur se trouve dans le fichier [testPathGenerator.py](local_planner_student/local_planner_student/testPathGenerator.py)
  - La consigne `path_to_goal` est exprimée en absolue dans n'importe quel repère à définir dans `header.frame_id` . Dans le cas du générateur, le `frame_id` envoyé est celui de la `/map`. **Attention** : A sa reception, la pose devra être convertie dans le repère de la TF odom. Cela peut se faire en une ligne avec la méthode `do_transform_pose` avec la transformation issue de la methode `lookup_transform`.

Pour évaluer le déplacement et l'environnement proche, votre noeud doit s'abonner aux 2 topics suivants :
+ /scan pour vérifier qu'il n'y ait pas d'obstacle
  - Pour faire simple, si un des points du laser a un obstacle à moins d'une valeur défini en ROSPARAM lors de l'execution d'un service, on considère qu'un obstacle empêche le succès de ce service. On affichera un log avec la distance de l'obstacle ainsi que l'angle de sa detection, une fois sur 100
+ /odom pour connaitre la position relative du robot par rapport à la tf odom


Pour déplacer le robot, vous devrez publier un topic de commande en velocité:
+ `/cmd_vel` de type twist pour piloter le robot en vitesse
  - Merci d'utiliser à cet effet les ROSPARAM `SAT_LINEAR` et `SAT_ANGULAR` passés au contructeur de localPlanner. Inspirez-vous ce qui est déjà fait avec `K_LINEAR`, `K_ANGULAR`. 
  - Prenez en compte aussi `ANGLE_TO_ALLOW_LINEAR` qui est un angle en radian au-dessus duquel on n'authorise pas de mouvement linéaire.   
  
Plus globalement, prenez en compte chacun des ROSPARAM passés au contructeur de localPlanner:

```python
        # Proportionnal coefficient for linear velocity
        self.K_linear = self._param('K_LINEAR', 1.0)
        # Proportionnal coefficient for angular velocity
        self.K_angular = self._param('K_ANGULAR', 4.0)
        # Max linear velocity
        self.Sat_linear = self._param('SAT_LINEAR', 2.0)                        
        # Max angular velocity
        self.Sat_angular = self._param('SAT_ANGULAR', 1.5708)
        # Distance below which we consider an obstacle
        self.Obstacle_range = self._param('OBSTACLE_RANGE', 0.5)
        # Above this value: angular control only. Below this value: angular and linear control together
        self.Angle_to_allow_linear = self._param('ANGLE_TO_ALLOW_LINEAR', 0.2)
        # Euclidian distance error to a waypoint allowing to move to a new waypoint
        self.Waypoint_eps = self._param('WAYPOINT_EPS', 0.16)
        # Euclidian distance error to the final waypoint below which we consider the position reached
        self.Destination_eps = self._param('DESTINATION_EPS', 0.003)
        # Angular error below which we consider the final orientation reached
        self.Angle_eps = self._param('ANGLE_EPS', 0.2)
```  
  




Voici la machine d'état permettant de passer d'un waypoint à un autre :

```mermaid
sequenceDiagram
    participant New Goal
    participant Reach in progress
    participant Last  position reached
    participant Last  pose reached
    loop for each waypoint
      New Goal->>Reach in progress: 
      Reach in progress->>Reach in progress: 

      Reach in progress->>New Goal:  If another waypoint AND dist < WAYPOINT_EPS
    end
    
    Reach in progress->>Last  position reached:  dist < DESTINATION_EPS
    Last  position reached->>Last  position reached: 
    Last  position reached->>Last  pose reached:  angle < goal angle
    Last  pose reached->>Last  pose reached : .
    
    
```
 
 ## Méthodologie


Imprégnez-vous de la structure du code. Complétez le template. Des commentaires `#TODO` indique dans les grandes lignes ce qu'il faut faire. Il est possible également de refaire tout le code (pas recommandé dans le temps imparti).

Commencez par les `#TODO` des callbacks de topics. Continuez par tous les autres `#TODO`. Celui de la méthode `path_service_callback` peut se faire éventuellement dans un second temps.

En effet, avant de tester le service associé à `path_service_callback` (`/pathService`), il faudra d'abord tester le service associé à `goal_service_callback` (`/goalService`), qui est nettement plus simple (un goal, plutôt qu'une liste de waypoints).

Lorque le progrmamme fonctionne bien avec le service `/goalService`, ajoutez la fonctionnalité du service `/pathService`. Pour finir veillez à l'orientation finale.




# FAQ

## Commands

### Service /goalService

```bash
ros2 service call /goalService local_planner_srvs/srv/LocalGoal "{
  'goal_pose2d': {
    'x': 1.0,
    'y': 1.0,
    'theta': 0.0
  }
}"
```
