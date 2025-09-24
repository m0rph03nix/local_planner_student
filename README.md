
# local_planner_student

## Binômes

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

Le turtelbot est tout trouvé pour cet usage. L'utilisation d'un simulateur facilitera la réalisation du TP. Il y a 2 possibilités. Les instructions se trouvent dans la partie du TP géré par Jacques Saraydaryan : https://github.com/jacques-saraydaryan/global_planner_short_path_student/tree/dev_ros2_jazzy


> Pour éviter un conflit avec les "autres ROS de la salle", faites bien 
  soit       
  `export ROS_LOCALHOST_ONLY=1` dans pour cloisonner la communication ROS à votre machine/container
  soit   
  `export ROS_DOMAIN_ID=<your_domain_id>` dans votre container et votre machine pour authoriser la communication ROS dans un domaine défini. Vous remplacerez `<your_domain_id>` par votre numéro de binôme (cf numéro de groupe du repo GIT de rendu qui vous a été créé par le prof. e.g. `S1_G**2**_...` ou `Groupe-**2**` alors mettez `2`), afin qu'il n'y ait pas de conflits entre les groupes.


Vous clonerez ce repo (`git clone https://github.com/m0rph03nix/local_planner_student.git`) à côté du TP `global_planner_short_path_student` si vous l'avez déjà fait.

Pour chaque repo de sujet de TP (global_planner_short_path_student et local_planner_student) vous pourrez taper la commande suivante : 
`git remote add rendu <lien https de votre repo GIT de rendu>`. 
> :bulb: [Managing Remote Repositories](https://docs.github.com/en/get-started/getting-started-with-git/managing-remote-repositories)

Ainsi, vous pourrez pousser vos codes sur le repo GIT de rendu qui vous a été créé par le prof ( e.g `git push rendu nom_de_ma_branche`), tout en gardant le lien avec les repo des sujets (qui eux sont par défaut sur `origin` et non sur `rendu`). 

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
      ros2 run local_planner_student testPathGenerator
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
 


Imprégnez-vous de la structure du code. Complétez le template. Des commentaires `#TODO` indique dans les grandes lignes ce qu'il faut faire. Il est possible également de refaire tout le code (pas recommandé dans le temps imparti).
Commencez par les `#TODO` des callbacks de topics. Continuez par tous les autres `#TODO`. Celui de la méthode `path_service_callback` peut se faire éventuellement dans un second temps.

En effet, avant de tester le service associé à `path_service_callback` (`/pathService`), il faudra d'abord tester le service associé à `goal_service_callback` (`/goalService`), qui est nettement plus simple (un goal, plutôt qu'une liste de waypoints).

Lorque le progrmamme fonctionne bien avec le service `/goalService`, ajoutez la fonctionnalité du service `/pathService`. Pour finir veillez à l'orientation finale.




# FAQ

## Errors

### tf_transforms

> :x: ModuleNotFoundError: No module named 'tf_transformations'
```bash
# Install this
sudo apt-get install python3-pip # if needed
pip3 install transforms3d

# Then...

# if you are root do this :
sudo apt-get install ros-humble-tf-transformations

# if you are not root, clone and build the sources in your workspace src folder :
git clone git@github.com:DLu/tf_transformations.git
```



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
