# Documentation du service Pick and Give

## Vue d'ensemble

Ce service ROS sert à effectuer des tâches de pick-and-give (prendre et donner un objet) à l'aide d'un bras robotique avec une intégration MoveIt!. Il permet de contrôler les bras et les pinces du robot pour saisir un objet à un endroit donné et le donner à un visiteur. Le service utilise des messages et services ROS pour interagir avec le robot et réaliser la tâche en réponse aux requêtes.

Le script est conçu pour fonctionner avec un robot disposant de deux bras, d'un torse et d'une pince sur chaque bras.

## Composants clés

- **Services ROS** : 
  - Le service `pick_and_give_service` écoute les requêtes et réalise la tâche de pick-and-give.
  - Il communique avec le service `/homing` pour ramener le robot à sa position d'origine après l'exécution de la tâche.

- **Intégration MoveIt!** : 
  - Le script utilise MoveIt! pour la planification et le contrôle des mouvements des bras, du torse, ainsi que de l'ouverture et la fermeture des pinces.

- **Logique de Pick and Place** : 
  - Le robot déplace son bras jusqu'à une position donnée pour saisir un objet avec sa pince.
  - Après avoir pris l'objet, il le déplace vers une position cible et le donne en ouvrant la pince.

## Classes

### `PickAndPlaceService`

Cette classe contient la logique principale pour le service de pick-and-place.

#### Méthodes :

- **`__init__(self)`** : 
  Initialise MoveIt! et le nœud ROS, attend le service de "homing", et configure les groupes nécessaires pour le torse, les bras et les pinces du robot.
  
- **`monter_buste(self, hauteur)`** : 
  Déplace le torse du robot à la hauteur spécifiée (donnée par `hauteur`).
  
- **`arm_move_to(self, number, arm)`** : 
  Déplace le bras spécifié (gauche ou droit) vers des positions articulaires prédéfinies basées sur le paramètre `number` (1 ou 2).
  
- **`go_to_height(self, n)`** : 
  Ajuste la position du torse en le déplaçant vers le haut ou vers le bas en fonction de la valeur `n`.
  
- **`take(self, flyerNb, take, arm, px, py, pz, ox, oy, oz, ow)`** : 
  Déplace le bras spécifié (gauche ou droit) aux coordonnées données (px, py, pz) et à l'orientation (ox, oy, oz, ow) pour soit saisir (take=True) soit tirer (take=False) un objet (l'objet doit être sorti du socle pour ensuite être donné).
  
- **`give(self, arm)`** : 
  Déplace le bras vers une position prédéfinie pour donner l'objet (position pour donner l'objet).
  
- **`mouvementPince(self, state, arm)`** : 
  Ouvre ou ferme la pince du bras spécifié (gauche ou droit) en fonction du paramètre `state` (`open` ou `close`).
  
- **`pick_and_place_handler(self, req)`** : 
  La fonction principale pour gérer le service ROS `pick_and_place`. Elle traite la requête, effectue la tâche de pick-and-place, et renvoie une réponse vide après la fin de la tâche.

## Edition du service

Pour lancer l'environement DEV, afin d'éditer le service de pick-and-give, suivez ces étapes :

1. **Connectez-vous au robot** : 
   (Connexion filaire recommandée)

2. **Vérifiez la connexion** : 
   Dans un terminal, lancez la commande 'rostopic list' pour vérifier que la connexion est correcte.

   ```bash
   rostopic list
   ```

3. **Activer conda** :
   A la racine du projet git, ouvrez un terminal et rentrez la commande suivante : 
   ```bash
   conda activate cesi-python
   ```

4. **Créez les fichiers nécessaires à l'environement DEV** :
   Dans le même terminal, rentrez la commande suivante : 
   ```bash
   catkin make
   ```

5. **Activer l'environement de DEV** :   
    Dans le même terminal, rentrez la commande suivante : 
   ```bash
   source devel/setup.bash
   ```

6. **Démarrer le service Pick and Give** : 
   Exécutez le script sur le robot en lançant le script Python :
   
   ```bash
   roslaunch pick_and_give pick_and_give.launch
   ```

7. **Appeler le service dans un terminal** : 
   Normalement, c'est la machine à état de Tiago qui fait les appels vers les services (dont celui-ci), si vous voulez appeler manuellement le service dans le cadre du DEV, vous pouvez appeler le service `pick_and_give` en envoyant une requête `PickAndGive` avec les coordonnées et le numéro de flyer appropriés. Cela déclenchera le robot pour réaliser la tâche de pick-and-place.

   Exemple :

   ```bash
   rosservice call /pick_and_place "coordinates:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    pose:
      position:
        x: 0.0
        y: 0.0
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
    flyerNb: 0"
   ```

8. **Appeler le service dans le code** :
   *todo*

## Dépendances

- **ROS (Robot Operating System)** : Le script est conçu pour être utilisé avec ROS.
- **MoveIt!** : Une bibliothèque ROS pour la planification des mouvements des robots.
- **Python3** : Le script est écrit en Python 3.
- **SciPy** : Utilisé pour manipuler les rotations via les quaternions.

## Remarques

- Assurez-vous que l'environnement du robot est correctement configuré avec MoveIt! et que tous les groupes de planification nécessaires pour les bras, le torse et les pinces sont disponibles.
- Ajustez les listes `joint_start_1_*` et `joint_start_2_*` si nécessaire pour correspondre aux configurations articulaires de votre robot.
- Les actions de la pince dépendent des positions articulaires définies dans les tableaux `joint_open_positions` et `joint_close_positions`. Ajustez ces valeurs selon les spécifications de votre pince ou selon votre besoin.

## Licence

Ce projet est sous licence MIT - voir le fichier [LICENSE](LICENSE) pour plus de détails.