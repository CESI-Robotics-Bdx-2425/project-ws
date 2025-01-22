# Documentation du service Homing

## Vue d'ensemble

Ce service ROS permet de faire un retour en position "home". La fonction "Home" existante n'étant pas bonne (elle fait bugger le robot), nous l'avons recodée.
Le script Python implémente un service ROS pour amener un robot à une position d'origine (Homing). Le service utilise MoveIt! pour planifier et exécuter les mouvements des articulations du robot afin de le ramener à une position de référence prédéfinie. 

## Composants clés

- **Service ROS** : 
  - Le service `homing` est un service ROS qui amène le robot à sa position d'origine (home). Il est exposé sur le service `/homing`.

- **MoveIt! Integration** : 
  - Le script utilise MoveIt! pour la planification des mouvements du robot et pour envoyer les commandes aux bras et au torse.

- **Position d'origine** : 
  - La position d'origine est une configuration d'articulation prédéfinie. Lorsque le service est appelé, le robot se déplace vers cette configuration.

## Classes

### `Homing`

Cette classe gère la logique du service Homing.

#### Méthodes :

- **`__init__(self)`** : 
  Initialise MoveIt! et le nœud ROS, configure les groupes de mouvement pour les bras et le torse du robot. Le service ROS `homing` est également enregistré ici.

- **`home(self, _)`** : 
  Cette méthode est appelée lorsque le service `/homing` est invoqué. Elle déplace le robot vers la position d'origine en envoyant les commandes de mouvement au groupe de mouvement configuré (`both_arms_torso`).

## Utilisation

Pour utiliser le service Homing, suivez ces étapes :

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
   roslaunch go_home homing.launch
   ```

7. **Appeler le service dans un terminal** : 
   Une fois le service démarré, vous pouvez appeler le service `homing` pour ramener le robot à sa position d'origine. Utilisez la commande suivante :

   ```bash
   rosservice call /homing
   ```

8. **Appeler le service dans le code** : 
   Si vous voulez appeler le service dans du code :

   ```py
   rospy.wait_for_service('/homing')
   
   home = rospy.ServiceProxy('homing', Empty)
   r = home()
   rospy.sleep(3)
   ```

## Dépendances

- **ROS (Robot Operating System)** : Le script est conçu pour être utilisé avec ROS.
- **MoveIt!** : Une bibliothèque ROS pour la planification des mouvements des robots.
- **Python3** : Le script est écrit en Python 3.

## Remarques

- Assurez-vous que l'environnement du robot est correctement configuré avec MoveIt! et que le groupe de planification pour les bras et le torse (`both_arms_torso`) est disponible.
- Les valeurs de `joint_home` représentent la position d'origine souhaitée du robot. Vous pouvez ajuster ces valeurs en fonction de la configuration exacte de votre robot.

## Licence

Ce projet est sous licence MIT - voir le fichier [LICENSE](LICENSE) pour plus de détails.