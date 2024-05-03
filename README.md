# Projet Temps-Réel INSA AE 2024

[![forthebadge](http://forthebadge.com/images/badges/built-with-love.svg)](http://forthebadge.com)![forthebadge](https://forthebadge.com/images/badges/made-with-c-plus-plus.svg)
![forthebadge](https://forthebadge.com/images/badges/0-percent-optimized.svg)![forthebadge](https://forthebadge.com/images/badges/works-on-my-machine.svg)
     
Ce projet a été réalisé dans le cadre du cours de temps réel de 4ème année.      
A l'aide de différentes tâches disponibles dans le fichier [tasks.cpp](software/raspberry/superviseur-robot/tasks.cpp), nous avons contrôlé différents aspects d'un robot à l'aide d'une Raspberry Pi.       

## Repertoires      
       
- hardware : contient les plans pour la partie mecanique du robot et de son chargeur, ainsi que les plans de conception des PCB du robot, du chargeur, de l'adaptateur Xbee pour la raspberry  et les plans des CAP du robot.
- software: rassemble les parties logicielles du robot, du chargeur, les bibliotheques et superviseur coté raspberry et l'interface Web.
- doc: contient les sujets de TD et TP.
- aruco_markers: Script de generation des tags (aruco) utilisés sur les robots.
- perso : contient les schéma de spécification et de conception qui retranscrisent les différentes tâches implémentés dans notre fichier [tasks.cpp](software/raspberry/superviseur-robot/tasks.cpp).      
     
## Démarrage
     
### Démarrer le robot
      
Rien de plus simple, assurez-vous qu'il soit chargé puis appuyez sur le bouton situé sur le côté de celui-ci.     
      
### Démarrer le superviseur
      
1. Lancez Netbeans et ouvrez le projet.
2. Cliquez droit sur le projet et choisir _Properities_.
3. Allez dans l'onglet _Build_.
4. Choisir _..._ pour _Build Host_.
5. Cliquez sur _Add_.
6. Remplir Hostname avec 10.105.0.x+101 l'adresse de la RaspberryPi (x est le numéro sur l'étiquette de la Raspberry Pi).
7. Cliquez _Next_.
8. Saisir _xenomai_ pour le _login_.
9. Cliquez _Next_.
10. Attendre l'ouverture de la communication.
11. Saisir _xenomai_ comme mot de passe.
12. Sélectionner _SFTP_ pour _Access project file via_.
13. Cliquez _Finish_.
14. Cliquez _Ok_.
15. Cliquez _Set As Default_.
16. Cliquez _Ok_.
17. Compilez en clquant sur le symbole du marteau.
18. Ouvrez un Terminal sur votre machine et tapez _ssh xenomai@10.105.0.X+101_.
19. Une fois connecté en ssh, tapez _cd ./dumber/software/monitor/monitor-python-qt_ et lancez ensuite _./monitor-python 10.105.0.x+101_ pour démarrer le programme sur le superviseur.     
      
### Démarrer le moniteur
       
Depuis le Terminal de votre PC, placez vous dans le répertoire du projet à la racine et tapez _cd software/monitor/monitor-python-qt_ et lancez _./monitor-python 10.105.0.X+101_.     
      
## Auteurs      
      
Les auteurs de ce projet sont :
* **Noël JUMIN** _alias_ [@NoNo47400](https://github.com/NoNo47400).
* **Clément MARCE** _alias_ [@Clemucho](https://github.com/Lab0x08).
        
Ce repo est un fork du projet [INSA Real Time](https://github.com/INSA-GEI/dumber/tree/evoxx-dumber-v3).       






