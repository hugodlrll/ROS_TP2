Hugo DELRUELLE

# Description #

Le noeud que j'ai développé est le noeud coach. Voici les fonctions qui sont implémentées :
    - Si la balle est dans le camp allié, il envoie l'instruction d'attaquer. Si elle est dans le camp adverse, il envoie l'instruction de défendre. (ball_callback())
    - Si l'énergie du joueur est inférieure à 20, le coach fait une demande de substitution pour changer le joueur. La substitution a été testée avec le code de l'arbitre et elle fonctionne (player_callbakc). En fonction du retour de l'arbitre, il envoie une réponse sur l'acceptation ou non de la substitution (request_substitution()) et l'effectue avec (execute_subsitution())
J'ai également implémenté l'urdf pour visualiser le terrain. Malheureusement il ne fonctionne pas avec les joueurs malgré mes diverses tentatives. Il y a ce bug à corriger. Le controller en revanche semble fonctionner car le commandes s'affichent dans le terminal lorsque j'appuie sur les boutons de la manette. Je n'ai pas eu le temps de tester l'implémentation de la balle.

RETEX module ROS :
Module permettant une bonne prise en main de l'outil ROS2. En revanche, les instructions du TP1 pourraient être plus précises pour mieux nous guider à travers la découverte de ROS2. Egalement, le contour du projet handball pourrait être défini sur une feuille de consigne, notamment au niveau des topics, actions à implémentées sur les noeuds... pour éviter de se perdre et améliorer le branchement finale entre les noeuds.


# ROS2 commandes #

1. Pour build : colcon build
2. pour exécuter :
    source /opt/ros/humble/setup.bash
    source install/setup.bash

    ros2 run node name
    ros2 launch <nom_du_package> <nom_du_fichier_launch>

3. Pour tester :
    Lancer rqt

    Rviz :
    LIBGL_ALWAYS_SOFTWARE=1 ros2 run rviz2 rviz2




    