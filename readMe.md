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

# Résumé séance 27/01/2026 #

-> request substitution approval fonctionne pour le coach lorsqu'un joueur a une énergie inférieure à 20
-> En fonction de la balle, le coach donne des ordres différents

-> reponse subtitution approval ne fonctionne pas coté arbitre
-> Le reste de l'arbitre n'a pas été testé pour le moment
    