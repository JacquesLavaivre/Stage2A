# Stage2A
Dépôt de travail de mon stage de 2e année

# Pour lancer la simulation il faut:

1) Lancer ROS
  ```bash
    $ roscore
  ```
2) Lancer VREP
  ```bash
    $ ./vrep.sh
  ``` 
3) Ouvrir la scène mylittlebotnoScript.ttt
  
4) Se placer dans le workspace
  ```bash
    $ cd workspace
  ``` 
5) Ensuite éxécuter les commandes suivantes:
  
    ```bash
    $ source devel/setup.bash
    $ catkin_make
    $ roslaunch stage robot.launch
  ``` 
 
