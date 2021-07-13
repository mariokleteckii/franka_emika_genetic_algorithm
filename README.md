# franka_emika_genetic_algorithm
To launch:

1. run gazebo simulation with position controllers `roslaunch franka_gazebo_moveit simulator_position.launch`
   This should start an empty Gazebo world with Franka Panda with **RED** link0 and end effector


2. run moveit planning `roslaunch franka_gazebo_moveit gazebo_ctrl_gui.launch` 

3. run rosbridge server `roslaunch rosbridge_server rosbridge_websocket.launch`

4. run python script `rosrun franka_gazebo_moveit simple_planning.py`

5. run .jar `java -jar ~/NextMove/target/NextMove-0.0.1-SNAPSHOT.jar 0.3 -0.4 0.65 0 0 0 1`

Minimal number of arguments is 7 (respresent next position). Eighth argument is population size (recommended 15) and ninth argument is the maximum number of generations of algorithm.


