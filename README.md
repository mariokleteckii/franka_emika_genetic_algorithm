# franka_emika_genetic_algorithm

To launch:

1. Create catkin workspace, add folder franka_gazebo_moveit in src folder and then build. 
2. run gazebo simulation with position controllers `roslaunch franka_gazebo_moveit 		            simulator_position.launch`
   This should start an empty Gazebo world with Franka Panda with **RED** link0 and end effector


3. run moveit planning `roslaunch franka_gazebo_moveit gazebo_ctrl_gui.launch` 
	In RViz:
		a) File -> Open Config -> moveit_start4.rviz -> Open
		b) Move Panda Arm a little bit forward and Plan&Execute it (It will prevent self collision)

4. run rosbridge server `roslaunch rosbridge_server rosbridge_websocket.launch`

5. run python script `rosrun franka_gazebo_moveit simple_planning.py`

6. run .jar `java -jar ~/NextMove/target/NextMove-0.0.1-SNAPSHOT.jar 0.1 0.3 0.7 0 0 0 1`

Minimal number of arguments is 7 (respresent next position). Eighth argument is population size (by default 15) and ninth argument is the maximum number of generations of algorithm (by default 10).


