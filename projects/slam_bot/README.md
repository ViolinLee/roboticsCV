### Launch Everything

Launch Kitching & Dining world in Gazebo:

``roslaunch udacity_bot world.launch``    

Launch teleop node for keyboard control:

``roslaunch udacity_bot teleop.launch``

Launch RTAB-Map mapping node:

``roslaunch udacity_bot mapping.launch``

Launch RViz GUI:

``roslaunch udacity_bot rviz.launch``   

###Note: 

Modify world launch command to:    

``roslaunch udacity_bot playground.launch``

to launch custom world. Other comands remain the same.    

The rtab-map parameters used in two simulation are differnet. Specific parameters can be found in the write-up PDF file. These parameters can be modify in the following file:   

``udacity_bot/launch/mapping.launch``
