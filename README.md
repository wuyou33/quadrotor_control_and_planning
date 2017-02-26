# MEAM620_Project1

In phase 1, I applied quintic trajectory planning and PID hover controller on position(xyz) and attitude(euler angles) for the quadrotor dynamic control so that it could follow path with accuracy and speed.
 
In phase 2, I developed an efficient and intuitive map data storage and achieved path planning in Dikjstra and Astar. With given boundary, obstacles, start and end, the algorithm can find a shortest path without collision quickly.
 
In phase 3, I post-processed the output of Astar/Dijkstra to reduce the trajectory length and time. I replaced segments between path points with no obstacles in between them with a straight line. 
 
In phase 4, I implemented control and trajectory generation alogorithm into a real world quadrotor Grazyflie 2.0 under VICON and ROS system for 3 tasks.
