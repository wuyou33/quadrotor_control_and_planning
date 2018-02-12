# Upenn MEAM620 Advanced Robotics, Project1

In phase 1, I applied quintic trajectory planning and PID hover controller on position(xyz) and attitude(euler angles) for the quadrotor dynamic control so that it could follow path with accuracy and speed.
 
In phase 2, I developed an efficient and intuitive map data storage and achieved path planning in Dikjstra and Astar. With given boundary, obstacles, start and end, the algorithm can find a shortest path without collision quickly (if exists).
 
In phase 3, I post-processed the output of Astar/Dijkstra to reduce the trajectory length and time. Basically, I replaced segments between path points with no obstacles in between them with a straight line. 
 
In phase 4, I implemented control and trajectory generation alogorithm into a real world quadrotor Grazyflie 2.0 with VICON and ROS system for 3 tasks. I tried diffent trajectory like 3rd order minimum accleration and 7th order minimum snap as well as different constrains at waypoints like zero or continuous velocity.

https://youtu.be/Vn3-uVWl6RQ ; https://youtu.be/dtDL0dvO6yI

