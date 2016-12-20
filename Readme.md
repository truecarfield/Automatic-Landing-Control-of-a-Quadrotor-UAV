The automatic landing control algorithm of a quadrotor UAV is edited within the framework of Hector Quadrotor. After downloading the packages into your workspace and before the compilation,the following command should be executed 

cd ~/your_workspace
rosdep install --from-path src --ignore-src --rosdistro indigo

This will install all dependencies of packages found in the src folder. 

Compared with original Hector Quadrotor, the following modifications has been made:

1. In package hector_quadrotor_controller, the a cpp file "landing_controller.cpp" has been generated, which is the source code of the landing control algorithm. 

2. In package hector_quadrotor_controller, the cpp file "timer.cpp" has been defined, which contains the timer functions that required by "landing_controller.cpp". the file "pid.cpp", where the PID controllers used in the original controllers are defined, has been modified, such that some new function has been added for the requirements from "landing_controler.cpp". 

3. In package hector_quadrotor_description, some new urdf files has been added, which are "box.urdf.xacro", "boxrobot.urdf.xacro". In those urdf files the features of target UGV for landing has been defined.

In order to launch the landing simulation, the launch file "outdoor_flight_gazebo.launch" is to be executed, the corresponding command is "roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch". There, the gazebo interface of the quadrotor controller has been configurated as the landing control algorithm in the package hector_quadrotor_controller. The quadrotor has been managed by a state machine, the default state after launch of the simulation is joystick state. By executing the command "roslaunch hector_quadrotor_teleop xbox_controller.launch", the speed control state with joystick of the quadrotor can be activated. Once the UGV is within the FOV of the quadrotor, the trajectory prediction algorithm will start working, after 5 seconds prediction, the quadrotor will try to chase the trajectory of the UGV blindly, once the quadrotor get the view of the UGV again, the tracking control with the feedback position information from UAV camera will work until the landing is finished.

