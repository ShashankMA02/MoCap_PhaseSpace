# MoCap_PhaseSpace
ROS Noetic

A. [Publisher] 

1. code .bashrc (comment or uncomment Noetic / Foxy accordingly,  last few lines)
2. focas-main@focaslab-01:~$ roslaunch phasespace_bringup phasespace_mocap.launch[terminal-1 command]
3. rqt , in another terminal


B. [Subscriber]
1. Build the workspace using catkin and also colcon build it in ROS1
2. Create a folder with below file and grant permission for the file (chmod +x <filename>
3. focas-main@focaslab-01:~/catkin_hexa_ws/Mocap_listener$ ./mocap_listen4.py  [terminal-2 comand]
