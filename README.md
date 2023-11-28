# sensorless_robots

Ashwin Subramanian

These scripts work with any ros simulation enviorment where the robot is publishing coordinates. Some configuration is needed depending on the map.


    Example:
    Clone and follow instructions: https://github.com/ai-winter/ros_motion_planning

    Disable pedestrians by setting pedestrians: "" in catkin_ws/src/ros_motion_planning/src/user_config/user_config.yaml

    Record a rosbag on /odom

    Run "python3 map.py" while simulation is open. Map parameters and obstacle csv will be generated. Crtl + C after "done" output

    Run "python3 overlay.py" for full paths and/or mcl visualization. Tweak parameters as neccesary.

 - *If using a different map than the example, Don't forget to change the readbag() function to change the topic and/or the data structure to get the position.
 - *Check "Display Options" and Params to configure trials as desired for either class.