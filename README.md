# sensorless_robots

Ashwin Subramanian

These scripts work with any ros simulation enviorment where the robot is publishing coordinates. Some configuration is needed depending on the map.


    Example:
    To generate map and path data:
        To use this code you need an existing ROS world and map setup with the ROS navigation stack.

        While recording a rosbag on /odom, /map, and /map_metadata, move your robot via a goal pose/poses. Name and save that bag in an appropriately named folder in /data.

        Run "python3 map.py" while simulation is open. Map parameters and obstacle csv will be generated. Crtl + C after "done" output. Move outputs to previous folder in /data.

    Run "python3 overlay.py" for full paths and/or mcl visualization. Tweak parameters as neccesary. 


 - *Check "Display Options" and Params to configure trials as desired for either class.