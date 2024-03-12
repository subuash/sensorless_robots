# sensorless_robots

Ashwin Subramanian

Despite an increase in the number of people who rely on manual wheelchairs, there are still substantial economic barriers to affordable and accessible localization systems. As a result, there is a pressing need to build a versatile yet low cost localization system for manual wheelchairs. Such systems allow users to self-navigate their environment, and foster greater self-sufficiency. Existing solutions rely on external sensors to assist with localization; creating a sensor-free system will result in a much cheaper, and more accessible, solution. This thesis investigates the application of a modified Monte Carlo Localization technique to successfully localize within a simulated indoor environment. By counting predicted locations within a 1-meter radius from the original location, we evaluate the efficacy of our algorithm and optimize its parameters. We find that our sensorless approach, given environments with realistic levels of noise, is able to successfully localize. Moreover, the localization algorithm can operate on a small number of particles, and performs with greater accuracy with knowledge of the starting position.

Paper: https://ir.library.oregonstate.edu/concern/honors_college_theses/n296x673z

Running scripts:
    Example:
    To generate map and path data:
        To use this code you need an existing ROS world and map setup with the ROS navigation stack.

        While recording a rosbag on /odom, /map, and /map_metadata, move your robot via a goal pose/poses. Name and save that bag in an appropriately named folder in /data.

        Run "python3 map.py" while simulation is open. Map parameters and obstacle csv will be generated. Crtl + C after "done" output. Move outputs to previous folder in /data.

    python overlay.py <particle_amount> <current_point visualization> <output_name> <start_variance> <path_variance> Tweak parameters as neccesary.


 - *Check "Display Options" and Params to configure trials as desired for either class.
