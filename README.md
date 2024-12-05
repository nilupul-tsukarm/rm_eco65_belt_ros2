check network connection of arm and PC
check belt arduino USB connection >>>> '/dev/ttyACM0' check gripper USB connection >>>>>>> '/dev/ttyUSB0'

jupyter-notebook script : /home/ubuntu/rm_eco65_ros2_test2.ipynb

source /opt/ros/humble/setup.bash

ros2 launch rm_driver rm_eco65_driver.launch.py

ros2 topic list # if needed ros2 topic echo /rm_driver/udp_arm_position # for get pos XYZ and rotation XYZW if needed ros2 topic echo /joint_states # for get joint values if needed

ros2 launch belt_speed_publisher belt_speed.launch.py ros2 topic echo /belt_speed # if needed
