############# these commands are for python scripts ###########

check network connection of arm and PC
check belt arduino USB connection >>>> '/dev/ttyACM0' check gripper USB connection >>>>>>> '/dev/ttyUSB0'

### wake up smart scape demo

terminal 1 : 
ros2 launch rm_driver rm_eco65_driver.launch.py
or 
ros2 launch rm_bringup rm_eco65_bringup.launch.py

terminal 2 :
ros2 launch belt_speed_publisher belt_speed.launch.py

terminal 3 :
ros2 launch dh_gripper dh_gripper.launch.py

terminal 4 : 
python3 ~/rm_eco65_belt_ros2/smart_scape_main.py

### for get data :
ros2 topic echo /joint_states #arm joints

ros2 topic echo /gripper_joint_states # gripper state 

ros2 topic echo /belt_speed 

### service calls

ros2 service call /set_gripper_position std_srvs/srv/SetBool "{data: true}"  # Open gripper

ros2 service call /set_gripper_position std_srvs/srv/SetBool "{data: false}" # Close gripper

ros2 service call /go_to_initial_position std_srvs/srv/Empty

ros2 service call /go_up_from_mid_belt_position std_srvs/srv/Empty

ros2 service call /go_to_mid_belt_position std_srvs/srv/Empty

ros2 service call /go_to_cup_A_position std_srvs/srv/Empty

ros2 service call /go_to_cup_B_position std_srvs/srv/Empty

###################### these for jupitor scripts ##############
check network connection of arm and PC
check belt arduino USB connection >>>> '/dev/ttyACM0' check gripper USB connection >>>>>>> '/dev/ttyUSB0'

jupyter-notebook script : /home/ubuntu/rm_eco65_ros2_test2.ipynb

source /opt/ros/humble/setup.bash

ros2 launch rm_driver rm_eco65_driver.launch.py

ros2 topic list # if needed ros2 topic echo /rm_driver/udp_arm_position # for get pos XYZ and rotation XYZW if needed ros2 topic echo /joint_states # for get joint values if needed

ros2 launch belt_speed_publisher belt_speed.launch.py ros2 topic echo /belt_speed # if needed
