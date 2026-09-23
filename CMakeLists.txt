cmake_minimum_required(VERSION 3.0.2)
project(amiga_can)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_python_setup()
catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/amiga_twist_to_can.py
    scripts/amiga_can_sniffer.py
    scripts/amiga_ros_teleop.py
    scripts/amiga_joy_teleop.py
    scripts/amiga_keyboard_teleop.py
    scripts/amiga_joy_teleop_evdev.py
    
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch" PATTERN "*.yaml"
)
