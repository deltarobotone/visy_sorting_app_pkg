cmake_minimum_required(VERSION 2.8.3)
project(visy_sorting_app_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation one_easy_protocol_pkg visy_neopixel_pkg visy_detector_pkg std_msgs rospy)

catkin_python_setup()

add_message_files(FILES GraspData.msg)
add_service_files(DIRECTORY srv FILES StartSorting.srv StopSorting.srv StartGraspPlanner.srv StopGraspPlanner.srv PickAndPlace.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS one_easy_protocol_pkg visy_neopixel_pkg visy_detector_pkg std_msgs rospy)

catkin_install_python(PROGRAMS
    scripts/${PROJECT_NAME}/sorting_app_node.py
    scripts/${PROJECT_NAME}/grasp_planner_node.py
    scripts/${PROJECT_NAME}/pick_and_place_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
