add_executable(test_check_pose_server test_check_pose_server.cpp)
target_link_libraries(test_check_pose_server PRIVATE ${PROJECT_NAME})
add_dependencies(test_check_pose_server ${PROJECT_NAME}_gencpp)

add_executable(test_plan_server test_plan_server.cpp)
target_link_libraries(test_plan_server PRIVATE ${PROJECT_NAME})
add_dependencies(test_plan_server ${PROJECT_NAME}_gencpp)

add_executable(test_back_to_home_server test_back2home_server.cpp)
target_link_libraries(test_back_to_home_server PRIVATE ${PROJECT_NAME})
add_dependencies(test_back_to_home_server ${PROJECT_NAME}_gencpp)

add_executable(test_good_cmd_pose test_good_cmd_pose.cpp)
target_link_libraries(test_good_cmd_pose PRIVATE ${PROJECT_NAME})
add_dependencies(test_good_cmd_pose ${PROJECT_NAME}_gencpp)

add_executable(test_moveit_planner test_moveit_planner.cpp)
target_link_libraries(test_moveit_planner PRIVATE ${PROJECT_NAME})
add_dependencies(test_moveit_planner ${PROJECT_NAME}_gencpp)

add_executable(test_z1_movej_arm test_z1_movej_arm)
target_link_libraries(test_z1_movej_arm PRIVATE ${Z1SDK_LIB})
target_include_directories(test_z1_movej_arm PRIVATE ${Z1SDK_INCLUDE_DIR})
