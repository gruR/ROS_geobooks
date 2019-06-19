cmake_minimum_required(VERSION 2.8.3)
project(knu_ros_lecture)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs

message_generation
cv_bridge
image_transport
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder


# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
add_service_files(
FILES
srvKnuRosLecture.srv
)


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


add_message_files(
FILES
knuRosLecture.msg
)
generate_messages(
DEPENDENCIES
std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
find_package(OpenCV REQUIRED)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES knu_ros_lecture
  CATKIN_DEPENDS roscpp std_msgs cv_bridge
  DEPENDS system_lib OpenCV
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/knu_ros_lecture.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable

#add_executable(talker src/talker.cpp)
#add_executable(listener src/listener.cpp)

#add_executable(turtlesim_random_move src/turtlesim_random_move.cpp)
#add_executable(gazebo_random_move src/gazebo_random_move.cpp)
#add_executable(turtle_odom_subscriber src/turtle_odom_subscriber.cpp)
#add_executable(turtlesim_pose src/turtlesim_pose.cpp)

#add_executable(user_msg_publisher_tutorial src/user_msg_publisher_tutorial.cpp)
#add_executable(user_msg_subscriber_tutorial src/user_msg_subscriber_tutorial.cpp)

#add_executable(server_srv_tutorial src/server_srv_tutorial.cpp)
#add_executable(client_srv_tutorial src/client_srv_tutorial.cpp)

#add_executable(server_srv_tutorial_param src/server_srv_tutorial_param.cpp)
#add_executable(client_srv_tutorial_param src/client_srv_tutorial_param.cpp)

#add_executable(turtle_position_move src/turtle_position_move.cpp)
#add_executable(opencv_intro src/opencv_intro.cpp)
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/knu_ros_lecture_node.cpp)
#add_executable(hello_world_node src/hello_world.cpp)
#add_executable(turtle_position_lrf_view src/turtlebot3_pose_lidar_viewer.cpp)
#add_executable(turtle_position_lrf_view src/turtlebot3_pose_lidar_viewer_register.cpp)
#add_executable(opencv_intro2 src/Turtlebot3_lidar_viewer.cpp)

#add_executable(hw2 src/hw2.cpp)
#add_executable(inout_test src/inout_test.cpp)
#add_executable(turtle_rpiCamera_detection_sample src/turtle_rpiCamera_detection_sample.cpp)




#add_executable(myopencv src/myopencv.cpp)
add_executable(myopencv_realimg src/myopencv_realimg.cpp)
#add_executable(follower src/follower.py)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
#add_dependencies(talker knu_ros_lecture_generate_messages_cpp)
#add_dependencies(listener knu_ros_lecture_generate_messages_cpp)

#add_dependencies(turtle_odom_subscriber knu_ros_lecture_generate_messages_cpp)
#add_dependencies(gazebo_random_move knu_ros_lecture_generate_messages_cpp)
#add_dependencies(turtlesim_random_move knu_ros_lecture_generate_messages_cpp)
#add_dependencies(turtlesim_pose knu_ros_lecture_generate_messages_cpp)

#add_dependencies(user_msg_publisher_tutorial knu_ros_lecture_generate_messages_cpp)
#add_dependencies(user_msg_subscriber_tutorial knu_ros_lecture_generate_messages_cpp)

#add_dependencies(server_srv_tutorial knu_ros_lecture_generate_messages_cpp)
#add_dependencies(client_srv_tutorial knu_ros_lecture_generate_messages_cpp)

#add_dependencies(server_srv_tutorial_param knu_ros_lecture_generate_messages_cpp)
#add_dependencies(client_srv_tutorial_param knu_ros_lecture_generate_messages_cpp)
#add_dependencies(opencv_intro knu_ros_lecture_generate_messages_cpp)
#add_dependencies(turtle_position_lrf_view knu_ros_lecture_generate_messages_cpp)
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(hello_world_node knu_ros_lecture_generate_messages_cpp)
#add_dependencies(turtle_position_lrf_view knu_ros_lecture_generate_messages_cpp)

#add_dependencies(turtle_position_move knu_ros_lecture_generate_messages_cpp)
#add_dependencies(opencv_intro2 knu_ros_lecture_generate_messages_cpp)

#add_dependencies(hw2 knu_ros_lecture_generate_messages_cpp)
#add_dependencies(inout_test knu_ros_lecture_generate_messages_cpp)
#add_dependencies(turtle_rpiCamera_detection_sample knu_ros_lecture_generate_messages_cpp)


#add_dependencies(myopencv myopencv_cpp)
add_dependencies(myopencv_realimg myopencv_realimg_cpp)

#add_dependencies(follower follower_py)
## Specify libraries to link a library or executable target against
#target_link_libraries(talker ${catkin_LIBRARIES})
#target_link_libraries(listener ${catkin_LIBRARIES})

#target_link_libraries(turtle_odom_subscriber ${catkin_LIBRARIES} )
#target_link_libraries(gazebo_random_move ${catkin_LIBRARIES} )
#target_link_libraries(turtlesim_random_move ${catkin_LIBRARIES} )
#target_link_libraries(turtlesim_pose ${catkin_LIBRARIES} )

#target_link_libraries(user_msg_publisher_tutorial ${catkin_LIBRARIES} )
#target_link_libraries(user_msg_subscriber_tutorial ${catkin_LIBRARIES} )

#target_link_libraries(server_srv_tutorial ${catkin_LIBRARIES} )
#target_link_libraries(client_srv_tutorial ${catkin_LIBRARIES} )

#target_link_libraries(server_srv_tutorial_param ${catkin_LIBRARIES} )
#target_link_libraries(client_srv_tutorial_param ${catkin_LIBRARIES} )
#target_link_libraries(opencv_intro ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#target_link_libraries(hello_world_node ${catkin_LIBRARIES})
#target_link_libraries(turtle_position_lrf_view ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
#target_link_libraries(turtle_position_lrf_view ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

#target_link_libraries(turtle_position_move ${catkin_LIBRARIES} )
#target_link_libraries(opencv_intro2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

#target_link_libraries(hw2 ${catkin_LIBRARIES} )
#target_link_libraries(inout_test ${catkin_LIBRARIES})
#target_link_libraries(turtle_rpiCamera_detection_sample ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#target_link_libraries(myopencv ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(myopencv_realimg ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#target_link_libraries(follower ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_knu_ros_lecture.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
