cmake_minimum_required(VERSION 2.8.3)
project(wp_con)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  bebop_msgs
  actionlib
  actionlib_msgs
  cv_bridge
  image_transport
  sensor_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

find_package(OpenCV REQUIRED)

generate_messages(
   DEPENDENCIES
#   bebop_msgs#   geometry_msgs#   nav_msgs#   std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}	
)

add_executable(Keyboard1 src/key_con.cpp)
target_link_libraries(Keyboard1 ${catkin_LIBRARIES})


add_executable(C_p6 src/c_p6.cpp)
target_link_libraries(C_p6 ${catkin_LIBRARIES})
add_dependencies(C_p6 C_p6_generate_messages_cpp)

add_executable(c_vy src/c_vy.cpp)
target_link_libraries(c_vy ${catkin_LIBRARIES})
add_dependencies(c_vy c_vy_generate_messages_cpp)

add_executable(c_vaz src/c_vaz.cpp)
target_link_libraries(c_vaz ${catkin_LIBRARIES})
add_dependencies(c_vaz c_vaz_generate_messages_cpp)

add_executable(vi_wp src/vi_waypoints.cpp)
target_link_libraries(vi_wp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
