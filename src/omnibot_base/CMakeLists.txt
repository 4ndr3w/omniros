cmake_minimum_required(VERSION 2.8.3)
project(omnibot_base)
find_package(catkin REQUIRED roscpp std_msgs tf)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/libomnibot)


add_executable(omnibot_differential_base src/omnibot_differential_base.cpp)
target_link_libraries(omnibot_differential_base ${catkin_LIBRARIES} omnibot atomic pthread)
