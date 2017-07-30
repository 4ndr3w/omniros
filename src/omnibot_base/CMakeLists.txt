cmake_minimum_required(VERSION 2.8.3)
project(omnibot_base)
find_package(catkin REQUIRED roscpp std_msgs tf)
set(CMAKE_CXX_FLAGS -std=c++11)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(omnibot_differential_base src/omnibot_differential_base.cpp src/fletcher.cpp)
target_link_libraries(omnibot_differential_base ${catkin_LIBRARIES} atomic pthread)
