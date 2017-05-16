cmake_minimum_required(VERSION 3.1)

project(libomnibot)
set(CMAKE_CXX_STANDARD 11)

add_library(omnibot OmniBot.cpp OmniBotStateReceiver.cpp OmniBotCommandSender.cpp)

add_executable(omnibot_console OmniBotConsole.cpp)
add_executable(omnibot_reboot OmniBotReboot.cpp)
add_executable(omnibot_enable OmniBotEnabler.cpp)

target_link_libraries(omnibot_reboot z)
target_link_libraries(omnibot_enable z)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
