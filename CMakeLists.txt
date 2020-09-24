cmake_minimum_required(VERSION 3.11)

add_executable(arm arm.cpp)
target_link_libraries(arm rofiHal)

