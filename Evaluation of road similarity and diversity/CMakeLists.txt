cmake_minimum_required(VERSION 3.20) 

project(Evaluation_Of_Road_Similarity C) 
set(CMAKE_C_STANDARD 23) 

add_subdirectory(lib/raylib/raylib-master) 
add_executable(rank src/rank.c lib/cJSON.c) 
add_executable(diversity src/diversity.c lib/cJSON.c) 
target_link_libraries(rank raylib)