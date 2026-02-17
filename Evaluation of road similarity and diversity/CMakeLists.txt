cmake_minimum_required(VERSION 3.20)

project(Evaluation_Of_Road_Similarity C)
set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)

include(FetchContent)

FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.0
)

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(raylib)

add_executable(rank
    src/rank.c
    lib/cJSON.c
)

add_executable(diversity
    src/diversity.c
    lib/cJSON.c
)

target_link_libraries(rank PRIVATE raylib)
target_link_libraries(diversity PRIVATE raylib)
