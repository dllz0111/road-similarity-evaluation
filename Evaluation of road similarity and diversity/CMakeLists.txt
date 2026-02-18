cmake_minimum_required(VERSION 3.20)
project(road C)

set(CMAKE_C_STANDARD 23)

include(FetchContent)


FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG 5.5
)

FetchContent_MakeAvailable(raylib)


add_executable(rank
        src/rank.c
        lib/cJSON.c
)

target_include_directories(rank PRIVATE
        lib
)

target_link_libraries(rank PRIVATE raylib)


add_executable(diversity
        src/diversity.c
        lib/cJSON.c
)

target_include_directories(diversity PRIVATE
        lib
)

target_link_libraries(diversity PRIVATE raylib)
