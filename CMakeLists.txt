cmake_minimum_required(VERSION 3.11)
project(acg)

include(FetchContent)
FetchContent_Declare(box2d-extern
                     GIT_REPOSITORY https://github.com/erincatto/box2d.git)
FetchContent_GetProperties(box2d-extern)
if(NOT box2d-extern_POPULATED)
  FetchContent_Populate(box2d-extern)
  message(STATUS ${box2d-extern_SOURCE_DIR})
  message(STATUS ${box2d-extern_BINARY_DIR})
  add_subdirectory(${box2d-extern_SOURCE_DIR} ${box2d-extern_BINARY_DIR})
endif()

add_subdirectory(utils/imgui_impl)
add_subdirectory(utils/box2d_draw)

#
add_library(acg INTERFACE)
target_include_directories(acg INTERFACE include)
target_link_libraries(acg INTERFACE box2d_draw)

# Examples
add_executable(falling_circle_with_wind examples/falling_circle_with_wind.cpp)
target_link_libraries(falling_circle_with_wind PRIVATE acg)
