cmake_minimum_required(VERSION 3.11)
project(acg)

add_subdirectory(3rdParty/box2d_draw)
add_subdirectory(3rdParty/NamedType)

add_library(acg INTERFACE)
target_include_directories(acg INTERFACE include)
target_link_libraries(acg INTERFACE box2d_draw NamedType)
target_compile_features(acg INTERFACE cxx_std_17)

# Examples
macro(add_example name)
  add_executable(${name} examples/${name}.cpp)
  target_link_libraries(${name} PRIVATE acg)
endmacro()

add_example(falling_circle_with_wind)
add_example(inverted_pendulum)
