add_library(imgui_impl imgui_impl_glfw.cpp imgui_impl_opengl3.cpp)
target_include_directories(imgui_impl PUBLIC ./)
target_link_libraries(imgui_impl PUBLIC imgui glad glfw)
