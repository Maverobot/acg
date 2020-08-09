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

add_subdirectory(imgui_impl)

add_library(box2d_draw draw.cpp settings.cpp)
target_include_directories(box2d_draw PUBLIC ${box2d-extern_SOURCE_DIR}/include
                                             ./)
target_link_libraries(box2d_draw PUBLIC box2d imgui_impl)
