add_library(box2d_draw draw.cpp settings.cpp)
target_include_directories(box2d_draw PUBLIC ${box2d-extern_SOURCE_DIR}/include
                                             ./)
target_link_libraries(box2d_draw PUBLIC box2d imgui_impl)
