find_package(Threads REQUIRED)

# all source files
set(SRC TriangleCutterMain.cpp)

# define the island target
sfml_add_example(physics_exr_1 GUI_APP
                 SOURCES ${SRC}
                 DEPENDS SFML::Graphics)

# external dependency headers
target_include_directories(physics_exr_1 SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/examples/physics_exr_1_triangle_cutter)

target_link_libraries(physics_exr_1 PRIVATE Threads::Threads)
