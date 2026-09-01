find_package(Threads REQUIRED)

# all source files
set(SRC main.cpp BezierCurve.cpp)

# define the island target
sfml_add_example(physics_exr_6 GUI_APP
                 SOURCES ${SRC}
                 DEPENDS SFML::Graphics)

# external dependency headers
target_include_directories(physics_exr_6 SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/examples/physics_exr_6_bezier)

target_link_libraries(physics_exr_6 PRIVATE Threads::Threads)
