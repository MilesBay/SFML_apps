find_package(Threads REQUIRED)

# all source files
set(SRC EulerIntegration.cpp Physics.cpp)

# define the island target
sfml_add_example(physics_exr_2 GUI_APP
                 SOURCES ${SRC}
                 DEPENDS SFML::Graphics)

# external dependency headers
target_include_directories(physics_exr_2 SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/examples/physics_exr_2_euler_integration)

target_link_libraries(physics_exr_2 PRIVATE Threads::Threads)
