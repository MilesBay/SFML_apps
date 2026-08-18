find_package(Threads REQUIRED)

# all source files
set(SRC RopeSimulation.cpp Physics.cpp)

# define the island target
sfml_add_example(physics_exr_4 GUI_APP
                 SOURCES ${SRC}
                 DEPENDS SFML::Graphics)

# external dependency headers
target_include_directories(physics_exr_4 SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/examples/physics_exr_4_spring)

target_link_libraries(physics_exr_4 PRIVATE Threads::Threads)
