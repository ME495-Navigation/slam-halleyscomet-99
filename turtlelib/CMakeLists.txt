cmake_minimum_required(VERSION 3.22)

project(turtlelib)

# --- Compiler Settings ---
# Enable C++23 support as per modern robotics standards
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Prevent in-source builds to keep the repository clean
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "No in-source builds allowed. Please create a separate build directory.")
endif()

# --- Dependencies ---
# Eigen3 is required for EKF matrix math. We find it here to use it during build.
find_package(Eigen3 REQUIRED)

# --- Library Target ---
# Create the turtlelib library from implementation files
add_library(turtlelib 
    src/geometry2d.cpp
    src/se2d.cpp
    src/svg.cpp
    src/diff_drive.cpp
    src/ekf.cpp
)

# Set include directories for the library
# PUBLIC link ensures that packages using turtlelib see these headers
target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link against Eigen3
# Using PUBLIC ensures that consumers (like nusim/nuslam) inherit Eigen3 headers/flags
target_link_libraries(turtlelib PUBLIC Eigen3::Eigen)

# Apply mandatory warning flags and conformance settings
target_compile_options(turtlelib PUBLIC -Wall -Wextra -Wpedantic)

# --- Executable Targets ---
# Standalone utilities linked against the library
add_executable(converter src/converter.cpp)

# Link the converter against the turtlelib library
target_link_libraries(converter turtlelib)

add_executable(frame_main src/frame_main.cpp)
target_link_libraries(frame_main turtlelib)

# --- Doxygen Documentation ---
# Find Doxygen and provide an option to build documentation
find_package(Doxygen)
option(BUILD_DOCS "Build the documentation" OFF)

if(DOXYGEN_FOUND AND BUILD_DOCS)
    # Configure Doxygen to use README.md as the landing page
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)
    # Use MathJax support
    set(DOXYGEN_USE_MATHJAX YES)
    # Generate documentation from include headers, source files, and README
    doxygen_add_docs(doxygen include/ src/ README.md ALL)
endif()

# --- Installation and Export Rules ---

# 1. Define which targets to install and assign them to an export set named 'turtlelib-targets'
install(TARGETS turtlelib converter frame_main
    EXPORT turtlelib-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# 2. Install header files
install(DIRECTORY include/turtlelib DESTINATION include)

# 3. Generate and install the 'targets' file. 
# This file contains the actual library locations and requirements (like Eigen3::Eigen).
install(EXPORT turtlelib-targets
    FILE turtlelib-targets.cmake
    NAMESPACE turtlelib::
    DESTINATION lib/cmake/turtlelib
)

# 4. Create a custom Config file.
# This is the file find_package(turtlelib) actually looks for.
# It MUST find Eigen3 first so that Eigen3::Eigen is defined before loading our targets.
include(CMakePackageConfigHelpers)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/turtlelib-config.cmake"
"include(CMakeFindDependencyMacro)
# Ensure Eigen3 is found by any package that finds turtlelib
find_dependency(Eigen3 REQUIRED)
# Include the exported targets generated in step 3
include(\"\${CMAKE_CURRENT_LIST_DIR}/turtlelib-targets.cmake\")
")

# 5. Install the generated config file
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/turtlelib-config.cmake"
    DESTINATION lib/cmake/turtlelib
)

# --- Unit Testing ---
include(CTest)
if(BUILD_TESTING)
    # Find Catch2 version 3
    find_package(Catch2 3 REQUIRED)

    # Test Target for Geometry2D primitives
    add_executable(test_geometry2d tests/test_geometry2d.cpp)
    target_link_libraries(test_geometry2d turtlelib Catch2::Catch2WithMain)
    add_test(NAME geometry2d_tests COMMAND test_geometry2d)

    # Test Target for SE(2) transformations
    add_executable(test_se2d tests/test_se2d.cpp)
    target_link_libraries(test_se2d turtlelib Catch2::Catch2WithMain)
    add_test(NAME se2d_tests COMMAND test_se2d)

    # Test Target for SVG
    add_executable(test_svg tests/test_svg.cpp)
    target_link_libraries(test_svg turtlelib Catch2::Catch2WithMain)
    add_test(NAME svg_tests COMMAND test_svg)

    # Test Target for DiffDrive
    add_executable(test_diff_drive tests/test_diff_drive.cpp)
    target_link_libraries(test_diff_drive turtlelib Catch2::Catch2WithMain)
    add_test(NAME diff_drive_tests COMMAND test_diff_drive)
endif()
