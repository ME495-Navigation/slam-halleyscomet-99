cmake_minimum_required(VERSION 3.22)

project(turtlelib)

# --- Compiler Settings ---
# Enable C++23 support as required by the assignment
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Prevent in-source builds to keep the repository clean
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "No in-source builds allowed. Please create a separate build directory.")
endif()

# --- Library Target ---
# Create the turtlelib library from geometry and SE(2) implementation files
add_library(turtlelib 
    src/geometry2d.cpp
    src/se2d.cpp
    src/svg.cpp
)

# Set include directories for the library
# Use BUILD_INTERFACE for local compilation and INSTALL_INTERFACE for exported use
target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Apply mandatory warning flags and conformance settings
target_compile_options(turtlelib PUBLIC -Wall -Wextra -Wpedantic)

# --- Executable Target ---
# Create the converter utility program
add_executable(converter src/converter.cpp)

# Link the converter against the turtlelib library
target_link_libraries(converter turtlelib)

add_executable(frame_main src/frame_main.cpp)
target_link_libraries(frame_main turtlelib)

install(TARGETS frame_main
        DESTINATION bin)

# --- Doxygen Documentation ---
# Find Doxygen and provide an option to build documentation
find_package(Doxygen)
option(BUILD_DOCS "Build the documentation" OFF)

if(DOXYGEN_FOUND AND BUILD_DOCS)
    # Configure Doxygen to use README.md as the landing page
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)
    # Generate documentation from include headers, source files, and README
    doxygen_add_docs(doxygen include/ src/ README.md ALL)
endif()

# --- Installation Rules ---

# Define installation for the library and the executable
install(TARGETS turtlelib converter
    EXPORT turtlelib-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install the headers into the system include directory
install(DIRECTORY include/turtlelib DESTINATION include)

# Export the targets so they can be found via find_package(turtlelib)
# This allows other packages to use turtlelib::turtlelib
install(EXPORT turtlelib-targets
    FILE turtlelib-config.cmake
    NAMESPACE turtlelib::
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
endif()