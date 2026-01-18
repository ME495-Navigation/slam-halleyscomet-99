cmake_minimum_required(VERSION 3.22)

project(turtlelib)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "No in source builds allowed.")
endif()

# --- Library Target ---
add_library(turtlelib src/geometry2d.cpp)

target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(turtlelib PUBLIC -Wall -Wextra -Wpedantic)

# --- Executable Target ---
add_executable(converter src/converter.cpp)

target_link_libraries(converter turtlelib)

# --- Doxygen Documentation ---
find_package(Doxygen)
option(BUILD_DOCS "Build the documentation" OFF)
if(DOXYGEN_FOUND AND BUILD_DOCS)
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)
    # Include headers, sources, and README in the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)
endif()

# --- Installation ---

# Install Library and Executable
install(TARGETS turtlelib converter
    EXPORT turtlelib-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install Header Files
install(DIRECTORY include/turtlelib DESTINATION include)

# Export Targets for use by other CMake projects (as turtlelib::turtlelib)
install(EXPORT turtlelib-targets
    FILE turtlelib-config.cmake
    NAMESPACE turtlelib::
    DESTINATION lib/cmake/turtlelib
)