# CMakeList.txt : CMake project for PI-DE-test, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Specify GCC compilers for CMake on Windows
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)


project ("PI-DE-test")


# Add source to this project's executable.
add_executable (PI-DE-test "PI-DE-test.cpp" "PI-DE-test.h" "Control.cpp" "Control.h" "Data_reader.cpp" "Global.cpp" "Global.h" "Nav.cpp" "Nav.h")

# Link the gpiod library
target_link_libraries(PI-DE-test gpiod)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET PI-DE-test PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
