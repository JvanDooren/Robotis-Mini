cmake_minimum_required(VERSION 3.10)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set name
SET(TARGET tput)

# add sources
SET(SOURCES src/main.cc)

# add the executable
add_executable(${TARGET} ${SOURCES})
