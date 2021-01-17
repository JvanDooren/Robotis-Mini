cmake_minimum_required(VERSION 3.10)

include(ExternalProject)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set name
set(TARGET bt210-throughput)

# add sources
SET(SOURCES src/main.cc)

# add the executable
add_executable(${TARGET} ${SOURCES})

set(LIB_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include)
target_include_directories(${TARGET} PUBLIC ${LIB_INCLUDE_DIR})

ExternalProject_Add(consoletable_ext
    GIT_REPOSITORY https://github.com/DenisSamilton/CppConsoleTable.git
    CONFIGURE_COMMAND ""
	BUILD_COMMAND
		COMMAND ${CMAKE_COMMAND} -E copy_directory <SOURCE_DIR>
				${LIB_INCLUDE_DIR}/CppConsoleTable
	INSTALL_COMMAND ""
	UPDATE_COMMAND ""
)

add_dependencies(${TARGET} consoletable_ext)
