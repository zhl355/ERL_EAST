CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(erl_astar)


################################################################################
# user build settings
option(BUILD_PYTHON "Build Python Bindings" OFF)
option(BUILD_MATLAB "Build MEX Bindings" OFF)
################################################################################


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")


# Find dependencies
INCLUDE_DIRECTORIES(include)


# Eigen3
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})


# Boost
FIND_PACKAGE(Boost REQUIRED COMPONENTS system filesystem)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# YAML-CPP
FIND_PACKAGE(PkgConfig REQUIRED)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)

##########################################################################################################
FIND_PACKAGE(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(erl_astar
  src/astar_nx.cpp)
target_link_libraries(erl_astar ${Boost_LIBRARIES})


##########################################################################################################
cs_install()
cs_export()
##########################################################################################################

