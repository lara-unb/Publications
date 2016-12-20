cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(transmitter)

#########################################################
# FINDING QIBUILD
#########################################################
find_package(qibuild)


#########################################################
# INCLUDE DIRECTORIES
#########################################################
include_directories(
${CMAKE_CURRENT_SOURCE_DIR}/include 
)


#########################################################
# SOURCES
#########################################################
set(_srcs
  #src/comms_robot.cpp
  src/TransmitterRobot.cpp
  
)

#set(CMAKE_BUILD_TYPE OPTIMIZED)
set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -msse -msse2 -msse3 -mmmx -DDEBUG ${ARCH_FLAGS}")
set(CMAKE_CXX_FLAGS_OPTIMIZED "-Wall -msse -msse2 -msse3 -O3 -mmmx -DDEBUG ${ARCH_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -msse -msse2 -msse3 -O3 -mmmx -DNDEBUG -DRELEASE ${ARCH_FLAGS}")


qi_create_bin(${PROJECT_NAME} main.cpp ${_srcs})
qi_use_lib(${PROJECT_NAME} rt ALCOMMON ALPROXIES BOOST)
