cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(comms_robot)

find_package(qibuild)

include_directories(
        "."
	"../../include"
)

#set(CMAKE_BUILD_TYPE OPTIMIZED)
set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -msse -msse2 -msse3 -mmmx -DDEBUG ${ARCH_FLAGS}")
set(CMAKE_CXX_FLAGS_OPTIMIZED "-Wall -msse -msse2 -msse3 -O3 -mmmx -DDEBUG ${ARCH_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -msse -msse2 -msse3 -O3 -mmmx -DNDEBUG -DRELEASE ${ARCH_FLAGS}")

set(TX_SRCS
  comms_robot.cpp
  )

add_executable(comms_robot ${TX_SRCS})

TARGET_LINK_LIBRARIES(comms_robot rt)

