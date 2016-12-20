cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(ChallengeNoWifi)

#########################################################
# FINDING QIBUILD
#########################################################
find_package(qibuild)


#########################################################
# INCLUDE DIRECTORIES
#########################################################
get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} PATH)

include_directories(
${CMAKE_CURRENT_SOURCE_DIR}/include
${PARENT_DIR}/motion/runswift
${PARENT_DIR}/common
)


#########################################################
# SOURCES
#########################################################
set(_srcs
    src/CommRobot.cpp
    src/ReceiverRobot.cpp
    src/TransmitterRobot.cpp
    ${PARENT_DIR}/common/ledAuxiliarFunctions.cpp
    ${PARENT_DIR}/common/motionAuxiliarFunctions.cpp
    ${PARENT_DIR}/common/motionData.cpp
    ${PARENT_DIR}/common/Conversions.cpp
    ${PARENT_DIR}/common/motionNAOqiFunction.cpp
)

#set(CMAKE_BUILD_TYPE OPTIMIZED)
set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -msse -msse2 -msse3 -mmmx -DDEBUG ${ARCH_FLAGS}")
set(CMAKE_CXX_FLAGS_OPTIMIZED "-Wall -msse -msse2 -msse3 -O3 -mmmx -DDEBUG ${ARCH_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -msse -msse2 -msse3 -O3 -mmmx -DNDEBUG -DRELEASE ${ARCH_FLAGS}")

qi_create_bin(${PROJECT_NAME} main.cpp ${_srcs})
qi_use_lib(${PROJECT_NAME} rt ALCOMMON ALPROXIES BOOST ALVISION OPENCV2_CORE OPENCV2_HIGHGUI)

