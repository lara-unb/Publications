cmake_minimum_required(VERSION 2.8)
project(motion)

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
${CMAKE_CURRENT_SOURCE_DIR}/runswift
${PARENT_DIR}/common
)

#########################################################
# INCLUDE FILES
#########################################################
set(SOURCES 
	"src/${PROJECT_NAME}.cpp"
    src/naoqi_module_loader.cpp
    runswift/motion/MotionAdapter.cpp
    runswift/motion/MotionOdometry.cpp
    runswift/motion/SonarRecorder.cpp
    runswift/motion/effector/AgentEffector.cpp
    runswift/motion/generator/ActionGenerator.cpp
    runswift/motion/generator/BodyModel.cpp
    runswift/motion/generator/ClippedGenerator.cpp
    runswift/motion/generator/DeadGenerator.cpp
    runswift/motion/generator/DistributedGenerator.cpp
    runswift/motion/generator/HeadGenerator.cpp
    runswift/motion/generator/NullGenerator.cpp
    runswift/motion/generator/PendulumModel.cpp
    runswift/motion/generator/RefPickupGenerator.cpp
    runswift/motion/generator/StandGenerator.cpp
    runswift/motion/generator/Walk2014Generator.cpp
    runswift/motion/generator/WalkCycle.cpp
    runswift/motion/generator/WalkEnginePreProcessor.cpp
    runswift/motion/touch/AgentTouch.cpp
    runswift/motion/touch/FeetState.cpp
    runswift/motion/touch/FilteredTouch.cpp
    runswift/motion/touch/NullTouch.cpp
    runswift/motion/touch/SensorOutput.cpp
    runswift/motion/touch/TorsoStateFilter.cpp
    runswift/perception/kinematics/Kinematics.cpp
    runswift/perception/kinematics/Parameters.cpp
    runswift/perception/kinematics/Pose.cpp
    runswift/perception/kinematics/Kinematics.cpp
    runswift/perception/kinematics/SonarFilter.cpp
    runswift/perception/vision/Camera.cpp
    runswift/types/XYZ_Coord.hpp
    runswift/gamecontroller/RoboCupGameControlData.cpp
    runswift/utils/options.cpp
    runswift/utils/Timer.hpp
    ../common/motionData.cpp
)

set(LIBAGENT_SOURCES
    runswift/libagent/libagent.cpp
    runswift/utils/options.cpp
    runswift/utils/Timer.hpp
)

set(CLIENT_SOURCES
    runswift/perception/kinematics/SonarFilter.cpp
    ../common/motionData.cpp
)

# The naoqi module
qi_create_lib(${PROJECT_NAME} SHARED ${SOURCES} SUBFOLDER naoqi)
qi_use_lib(${PROJECT_NAME} ALCOMMON BOOST)
qi_stage_lib(${PROJECT_NAME})

#########################################################
# ADD CLIENT
#########################################################

# Create an Executable that can call the motionCapture Module (locally or remotely)
qi_create_bin("${PROJECT_NAME}Client" "${PROJECT_NAME}Client.cpp" ${CLIENT_SOURCES})
qi_use_lib("${PROJECT_NAME}Client" ALCOMMON ALPROXIES BOOST)


#########################################################
# LIBAGENT
#########################################################
qi_create_lib(agent SHARED ${LIBAGENT_SOURCES} SUBFOLDER naoqi)
qi_use_lib(agent ALCOMMON BOOST ALPROXIES)
qi_stage_lib(agent)

#########################################################
# OTHER THINGS
#########################################################
#Disable BOOST Useless warning
add_definitions(-DBOOST_SIGNALS_NO_DEPRECATION_WARNING)
