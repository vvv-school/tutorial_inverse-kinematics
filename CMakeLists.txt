# requires minimum cmake version
cmake_minimum_required(VERSION 3.0)

# enforce C++11 features
set(CMAKE_CXX_STANDARD 11)

# produce the cmake var PROJECT_NAME
project(tutorial_inverse-kinematics)

# mandatory use of these packages
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

find_package(OpenCV REQUIRED)

# helpers defining certain macros (e.g. "yarp_install")
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

# import math symbols from standard cmath
add_definitions(-D_USE_MATH_DEFINES)

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ctrlLib_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# robot
add_executable(${PROJECT_NAME}-robot ${CMAKE_SOURCE_DIR}/src/robot.cpp)
target_link_libraries(${PROJECT_NAME}-robot ${YARP_LIBRARIES} ctrlLib ${OpenCV_LIBRARIES})
install(TARGETS ${PROJECT_NAME}-robot DESTINATION bin)

# controller
add_executable(${PROJECT_NAME}-controller ${CMAKE_SOURCE_DIR}/src/controller.cpp)
target_link_libraries(${PROJECT_NAME}-controller ${YARP_LIBRARIES} ctrlLib)
install(TARGETS ${PROJECT_NAME}-controller DESTINATION bin)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

