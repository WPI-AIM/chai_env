#  Software License Agreement (BSD License)
#  Copyright (c) 2019, AMBF.
#  (www.aimlab.wpi.edu)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of authors nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: Adnan Munawar $
#  $Date:  $
#  $Rev: $

cmake_minimum_required(VERSION 2.8.3)
project(chai_env)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
geometry_msgs
chai_msgs
tf)

catkin_python_setup()

catkin_package( INCLUDE_DIRS include
  LIBRARIES chai_env chai_obj chai_world
  CATKIN_DEPENDS chai_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include ${catkin_INCLUDE_DIRS})

set (ENV_HDR_FILES include/${PROJECT_NAME}/chai_env.h
                   include/${PROJECT_NAME}/Object.h
                   include/${PROJECT_NAME}/RosComBase.h
                   include/${PROJECT_NAME}/ObjectRosCom.h
	                 include/${PROJECT_NAME}/CmdWatchDog.h)

set (ENV_SRC_FILES src/chai_env.cpp
                   src/Object.cpp
                   src/ObjectRosCom.cpp)

set (OBJ_HDR_FILES include/${PROJECT_NAME}/Object.h
                   include/${PROJECT_NAME}/RosComBase.h
                   include/${PROJECT_NAME}/ObjectRosCom.h
	                 include/${PROJECT_NAME}/CmdWatchDog.h)

set (OBJ_SRC_FILES src/Object.cpp src/ObjectRosCom.cpp)

set (WORLD_HDR_FILES include/${PROJECT_NAME}/World.h
                     include/${PROJECT_NAME}/RosComBase.h
                     include/${PROJECT_NAME}/WorldRosCom.h
                     include/${PROJECT_NAME}/CmdWatchDog.h  )

set (WORLD_SRC_FILES src/World.cpp src/WorldRosCom.cpp)

## Declare a C++ library
add_library(chai_env ${ENV_HDR_FILES} ${ENV_SRC_FILES})
add_library(chai_obj ${OBJ_HDR_FILES} ${OBJ_SRC_FILES})
add_library(chai_world ${WORLD_HDR_FILES} ${WORLD_SRC_FILES})

add_dependencies(chai_env ${catkin_EXPORTED_TARGETS})
add_dependencies(chai_obj ${catkin_EXPORTED_TARGETS})
add_dependencies(chai_world ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(chai_env ${catkin_LIBRARIES})
target_link_libraries(chai_obj ${catkin_LIBRARIES})
target_link_libraries(chai_world ${catkin_LIBRARIES})

set (CHAI_ENV_INCLUDE_DIRS include)
set (CHAI_ENV_LIBRARIES chai_env chai_obj chai_world)
