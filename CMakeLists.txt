project( ros_queue_to_disk )
cmake_minimum_required(VERSION 2.8)

set(BOOST_LIBS thread)
find_package( Boost COMPONENTS ${BOOST_LIBS} REQUIRED)

add_library(${PROJECT_NAME} INTERFACE )
target_include_directories(${PROJECT_NAME} INTERFACE include ${Boost_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -lstdc++fs")

#
#find_package(catkin REQUIRED
#  COMPONENTS
#    roscpp
#    )
#
#set( _INCLUDE_DIRS
#            include
#            ${catkin_INCLUDE_DIRS}
#       )
#
#set( _LIBRARIES
#               ${catkin_LIBRARIES}
#        )
#
#set( ${PROJECT_NAME}_INCLUDE_DIRS ${_INCLUDE_DIRS}  CACHE INTERNAL "" FORCE )
#set( ${PROJECT_NAME}_LIBRARIES  ${_LIBRARIES} CACHE INTERNAL "" FORCE )
#
#include_directories( ${INCLUDE_DIRS} )
#
##add_library( ${PROJECT_NAME}_LIBRARIES SHARED ${SRCS} ${HDRS} )
#target_link_libraries( ros_queue_to_disk ${_LIBRARIES} )
#
#-lstdc++fs 