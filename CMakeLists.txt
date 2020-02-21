project( ros_queue_to_disk )
cmake_minimum_required(VERSION 2.8)

set(WITH_DEBUG_TEST_IMAGE 0)
if(WITH_DEBUG_TEST_IMAGE)
  add_definitions(-DDEBUG_TEST_IMAGE)
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    message_generation
    )


add_service_files(
  DIRECTORY
  srv
  FILES
  TestInt.srv
  TestImage.srv
)

generate_messages(
  DEPENDENCIES
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include/
  CATKIN_DEPENDS message_runtime 
)

set( _INCLUDE_DIRS
            ${catkin_INCLUDE_DIRS}
            include/
      )

set( _LIBRARIES
            ${catkin_LIBRARIES}
      )


include_directories( ${_INCLUDE_DIRS} )
link_libraries(stdc++fs)


# test_int
if(NOT WITH_DEBUG_TEST_IMAGE)
  
add_executable(test_int_server tests/test_int_server.cpp)
target_link_libraries( test_int_server ${_LIBRARIES} )
set_target_properties( test_int_server PROPERTIES LINKER_LANGUAGE CXX)

add_executable(test_int_client tests/test_int_client.cpp)
target_link_libraries( test_int_client ${_LIBRARIES} )
set_target_properties( test_int_client PROPERTIES LINKER_LANGUAGE CXX)

endif()

# test_image
add_executable(test_image_server tests/test_image_server.cpp)
target_link_libraries( test_image_server ${_LIBRARIES} )
set_target_properties( test_image_server PROPERTIES LINKER_LANGUAGE CXX)

add_executable(test_image_client tests/test_image_client.cpp)
target_link_libraries( test_image_client ${_LIBRARIES} ${OpenCV_LIBS} )
set_target_properties( test_image_client PROPERTIES LINKER_LANGUAGE CXX)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)