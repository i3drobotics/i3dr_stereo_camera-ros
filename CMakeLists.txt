cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 11)
project(i3dr_stereo_camera)

find_package(catkin REQUIRED COMPONENTS
                roscpp std_msgs tf rospy dynamic_reconfigure image_geometry message_generation 
                camera_info_manager image_transport stereo_msgs cv_bridge
            )

find_package(Boost REQUIRED COMPONENTS thread system)

# build with i3dr algorithm: 'catkin_make -DENABLE_I3DR_ALG=ON'
# build without i3dr algorithm: 'catkin_make -DENABLE_I3DR_ALG=OFF'
option(ENABLE_I3DR_ALG "Use I3DR Matching algorithm" OFF)
if (ENABLE_I3DR_ALG)
	#Add FindPhobos.cmake directory to CMAKE_MODULE_PATH
	set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
	find_package(Phobos REQUIRED)
	add_definitions(-DENABLE_I3DR_ALG)
else (ENABLE_I3DR_ALG)
	find_package(OpenCV REQUIRED)
endif()

find_package(PCL 1.2 REQUIRED)

include_directories(
	include
	${Phobos_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_service_files(
	FILES
    SetFloat.srv
	SetInt.srv
	SaveStereo.srv
	SaveRectified.srv
	#ConfigLaser.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
 
generate_dynamic_reconfigure_options(
	cfg/i3DR_Disparity.cfg
	cfg/tiscamera_settings.cfg
)

catkin_package( 
    INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp nodelet std_msgs sensor_msgs message_runtime
	LIBRARIES ${PROJECT_NAME}
#  DEPENDS system_lib
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if (PHOBOS_FOUND)
	add_executable(
		generate_disparity src/generate_disparity.cpp src/matcherJrsgm.cpp src/iniReader.cpp
	)
else (PHOBOS_FOUND)
	message(WARNING "I3DR matching library not found. Will not be able to generate disparity with I3DR matching algorithm (2)")
	add_executable(
		generate_disparity src/generate_disparity.cpp
	)
endif()

target_link_libraries(
	generate_disparity
	${Phobos_LIBRARIES}
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

add_executable(
	generate_point_cloud src/generate_point_cloud.cpp
)
target_link_libraries(
	generate_point_cloud
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(
	rectify src/rectify.cpp
)
target_link_libraries(
	rectify
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

add_executable(
	disparity_to_depth src/disparity_to_depth.cpp
)
target_link_libraries(
	disparity_to_depth
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

add_dependencies(generate_disparity ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(generate_disparity ${PROJECT_NAME}_gencfg)