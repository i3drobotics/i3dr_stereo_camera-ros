cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 11)
project(i3dr_stereo_camera)

find_package(catkin REQUIRED COMPONENTS
                roscpp std_msgs tf rospy dynamic_reconfigure message_generation 
                camera_info_manager image_transport stereo_msgs cv_bridge
            )

find_package(Boost REQUIRED COMPONENTS thread system)

find_package(OpenCV REQUIRED)

find_package(PCL 1.2 REQUIRED)

include_directories(
	include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_service_files(
    FILES
    SetFloat.srv
    SetInt.srv
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
	CATKIN_DEPENDS roscpp nodelet std_msgs sensor_msgs
	LIBRARIES ${PROJECT_NAME}
#  DEPENDS system_lib
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(
	generate_disparity src/generate_disparity.cpp src/matcherJrsgm.cpp src/iniReader.cpp
)

target_link_libraries(
	generate_disparity
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

add_dependencies(generate_disparity ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(generate_disparity ${PROJECT_NAME}_gencfg)