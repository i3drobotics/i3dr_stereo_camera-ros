cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)
project(i3dr_stereo_camera)

find_package(catkin REQUIRED COMPONENTS
                roscpp std_msgs tf rospy dynamic_reconfigure image_geometry message_generation 
                camera_info_manager image_transport stereo_msgs cv_bridge
            )

find_package(Boost REQUIRED COMPONENTS thread system)

option(WITH_QT "Build QT Application" OFF)
if(WITH_QT)
	set(CMAKE_AUTOMOC ON)
	set(CMAKE_AUTOUIC ON)
	set(CMAKE_AUTORCC ON)
	set(CMAKE_INCLUDE_CURRENT_DIR ON)
	find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
	find_package(VTK 6 REQUIRED)
	include(${VTK_USE_FILE})
endif()

# Build with cuda: 'catkin_make -DWITH_CUDA=ON'
# Enables OpenCV CUDA Stereo matchers
# ROS package: 'vision_opencv' must be re-compiled with cuda enabled opencv library
# Install OpenCV 3.3.1 with CUDA enabled (-DWITH_CUDA) to /usr/local
# Source OpenCV install location after sourcing ros (add this to ~/.bashrc):
	# source /opt/ros/kinetic/setup.bash
	# export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
	# export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
option(WITH_CUDA "Use OpenCV CUDA Libraries" OFF)
if (WITH_CUDA)
	add_definitions(-DWITH_CUDA)
	set(OpenCV_DIR ${CMAKE_CURRENT_BINARY_DIR}/opencv3)
endif()

# build with i3dr algorithm: 'catkin_make -DWITH_I3DRSGM=ON'
option(WITH_I3DRSGM "Use I3DR Matching algorithm" OFF)
if (WITH_I3DRSGM)
	set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
	find_package(Phobos REQUIRED)
	add_definitions(-DWITH_I3DRSGM)
	if (NOT PHOBOS_FOUND)
		message(FATAL_ERROR "Failed to load Phobos library for I3DRSGM.")
	endif()
else (WITH_I3DRSGM)
	find_package(OpenCV 3)
	if (NOT OpenCV_FOUND)
		find_package(OpenCV 4 REQUIRED)
	endif()
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
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
	cfg/i3DR_Disparity.cfg
	cfg/i3DR_pointCloud.cfg
	cfg/tiscamera_settings.cfg
)

catkin_package( 
    INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp nodelet std_msgs sensor_msgs message_runtime
	LIBRARIES ${PROJECT_NAME}
)

link_directories(
	${PCL_LIBRARY_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/bin
)

add_definitions(${PCL_DEFINITIONS})

if (WITH_I3DRSGM)
	add_executable(
		init_stereo_matchers src/init_stereo_matchers.cpp 
		src/stereoMatcher/abstractStereoMatcher.cpp 
		src/stereoMatcher/I3DRSGM.cpp 
		src/stereoMatcher/matcherI3DRSGM.cpp 
	)
	target_link_libraries(
		init_stereo_matchers
		${Phobos_LIBRARIES}
		${catkin_LIBRARIES}
		${OpenCV_LIBRARIES}
		cudaMem
	)
	if (WITH_CUDA)
		add_executable(
			generate_disparity src/generate_disparity.cpp 
			src/stereoMatcher/abstractStereoMatcher.cpp 
			src/stereoMatcher/matcherOpenCVBlock.cpp 
			src/stereoMatcher/matcherOpenCVSGBM.cpp 
			src/stereoMatcher/I3DRSGM.cpp 
			src/stereoMatcher/matcherI3DRSGM.cpp 
			src/stereoMatcher/matcherOpenCVBlockCuda.cpp
			src/stereoMatcher/matcherOpenCVBPCuda.cpp
			src/stereoMatcher/matcherOpenCVCSBPCuda.cpp
		)
		target_link_libraries(
			generate_disparity
			${Phobos_LIBRARIES}
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
			${PCL_LIBRARIES}
			cudaMem
		)
	else (WITH_CUDA)
		message(WARNING "Not using CUDA. Will not be able to generate disparity with StereoBMCuda (3)")
		add_executable(
			generate_disparity src/generate_disparity.cpp 
			src/stereoMatcher/abstractStereoMatcher.cpp 
			src/stereoMatcher/matcherOpenCVBlock.cpp 
			src/stereoMatcher/matcherOpenCVSGBM.cpp 
			src/stereoMatcher/I3DRSGM.cpp 
			src/stereoMatcher/matcherI3DRSGM.cpp 
		)
		target_link_libraries(
			generate_disparity
			${Phobos_LIBRARIES}
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
			${PCL_LIBRARIES}
			cudaMem
		)	
	endif()
else (WITH_I3DRSGM)
	message(WARNING "I3DR matching library not found. Will not be able to generate disparity with I3DR matching algorithm (2)")
	add_executable(
		init_stereo_matchers src/init_stereo_matchers.cpp 
		src/stereoMatcher/abstractStereoMatcher.cpp 
		src/stereoMatcher/matcherI3DRSGM.cpp 
	)
	target_link_libraries(
		init_stereo_matchers
		${catkin_LIBRARIES}
		${OpenCV_LIBRARIES}
		cudaMem
	)
	if (WITH_CUDA)
		add_executable(
			generate_disparity src/generate_disparity.cpp 
			src/stereoMatcher/abstractStereoMatcher.cpp 
			src/stereoMatcher/matcherOpenCVBlock.cpp 
			src/stereoMatcher/matcherOpenCVSGBM.cpp
			src/stereoMatcher/matcherOpenCVBlockCuda.cpp
			src/stereoMatcher/matcherOpenCVBPCuda.cpp
			src/stereoMatcher/matcherOpenCVCSBPCuda.cpp
		)
		target_link_libraries(
			generate_disparity
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
			${PCL_LIBRARIES}
		)
	else (WITH_CUDA)
		message(WARNING "Not using CUDA. Will not be able to generate disparity with StereoBMCuda (3)")
		add_executable(
			generate_disparity src/generate_disparity.cpp 
			src/stereoMatcher/abstractStereoMatcher.cpp 
			src/stereoMatcher/matcherOpenCVBlock.cpp 
			src/stereoMatcher/matcherOpenCVSGBM.cpp
		)
		target_link_libraries(
			generate_disparity
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
			${PCL_LIBRARIES}
		)	
	endif()
endif()

if(WITH_QT)
	add_executable(
		stereo_gui src/stereo_gui.cpp
		include/StereoGUI/StereoGUI.h
		src/StereoGUI/StereoGUI.cpp
		src/StereoGUI/StereoGUI.ui
		#src/StereoGUI/resources.qrc
	)
	target_link_libraries(
		stereo_gui
		${catkin_LIBRARIES}
		${OpenCV_LIBRARIES}
		${PCL_LIBRARIES}
		${QT_LIBRARIES}
		${VTK_LIBRARIES}
	)
endif()

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
	${PCL_LIBRARIES}
)

add_executable(
	crop_image_by_disparity src/crop_image_by_disparity.cpp
)
target_link_libraries(
	crop_image_by_disparity
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

add_dependencies(generate_disparity ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(rectify ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(init_stereo_matchers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(generate_disparity ${PROJECT_NAME}_gencfg)
