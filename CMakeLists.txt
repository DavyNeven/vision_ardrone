cmake_minimum_required(VERSION 2.8.3)
project(vision_ardrone)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
roscpp 
ardrone_autonomy 
std_msgs 
image_transport 
sensor_msgs 
cv_bridge
message_generation
)

# Compile third party libs
include(ExternalProject)
ExternalProject_Add(thirdpartylib
	URL ${PROJECT_SOURCE_DIR}/thirdparty/thirdparty.tar.gz
	PREFIX ${CMAKE_BINARY_DIR}/thirdparty
	CONFIGURE_COMMAND ""
	BUILD_COMMAND make
	INSTALL_COMMAND ""
	BUILD_IN_SOURCE 1
)


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ControlCommand.msg
   Detection.msg
   Status.msg
   Identity.msg
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
CATKIN_DEPENDS roscpp std_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
${catkin_INCLUDE_DIRS}
)

include(/usr/dlib-18.16/dlib/cmake)

# set required libs and headers
include_directories(
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/libcvd/include
)

link_directories(
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/libcvd/lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations

## Declare a cpp library
#add_library(dlib
 #  /usr/dlib-18.10/dlib/all/source.cpp
 #)

## Declare a cpp executable

add_executable(ardrone_controller src/Controller/Controller.cpp src/tools/PDcontroller.cpp src/tools/PDcontroller.h)
target_link_libraries(ardrone_controller ${catkin_LIBRARIES})
set_target_properties(ardrone_controller PROPERTIES COMPILE_FLAGS "-D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3")
add_dependencies(ardrone_controller ${PROJECT_NAME}_gencpp)

add_executable(ardrone_ub_detector src/Detectors/UB_detector.cpp src/tools/LKT_Tracker.cpp src/tools/LKT_Tracker.h src/tools/ObjectDetector.cpp src/tools/ObjectDetector.h)
target_link_libraries(ardrone_ub_detector ${catkin_LIBRARIES})
set_target_properties(ardrone_ub_detector PROPERTIES COMPILE_FLAGS "-D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3")
add_dependencies(ardrone_ub_detector ${PROJECT_NAME}_gencpp)


add_executable(ardrone_face_detector src/Detectors/FACE_detector.cpp src/tools/LKT_Tracker.cpp src/tools/LKT_Tracker.h)
target_link_libraries(ardrone_face_detector ${catkin_LIBRARIES} dlib)
set_target_properties(ardrone_face_detector PROPERTIES COMPILE_FLAGS "-D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3")
add_dependencies(ardrone_face_detector ${PROJECT_NAME}_gencpp)

add_executable(ardrone_recognizer src/Recognizer/Face_Recognizer.cpp src/Recognizer/Face_Recognizer.h src/Recognizer/Recognizer.cpp src/tools/PreprocessFace.cpp src/tools/PreprocessFace.h)
target_link_libraries(ardrone_recognizer ${catkin_LIBRARIES} dlib)
set_target_properties(ardrone_recognizer PROPERTIES COMPILE_FLAGS "-D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3")
add_dependencies(ardrone_recognizer ${PROJECT_NAME}_gencpp)







#------------------GUI-----------------#
set(GUI_SOURCE_FILES
    src/GUI/main_GUI.cpp
    src/GUI/mainwindow.cpp
    src/GUI/RosThread.cpp
)

set(GUI_HEADER_FILES
    src/GUI/mainwindow.h
    src/GUI/ui_mainwindow.h
    src/GUI/RosThread.h
)

set(GUI_UI_FILES
    src/GUI/mainwindow.ui
)

# *.qrc
set(GUI_RESOURCE_FILES
)

# do QT stuff
ADD_DEFINITIONS( -Wall )
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
QT4_ADD_RESOURCES(GUI_RESOURCE_FILES_CPP ${GUI_RESOURCE_FILES})
QT4_WRAP_UI(GUI_UI_FILES_HPP ${GUI_UI_FILES})
QT4_WRAP_CPP(GUI_HEADER_FILES_HPP ${GUI_HEADER_FILES})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# build!
add_executable(ardrone_gui ${GUI_SOURCE_FILES} ${GUI_RESOURCE_FILES_CPP} ${GUI_UI_FILES_HPP} ${GUI_HEADER_FILES_HPP})
target_link_libraries(ardrone_gui ${QT_LIBRARIES} cvd ${catkin_LIBRARIES})
add_dependencies(ardrone_gui thirdpartylib ${PROJECT_NAME}_gencpp)







