cmake_minimum_required(VERSION 3.0)
project( pylon_opencv )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Pylon 
list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} )
find_package(Pylon)
include_directories(${Pylon_INCLUDE_DIRS})

add_executable( main main.cpp
includes/robot.cpp includes/robot.h
includes/gripper.cpp includes/gripper.h
includes/cornerPoints.h includes/cornerPoints.cpp
includes/undistortion.h includes/undistortion.cpp
includes/drawCoordinates.cpp includes/drawCoordinates.h
includes/homograf.cpp includes/homograf.h
includes/readVar.cpp includes/readVar.h
includes/calibrationPhotos.cpp includes/calibrationPhotos.h
includes/cameraCalibration.cpp includes/cameraCalibration.h
includes/objectDetection.cpp includes/objectDetection.h
includes/data.cpp includes/data.h
includes/createFile.cpp includes/createFile.h 
includes/CalcVelocity.cpp includes/CalcVelocity.h
includes/RobotThrow.cpp includes/RobotThrow.h
includes/throwVector.cpp includes/throwVector.h
includes/jointVelocity.cpp includes/jointVelocity.h
includes/angle.cpp includes/angle.h
)
find_package(ur_rtde REQUIRED)
target_link_libraries( main ${OpenCV_LIBS} ${Pylon_LIBRARIES} ur_rtde::rtde)