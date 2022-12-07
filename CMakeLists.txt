cmake_minimum_required(VERSION 3.16.3)
project(fpsCount)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(SYSTEM_ARCHITECTURE x64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(SYSTEM_ARCHITECTURE arm64)
endif()

link_directories(lib/${SYSTEM_ARCHITECTURE})
set(GIGE_CAM libMVSDK.so)



add_executable(demo_aruco src/demo_aruco.cpp)
target_link_libraries(demo_aruco ${GIGE_CAM} ${OpenCV_LIBRARIES})


# add_executable(aruco_gige_cam_calibration src/aruco_gige_cam_calibration.cpp)
# target_link_libraries(aruco_gige_cam_calibration ${GIGE_CAM} ${OpenCV_LIBRARIES})





# add_executable(charuco_gige_cam_calibration src/charuco_gige_cam_calibration.cpp)
# target_link_libraries(charuco_gige_cam_calibration ${GIGE_CAM} ${OpenCV_LIBRARIES})

