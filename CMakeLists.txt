cmake_minimum_required(VERSION 3.16)
project(ball_capture LANGUAGES CXX)

option(BALL_BUILD_CAPTURE "Build the camera and video capture tool" ON)
option(BALL_BUILD_APRILTAG_PNP "Build the AprilTag pose tool" OFF)
option(BALL_WITH_HIKROBOT "Build with the migrated Hikrobot USB3 camera driver" OFF)

add_library(ball_collect STATIC tasks/ball_collect/state_machine.cpp)
target_compile_features(ball_collect PUBLIC cxx_std_17)
target_include_directories(ball_collect PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(ball_collector src/collector_main.cpp)
target_link_libraries(ball_collector PRIVATE ball_collect)

if(NOT BALL_BUILD_CAPTURE AND NOT BALL_BUILD_APRILTAG_PNP)
  return()
endif()

find_package(OpenCV 4 REQUIRED COMPONENTS core imgcodecs videoio)
if(BALL_BUILD_CAPTURE)
  add_executable(ball_capture src/main.cpp tools/exiter.cpp)
  target_compile_features(ball_capture PRIVATE cxx_std_17)
  target_include_directories(ball_capture PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(ball_capture PRIVATE ${OpenCV_LIBS})
  list(APPEND camera_tools ball_capture)
endif()

if(BALL_BUILD_APRILTAG_PNP)
  if(OpenCV_VERSION VERSION_LESS "4.7")
    set(aruco_component aruco)
  else()
    set(aruco_component objdetect)
  endif()
  find_package(OpenCV 4 REQUIRED COMPONENTS core imgcodecs videoio calib3d ${aruco_component})
  add_executable(apriltag_pnp tools/apriltag_pnp.cpp tools/exiter.cpp)
  target_compile_features(apriltag_pnp PRIVATE cxx_std_17)
  target_include_directories(apriltag_pnp PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(apriltag_pnp PRIVATE ${OpenCV_LIBS})
  list(APPEND camera_tools apriltag_pnp)
endif()

if(BALL_WITH_HIKROBOT)
  if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR
     NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    message(FATAL_ERROR "The migrated MVS SDK libraries require Linux aarch64")
  endif()

  find_package(fmt REQUIRED)
  find_package(spdlog REQUIRED)
  find_package(Threads REQUIRED)

  set(MVS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/io/hikrobot")
  set(MVS_LIB_DIR "${MVS_DIR}/lib/arm64")
  foreach(camera_tool IN LISTS camera_tools)
    target_sources(${camera_tool} PRIVATE
      src/camera_base.cpp
      io/hikrobot/hikrobot.cpp
      io/hikrobot/hik_camera_manager.cpp
      tools/logger.cpp
    )
    target_include_directories(${camera_tool} PRIVATE "${MVS_DIR}/include")
    target_compile_definitions(${camera_tool} PRIVATE BALL_WITH_HIKROBOT=1)
    target_link_libraries(${camera_tool} PRIVATE
      "${MVS_LIB_DIR}/libMvCameraControl.so"
      fmt::fmt spdlog::spdlog Threads::Threads ${CMAKE_DL_LIBS}
    )
    # Preserve the original build's RPATH so SDK dependencies can find sibling libraries.
    target_link_options(${camera_tool} PRIVATE -Wl,--disable-new-dtags)
    set_target_properties(${camera_tool} PROPERTIES BUILD_RPATH "${MVS_LIB_DIR}")
  endforeach()
endif()
