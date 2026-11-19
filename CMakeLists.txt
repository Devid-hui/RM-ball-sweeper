cmake_minimum_required(VERSION 3.16)
project(ball_capture LANGUAGES CXX)

option(BALL_BUILD_CAPTURE "Build the camera and video capture tool" ON)
option(BALL_WITH_HIKROBOT "Build with the migrated Hikrobot USB3 camera driver" OFF)

add_library(ball_collect STATIC tasks/ball_collect/state_machine.cpp)
target_compile_features(ball_collect PUBLIC cxx_std_17)
target_include_directories(ball_collect PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(ball_collector src/collector_main.cpp)
target_link_libraries(ball_collector PRIVATE ball_collect)

if(NOT BALL_BUILD_CAPTURE)
  return()
endif()

find_package(OpenCV 4 REQUIRED COMPONENTS core imgcodecs videoio)
add_executable(ball_capture src/main.cpp tools/exiter.cpp)
target_compile_features(ball_capture PRIVATE cxx_std_17)
target_include_directories(ball_capture PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ball_capture PRIVATE ${OpenCV_LIBS})

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
  target_sources(ball_capture PRIVATE
    src/camera_base.cpp
    io/hikrobot/hikrobot.cpp
    io/hikrobot/hik_camera_manager.cpp
    tools/logger.cpp
  )
  target_include_directories(ball_capture PRIVATE "${MVS_DIR}/include")
  target_compile_definitions(ball_capture PRIVATE BALL_WITH_HIKROBOT=1)
  target_link_libraries(ball_capture PRIVATE
    "${MVS_LIB_DIR}/libMvCameraControl.so"
    fmt::fmt spdlog::spdlog Threads::Threads ${CMAKE_DL_LIBS}
  )
  # Preserve the original build's RPATH so SDK dependencies can find sibling libraries.
  target_link_options(ball_capture PRIVATE -Wl,--disable-new-dtags)
  set_target_properties(ball_capture PROPERTIES BUILD_RPATH "${MVS_LIB_DIR}")
endif()
