cmake_minimum_required(VERSION 2.8.3)
project(trajectory_tracker_rviz_plugins)

find_package(catkin REQUIRED
  COMPONENTS
    pluginlib
    rviz
    trajectory_tracker_msgs
)
catkin_package(
  CATKIN_DEPENDS
    pluginlib
    rviz
    trajectory_tracker_msgs
)

include_directories(${catkin_INCLUDE_DIRS} include)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})

  qt4_wrap_cpp(MOC_FILES
    include/trajectory_tracker_rviz_plugins/path_with_velocity_display.h
  )
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)

  qt5_wrap_cpp(MOC_FILES
    include/trajectory_tracker_rviz_plugins/path_with_velocity_display.h
  )
endif()

find_file(HAVE_VALIDATE_QUATERNION_H
  NAMES rviz/validate_quaternions.h
  HINTS ${catkin_INCLUDE_DIRS}
)
if(HAVE_VALIDATE_QUATERNION_H)
  add_definitions(-DHAVE_VALIDATE_QUATERNION_H)
endif()

add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME}
  src/path_with_velocity_display.cpp
  ${MOC_FILES}
)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
