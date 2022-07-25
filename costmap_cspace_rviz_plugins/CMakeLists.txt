cmake_minimum_required(VERSION 3.1.3)
project(costmap_cspace_rviz_plugins)

set(PACKAGE_DEPENDS
  roscpp
  rviz
  costmap_cspace_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDS})

catkin_package(CATKIN_DEPENDS ${PACKAGE_DEPENDS})

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})

  qt4_wrap_cpp(MOC_FILES
    include/costmap_cspace_rviz_plugins/c_space_3d_display.h
  )
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)

  qt5_wrap_cpp(MOC_FILES
    include/costmap_cspace_rviz_plugins/c_space_3d_display.h
  )
endif()

add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME}
  src/c_space_3d_display.cpp
  ${MOC_FILES}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)
  set(ROSLINT_CPP_OPTS "--filter=-runtime/references,-build/c++11")
  roslint_cpp()
  roslint_add_test()
endif()

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
