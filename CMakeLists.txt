cmake_minimum_required(VERSION 3.0.2)
project(rviz_plugin_urdf_composer)

## Option to enable or disable compiling of SRV files only
option(COMPILE_ONLY_SRV "Compile only services (SRV files)" ON)

## Find catkin macros and libraries
if(NOT COMPILE_ONLY_SRV)
  find_package(catkin REQUIRED COMPONENTS
    std_srvs
    geometry_msgs
    message_generation
    std_msgs
    pluginlib
    tf2_geometry_msgs
    rviz
    robot_state_publisher
  )
else()
  find_package(catkin REQUIRED COMPONENTS
    std_srvs
    message_generation
    std_msgs
  )
endif()

## System dependencies are found with CMake's conventions
set(OGRE_INCLUDE_DIRS /usr/include/OGRE)
set(OGRE_LIBRARIES /usr/lib/libOgreMain.so)

if(NOT COMPILE_ONLY_SRV)
  ## This setting causes Qt's "MOC" generation to happen automatically.
  set(CMAKE_AUTOMOC ON)

  ## Include Qt based on rviz version
  if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    include(${QT_USE_FILE})
  else()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    set(QT_LIBRARIES Qt5::Widgets)
  endif()

  ## Define the plugin description and resources
  add_definitions(-DQT_NO_KEYWORDS)
endif()

################################################
## Declare ROS messages and services
################################################
add_service_files(
  FILES
  ManageModules.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

if(NOT COMPILE_ONLY_SRV)
  ################################################
  ## catkin specific configuration
  ################################################
  catkin_package(
    LIBRARIES rviz_plugin_urdf_composer
    CATKIN_DEPENDS message_runtime
    DEPENDS OGRE
  )

  ###########
  ## Build ##
  ###########
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
  )

  ## Declare C++ libraries
  add_library(${PROJECT_NAME}_lib2 src/tools.cpp)
  target_link_libraries(${PROJECT_NAME}_lib2 ${catkin_LIBRARIES})

  set(SRC_FILES
    src/urdf_composer.cpp
    src/urdf_vis_update.cpp
  )

  add_library(${PROJECT_NAME} ${SRC_FILES})
  target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME}_lib2)

  ## Declare executables
  add_executable(module_selection src/reconcycle_module_selection.cpp)
  target_link_libraries(module_selection ${catkin_LIBRARIES} ${PROJECT_NAME}_lib2)

  #############
  ## Install ##
  #############
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

  install(DIRECTORY media/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media
  )

  install(DIRECTORY icons/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons
  )
else()
  ################################################
  ## If only SRV is compiled, still include the necessary files for the service
  ################################################
  catkin_package(
    CATKIN_DEPENDS message_runtime
  )
endif()

#############
## Testing ##
#############
# Uncomment if you want to add gtest-based tests
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rviz_plugin_urdf_composer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
