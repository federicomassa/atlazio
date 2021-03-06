##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.0.2)
project(atlazio)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp 
  sensor_msgs
  geometry_msgs
  rosbag
  nav_msgs)
#find_package(QCustomPlot REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)

find_package(Boost REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(QCustomPlot_PATH . CACHE STRING "Path to qcustomplot.cpp and .h")

set(atlazio_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/atlazio)

# This is necessary to find ui generated headers in build folder

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

set(SOURCES src/atlazio.cpp src/main_window.cpp src/ros_monitor.cpp src/ros_node.cpp 
src/bag_reader.cpp
src/topic.cpp
src/topiccombobox.cpp
${QCustomPlot_PATH}/qcustomplot.cpp
)

# All headers that contatins Qt classes. Needed to force moc (don't know why automoc is not working on them)
set(HEADERS_MOC include/atlazio/main_window.h include/atlazio/ros_monitor.h
  include/atlazio/ros_node.h
  include/atlazio/bag_reader.h
  include/atlazio/topiccombobox.h
)

set(RESOURCES resources/images.qrc)

##############################################################################
# Binaries
##############################################################################

add_executable(atlazio ${SOURCES} ${HEADERS_MOC} ${RESOURCES})
target_link_libraries(atlazio 
Qt5::Widgets
Qt5::Core
Qt5::PrintSupport
${catkin_LIBRARIES} 
#${QCustomPlot_LIBRARIES}
)

target_include_directories(atlazio PRIVATE ${QCustomPlot_PATH})
target_include_directories(atlazio PRIVATE ${atlazio_INCLUDE_DIRS})
install(TARGETS atlazio RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})