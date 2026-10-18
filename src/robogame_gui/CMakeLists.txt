cmake_minimum_required(VERSION 3.16)
project(robogame_gui)

# moc the .hh header too.
cmake_policy(SET CMP0100 NEW)

find_package(ament_cmake REQUIRED)
find_package(gz_gui_vendor REQUIRED)
find_package(gz-gui REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Quick Qml)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)

# gz-gui finds a plugin by its library name, libRobogameView.so, and its
# QML at :/RobogameView/RobogameView.qml.
add_library(RobogameView SHARED
  src/RobogameView.cc
  src/RobogameView.hh
  src/RobogameView.qrc
)
target_link_libraries(RobogameView
  gz-gui::gz-gui
  gz-msgs::gz-msgs
  gz-plugin::register
  gz-transport::gz-transport
  Qt6::Core Qt6::Quick Qt6::Qml
)

# sim.launch.py adds this to GZ_GUI_PLUGIN_PATH.
install(TARGETS RobogameView LIBRARY DESTINATION lib/${PROJECT_NAME})

ament_package()
