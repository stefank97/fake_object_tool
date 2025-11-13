cmake_minimum_required(VERSION 3.10)
project(fake_object_tool)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

#Für QT MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
qt5_wrap_cpp(MOC_FILES
  include/fake_object_tool/rectangle_tool.hpp
)

include_directories(include)

add_library(fake_object_tool SHARED
  src/rectangle_tool.cpp
  ${MOC_FILES}
)
target_include_directories(fake_object_tool PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fake_object_tool
  rviz_common
  rviz_rendering
  rclcpp
  geometry_msgs
  pluginlib
)

target_link_libraries(fake_object_tool Qt5::Widgets)
# RViz-Plugin beim ament index registrieren
pluginlib_export_plugin_description_file(rviz_common rviz_plugin.xml)

install(TARGETS fake_object_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES rviz_plugin.xml DESTINATION share/${PROJECT_NAME})


ament_package()