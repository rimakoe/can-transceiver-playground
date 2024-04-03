cmake_minimum_required(VERSION 3.8)
project(can_transciever)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(STANDARD_CPP_LIB_DIR /usr/local/lib)
set(STANDARD_CPP_INCLUDE_DIR /usr/local/include)

include_directories(include ${STANDARD_CPP_INCLUDE_DIR}/can-transciever-lib)
add_executable(${PROJECT_NAME} src/ros2can_transciever.cpp)
target_link_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES} ${STANDARD_CPP_LIB_DIR}/can-transciever-lib/libcan-transciever-lib.so)
ament_target_dependencies(${PROJECT_NAME} rclcpp)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
