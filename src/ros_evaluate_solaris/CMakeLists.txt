cmake_minimum_required(VERSION 3.8)
project(ros_evaluate_solaris)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw_fastrtps_cpp REQUIRED)

set(RMW_IMPLEMENTATION "rmw_fastrtps_cpp")
add_definitions(-DRMW_IMPLEMENTATION=${RMW_IMPLEMENTATION})

set(node_plugins "")

include_directories(include)

set(executable_pub pub_node)
set(executable_sub sub_node)
set(executable_self self_node)
set(executable_intra intra_nodes)
set(executable_loanp loan_pub)
set(executable_stat  stat_sub)

set(dependencies
  rclcpp
)

add_executable(${executable_pub}
  src/pub_node.cpp
)

add_executable(${executable_sub}
  src/sub_node.cpp
)

add_executable(${executable_self}
  src/self_node.cpp
)

add_executable(${executable_intra}
  src/intra_nodes.cpp
)

add_executable(${executable_loanp}
  src/loan_pub.cpp
)

add_executable(${executable_stat}
  src/sub_statistics.cpp
)

ament_target_dependencies(${executable_pub}
  ${dependencies}
)

ament_target_dependencies(${executable_sub}
  ${dependencies}
)

ament_target_dependencies(${executable_self}
  ${dependencies}
)

ament_target_dependencies(${executable_intra}
  ${dependencies}
)

ament_target_dependencies(${executable_loanp}
  ${dependencies}
)

ament_target_dependencies(${executable_stat}
  ${dependencies}
)

add_library(pub_node_component SHARED
  src/pub_node_component.cpp)
target_compile_definitions(pub_node_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(pub_node_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(pub_node_component "solaris_test::PubNodeComponent")
set(node_plugins "${node_plugins}solaris_test::PubNodeComponent;$<TARGET_FILE:PubNodeComponent>\n")

add_library(sub_node_component SHARED
  src/sub_node_component.cpp)
target_compile_definitions(sub_node_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(sub_node_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(sub_node_component "solaris_test::SubNodeComponent")
set(node_plugins "${node_plugins}solaris_test::SubNodeComponent;$<TARGET_FILE:SubNodeComponent>\n")

install(TARGETS
  pub_node_component
  sub_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  TARGETS ${executable_pub}
  TARGETS ${executable_sub}
  TARGETS ${executable_self}
  TARGETS ${executable_intra}
  TARGETS ${executable_loanp}
  TARGETS ${executable_stat}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
