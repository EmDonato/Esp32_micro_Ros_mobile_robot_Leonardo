cmake_minimum_required(VERSION 3.8)
project(leonardo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# >>> AGGIUNTA: trova anche le dipendenze per messaggi personalizzati
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)  # per usare Header
find_package(rosidl_default_generators REQUIRED)  # >>> AGGIUNTA

# >>> AGGIUNTA: genera il messaggio custom
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SimpleOdometry.msg"
  DEPENDENCIES std_msgs
)

# 1) Dichiarazione dell'eseguibile
add_executable(control_teleop src/control_teleop.cpp)

# 2) Includi anche i file generati da rosidl
target_include_directories(control_teleop
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 3) Dipendenze ament
ament_target_dependencies(
  control_teleop
  rclcpp
  geometry_msgs
  std_msgs
)

# >>> AGGIUNTA: linka l'interfaccia al tuo eseguibile
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(control_teleop ${cpp_typesupport_target})

# 4) Install dell'eseguibile
install(
  TARGETS control_teleop
  DESTINATION lib/${PROJECT_NAME}
)

# 5) Install degli header
install(
  DIRECTORY include/
  DESTINATION include
)

# install dei file launch nella directory share/<package>
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# >>> AGGIUNTA: chiusura per pacchetto con messaggi
ament_export_dependencies(rosidl_default_runtime)

ament_package()

