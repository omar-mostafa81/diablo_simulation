cmake_minimum_required(VERSION 3.5)
project(diablo_simulation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(motion_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


set(dependencies
  rclcpp
  motion_msgs
  nav_msgs
  geometry_msgs
)

set(executable_name path_follower_node)
set(executable2_name diablo_simulation)
set(executable3_name save_to_csv)
set(executable4_name path_jacobian_node)

set(CMAKE_CXX_STANDARD 17)

add_executable(${executable_name} 
      src/path_follower_node.cpp
  )

add_executable(${executable2_name} 
      src/diablo_simulation.cpp
  )  

add_executable(${executable3_name} 
      src/save_to_csv.cpp
  )    
  
  add_executable(${executable4_name} 
      src/path_jacobian_node.cpp
  )    
  
ament_target_dependencies(${executable_name} ${dependencies})
ament_target_dependencies(${executable2_name} ${dependencies})
ament_target_dependencies(${executable3_name} ${dependencies})
ament_target_dependencies(${executable4_name} ${dependencies})

install(TARGETS ${library_name}
  DESTINATION lib)

install(TARGETS 
	${executable_name}
	${executable2_name}
	${executable3_name}
	${executable4_name}
	  DESTINATION lib/${PROJECT_NAME}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
