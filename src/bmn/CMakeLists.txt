cmake_minimum_required(VERSION 3.8)
project(bmn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Haply stuff ##########################################################################################
# set the version to download, dont use debug version
set(VERSION "0.1.2")
set(API_DIR "${CMAKE_SOURCE_DIR}/deps/HardwareAPI")
## Download and extract the latest version of the HardwareAPI
set(PACKAGE "HardwareAPI.${VERSION}.linux.tar.gz")
file(DOWNLOAD "https://cdn.haply.co/r/33857671/${PACKAGE}" "${PACKAGE}")
file(ARCHIVE_EXTRACT INPUT ${PACKAGE} DESTINATION ${API_DIR})
## Required include and define directives for making use of the API.
include_directories(${API_DIR})
add_definitions(-DUNICODE -D_UNICODE)
## Additional compiler specific directives
add_compile_options(-ggdb -Wall -Wextra -pedantic -mavx2)
add_compile_options(-Wno-deprecated-declarations)
add_compile_options(-march=native)
link_libraries(${API_DIR}/libHaply.HardwareAPI.a)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(commsmsgs REQUIRED)

add_executable(bmn_node src/bubblemethod.cpp)
ament_target_dependencies(bmn_node rclcpp Eigen3 geometry_msgs std_msgs commsmsgs)

install(TARGETS
  bmn_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_package()
