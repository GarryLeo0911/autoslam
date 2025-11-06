cmake_minimum_required(VERSION 3.8)
project(autoslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install config files (only if directory contains files)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config" AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config")
    file(GLOB CONFIG_FILES "${CMAKE_CURRENT_SOURCE_DIR}/config/*")
    if(CONFIG_FILES)
        install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
    endif()
endif()

# Install scripts (only if directory contains files)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts" AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
    file(GLOB SCRIPT_FILES "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
    if(SCRIPT_FILES)
        install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME}/
                USE_SOURCE_PERMISSIONS)
    endif()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()