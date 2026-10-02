cmake_minimum_required(VERSION 3.16)
project(nav_safety_profile LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} INTERFACE yaml-cpp OpenSSL::Crypto)

install(DIRECTORY include/ DESTINATION include)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../../config/safety_profile/sitl_current_as_is.yaml"
  DESTINATION share/${PROJECT_NAME}/config)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenSSL yaml-cpp)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safety_profile test/test_safety_profile.cpp)
  target_link_libraries(test_safety_profile ${PROJECT_NAME})
  target_compile_definitions(test_safety_profile PRIVATE
    SAFETY_PROFILE_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/safety_profile/sitl_current_as_is.yaml")

  add_executable(profile_hash_probe test/profile_hash_probe.cpp)
  target_link_libraries(profile_hash_probe ${PROJECT_NAME})
  target_compile_definitions(profile_hash_probe PRIVATE
    SAFETY_PROFILE_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/safety_profile/sitl_current_as_is.yaml")
  add_test(NAME test_safety_profile_cross_language_hash
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/safety_profile/tests/test_profile_hash_cross_language.py
      $<TARGET_FILE:profile_hash_probe>
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/safety_profile/sitl_current_as_is.yaml)
endif()

ament_package()
