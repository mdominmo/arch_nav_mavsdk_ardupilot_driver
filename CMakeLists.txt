cmake_minimum_required(VERSION 3.16)
project(arch_nav_mavsdk VERSION 0.0.1 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

if(NOT TARGET arch_nav::arch_nav)
  find_package(arch_nav REQUIRED)
endif()
find_package(MAVSDK REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/mavsdk_command_dispatcher.cpp
  src/mavsdk_platform_driver.cpp
  src/mavsdk_config_loader.cpp
  src/mavsdk_registration.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    arch_nav::arch_nav
  PRIVATE
    MAVSDK::mavsdk
    yaml-cpp
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/arch_nav/drivers
)

install(DIRECTORY config/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/arch_nav_mavsdk_driver/config
)
