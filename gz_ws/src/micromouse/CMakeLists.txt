cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz-gui-micromouse-plugin)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set (CMAKE_AUTOMOC ON)

# Find Qt5
find_package (Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
)

# Find the Gazebo SIM library
find_package(gz-sim8 REQUIRED COMPONENTS gui)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GZ-GUI_CXX_FLAGS}")

QT5_ADD_RESOURCES(resources_RCC micromouse.qrc)

# Generate examples
add_library(MicromousePlugin SHARED ${headers_MOC}
  MicromousePlugin.cc
  ${resources_RCC}
)
target_link_libraries(MicromousePlugin
  PRIVATE gz-sim8::gui
  ${Qt5Core_LIBRARIES}
  ${Qt5Qml_LIBRARIES}
  ${Qt5Quick_LIBRARIES}
  ${Qt5QuickControls2_LIBRARIES}
)

target_compile_options(MicromousePlugin PRIVATE
  $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
)
