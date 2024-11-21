cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(encoder)

find_package(gz-cmake3 REQUIRED)
find_package(gz-sensors8 REQUIRED)
find_package(gz-sim8 REQUIRED)  

add_library(${PROJECT_NAME} SHARED Encoder.cc)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    gz-sensors8::gz-sensors8
    gz-sim8::gz-sim8
)

target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Werror>
)
