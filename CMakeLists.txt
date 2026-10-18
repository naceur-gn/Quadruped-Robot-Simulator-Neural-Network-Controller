cmake_minimum_required(VERSION 3.20)
project(robot_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(FetchContent)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 3.4
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(glfw)

add_executable(robot_sim
  src/main.cpp
  src/gl.cpp
  src/render/shader.cpp
  src/render/mesh.cpp
  src/render/camera.cpp
  src/physics/physics.cpp
  src/robot/quadruped.cpp
  src/robot/stabilizer.cpp
  src/scene/scene.cpp
)

target_include_directories(robot_sim PRIVATE src)
target_link_libraries(robot_sim PRIVATE glfw)

if(WIN32)
  target_link_libraries(robot_sim PRIVATE opengl32)
endif()

if(MSVC)
  target_compile_options(robot_sim PRIVATE /W4)
else()
  target_compile_options(robot_sim PRIVATE -Wall -Wextra)
endif()
