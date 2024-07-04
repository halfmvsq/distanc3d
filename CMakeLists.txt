cmake_minimum_required(VERSION 3.16.4 FATAL_ERROR)
project(distanc3d LANGUAGES CXX)

include(FetchContent)

FetchContent_Declare(
  glm
  GIT_REPOSITORY https://github.com/g-truc/glm.git
  GIT_TAG 0af55ccecd98d4e5a8d1fad7de25ba429d60e863 # 1.0.1
)

FetchContent_MakeAvailable(glm)
message(STATUS "GLM source: ${glm_SOURCE_DIR}")

add_subdirectory(example)
