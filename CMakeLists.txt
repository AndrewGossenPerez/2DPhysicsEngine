cmake_minimum_required(VERSION 3.10)
project(2DPhysicsEngine LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(external/glfw)

add_library(glad STATIC ${CMAKE_CURRENT_SOURCE_DIR}/external/glad.c)

target_include_directories(glad PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/external)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/world.cpp
    src/RigidBody.cpp
    src/collision.cpp
    src/visuals.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PRIVATE glad glfw)
