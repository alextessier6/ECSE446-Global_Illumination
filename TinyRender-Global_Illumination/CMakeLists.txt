cmake_minimum_required(VERSION 3.5)

project(tinyrender)

find_package(SDL2 REQUIRED)
if(SDL_FOUND)
    message("SDL Found")
endif()
include_directories(${SDL2_INCLUDE_DIRS})

#string(STRIP ${SDL2_LIBRARIES} SDL2_LIBRARIES)

find_package(OpenGL REQUIRED)
if(OPENGL_FOUND)
    message("OpenGL Found")
endif()
include_directories(${OPENGL_INCLUDE_DIRS})

find_package(GLEW REQUIRED)
if(GLEW_FOUND)
    message("GLEW Found")
endif()
include_directories(${GLEW_INCLUDE_DIRS})

find_package(Boost COMPONENTS filesystem system REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories("src")
include_directories("externals/")
include_directories("externals/glm/")

set(OpenMP_C "${CMAKE_C_COMPILER}")
set(OpenMP_C_FLAGS "-fopenmp -Wno-unused-command-line-argument")
set(OpenMP_C_LIB_NAMES "libomp" "libgomp" "libiomp5")
set(OpenMP_libomp_LIBRARY ${OpenMP_C_LIB_NAMES})
set(OpenMP_libgomp_LIBRARY ${OpenMP_C_LIB_NAMES})
set(OpenMP_libiomp5_LIBRARY ${OpenMP_C_LIB_NAMES})

set(OpenMP_CXX "${CMAKE_CXX_COMPILER}")
set(OpenMP_CXX_FLAGS "-fopenmp -Wno-unused-command-line-argument")
set(OpenMP_CXX_LIB_NAMES "libomp" "libgomp" "libiomp5")
set(OpenMP_libomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})
set(OpenMP_libgomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})
set(OpenMP_libiomp5_LIBRARY ${OpenMP_CXX_LIB_NAMES})

include_directories( "/usr/local/opt" )

find_package(OpenMP REQUIRED)
if(NOT TARGET OpenMP::OpenMP_CXX)
    add_library(OpenMP_TARGET INTERFACE)
    add_library(OpenMP::OpenMP_CXX ALIAS OpenMP_TARGET)
    target_compile_options(OpenMP_TARGET INTERFACE ${OpenMP_CXX_FLAGS})
    find_package(Threads REQUIRED)
    target_link_libraries(OpenMP_TARGET INTERFACE Threads::Threads)
    target_link_libraries(OpenMP_TARGET INTERFACE ${OpenMP_CXX_FLAGS})
endif()

file(GLOB_RECURSE srcs src/*.cpp src/*.h)
add_executable(tinyrender ${srcs})

if(WIN32)
    target_link_libraries(tinyrender ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} SDL2::SDL2 SDL2::SDL2main)
elseif(APPLE)
    target_link_libraries(tinyrender OpenMP::OpenMP_CXX ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} ${SDL2_LIBRARIES})
else()
    target_link_libraries(tinyrender stdc++fs ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} ${SDL2_LIBRARIES})
endif()