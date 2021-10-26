cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_cxx)

add_compile_options(-std=c++11)

find_package(Boost REQUIRED)
include_directories(
    ${Boost_INCLUDE_DIR}    
    $ENV{UNITREE_SDK_PATH}/include
    # unitree_legged_sdk/include
)

link_directories($ENV{UNITREE_SDK_PATH}/lib)
# link_directories(unitree_legged_sdk/lib)
message(STATUS $ENV{Boost_INCLUDE_DIR})

# add julia interface 
set(JlCxx_DIR $ENV{HOME}/dev/libcxxwrap-julia-build)
set(Julia_PREFIX /opt/julia-1.6.1)
set(Julia_EXECUTABLE /opt/julia-1.6.1/bin/julia)
# set(Julia_EXECUTABLE /usr/local/bin/julia)
# set(Julia_PREFIX /usr/local/bin/)

find_package(JlCxx)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

message(STATUS ${CMAKE_INSTALL_PREFIX})

add_library(julia_a1_interface SHARED src/julia_interface.cpp)
target_link_libraries(julia_a1_interface JlCxx::cxxwrap_julia libunitree_legged_sdk_arm64.so)

install(TARGETS
  julia_a1_interface
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
RUNTIME DESTINATION lib)

# cmake -DJulia_EXECUTABLE=/usr/local/bin/julia ../libcxxwrap-julia
# cmake -DJulia_PREFIX=/usr/local/bin/ ../libcxxwrap-julia