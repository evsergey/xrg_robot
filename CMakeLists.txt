cmake_minimum_required(VERSION 3.13)
project(xrg_robots VERSION 1.0.0.0 LANGUAGES C CXX)

include_directories(include)
file(GLOB cpp_sources "src/*.cpp")
file(GLOB headers "include/*.h")

add_executable(test test/main.cpp)
target_sources(test PRIVATE ${cpp_sources})
target_link_libraries(test wiringPi pthread crypt rt)

install(TARGETS test DESTINATION bin)
