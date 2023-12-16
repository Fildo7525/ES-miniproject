cmake_minimum_required(VERSION 3.16.3)
project(testIP)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(dma_dir)

set(SRC
	main.cpp
)

add_executable(testIP main.cpp)

target_link_libraries(
	testIP
PUBLIC
	drivers
	dma
)

