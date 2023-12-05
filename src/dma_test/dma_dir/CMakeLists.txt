cmake_minimum_required(VERSION 3.3.2)
project(dma)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(FILES
	dma.cpp
	dma.hpp
)

add_library(dma ${FILES})

target_include_directories(
	dma
PUBLIC
	../lib/
	${CMAKE_SOURCE_DIR}
)

