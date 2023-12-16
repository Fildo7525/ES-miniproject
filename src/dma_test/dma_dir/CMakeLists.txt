add_subdirectory(drivers)

set(FILES
	dma.cpp
	dma.hpp
)

add_library(dma ${FILES})


target_include_directories(
	dma
PUBLIC
	../lib/
	dma_dir
	${CMAKE_SOURCE_DIR}
)

target_link_libraries(
	dma
PUBLIC
	drivers
)

