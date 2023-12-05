set(SRC
	xnn_inference.c
)

set(HEADERS
	xnn_inference.h
	xnn_inference_hw.h
	xnn_inference_linux.c
	xnn_inference_sinit.c
)

add_library(drivers ${SRC} ${HEADERS})


