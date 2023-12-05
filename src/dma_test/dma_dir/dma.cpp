#include "dma.hpp"

DMA::DMA()
	: pmem()
	, dma(UIO_DMA_N, 0x10000)
{
	int status = XImge_processor_Initialize(&neuralNetwork, "imge_processor");
	XNn_inference_Initialize(&neuralNetwork, "nn_inference")
	if (status != XST_SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "Error: Could not initialize the IP core.");
		exit(-1);
	}

	printf("DMA constructor\n");
}

