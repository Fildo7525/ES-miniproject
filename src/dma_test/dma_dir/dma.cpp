#include "dma.hpp"

DMA::DMA()
	: pmem()
	, dma(UIO_DMA_N, 0x10000)
{
	printf("DMA constructor\n");
}

