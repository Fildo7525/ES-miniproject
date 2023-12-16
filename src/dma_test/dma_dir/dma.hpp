#pragma once

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <iostream>

#include <time.h>
#include <chrono>

#include <math.h>

#include "axi_dma_controller.h"
#include "reserved_mem.hpp"
#include "drivers/xnn_inference.h"

#define DEVICE_FILENAME "/dev/reservedmemLKM"
// #define LENGTH 0x007fffff // Length in bytes
#define P_START 0x70000000
#define P_OFFSET 0

#define UIO_DMA_N 0


/**
 * @class DMA
 * @brief Wrapper class for the DMA and IP block for communication.
 *
 * TODO: Check if everything is working as it should with the IP block.
 *
 */
class DMA
{
public:
	// Constructor
	DMA(/*xnnipname %ip*/);
	~DMA();

	/**
	 * @brief Send data to generated IP with DMA.
	 *
	 * This method will send data @c buff to the shared memory @c pmem.
	 * DMA will then take them and send them to the IP block. The result
	 * is processed by DMA by writing it to the shared memory. We can
	 * then read it from there.
	 *
	 * @param buff Data to be processed by the IP block.
	 * @param out Pointer to the object were the processed data will be written.
	 * @param inputLength Length of the input data.
	 * @param outputLength Length of the output data.
	 */
	void useIpBlock(float *buff,
					float *out,
					const long inputLength,
					const long outputLength);

	/**
	 * @brief Send data to the IP and start it.
	 *
	 * @param buff Data to be processed by the IP block.
	 * @param inputLength Length of the input data.
	 * @return The size of the written data.
	 */
	uint32_t sendData(float *buff,
					  const long inputLength);

	/**
	 * @brief Receive data from the IP and write it to the output buffer.
	 *
	 * @param buff Data buffer for the output data.
	 * @param outputLength Length of the output buffer.
	 * @return The size of the read data.
	 */
	uint32_t getData(float *buff,
					 const long outputLength);

private:

	// DMA controller for sharing the data.
	AXIDMAController dma;

	// Shared memory used by the DMA
	Reserved_Mem pmem;

	XNn_inference neuralNetwork;
};

