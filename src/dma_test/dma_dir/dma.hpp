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
	~DMA() {
		XNn_inference_Release(&neuralNetwork);
	}

	/**
	 * @brief Send data to generated IP with DMA.
	 *
	 * This method will send data @c buff to the shared memory @c pmem.
	 * DMA will then take them and send them to the IP block. The result
	 * is processed by DMA by writing it to the shared memory. We can
	 * then read it from there.
	 *
	 * @tparam T Type of data to be send to the IP block.
	 * @param buff Data to be processed by the IP block.
	 * @param out Pointer to the object were the processed data will be written.
	 * @param inputLength Length of the input data.
	 * @param outputLength Length of the output data.
	 */
	template<typename T>
	void useIpBlock(T *buff,
					T *out,
					const long inputLength,
					const long outputLength);

	/**
	 * @brief Send data to the IP and start it.
	 *
	 * @tparam T Type of data to be send to the IP block.
	 * @param buff Data to be processed by the IP block.
	 * @param inputLength Length of the input data.
	 * @return The size of the written data.
	 */
	template<typename T>
	uint32_t sendData(T *buff,
					  const long inputLength);

	/**
	 * @brief Receive data from the IP and write it to the output buffer.
	 *
	 * @tparam T Type of the data that was sent to the IP.
	 * @param buff Data buffer for the output data.
	 * @param outputLength Length of the output buffer.
	 * @return The size of the read data.
	 */
	template<typename T>
	uint32_t getData(T *buff,
					 const long outputLength);

private:

	// DMA controller for sharing the data.
	AXIDMAController dma;

	// Shared memory used by the DMA
	Reserved_Mem pmem;

	XNn_inference neuralNetwork;
};

template<typename T>
void DMA::useIpBlock(T *buff,
					 T *out,
					 const long inputLength,
					 const long outputLength)
{
	sendData(buff, inputLength);
	getData(out, outputLength);
}

template<typename T>
uint32_t DMA::sendData(T *buff, const long inputLength)
{
	// Transfer data to the shared memeory.
	auto ret = pmem.transfer(buff, P_OFFSET, inputLength);

	dma.MM2SReset();
	dma.S2MMReset();

	dma.MM2SHalt();
	dma.S2MMHalt();

	dma.MM2SInterruptEnable();
	dma.S2MMInterruptEnable();

	dma.MM2SSetSourceAddress(P_START + P_OFFSET);
	dma.S2MMSetDestinationAddress(P_START + P_OFFSET);

	// Here we will wate for the ip to be ready
	while(!XNn_inference_IsReady(&neuralNetwork)) {}

	// Start the ip
	XNn_inference_Start(&neuralNetwork);

	dma.MM2SStart();
	dma.S2MMStart();

	dma.MM2SSetLength(inputLength); //! WIll only work up to 2^23

	return ret;
}

template<typename T>
uint32_t DMA::getData(T *buff, const long outputLength)
{
	dma.S2MMSetLength(outputLength);

	while (!dma.MM2SIsSynced()) {}
	while(!dma.S2MMIsSynced()) {}
	while(!Xipname_IsDone(&neuralNetwork)) {}

	return pmem.gather(buff, P_OFFSET, outputLength);
}

