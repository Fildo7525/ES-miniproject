#include "dma.hpp"

DMA::DMA()
	: pmem()
	, dma(UIO_DMA_N, 0x10000)
{
	int status = XNn_inference_Initialize(&neuralNetwork, "nn_inference");
	if (status != XST_SUCCESS) {
		std::cout << "The IP nn_inference could not be initialized" << std::endl;
		exit(-1);
	}

	printf("DMA constructor\n");
}

DMA::~DMA() {
	XNn_inference_Release(&neuralNetwork);
}

void DMA::useIpBlock(float *buff,
					 float *out,
					 const long inputLength,
					 const long outputLength)
{
	sendData(buff, inputLength);
	getData(out, outputLength);
}

uint32_t DMA::sendData(float *buff, const long inputLength)
{
	std::cout << "Transfering data\n";
	// Transfer data to the shared memeory.
	auto ret = pmem.transfer(buff, P_OFFSET, inputLength);

	std::cout << "Reseting S2MM and MM2S\n";
	dma.MM2SReset();
	dma.S2MMReset();

	std::cout << "Halting\n";
	dma.MM2SHalt();
	dma.S2MMHalt();

	std::cout << "Enebling interrupts\n";
	dma.MM2SInterruptEnable();
	dma.S2MMInterruptEnable();

	std::cout << "Setting source\n";
	dma.MM2SSetSourceAddress(P_START + P_OFFSET);
	dma.S2MMSetDestinationAddress(P_START + P_OFFSET);

	std::cout << "Waiting for the IP to be ready\n";
	// Here we will wate for the ip to be ready
	while(!XNn_inference_IsReady(&neuralNetwork)) {}

	std::cout << "Staring\n";
	// Start the ip
	XNn_inference_Start(&neuralNetwork);

	dma.MM2SStart();
	dma.S2MMStart();


	std::cout << "setting the length of MM2S\n";
	dma.MM2SSetLength(inputLength); //! WIll only work up to 2^23

	return ret;
}

uint32_t DMA::getData(float *buff, const long outputLength)
{
	std::cout << "setting the length of S2MM\n";
	dma.S2MMSetLength(outputLength);

	std::cout << "Sync MM2S\n";
	while (!dma.MM2SIsSynced()) {}
	std::cout << "Sync S2MM\n";
	while(!dma.S2MMIsSynced()) {}
	std::cout << "Wait for NN\n";
	while(!XNn_inference_IsDone(&neuralNetwork)) {}

	std::cout << "Gather data" << std::endl;
	return pmem.gather(buff, P_OFFSET, outputLength);
}

