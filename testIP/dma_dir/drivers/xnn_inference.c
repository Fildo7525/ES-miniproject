// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "xnn_inference.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XNn_inference_CfgInitialize(XNn_inference *InstancePtr, XNn_inference_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XNn_inference_Start(XNn_inference *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNn_inference_ReadReg(InstancePtr->Control_BaseAddress, XNN_INFERENCE_CONTROL_ADDR_AP_CTRL) & 0x80;
    XNn_inference_WriteReg(InstancePtr->Control_BaseAddress, XNN_INFERENCE_CONTROL_ADDR_AP_CTRL, Data | 0x01);
}

u32 XNn_inference_IsDone(XNn_inference *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNn_inference_ReadReg(InstancePtr->Control_BaseAddress, XNN_INFERENCE_CONTROL_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XNn_inference_IsIdle(XNn_inference *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNn_inference_ReadReg(InstancePtr->Control_BaseAddress, XNN_INFERENCE_CONTROL_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XNn_inference_IsReady(XNn_inference *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNn_inference_ReadReg(InstancePtr->Control_BaseAddress, XNN_INFERENCE_CONTROL_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XNn_inference_EnableAutoRestart(XNn_inference *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNn_inference_WriteReg(InstancePtr->Control_BaseAddress, XNN_INFERENCE_CONTROL_ADDR_AP_CTRL, 0x80);
}

void XNn_inference_DisableAutoRestart(XNn_inference *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNn_inference_WriteReg(InstancePtr->Control_BaseAddress, XNN_INFERENCE_CONTROL_ADDR_AP_CTRL, 0);
}

void XNn_inference_InterruptGlobalEnable(XNn_inference *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNn_inference_WriteReg(InstancePtr->Control_BaseAddress, XNN_INFERENCE_CONTROL_ADDR_GIE, 1);
}

void XNn_inference_InterruptGlobalDisable(XNn_inference *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNn_inference_WriteReg(InstancePtr->Control_BaseAddress, XNN_INFERENCE_CONTROL_ADDR_GIE, 0);
}

void XNn_inference_InterruptEnable(XNn_inference *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XNn_inference_ReadReg(InstancePtr->Control_BaseAddress, XNN_INFERENCE_CONTROL_ADDR_IER);
    XNn_inference_WriteReg(InstancePtr->Control_BaseAddress, XNN_INFERENCE_CONTROL_ADDR_IER, Register | Mask);
}

void XNn_inference_InterruptDisable(XNn_inference *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XNn_inference_ReadReg(InstancePtr->Control_BaseAddress, XNN_INFERENCE_CONTROL_ADDR_IER);
    XNn_inference_WriteReg(InstancePtr->Control_BaseAddress, XNN_INFERENCE_CONTROL_ADDR_IER, Register & (~Mask));
}

void XNn_inference_InterruptClear(XNn_inference *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNn_inference_WriteReg(InstancePtr->Control_BaseAddress, XNN_INFERENCE_CONTROL_ADDR_ISR, Mask);
}

u32 XNn_inference_InterruptGetEnabled(XNn_inference *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XNn_inference_ReadReg(InstancePtr->Control_BaseAddress, XNN_INFERENCE_CONTROL_ADDR_IER);
}

u32 XNn_inference_InterruptGetStatus(XNn_inference *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XNn_inference_ReadReg(InstancePtr->Control_BaseAddress, XNN_INFERENCE_CONTROL_ADDR_ISR);
}

