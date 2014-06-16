#pragma once
#include "Processes.h"
#include "CommProtocol.h"
//
// This file contains the object providing communication to the
// TCM. It will set up the module and parse packets received
// Process is a base class that provides TCM with cooperative
// parallel processing. The Control method will be
// called by a process manager on a continuous basis.
//
class TCM : public Process, public CommHandler {
	public:
		TCM(SerPort * serPort);
		~TCM();
	protected:
		CommProtocol * mComm;
		UInt32 mStep, mTime, mResponseTime;
		void UInt16 dataLen = HandleComm(UInt8 frameType, void * dataPtr = NULL,0);
		void UInt16 dataLen = SendComm(UInt8 frameType, void * dataPtr = NULL,0);
		void Control();
};

