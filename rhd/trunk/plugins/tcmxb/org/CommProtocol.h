#pragma once
//#include "SystemSerPort.h"
#include "Processes.h"

//
//CommHandler is a base class that provides a callback for
//incoming messages.
//
class CommHandler
{
	public:
	// Call back to be implemented in derived class.
	virtual void HandleComm(UInt8 frameType, void * dataPtr = null, UInt16 dataLen = 0) {}
};

//
// CommProtocol handles the actual serial communication with the
//module.
// Process is a base class that provides CommProtocol with
// cooperative parallel processing. The Control method will be
// called by a process manager on a continuous basis.
//
class CommProtocol : public Process{
	public:
	enum
	{
		// Frame IDs (Commands)
		kGetModInfo = 1,// 1
		kModInfoResp,		// 2
		kSetDataComponents,	// 3
		kGetData,			// 4
		kDataResp,			//5
		// Data Component IDs
		kHeading = 5,		//5 - type Float32
		kTemperature = 7,	//7 - type Float32
		kPAligned = 21,		//21 - type Float32
		kRAligned,			//22 - type Float32
		kIZAligned,			//23 - type Float32
		kPAngle,			//24 - type Float32
		kRAngle,			//25 - type Float32

	};
	enum
	{
		kBufferSize = 512, 	// maximum size of our input buffer
		kPacketMinSize = 5	// minimum size of a serial packet
	};

// SerPort is a serial communication object abstracting
// the hardware implementation
CommProtocol(CommHandler * handler = NULL, SerPort* serPort = NULL);
void Init(UInt32 baud = 38400);
void SendData(UInt8 frame, void * dataPtr = NULL, UInt32 len = 0);
void SetBaud(UInt32 baud);
	protected:
		CommHandler * mHandler;
		SerPort * mSerialPort;
		UInt8 mOutData[kBufferSize], mInData[kBufferSize];
		UInt16 mExpectedLen;
		UInt32 mOutLen, mOldInLen, mTime, mStep;
		UInt16 CRC(void * data, UInt32 len);
		void Control();
};

