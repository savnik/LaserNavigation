#include "TCM.h"
#include "TickGenerator.h"

const UInt8 kDataCount = 4;
// We will be requesting 4 componets (Heading, pitch, roll,
// temperature)

//
// This object polls the TCM module once a second for
// heading, pitch, roll and temperature.
//
TCM::TCM(SerPort * serPort): Process("TCM")
{
	// Let the CommProtocol know this object will handle any
	// serial data returned by the module
	mComm = new CommProtocol(this, serPort);
	mTime = 0;
	mStep = 1;
}

TCM::~TCM()
{
}

//
// Called by the CommProtocol object when a frame is completely received
//
void TCM::HandleComm(UInt8 frameType, void * dataPtr, UInt16 dataLen)
{
	UInt8 * data = (UInt8 *)dataPtr;
	switch(frameType)
	{
		case CommProtocol::kDataResp:
		{
			// Parse the data response
			UInt8 count = data[0];
			// The number of data elements returned
			UInt32 pntr = 1;
			// Used to retrieve the returned elements
			// The data elements we requested
			Float32 heading, pitch, roll, temperature;
			if(count != kDataCount)
			{
				// Message is a function that displays a C formatted string
				// (similar to printf)
				Message("Received %u data elements instead of the %u requested\r\n", (UInt16)count,(UInt16)kDataCount);
				return;
			}
			// loop through and collect the elements
			while(count)
			{
				// The elements are received as {type (ie. kHeading), data}
				switch(data[pntr++])
				// read the type and go to the first byte of the data
				{
				// Only handling the 4 elements we are looking for
				case CommProtocol::kHeading:
				{
					// Move(source, destination, size (bytes)). Move copies the
					// specified number of bytes from the source pointer to the
					// destination pointer. Store the heading.
					Move(&(data[pntr]),&heading,sizeof(heading));
					// increase the pointer to point to the next data element type
					pntr += sizeof(heading);
					break;
				}
				case CommProtocol::kPAngle:
				{
					// Move(source, destination, size (bytes)). Move copies the
					// specified number of bytes from the source pointer to the
					// destination pointer. Store the pitch.
					Move(&(data[pntr]),&pitch,sizeof(pitch));
					// increase the pointer to point to the next data element type
					pntr += sizeof(pitch);
					break;
				}
				case CommProtocol::kRAngle:
				{
					// Move(source, destination, size (bytes)). Move copies the
					// specified number of bytes from the source pointer to the
					// destination pointer. Store the roll.
					Move(&(data[pntr]),&roll,sizeof(roll));
					// increase the pointer to point to the next data element type
					pntr += sizeof(roll);
					break;
				}
				case CommProtocol::kTemperature:
				{
					// Move(source, destination, size (bytes)). Move copies the
					// specified number of bytes from the source pointer to the
					// destination pointer. Store the heading.
					Move(&(data[pntr]), &temperature,sizeof(temperature));
					// increase the pointer to point to the next data element type
					pntr += sizeof(temperature);
					break;
				}
				default:
					// Message is a function that displays a formatted string
					// (similar to printf)
					Message("Unknown type: %02X\r\n",data[pntr - 1]);
					// unknown data type, so size is unknown, so skip everything
					return;
					break;
				}
				count--;
				// One less element to read in
			}
			// Message is a function that displays a formatted string
			// (similar to printf)
			Message("Heading:%f,Pitch:%f,Roll: %f,Temperature: %f\r\n", heading, pitch, roll,temperature);
			mStep--;
			// send next data request
			break;
		}
		default:
		{
			// Message is a function that displays a formatted string
			// (similar to printf)
			Message("Unknown frame %02X received\r\n", (UInt16)frameType);
			break;
		}
	}
}

//
// Have the CommProtocol build and send the frame to the module.
//
void TCM::SendComm(UInt8 frameType, void * dataPtr, UInt16 dataLen)
{
	if(mComm) mComm->SendData(frameType, dataPtr, dataLen);
	// Ticks is a timer function. 1 tick = 10msec.
	mResponseTime = Ticks() + 300;
	// Expect a response
	within 3 seconds
}

//
// This is called each time this process gets a turn to execute.
//
void TCM::Control()
{
	switch(mStep)
	{
	case 1:
	{
		UInt8 pkt[kDataCount + 1];
		// the compents we are requesting, preceded by the number of
		// components being requested
		pkt[0]=kDataCount;
		pkt[1]=CommProtocol::kHeading;
		pkt[2]=CommProtocol::kPAngle;
		pkt[3]=CommProtocol::kRAngle;
		pkt[4]=CommProtocol::kTemperature;

		SendComm(CommProtocol::kSetDataComponents,pkt,kDataCount + 1);
		// Ticks is a timer function. 1 tick = 10msec.
		mTime = Ticks() + 100;
		// Taking a sample in 1s.
		mStep++;
		// go to next step of process
		break;
	}
	case 2:
	{
		// Ticks is a timer function. 1 tick = 10msec.
		if(Ticks() > mTime)
		{
			// tell the module to take a sample
			SendComm(CommProtocol::kGetData);
			mTime = Ticks() + 100; // take a sample everysecond
			mStep++;
		}
		break;
	}
	case 3:
	{
		// Ticks is a timer function. 1 tick = 10msec.
		if(Ticks() > mResponseTime)
		{
			Message("No response from the module. Check connection and try again\r\n");
			mStep = 0;
		}
		break;
	}

	default:
		break;
	}
}
