#include "CommProtocol.h"
// import an object that will provide a 10mSec tick count through
// a function called Ticks()
#include "TickGenerator.h"
// SerPort is an object that controls the physical serial
// interface. It handles sending out
// the characters, and buffers the characters read in until
// we are ready for them.
//
CommProtocol::CommProtocol(CommHandler * handler, SerPort * serPort)
: Process("CommProtocol")
{
mHandler = handler;
// store the object that will parse the data when it is fully
// received
mSerialPort = serPort;
Init();
}
// Initialize the serial port and variables that will control
// this process
void CommProtocol::Init(UInt32 baud)
{
SetBaud(baud);
mOldInLen = 0;
// no data previously received
mStep = 1;
// goto the first step of our process
}
//
// Put together the frame to send to the module
//
void CommProtocol::SendData(UInt8 frameType, void * dataPtr, UInt32
len)
{
UInt8 * data = (UInt8 *)dataPtr;
// the data to send
UInt32 index = 0;
// our location in the frame we are putting together
UInt16 crc;
// the CRC to add to the end of the packet
UInt16 count;
// the total length the packet will be
count = (UInt16)len + kPacketMinSize;
// exit without sending if there is too much data to fit
// inside our packet
if(len > kBufferSize - kPacketMinSize) return;
//
//
//
//
Store the total len of the packet including the len bytes
(2), the frame ID (1),
the data (len), and the crc (2). If no data is sent, the
min len is 5
mOutData[index++] = count >> 8;
mOutData[index++] = count & 0xFF;
// store the frame ID
mOutData[index++] = frameType ;
// copy the data to be sent
while(len--) mOutData[index++] = *data++;
// compute and add the crc
crc = CRC(mOutData, index);
mOutData[index++] = crc >> 8 ;
mOutData[index++] = crc & 0xFF ;
// Write block will copy and send the data out the serial port
mSerialPort->WriteBlock(mOutData, index);
}
//
// Call the functions in serial port necessary to change the
// baud rate
//
void CommProtocol::SetBaud(UInt32 baud)
{
mSerialPort->SetBaudRate(baud);
mSerialPort->InClear();
// clear any data that was already waiting in the buffer
}
//
// Update the CRC for transmitted and received data using the
// CCITT 16bit algorithm (X^16 + X^12 + X^5 + 1).
//
UInt16 CommProtocol::CRC(void * data, UInt32 len)
{
UInt8 * dataPtr = (UInt8 *)data;
UInt32 index = 0;
UInt16 crc = 0;
while(len--)
{
crc = (unsigned char)(crc >> 8) | (crc << 8);
crc ^= dataPtr[index++];
crc ^= (unsigned char)(crc & 0xff) >> 4;
crc ^= (crc << 8) << 4;
crc ^= ((crc & 0xff) << 4) << 1;
}
return crc;
}
//
// This is called each time this process gets a turn to execute.
//
void CommProtocol::Control()
{
// InLen returns the number of bytes in the input buffer of
//the serial object that are available for us to read.
UInt32 inLen = mSerialPort->InLen();
//
//
//
//
switch(mStep)
{
case 1:
{
wait for length bytes to be received by the serial object
if(inLen >= 2)
{
Read block will return the number of requested (or available)
bytes that are in the serial objects input buffer.
read the byte count
mSerialPort->ReadBlock(mInData, 2);
// byte count is ALWAYS transmitted in big endian, copy byte
// count to mExpectedLen to native endianess
mExpectedLen
=
(mInData[0]
<<
mInData[1];
8)
|
// Ticks is a timer function. 1 tick = 10msec.
// wait up to 1/2s for the complete frame (mExpectedLen) to be
// received
mTime = Ticks() + 50 ;
mStep++ ;
// goto the next step in the process
}
break ;
}
case 2:
{
// wait for msg complete or timeout
if(inLen >= mExpectedLen - 2)
{
UInt16 crc, crcReceived;
// calculated and received crcs.
// Read block will return the number of
// requested (or available) bytes that are in the
// serial objects input buffer.
mSerialPort->ReadBlock(&mInData[2],
mExpectedLen - 2);
// in CRC verification, don't include the CRC in the recalculation
(-2)
crc = CRC(mInData, mExpectedLen - 2);
// CRC is also ALWAYS transmitted in big endian
crcReceived = (mInData[mExpectedLen - 2] <<
8) | mInData[mExpectedLen - 1] ;
if(crc == crcReceived)
{
// the crc is correct, so pass the frame up for processing.
if(mHandler)
mHandler-
>HandleComm(mInData[2], &mInData[3], mExpectedLen - kPacketMinSize);
}
else
{
// crc's don't match so clear everything that is currently in the
// input buffer since the data is not reliable.
mSerialPort->InClear();
}
// go back to looking for the length bytes.
mStep = 1 ;
}
else
{
// Ticks is a timer function. 1 tick = 10msec.
if(Ticks() > mTime)
{
// Corrupted message. We did not get the length we were
// expecting within 1/2sec of receiving the length bytes. Clear
// everything in the input buffer since the data is unreliable
mSerialPort->InClear();
mStep = 1 ;
// Look for the next length bytes
}
}
break ;
}
default:
break ;
}
}

