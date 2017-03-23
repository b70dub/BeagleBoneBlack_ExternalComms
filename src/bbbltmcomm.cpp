/*-----------------------------------------------------------------------------
 *
 * File: bbbltmcomm.cpp
 * Author: Brian Ankeny
 * Platform: Beaglebone Black
 * Eclipse Luna Service Release 2 (4.4.2)
 * This project supports multiple threads and provides a method of communicating with an external device via uart ttyO4 the Beaglebone Black.
 * Note that only tty00 is enabled by default. You will need to modify uEnv.txt via an editor such as nano to
 * enable any other uarts that you require. See http://beaglebone.cameon.net/home/serial-ports-uart for
 * instructions on how to accomplish this.
 *
 * also included is support for socket communications from the beaglebone black to an external pc/device as
 * well as support for sending an email from the beaglebone black.
 *-----------------------------------------------------------------------------
*/
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono> // C++11  // std::chrono::steady_clock
#include <unistd.h>
#include "serialib.h"
#include <signal.h>

#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>

#include <netdb.h>


#define SOCKSERVADD "192.168.1.12"
#define SOCKPORTNUM "32000"
#define         DEVICE_PORT             "/dev/ttyO4"


using namespace std;
using namespace serialcomms;

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::monotonic_clock;

//#define _GLIBCXX_USE_NANOSLEEP


auto TCPClient_startTime = monotonic_clock::now();
auto TCPClient_CurrentTime = monotonic_clock::now();

auto TCPClient_TeststartTime = monotonic_clock::now();
auto TCPClient_TestCurrentTime = monotonic_clock::now();



auto Serial_startTime = monotonic_clock::now();
auto Serial_CurrentTime = monotonic_clock::now();


//Serial Comm Flags
bool 	bKillThread_BBBLTM_SerialComms = false,

		bSerialCommsRunning = false,
		bSerialData_DataReadyToSend = false,
		bSerialData_ReceiveDecodeComplete = false,
		bSerial_ReqToSendControllerUpdatesToLTM = false,
		bSerial_ReqToSendCheckForConnStatus = false,
		bSerial_SendConnStatusRequest = false,
		bSerial_ReqToSendDataReceivedACK = false,
		bSerial_RequestControllerUpdate = false,
		bSerial_SentDataReceivedByLTM = false,
		bSerial_RequestToSendDataToClient = false,
		bSerial_ControllerUpdatesSentToLTM = false,

		bEmail_SendDeviationAlert = false,
		bEmail_SendMaintenanceAlert = false,

		bTestSerialEnabled = true,							//TEST FLAG!!!!!
		bTest_EchoBackData = false;

// TCP Server Flags
bool	bKillThread_BBBLTM_TCPServerComms = false,
		bTCPServerCommsRunning = false,

		bTCPServer_DataReadyToSend = false,
		bTCPServer_ReceiveDecodeComplete = false,
		bTCPServer_ReqToSendDataReceivedACK = false,
		bTCPServer_SentDataReceivedByRemoteBBB = false,
		bOkToContinue = false;

// TCP Client Flags
bool	bKillThread_BBBLTM_TCPClientComms = false,
		bTCPClientCommsRunning = false,
		bTCPClient_ReqToSendControllerUpdatesToServer = false,
		bTCPClient_DataReadyToSend = false,
		bTCPClient_ReceiveDecodeComplete = false,
		bTCPClient_ReqToSendDataReceivedACK = false,
		bTCPClient_SentDataReceivedByRemoteBBB = false,
		bTCPClient_RequestToSendDataToLTM = false,
		bTCPClient_ReqToSendCheckForConnStatus = false,
		bTCPClient_SentDataReceivedByServer = false,
		bTCPClient_ControllerUpdatesSentToServer = false,
		bTCPClient_SendConnStatusRequest = false,   			//not coded yet
		bTCPSentDataToServerTEST = false,
		bTCPClient_SendTestDataToServer = false,
		bTCPSendDataToServerTEST = false;

//general flags
bool 	bKillKeyboardInput = false,
		bKeyboardMonitorRunning = false;

char chKey;

//Threads
void BBBLTM_SerialComms();
void BBBLTM_TCPServer();
void BBBLTM_TCPClient();
void KeyboardMon();

//Serial Comms Functions
void Func_SerialHandleAndSendData();
uint16_t Func_SerialHandleReceivedData (uint16_t iNumBytesRead);
static uint16_t SerialComputeChecksum(uint8_t* bytes, uint16_t length);

//TCP Socket Server Functions
void Func_TCPServerHandleAndSendData(int TCPServer_newsockfd);
uint16_t Func_TCPServerHandleReceivedData (uint16_t iNumBytesRead);
static uint16_t TCPServerComputeChecksum(uint8_t* bytes, uint16_t length);

//TCP Socket Client Functions
void Func_TCPClientHandleAndSendData(int sockfd);
uint16_t Func_TCPClientHandleReceivedData (uint16_t iNumBytesRead);
static uint16_t TCPClientComputeChecksum(uint8_t* bytes, uint16_t length);


//void Func_GetLocalIPAddr(char* IPAddrString);


serialib LS;    										// Object of the serialib class

//Serial int vars
uint16_t 	iSerial_Ret = 0,                                        // Used for return values
			iSerial_ReceivedPayloadLen = 0,
			iSerial_ReceivedLocalControllerNumber = 0,
			iSerial_ReceivedLocalControllerPhase = 0,
			iSerial_ReceivedLocalControllerRecipeNum = 0,
			iSerial_ReceivedLocalControllerBasketNum = 0,
			iSerial_FrameStartCount = 0,
			iSerial_ResendDelay = 0;

int 		iPortHandle = 0;


//Serial Char Arrays
char	 	SerialInBuffer[128],
			SerialOutBuffer[128],
			Serial_ReceivedCRCBuffer[2],
			Serial_ReceivedPayloadBuffer[128];

//Serial Chars
char		chSerial_ReceivedCommand,
			chSerial_ReceivedPayloadLen,
			chSerial_SendCommand,
			chSerial_SendPayloadLen;

//TCP Server Char Arrays
char	 	TCPServer_InBuffer[128],
			TCPServer_OutBuffer[128],
			TCPServer_ReceivedCRCBuffer[2],
			TCPServer_ReceivedPayloadBuffer[128];

//TCP Server Chars
char		chTCPServer_ReceivedCommand,
			chTCPServer_ReceivedPayloadLen,
			chTCPServer_SendCommand,
			chTCPServer_SendPayloadLen;

//TCP Server int Vars
uint16_t 	iTCPServer_Ret = 0,                                        // Used for return values
			iTCPServer_ReceivedPayloadLen = 0,
			iTCPServer_ReceivedLocalControllerNumber = 0,
			iTCPServer_ReceivedLocalControllerPhase = 0,
			iTCPServer_ReceivedLocalControllerRecipeNum = 0,
			iTCPServer_ReceivedLocalControllerBasketNum = 0,
			iTCPServer_FrameStartCount = 0;


//TCP Client Char Arrays
char	 	TCPClient_InBuffer[128],
			TCPClient_OutBuffer[128],
			TCPClient_ReceivedPayloadBuffer[128],
			TCPClient_ReceivedCRCBuffer[2],
			TCPClient_SendPayloadBuffer[128],
			TCPClient_SendCRCBuffer[2];

//TCP Client Chars
char		chTCPClient_SendCommand,
			chTCPClient_SendPayloadLen,
			chTCPClient_ReceivedCommand,
			chTCPClient_ReceivedPayloadLen;

//TCP Client int Vars
uint16_t 	iTCPClient_Ret = 0,                                        // Used for return values
			iTCPClient_ReceivedCRC = 0,
			iTCPClient_ReceivedPayloadLen = 0,
			iTCPClient_ReceivedLocalControllerNumber = 0,
			iTCPClient_ReceivedLocalControllerPhase = 0,
			iTCPClient_ReceivedLocalControllerRecipeNum = 0,
			iTCPClient_ReceivedLocalControllerBasketNum = 0,
			iTCPClient_FrameStartCount = 0,
			iTCPClient_ResendDelay = 1000; //milliseconds



volatile sig_atomic_t iRunProgram = 1;

uint8_t chByte[1];
char Buffer[128];

char ETH0_IPAddr[20];

void my_function(int sig){ // can be called asynchronously
	bKillThread_BBBLTM_SerialComms = true; // set flag to kill thread
	bKillThread_BBBLTM_TCPServerComms = true;
	bKillThread_BBBLTM_TCPClientComms = true;
	bKillKeyboardInput = true;
	std::cout << "caught!..Press enter to exit" << std::endl;
}


int main(int argc, char **argv)
{
	printf("in main 1\n");
	//Get the IP adress of the local device ETH0 interface
//	Func_GetLocalIPAddr(ETH0_IPAddr);

	signal(SIGINT,my_function);       //for Ctrl+C


//	//Create thread for RS232 comms with LTM
	std::thread t1(BBBLTM_SerialComms);


	printf("in main 2\n");
	//Create thread for TCP socket server comms with other BeagleBone Black - serve data
//	std::thread t2(BBBLTM_TCPServer);


	printf("in main 3\n");

	//Create thread for TCP socket client comms with other BeagleBone Black -request data
//	std::thread t3(BBBLTM_TCPClient);

	printf("in main 4\n");

	//Create thread for TCP socket client comms with other BeagleBone Black -request data
//	std::thread t4(KeyboardMon);

	while(iRunProgram == 1)
	{
		if(!bSerialCommsRunning && !bTCPServerCommsRunning && !bTCPClientCommsRunning)
			iRunProgram = 0;


		//printf("still in main 4\n");
		usleep(100000);			//Sleep for 100 ms
	}

	t1.join();
//	t2.join();
//	t3.join();
//	t4.join();

	printf("Exiting program\n");
	usleep(1000000);			//Sleep for 1000 ms

	return 0;
}


//Get keyboard input for testing
void KeyboardMon()
{
	bKeyboardMonitorRunning = true;
	bKillKeyboardInput = false;
	std::cout << "KeyboardMon Task Started" << std::endl;
	char input;
	while(!bKillKeyboardInput)
	{

		printf("KeyboardMonitor waiting for key entry!!!\n");

		scanf(" %c\n", &input);

		std::cout << "after scanf" << std::endl;

		printf("test %c\n", input);

		//input=getchar();
		if(input == 't') // Compare input to 'q' character
		{
			printf("test2 %c\n", input);
			bTCPSendDataToServerTEST = true;
		}

		else if(input == 'g') // Compare input to 'g' character
		{
			fprintf(stdout, "Key %c pressed!\n", input);
			bOkToContinue = true;
		}



		this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	std::cout << "KeyboardMon Task killed" << std::endl;
}








void BBBLTM_SerialComms()
{
	bool 	bSerialCommPortError = false;

	int 	iSerial_ReceiveBytesRead = 0,
			iSerial_ReceivedBytesBackup = 0;

	bKillThread_BBBLTM_SerialComms = false;
	bSerialCommsRunning = true;

	std::cout << "Serial Coms Task Started" << std::endl;

	// Open serial port
	iSerial_Ret=LS.Open(DEVICE_PORT,115200);                                        // Open serial link at 115200 bauds
	if (iSerial_Ret!=1) {                                                           // If an error occured...
		printf ("Error while opening port. Permission problem ? :  %d\n",iSerial_Ret);        // ... display a message ...
		bSerialCommPortError = true;                                                  // ... quit the application
	}
	else
		printf ("Serial port opened successfully !\n");

	if(!bSerialCommPortError)
	{
		while(!bKillThread_BBBLTM_SerialComms)
		{
			//std::cout << "Serial comms still running " << std::endl;



			 //Reset the resend timer (stop resending) as the data was received correctly
			 if(bSerial_SentDataReceivedByLTM)
			 {
				 printf("Received ACK from LTM stopping resend process\n");

				 bSerial_ControllerUpdatesSentToLTM 	= false;
				 bSerial_SentDataReceivedByLTM			= false;
			 }

			//Check the current elapsed time and resend if needed
			if(bSerial_ControllerUpdatesSentToLTM && !bSerial_SentDataReceivedByLTM)
			{
				Serial_CurrentTime = monotonic_clock::now();

				if((duration_cast<milliseconds>(Serial_CurrentTime - Serial_startTime).count()) >= iSerial_ResendDelay)
					bSerial_ReqToSendControllerUpdatesToLTM = true;
			}

			//Check for data that is ready to send
			if(bSerial_ReqToSendControllerUpdatesToLTM || bSerial_SendConnStatusRequest || bSerial_ReqToSendDataReceivedACK || bTest_EchoBackData)
			{
				std::cout << "Handling data to send" << std::endl;

				Func_SerialHandleAndSendData();
			}

			//Otherwise listen for incoming data!!!
			else
			{
				//std::cout << "before serialib Read" << std::endl;
				// Read an array from the serial device -returns number of bytes read

			//	iSerial_ReceiveBytesRead = LS.Read(SerialInBuffer,128,5000);                                // Read a maximum of 128 characters with a timeout of 5 seconds

				iSerial_ReceiveBytesRead=LS.ReadString(SerialInBuffer,'\n',128,1);
			//std::cout << "after serialib Read" << std::endl;

				// The final character of the string must be a line feed ('\n')
				if (iSerial_ReceiveBytesRead>0)
				{
					//Signal data received to start packet decode process
					bSerialData_ReceiveDecodeComplete = false;

					printf ("\n");
					printf ("iSerial_ReceiveBytesRead : %d\n",iSerial_ReceiveBytesRead);

					printf ("Hex read from serial port : ");
					for(int i = 0; i <=iSerial_ReceiveBytesRead; i++)
						printf ("%02X  ",SerialInBuffer[i]);


					if(Func_SerialHandleReceivedData(iSerial_ReceiveBytesRead))
					{
						printf ("Got here - bSerial_ReqToSendDataReceivedACK = true \n");
						bSerial_ReqToSendDataReceivedACK = true;
					}
				}

			}

			this_thread::sleep_for(std::chrono::milliseconds(20));
		}
	}

	// Close the connection with the device
	LS.Close();

	bSerialCommsRunning = false;

	std::cout << "Serial Coms Task Killed" << std::endl;

	std::terminate();
}



//************************************************************************************************************************//

//			** void Func_SerialHandleAndSendData(uint16_tiPortNumber) **

//************************************************************************************************************************//
//Notes: This function frames the new data to send

	// PACKET FRAMING FORMAT - total packet length minus flags len+4
	//U8 sflag;   //0x7e start of packet end of packet flag from HDLC
	//U8 cmd;     //what to do.
	//U8 len;     // payload length
	//U8 payload[len];  // could be zero len
	//U16 crc;
	//U8 eflag;   //end of frame flag


	// Received Data Command List:
	//=====================================================
	// 0x01 - Received status updates from another controller (includes Serial Comm connection status of the other controller  BBB handles timing for all controller connection statuses.
	// 0x02 - Received deviation alert
	// 0x03 - Received maintenance alert
	// 0x04 - Unused
	// .
	// .
	// 0x99 - Received ACK from last comm request to another controller

	// Send Data Command List:
	//=====================================================
	// 0x01 - Send status updates to local controller from other remote controller(s)
	// 0x02 - Send request for connection status information from local controller
	// 0x03 - Unused
	// 0x04 - Unused
	// 0x05 - Unused
	// .
	// .
	// 0x99 - Send data recieved ACK

void Func_SerialHandleAndSendData()
{
	std::cout << "Func_SerialHandleAndSendData running" << std::endl;
	bool	bSerial_SendControllerUpdatesToLTM	= false,
			bSerialData_SendCheckForConnectionStatus	= false,
			bSerialData_SendDataReceivedACK				= false;

	uint16_t	iSerial_SendPayloadLen = 0,
				iSerial_ThisControllerNum = 99,
				iSerial_SendCalculatedCRC = 0;

	char	Serial_SendPayloadBuffer[128];

		//Send status update from remote controller to local controller
		if(bSerial_ReqToSendControllerUpdatesToLTM)
		{
			chSerial_SendCommand = chTCPServer_ReceivedCommand;
			chSerial_SendPayloadLen = chTCPServer_ReceivedPayloadLen;
			iSerial_SendPayloadLen = chSerial_SendPayloadLen;

			for(uint16_t i = 0; i<=iSerial_SendPayloadLen; i++)
				Serial_SendPayloadBuffer[i] = TCPServer_ReceivedPayloadBuffer[i];

			bSerial_SendControllerUpdatesToLTM = true;
		}

		else if(bSerial_ReqToSendCheckForConnStatus)
		{
			chSerial_SendCommand = 0x02;
			iSerial_SendPayloadLen = 0;
			chSerial_SendPayloadLen = iSerial_SendPayloadLen & 0xFF;  //allows up to 255 bytes for payload length but we are currently limited by the Serial BufferIN and BufferOUT length of 128 bytes
			bSerialData_SendCheckForConnectionStatus = true;
		}

		else if(bSerial_ReqToSendDataReceivedACK)
		{
			chSerial_SendCommand = 0x99;

			iSerial_SendPayloadLen = 1;
			chSerial_SendPayloadLen = iSerial_SendPayloadLen & 0xFF;  //allows up to 255 bytes for payload length but we are currently limited by the Serial BufferIN and BufferOUT length of 128 bytes

			Serial_SendPayloadBuffer[0] = iSerial_ThisControllerNum & 0xFF;

			bSerialData_SendDataReceivedACK= true;
		}



		if(bSerial_SendControllerUpdatesToLTM || bSerialData_SendCheckForConnectionStatus || bSerialData_SendDataReceivedACK || bTest_EchoBackData)
		{

			//Frame Serial Data for sending from BBB to LTM

			//Set the start of Frame Flag
			SerialOutBuffer[0] = 0x7e;

			SerialOutBuffer[1] = chSerial_SendCommand;
			SerialOutBuffer[2] = chSerial_SendPayloadLen;

			//Format payload
			if(!bTestSerialEnabled)		//Send phase and i/o data received from other BBB
			{
				//Load the payload buffer if the length is greater than 0
				if(iSerial_SendPayloadLen > 0)
				{
				 for(uint16_t i = 1; i <= iSerial_SendPayloadLen; i++)
					 SerialOutBuffer[2+i] = Serial_SendPayloadBuffer[i-1];
				}

				//Create the crc
				//Func_Serial_MakeCRC(iSerial_SendPayloadLen);
				iSerial_SendCalculatedCRC = SerialComputeChecksum(reinterpret_cast<uint8_t*> (SerialOutBuffer), (iSerial_SendPayloadLen + 2));


				//Convert the unsigned int16 CRC to an unsigned char array
				SerialOutBuffer[2 + iSerial_SendPayloadLen + 1] = iSerial_SendCalculatedCRC & 0xFF;
				SerialOutBuffer[2 + iSerial_SendPayloadLen + 2] = iSerial_SendCalculatedCRC >> 8;

				//Set the end of Frame Flag
				SerialOutBuffer[iSerial_SendPayloadLen + 6] = 0x9e;
			}
			else 						//Controller test  -- echo back previous data
			{
				for(uint16_t i = 0; i<=(iSerial_ReceivedPayloadLen + 6); i++)
					SerialOutBuffer[i] = SerialInBuffer[iSerial_FrameStartCount + i];

				bTest_EchoBackData = false;
			}

			//Set bit to enable sending the data
			bSerialData_DataReadyToSend = true;
		}


		if(bSerialData_DataReadyToSend)
		{
			// Write the contents of SerialOutBuffer on the serial port
			iSerial_Ret=LS.Write(SerialOutBuffer, (iSerial_SendPayloadLen + 6));                                             // Send the command on the serial port
			if (iSerial_Ret!=1) {                                                           // If the writting operation failed ...
				printf ("Error while writing data\n");                              // ... display a message ...
	                                 // Send failure...maybe due to packet collision so trigger a read operation next cycle
			}
			else
			{
				bSerialData_DataReadyToSend = false;

				//Reset bits for next time

				if(bSerial_SendControllerUpdatesToLTM)
				{

					printf("Got here - bSerial_ReqToSendControllerUpdatesToLTM = false \n");
					printf("Got here - bSerial_SendControllerUpdatesToLTM = false \n");
					bSerial_ReqToSendControllerUpdatesToLTM				= false;
					bSerial_SendControllerUpdatesToLTM	= false;


					//Start the resend delay timer
					bSerial_ControllerUpdatesSentToLTM	= true;
					Serial_startTime = monotonic_clock::now();
				}

				 else if(bSerialData_SendCheckForConnectionStatus)
				 {
//								 printf("Got here - bSerial_ReqToSendCheckForConnStatus = false \n");
//								 printf("Got here - bSerialData_SendCheckForConnectionStatus = false \n");
					 bSerial_ReqToSendCheckForConnStatus		= false;
					 bSerialData_SendCheckForConnectionStatus				= false;
				 }

				 else if(bSerialData_SendDataReceivedACK)
				 {
					 printf("Got here - bSerial_ReqToSendDataReceivedACK = false \n");
					 printf("Got here - bSerialData_SendDataReceivedACK = false \n");
					 bSerial_ReqToSendDataReceivedACK			= false;
					 bSerialData_SendDataReceivedACK		= false;
				 }

				printf ("Write operation is successful \n");

			}

		}
		std::cout << "Func_SerialHandleAndSendData exiting" << std::endl;
}


//************************************************************************************************************************//

//										** uint16_t Func_SerialHandleReceivedData (uint16_t iNumBytesRead) **

//************************************************************************************************************************//
//Notes: This function decodes the received data and reports if the data is valid

// PACKET FRAMING FORMAT - total packet length minus flags len+4
				//U8 sflag;   //0x7e start of packet end of packet flag from HDLC
				//U8 cmd;     //what to do.
				//U8 len;     // payload length
				//U8 payload[len];  // could be zero len
				//U16 crc;
				//U8 eflag;   //end of frame flag


	// Received Data Command List:
	//=====================================================
	// 0x01 - Received status updates from another controller (includes Serial Comm connection status of the other controller  BBB handles timing for all controller connection statuses.
	// 0x02 - Received deviation alert
	// 0x03 - Received maintenance alert
	// 0x04 - Unused
	// .
	// .
	// 0x99 - Received ACK from last comm request to another controller

	// Send Data Command List:
	//=====================================================
	// 0x01 - Send status updates to local controller from other remote controller(s)
	// 0x02 - Send request for connection status information from local controller
	// 0x03 - Unused
	// 0x04 - Unused
	// 0x05 - Unused
	// .
	// .
	// 0x99 - Send data recieved ACK


uint16_t Func_SerialHandleReceivedData (uint16_t iNumBytesRead)
{

	uint16_t		iSerial_ReceivedCRC = 0,
					iSerial_FrameStartCount = 0,
					iSerial_Receive_CalculatedCRC = 0;


	//Data Received so lets check what we got
	for(uint16_t i = 0; i<=iNumBytesRead; i++)
	{
		//Find start of data packet
		if(SerialInBuffer[i] == 0x7e)
		{

			int iByteCount = 0;

			for(int iTempCount = 0; iTempCount <= iNumBytesRead; iTempCount++)
			{
				iByteCount++;

				if(SerialInBuffer[iTempCount+i] == 0x0A)
					break;
			}
			printf (" \n");
			printf ("Actual Bytes Received = %d \n", iByteCount);

			iSerial_FrameStartCount = i;
			chSerial_ReceivedCommand = SerialInBuffer[iSerial_FrameStartCount+1];
			chSerial_ReceivedPayloadLen = SerialInBuffer[iSerial_FrameStartCount+2];

			//Convert char to int
			iSerial_ReceivedPayloadLen = chSerial_ReceivedPayloadLen;

			if(iSerial_ReceivedPayloadLen > 0)
			{
				//load the payload buffer
				for(uint8_t iPayloadCount = 0; iPayloadCount <= iSerial_ReceivedPayloadLen; iPayloadCount++)
					Serial_ReceivedPayloadBuffer[iPayloadCount] = SerialInBuffer[iSerial_FrameStartCount + 2 + (iPayloadCount + 1)];
			}

			//load the crc
			for(uint8_t iCRCBufferCount = 0; iCRCBufferCount <= 1; iCRCBufferCount++)
			{
				Serial_ReceivedCRCBuffer[iCRCBufferCount] = SerialInBuffer[iSerial_FrameStartCount + 2 + iSerial_ReceivedPayloadLen + (iCRCBufferCount + 1)];
				printf ("Serial_ReceivedCRCBuffer[%d] = %02X \n", iCRCBufferCount, SerialInBuffer[iSerial_FrameStartCount + 2 + iSerial_ReceivedPayloadLen + (iCRCBufferCount + 1)]);
			}

			//Convert the unsigned char array to unsigned int16
			iSerial_ReceivedCRC = Serial_ReceivedCRCBuffer[0] | (uint16_t(Serial_ReceivedCRCBuffer[1]) <<8);


			printf ("SerialInBuffer[%d] = %02X \n", i, SerialInBuffer[i]);
			printf ("chSerial_ReceivedCommand = %02X \n", chSerial_ReceivedCommand);
			printf ("chSerial_ReceivedPayloadLen = %d \n", iSerial_ReceivedPayloadLen);

			for(uint8_t iPayloadCount = 0; iPayloadCount <= (iSerial_ReceivedPayloadLen - 1); iPayloadCount++)
				printf ("Serial_ReceivedPayloadBuffer[%d] = %02X \n", iPayloadCount, Serial_ReceivedPayloadBuffer[iPayloadCount]);

			printf ("iSerial_ReceivedCRC = %d \n", iSerial_ReceivedCRC);


			// Received Data Command List:
				//=====================================================
				// 0x01 - Received status updates from another controller (includes Serial Comm connection status of the other controller  BBB handles timing for all controller connection statuses.
				// 0x02 - Received deviation alert
				// 0x03 - Received maintenance alert
				// 0x04 - Unused
				// .
				// .
				// 0x99 - Received ACK from last comm request to another controller


			iSerial_Receive_CalculatedCRC = SerialComputeChecksum(reinterpret_cast<uint8_t*> (&SerialInBuffer[iSerial_FrameStartCount]), (iSerial_ReceivedPayloadLen + 2));

			//Check CRC
			if((iSerial_ReceivedCRC == iSerial_Receive_CalculatedCRC) && (iByteCount == iNumBytesRead))
			{
				printf ("CRC Check PASSED!! \n");
				bSerialData_ReceiveDecodeComplete = true;
				bSerial_ReqToSendDataReceivedACK = true;


				//Determine payload action
				if(chSerial_ReceivedCommand == 0X01)		//Send Controller phase and i/o data
				{
					iSerial_ReceivedLocalControllerNumber = Serial_ReceivedPayloadBuffer[0];
					iSerial_ReceivedLocalControllerPhase = Serial_ReceivedPayloadBuffer[1];
					iSerial_ReceivedLocalControllerRecipeNum = Serial_ReceivedPayloadBuffer[2];
					iSerial_ReceivedLocalControllerBasketNum = Serial_ReceivedPayloadBuffer[3];

					if(bTestSerialEnabled)
					{
						printf ("Echoing data back to LTM!! \n");
						bTest_EchoBackData = true;
					}
					else
						bSerial_RequestToSendDataToClient = true;
					//Serial_ReceivedPayloadBuffer[1 to iSerial_ReceivedPayloadLen] == i/o values;
				}

				else if(chSerial_ReceivedCommand == 0X03)		//Send Deviation Email Alert
					bEmail_SendDeviationAlert = true;

				else if(chSerial_ReceivedCommand == 0X04)		//Send Maintenance Email Alert
					bEmail_SendMaintenanceAlert = true;

				else if(chSerial_ReceivedCommand == 0X99)		//Data Received ACK
					bSerial_SentDataReceivedByLTM = true;

				else											//Do nothing with the data as the command is invalid
					return 0;

				return 1;
			}
			else
			{
				if(iByteCount != iNumBytesRead)
				{
					printf ("Length does not match!! \n");
					printf ("iByteCount used = %d \n", iByteCount);
					printf ("iSerial_ReceiveBytesRead used = %d \n", iNumBytesRead);
				}

				if(iSerial_ReceivedCRC != iSerial_Receive_CalculatedCRC)
				{
					printf ("CRC Check FAILED!! \n");
					printf ("iSerial_ReceivedCRC = %d \n", iSerial_ReceivedCRC);
					printf ("Calculated CRC = %d \n", iSerial_Receive_CalculatedCRC);
				}

				return 0;
			}
		}
	}
	//Oh Snap! We didnt detect a start of packet bit
	return 0;
}



//************************************************************************************************************************//

//								** static uint16_t SerialComputeChecksum(uint8_t* data_p, uint16_t length) **

//************************************************************************************************************************//
//Notes: This function creates a CRC

static uint16_t SerialComputeChecksum(uint8_t* data_p, uint16_t length){
	uint8_t x;
	uint16_t crc = 0xFFFF;

   while (length--){

	   printf ("length = %u \n", length);

//	   printf ("*data_p++ = %u \n", *data_p);


       x = crc >> 8 ^ *data_p++;

//     printf ("csX1 = %u \n", x);
       printf ("crc >> 8 = %u \n", crc >> 8);

       printf ("x>>4 = %u \n", x>>4);


       x ^= x>>4;


//       printf ("csX2 = %u \n", x);

       printf ("crc << 8 = %u \n", crc << 8);

       printf ("x << 12 = %u \n", x << 12);


       crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);

       printf ("CRC = %u \n", crc);
   }
   return crc;
}



void BBBLTM_TCPServer()
{
	int iTCPServer_ReceiveBytesRead = 0;

	bool 	bTCPServerPortError = true;

	bKillThread_BBBLTM_TCPServerComms = false;
	bTCPServerCommsRunning = true;

	std::cout << "BBBLTM_TCPServer Task Started" << std::endl;

	 int TCPServer_sockfd, TCPServer_newsockfd, portno;
	 socklen_t clilen;
	 char buffer[256];
	 struct sockaddr_in serv_addr, cli_addr;
	 int n,yes = 1;

	 struct timeval tv;
	 tv.tv_sec = 3;   /*2 sec timeout*/
	 tv.tv_usec = 0;  // must be initialized


	 while(bTCPServerPortError && !bKillThread_BBBLTM_TCPServerComms)
	 {
		 bTCPServerPortError = false;

		 TCPServer_sockfd = socket(AF_INET, SOCK_STREAM, 0);
		 if (TCPServer_sockfd < 0)
		 {
			printf("ERROR opening socket. \n");
			bTCPServerPortError = true;
		 }

	//	 if(setsockopt(TCPServer_sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) <0)
		//	 printf("\nERROR setting socket options due to : [%s]\n",(char*)strerror(errno));

		 bzero((char *) &serv_addr, sizeof(serv_addr));
		 portno = atoi (SOCKPORTNUM);
		 serv_addr.sin_family = AF_INET;
		 serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
		 serv_addr.sin_port = htons(portno);

		 if (::bind(TCPServer_sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) <  0)
		 {
			 printf("\nERROR on binding due to : [%s]\n",(char*)strerror(errno));
			 printf("\nPort number : %d\n",portno);
			// printf("ERROR on binding. \n");
			 bTCPServerPortError = true;
		 }
		 if(!bTCPServerPortError)
		 {
			 listen(TCPServer_sockfd,5);
			 clilen = sizeof(cli_addr);
			 TCPServer_newsockfd = accept(TCPServer_sockfd,
						 (struct sockaddr *) &cli_addr,
						 &clilen);
			 if (TCPServer_newsockfd < 0)
			 {
				 printf("ERROR on accept. \n");
				 bTCPServerPortError = true;
			 }
		 }
		 if(bTCPServerPortError)
		 {
			 printf("Closing socket and port for retry \n");
			 close(TCPServer_newsockfd);
			 close(TCPServer_sockfd);
			 usleep(1000000);			//Sleep for 1000 ms
		 }
	 }

	 if(!bTCPServerPortError)
	 {
		 while(!bKillThread_BBBLTM_TCPServerComms)
		 {
			 //std::cout << "TCP comms still running " << std::endl;

			//Check for data that is ready to send
			if(bTCPServer_ReqToSendDataReceivedACK)
			{
				std::cout << "Handling data to send ACK" << std::endl;

				Func_TCPServerHandleAndSendData(TCPServer_newsockfd);

				//Wait to review data before continuing
				printf("Waiting to continue... Enter 'ctrl-g' and press enter\n");
				while(!bOkToContinue)
					usleep(1000000);			//Sleep for 1000 ms

				bOkToContinue = false;
				printf("After bOkToContinue loop\n");
			}

			//Otherwise listen for incoming data!!!
			else
			{
				printf ("listening for data \n");

				// Read an array from the serial device -returns number of bytes read
				//bzero(TCPServer_InBuffer,128);
				iTCPServer_ReceiveBytesRead = read(TCPServer_newsockfd,TCPServer_InBuffer,127);


				printf ("after the read");
				// The final character of the string must be a line feed ('\n')
				if (iTCPServer_ReceiveBytesRead>0)
				//((iTCPServer_ReceiveBytesRead = read(TCPServer_newsockfd,TCPServer_InBuffer,127))>0)
				{
					//Signal data received to start packet decode process
					bTCPServer_ReceiveDecodeComplete = false;

					printf ("\n");
					printf ("iTCPServer_ReceiveBytesRead : %d\n",iTCPServer_ReceiveBytesRead);

					printf ("String read from TCP port : ");
					printf ("%s \n",TCPServer_InBuffer);

					printf ("Hex read from TCP port : ");
					for(int i = 0; i <=iTCPServer_ReceiveBytesRead; i++)
						printf ("%02X  ",TCPServer_InBuffer[i]);

					printf ("char print out\n");
					for(int i = 0; i <=iTCPServer_ReceiveBytesRead; i++)
							printf ("%c  ",TCPServer_InBuffer[i]);

					if(Func_TCPServerHandleReceivedData(iTCPServer_ReceiveBytesRead))
					{
						printf ("Got here - bTCPServer_ReqToSendDataReceivedACK = true \n");
						bTCPServer_ReqToSendDataReceivedACK = true;
					}
				}

				if (iTCPServer_ReceiveBytesRead < 0) printf("\nERROR reading from socket due to : [%s]\n",(char*)strerror(errno));

				if(iTCPServer_ReceiveBytesRead == 0)
				{
					for(int i = 0; i <=iTCPServer_ReceiveBytesRead; i++)
						TCPServer_InBuffer[i] = '\0';

				}


			}

			this_thread::sleep_for(std::chrono::milliseconds(2));

		 }
	 }

	 close(TCPServer_newsockfd);
	 usleep(1000000);			//Sleep for 1000 ms
	 close(TCPServer_sockfd);
	 usleep(1000000);			//Sleep for 1000 ms



	bTCPServerCommsRunning = false;

	std::cout << "TCP Server Coms Task Killed" << std::endl;

	std::terminate();
}


//************************************************************************************************************************//

//			** void Func_TCPServerHandleAndSendData(uint16_tiPortNumber) **

//************************************************************************************************************************//
//Notes: This function frames the new data to send

	// PACKET FRAMING FORMAT - total packet length minus flags len+4
	//U8 sflag;   //0x7e start of packet end of packet flag from HDLC
	//U8 cmd;     //what to do.
	//U8 len;     // payload length
	//U8 payload[len];  // could be zero len
	//U16 crc;
	//U8 eflag;   //end of frame flag


	// Received Data Command List:
	//=====================================================
	// 0x01 - Received status updates from another controller (includes Serial Comm connection status of the other controller  BBB handles timing for all controller connection statuses.
	// 0x02 - Received deviation alert
	// 0x03 - Received maintenance alert
	// 0x04 - Unused
	// .
	// .
	// 0x99 - Received ACK from last comm request to another controller

	// Send Data Command List:
	//=====================================================
	// 0x01 - Send status updates to local controller from other remote controller(s)
	// 0x02 - Send request for connection status information from local controller
	// 0x03 - Unused
	// 0x04 - Unused
	// 0x05 - Unused
	// .
	// .
	// 0x99 - Send data recieved ACK

void Func_TCPServerHandleAndSendData(int TCPServer_newsockfd)
{
	std::cout << "Func_TCPServerHandleAndSendData running" << std::endl;
	bool	TCPServer_SendDataReceivedACK				= false;

	char	TCPServer_SendPayloadBuffer[128];

	uint16_t	iTCPServer_SendPayloadLen = 0,
				iTCPServer_ThisControllerNum = 22,
				iTCPServer_SendCalculatedCRC = 0;

		if(bTCPServer_ReqToSendDataReceivedACK)
		{
			chTCPServer_SendCommand = 0x99;

			iTCPServer_SendPayloadLen = 1;
			chTCPServer_SendPayloadLen = iTCPServer_SendPayloadLen & 0xFF;  //allows up to 255 bytes for payload length but we are currently limited by the Serial BufferIN and BufferOUT length of 128 bytes

			TCPServer_SendPayloadBuffer[0] = iTCPServer_ThisControllerNum & 0xFF;

			TCPServer_SendDataReceivedACK= true;
		}



		if(TCPServer_SendDataReceivedACK)
		{
			//Frame Serial Data for sending from BBB to LTM

			//Set the start of Frame Flag
			TCPServer_OutBuffer[0] = 0x7e;

			TCPServer_OutBuffer[1] = chTCPServer_SendCommand;
			TCPServer_OutBuffer[2] = chTCPServer_SendPayloadLen;

			//Format payload

			//Load the payload buffer if the length is greater than 0
			if(iTCPServer_SendPayloadLen > 0)
			{
			 for(uint16_t i = 1; i <= iTCPServer_SendPayloadLen; i++)
				 TCPServer_OutBuffer[2+i] = TCPServer_SendPayloadBuffer[i-1];

			 	 printf ("TCPServer_SendPayloadBuffer[0] = %02X  ",TCPServer_SendPayloadBuffer[0]);
			 	 printf ("TCPServer_OutBuffer[3] = %02X  ",TCPServer_OutBuffer[3]);
			}

			//Create the crc
			//Func_Serial_MakeCRC(iTCPServer_SendPayloadLen);
			iTCPServer_SendCalculatedCRC = TCPServerComputeChecksum(reinterpret_cast<uint8_t*> (TCPServer_OutBuffer), (iTCPServer_SendPayloadLen + 2));


			//Convert the unsigned int16 CRC to an unsigned char array
			TCPServer_OutBuffer[2 + iTCPServer_SendPayloadLen + 1] = iTCPServer_SendCalculatedCRC & 0xFF;
			TCPServer_OutBuffer[2 + iTCPServer_SendPayloadLen + 2] = iTCPServer_SendCalculatedCRC >> 8;

			//Set the end of Frame Flag
			TCPServer_OutBuffer[iTCPServer_SendPayloadLen + 5] = 0x9e;

			TCPServer_OutBuffer[iTCPServer_SendPayloadLen + 6] = '\n';
			TCPServer_OutBuffer[iTCPServer_SendPayloadLen + 7] = '\0';

			//Set bit to enable sending the data
			bTCPServer_DataReadyToSend = true;
		}


		if(bTCPServer_DataReadyToSend)
		{

			printf ("sending the following : ");
			for(int i = 0; i <=(iTCPServer_SendPayloadLen + 5); i++)
					printf ("%02X  ",TCPServer_OutBuffer[i]);


		//	iTCPClient_Ret = write(sockfd,TCPClient_OutBuffer,(iTCPClient_SendPayloadLen + 7));// Send the command on the TCPClient port

			// Write the contents of TCPServer_OutBuffer on the TCP port
			iTCPServer_Ret = write(TCPServer_newsockfd,TCPServer_OutBuffer,(iTCPServer_SendPayloadLen + 7));

			//Display a message if the write operation fails
			if (iTCPServer_Ret <= 0)
				printf("\nError while writing data due to : [%s]\n",(char*)strerror(errno));

			else
			{
				bTCPServer_DataReadyToSend = false;

				//Reset bits for next time

				 if(TCPServer_SendDataReceivedACK)
				 {
					 printf("Got here - bTCPServer_ReqToSendDataReceivedACK = false \n");
					 printf("Got here - TCPServer_SendDataReceivedACK = false \n");
					 bTCPServer_ReqToSendDataReceivedACK			= false;
					 TCPServer_SendDataReceivedACK		= false;
				 }

				printf ("Write operation is successful \n");

			}

		}
		std::cout << "Func_TCPServerHandleAndSendData exiting" << std::endl;
}

//************************************************************************************************************************//

//										** uint16_t Func_TCPServerHandleReceivedData (uint16_t iNumBytesRead) **

//************************************************************************************************************************//
//Notes: This function decodes the received data and reports if the data is valid

// PACKET FRAMING FORMAT - total packet length minus flags len+4
				//U8 sflag;   //0x7e start of packet end of packet flag from HDLC
				//U8 cmd;     //what to do.
				//U8 len;     // payload length
				//U8 payload[len];  // could be zero len
				//U16 crc;
				//U8 eflag;   //end of frame flag


	// Received Data Command List:
	//=====================================================
	// 0x01 - Received status updates from another controller (includes Serial Comm connection status of the other controller  BBB handles timing for all controller connection statuses.
	// 0x02 - Received deviation alert
	// 0x03 - Received maintenance alert
	// 0x04 - Unused
	// .
	// .
	// 0x99 - Received ACK from last comm request to another controller

	// Send Data Command List:
	//=====================================================
	// 0x01 - Send status updates to local controller from other remote controller(s)
	// 0x02 - Send request for connection status information from local controller
	// 0x03 - Unused
	// 0x04 - Unused
	// 0x05 - Unused
	// .
	// .
	// 0x99 - Send data recieved ACK


uint16_t Func_TCPServerHandleReceivedData (uint16_t iNumBytesRead)
{
	uint16_t		iTCPServer_ReceivedCRC = 0,
					iTCPServer_Receive_CalculatedCRC = 0;


	//Data Received so lets check what we got
	for(uint16_t i = 0; i<=iNumBytesRead; i++)
	{
		//Find start of data packet
		if(TCPServer_InBuffer[i] == 0x7e)
		{
			int iByteCount = 0;

			for(int iTempCount = 0; iTempCount <= iNumBytesRead; iTempCount++)
			{
				iByteCount++;

				if(TCPServer_InBuffer[iTempCount+i] == 0x0A)
					break;
			}
			printf (" \n");
			printf ("Actual Bytes Received = %d \n", iByteCount);

			iTCPServer_FrameStartCount = i;
			chTCPServer_ReceivedCommand = TCPServer_InBuffer[iTCPServer_FrameStartCount+1];
			chTCPServer_ReceivedPayloadLen = TCPServer_InBuffer[iTCPServer_FrameStartCount+2];

			//Convert char to int
			iTCPServer_ReceivedPayloadLen = chTCPServer_ReceivedPayloadLen;

			if(iTCPServer_ReceivedPayloadLen > 0)
			{
				//load the payload buffer
				for(uint8_t iPayloadCount = 0; iPayloadCount <= iTCPServer_ReceivedPayloadLen; iPayloadCount++)
					TCPServer_ReceivedPayloadBuffer[iPayloadCount] = TCPServer_InBuffer[iTCPServer_FrameStartCount + 2 + (iPayloadCount + 1)];
			}

			//load the crc
			for(uint8_t iCRCBufferCount = 0; iCRCBufferCount <= 1; iCRCBufferCount++)
			{
				TCPServer_ReceivedCRCBuffer[iCRCBufferCount] = TCPServer_InBuffer[iTCPServer_FrameStartCount + 2 + iTCPServer_ReceivedPayloadLen + (iCRCBufferCount + 1)];
				printf ("TCPServer_ReceivedCRCBuffer[%d] = %02X \n", iCRCBufferCount, TCPServer_InBuffer[iTCPServer_FrameStartCount + 2 + iTCPServer_ReceivedPayloadLen + (iCRCBufferCount + 1)]);
			}

			//Convert the unsigned char array to unsigned int16
			iTCPServer_ReceivedCRC = TCPServer_ReceivedCRCBuffer[0] | (uint16_t(TCPServer_ReceivedCRCBuffer[1]) <<8);


			printf ("TCPServer_InBuffer[%d] = %02X \n", i, TCPServer_InBuffer[i]);
			printf ("chTCPServer_ReceivedCommand = %02X \n", chTCPServer_ReceivedCommand);
			printf ("chTCPServer_ReceivedPayloadLen = %d \n", iTCPServer_ReceivedPayloadLen);

			for(uint8_t iPayloadCount = 0; iPayloadCount <= (iTCPServer_ReceivedPayloadLen - 1); iPayloadCount++)
				printf ("TCPServer_ReceivedPayloadBuffer[%d] = %02X \n", iPayloadCount, TCPServer_ReceivedPayloadBuffer[iPayloadCount]);

			printf ("iTCPServer_ReceivedCRC = %d \n", iTCPServer_ReceivedCRC);


			// Received Data Command List:
				//=====================================================
				// 0x01 - Received status updates from another controller (includes Serial Comm connection status of the other controller  BBB handles timing for all controller connection statuses.
				// 0x02 - Received deviation alert
				// 0x03 - Received maintenance alert
				// 0x04 - Unused
				// .
				// .
				// 0x99 - Received ACK from last comm request to another controller


			iTCPServer_Receive_CalculatedCRC = TCPServerComputeChecksum(reinterpret_cast<uint8_t*> (&TCPServer_InBuffer[iTCPServer_FrameStartCount]), (iTCPServer_ReceivedPayloadLen + 2));

			//Check CRC
			if((iTCPServer_ReceivedCRC == iTCPServer_Receive_CalculatedCRC) && (iByteCount == iNumBytesRead))
			{
				printf ("CRC Check PASSED!! \n");
				bTCPServer_ReceiveDecodeComplete = true;
				bTCPServer_ReqToSendDataReceivedACK = true;


				//Determine payload action
				if(chTCPServer_ReceivedCommand == 0X01)		//Send Controller phase and i/o data
				{
					iTCPServer_ReceivedLocalControllerNumber = TCPServer_ReceivedPayloadBuffer[0];
					iTCPServer_ReceivedLocalControllerPhase = TCPServer_ReceivedPayloadBuffer[1];
					iTCPServer_ReceivedLocalControllerRecipeNum = TCPServer_ReceivedPayloadBuffer[2];
					iTCPServer_ReceivedLocalControllerBasketNum = TCPServer_ReceivedPayloadBuffer[3];

					printf ("iTCPServer_ReceivedLocalControllerNumber = %d \n", iTCPServer_ReceivedLocalControllerNumber);
					printf ("iTCPServer_ReceivedLocalControllerPhase = %d \n", iTCPServer_ReceivedLocalControllerPhase);
					printf ("iTCPServer_ReceivedLocalControllerRecipeNum  = %d \n", iTCPServer_ReceivedLocalControllerRecipeNum);
					printf ("iTCPServer_ReceivedLocalControllerBasketNum  = %d \n", iTCPServer_ReceivedLocalControllerBasketNum);
					bSerial_ReqToSendControllerUpdatesToLTM = true;
					//TCPServer_ReceivedPayloadBuffer[1 to iTCPServer_ReceivedPayloadLen] == i/o values;
				}

				else if(chTCPServer_ReceivedCommand == 0X99)		//Data Received ACK
					bTCPServer_SentDataReceivedByRemoteBBB = true;

				else											//Do nothing with the data as the command is invalid
					return 0;

				return 1;
			}
			else
			{
				if(iByteCount != iNumBytesRead)
				{
					printf ("Length does not match!! \n");
					printf ("iByteCount used = %d \n", iByteCount);
					printf ("iTCPServer_ReceiveBytesRead used = %d \n", iNumBytesRead);
				}

				if(iTCPServer_ReceivedCRC != iTCPServer_Receive_CalculatedCRC)
				{
					printf ("CRC Check FAILED!! \n");
					printf ("iTCPServer_ReceivedCRC = %d \n", iTCPServer_ReceivedCRC);
					printf ("Calculated CRC = %d \n", iTCPServer_Receive_CalculatedCRC);
				}

				return 0;
			}
		}
	}
	//Oh Snap! We didnt detect a start of packet bit
	return 0;
}


//************************************************************************************************************************//

//								** static uint16_t TCPComputeChecksum(uint8_t* data_p, uint16_t length) **

//************************************************************************************************************************//
//Notes: This function creates a CRC

static uint16_t TCPServerComputeChecksum(uint8_t* data_p, uint16_t length){
	uint8_t x;
	uint16_t crc = 0xFFFF;

   while (length--){

//	   printf ("length = %u \n", length);
//	   printf ("*data_p++ = %u \n", *data_p);


       x = crc >> 8 ^ *data_p++;

//       printf ("csX1 = %u \n", x);
//       printf ("crc >> 8 = %u \n", crc >> 8);
//       printf ("x>>4 = %u \n", x>>4);


       x ^= x>>4;


//       printf ("csX2 = %u \n", x);
//       printf ("crc << 8 = %u \n", crc << 8);
//       printf ("x << 12 = %u \n", x << 12);


       crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);

 //      printf ("CRC = %u \n", crc);
   }
   return crc;
}










void BBBLTM_TCPClient()
{
	std::cout << "BBBLTM_TCPClient Task Started" << std::endl;

	int iTCPClient_ReceiveBytesRead = 0;

	bool 	bTCPClientPortError = false,
			bServerConnected = false,
			bTCPClient_ListenForACK = false;

	bKillThread_BBBLTM_TCPClientComms = false;
	bTCPClientCommsRunning = true;


	int sockfd, portno, n, flag = 1;
	struct sockaddr_in serv_addr;
	struct hostent *server;

	struct timeval tv;
	tv.tv_sec = 2;   /*2 sec timeout*/
	tv.tv_usec = 0;  // must be initialized


	portno = atoi(SOCKPORTNUM);
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	printf("Port # = %d \n",portno);

	if (sockfd < 0)
	{
		printf("ERROR opening socket\n");
		bTCPClientPortError = true;
	}
	server = gethostbyname(SOCKSERVADD);
	if (server == NULL) {
		fprintf(stderr,"ERROR, no such host\n");
		bTCPClientPortError = true;
	}


	while(!bServerConnected)
	{
		bzero((char *) &serv_addr, sizeof(serv_addr));
		serv_addr.sin_family = AF_INET;
		bcopy((char *)server->h_addr,
			 (char *)&serv_addr.sin_addr.s_addr,
			 server->h_length);
		serv_addr.sin_port = htons(portno);
		if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
		{
			printf("\nCannot connect to server due to : [%s]\n",(char*)strerror(errno));
		}
		else{
			bServerConnected = true;
			printf("Connected to server\n");
		}

		usleep(1000000);			//Sleep for 1000 ms
	}

	 if(!bTCPClientPortError)
	 {


		 while(!bKillThread_BBBLTM_TCPClientComms)
		 {
			// printf("Client running\n");

			 //std::cout << "TCP comms still running " << std::endl;

			 //Reset the resend timer (stop resending) as the data was received correctly
			 if(bTCPClient_SentDataReceivedByServer)
			 {
				 printf("Received ACK from remote device stopping resend process\n");

				 bTCPClient_ControllerUpdatesSentToServer 	= false;
				 bTCPSentDataToServerTEST					= false;
				 bTCPClient_SentDataReceivedByServer		= false;
			 }

			 //Check the current elapsed time and resend if needed
			 if(bTCPClient_ControllerUpdatesSentToServer && !bTCPClient_SentDataReceivedByServer)
			 {
				 printf("Sending updates to Server\n");

				 TCPClient_CurrentTime = monotonic_clock::now();

				 if((duration_cast<milliseconds>(TCPClient_CurrentTime - TCPClient_startTime).count()) >= iTCPClient_ResendDelay)
					 bTCPClient_ReqToSendControllerUpdatesToServer = true;
			 }

			 //Check the current elapsed time and resend if needed
			 if(bTCPSentDataToServerTEST && !bTCPClient_SentDataReceivedByServer)
			 {
				 TCPClient_TestCurrentTime = monotonic_clock::now();

				 if((duration_cast<milliseconds>(TCPClient_TestCurrentTime - TCPClient_TeststartTime).count()) >= iTCPClient_ResendDelay)
				 {
					 std::cout << "Resending Test Data" << std::endl;

					 bTCPSendDataToServerTEST = true;
				 }
			 }

			//Check for data that is ready to send
			if(bTCPClient_ReqToSendControllerUpdatesToServer || bTCPClient_SendConnStatusRequest || bTCPClient_ReqToSendDataReceivedACK || bTCPSendDataToServerTEST)
			{
				if(bTCPSendDataToServerTEST)
					std::cout << "sending test data" << std::endl;

				Func_TCPClientHandleAndSendData(sockfd);
				bTCPClient_ListenForACK = true;
			}

			//Otherwise listen for incoming data!!!
			if(bTCPClient_ListenForACK)
			{
				printf("listening for data\n");
				// Read an array from the serial device -returns number of bytes read
				bzero(TCPClient_InBuffer,128);
				iTCPClient_ReceiveBytesRead = read(sockfd,TCPClient_InBuffer,127);
				printf("after read\n");
				if (iTCPClient_ReceiveBytesRead < 0) printf("ERROR reading from socket. \n");

				// The final character of the string must be a line feed ('\n')
				if (iTCPClient_ReceiveBytesRead>0)
				{
					//Signal data received to start packet decode process
					bTCPClient_ReceiveDecodeComplete = false;

					printf ("\n");
					printf ("iTCPClient_ReceiveBytesRead : %d\n",iTCPClient_ReceiveBytesRead);

					printf ("Hex read from TCP port : ");
					for(int i = 0; i <=iTCPClient_ReceiveBytesRead; i++)
						printf ("%02X  ",TCPClient_InBuffer[i]);

					if(Func_TCPClientHandleReceivedData(iTCPClient_ReceiveBytesRead))
					{
						printf ("Got here - bTCPClient_ReqToSendDataReceivedACK = true \n");
						//bTCPClient_ReqToSendDataReceivedACK = true;
					}
				}

				bTCPClient_ListenForACK = false;

			}

			this_thread::sleep_for(std::chrono::milliseconds(20));

		 }
	 }

	 close(sockfd);

	 usleep(1000000);			//Sleep for 1000 ms

	bTCPClientCommsRunning = false;

	std::cout << "TCP Client Coms Task Killed" << std::endl;

	std::terminate();
}



//************************************************************************************************************************//

//			** void Func_TCPClientHandleAndSendData(uint16_tiPortNumber) **

//************************************************************************************************************************//
//Notes: This function frames the new data to send

	// PACKET FRAMING FORMAT - total packet length minus flags len+4
	//U8 sflag;   //0x7e start of packet end of packet flag from HDLC
	//U8 cmd;     //what to do.
	//U8 len;     // payload length
	//U8 payload[len];  // could be zero len
	//U16 crc;
	//U8 eflag;   //end of frame flag


	// Received Data Command List:
	//=====================================================
	// 0x01 - Received status updates from another controller (includes TCPClient Comm connection status of the other controller  BBB handles timing for all controller connection statuses.
	// 0x02 - Received deviation alert
	// 0x03 - Received maintenance alert
	// 0x04 - Unused
	// .
	// .
	// 0x99 - Received ACK from last comm request to another controller

	// Send Data Command List:
	//=====================================================
	// 0x01 - Send status updates to local controller from other remote controller(s)
	// 0x02 - Send request for connection status information from local controller
	// 0x03 - Unused
	// 0x04 - Unused
	// 0x05 - Unused
	// .
	// .
	// 0x99 - Send data recieved ACK

void Func_TCPClientHandleAndSendData(int sockfd)
{
	std::cout << "Func_TCPClientHandleAndSendData running" << std::endl;
	bool	bTCPClient_SendControllerUpdatesToServer	= false,
			bTCPClient_SendCheckForConnectionStatus		= false,
			bTCPClient_SendDataReceivedACK				= false,
			bTCPClient_SendTestDataToServer 			= false;


	uint16_t	iTCPClient_SendPayloadLen = 0,
				iTCPClient_ThisControllerNum = 99,
				iTCPClient_SendCalculatedCRC = 0;

	char	TCPClient_SendPayloadBuffer[128];

		//Send status update from remote controller to local controller
		if(bTCPClient_ReqToSendControllerUpdatesToServer)
		{
			chTCPClient_SendCommand = chTCPClient_ReceivedCommand;
			chTCPClient_SendPayloadLen = chTCPClient_ReceivedPayloadLen;
			iTCPClient_SendPayloadLen = chTCPClient_SendPayloadLen;

			for(uint16_t i = 0; i<=iTCPClient_SendPayloadLen; i++)
				TCPClient_SendPayloadBuffer[i] = TCPClient_ReceivedPayloadBuffer[i];

			bTCPClient_SendControllerUpdatesToServer = true;
		}

		else if(bTCPClient_ReqToSendCheckForConnStatus)
		{
			chTCPClient_SendCommand = 0x02;
			iTCPClient_SendPayloadLen = 0;
			chTCPClient_SendPayloadLen = iTCPClient_SendPayloadLen & 0xFF;  //allows up to 255 bytes for payload length but we are currently limited by the TCPClient BufferIN and BufferOUT length of 128 bytes
			bTCPClient_SendCheckForConnectionStatus = true;
		}

		else if(bTCPClient_ReqToSendDataReceivedACK)
		{
			chTCPClient_SendCommand = 0x99;

			iTCPClient_SendPayloadLen = 1;
			chTCPClient_SendPayloadLen = iTCPClient_SendPayloadLen & 0xFF;  //allows up to 255 bytes for payload length but we are currently limited by the TCPClient BufferIN and BufferOUT length of 128 bytes

			TCPClient_SendPayloadBuffer[0] = iTCPClient_ThisControllerNum & 0xFF;

			bTCPClient_SendDataReceivedACK= true;
		}
		else if(bTCPSendDataToServerTEST)
		{
			printf ("Got here - framing test packet\n");
			chTCPClient_SendCommand = 0x01;
			chTCPClient_SendPayloadLen = 0x04;
			iTCPClient_SendPayloadLen = chTCPClient_SendPayloadLen;
			TCPClient_SendPayloadBuffer[0] = 0x01;
			TCPClient_SendPayloadBuffer[1] = 0x02;
			TCPClient_SendPayloadBuffer[2] = 0x03;
			TCPClient_SendPayloadBuffer[3] = 0x04;

//			for(uint16_t i = 0; i<=iTCPClient_SendPayloadLen; i++)
//				TCPClient_SendPayloadBuffer[i] = TCPClient_ReceivedPayloadBuffer[i];

			bTCPClient_SendTestDataToServer = true;
		}



		if(bTCPClient_SendControllerUpdatesToServer || bTCPClient_SendCheckForConnectionStatus || bTCPClient_SendDataReceivedACK || bTCPClient_SendTestDataToServer)
		{

			printf ("Got here - calc crc\n");
			//Frame TCPClient Data for sending from BBB to LTM


			//Set the start of Frame Flag
			TCPClient_OutBuffer[0] = 0x7e;

			TCPClient_OutBuffer[1] = chTCPClient_SendCommand;
			TCPClient_OutBuffer[2] = chTCPClient_SendPayloadLen;

			//Load the payload buffer if the length is greater than 0
			if(iTCPClient_SendPayloadLen > 0)
			{
			 for(uint16_t i = 1; i <= iTCPClient_SendPayloadLen; i++)
				 TCPClient_OutBuffer[2+i] = TCPClient_SendPayloadBuffer[i-1];
			}

			//Create the crc
			//Func_TCPClient_MakeCRC(iTCPClient_SendPayloadLen);
			iTCPClient_SendCalculatedCRC = TCPClientComputeChecksum(reinterpret_cast<uint8_t*> (TCPClient_OutBuffer), (iTCPClient_SendPayloadLen + 2));

			//Convert the unsigned int16 CRC to an unsigned char array
			TCPClient_OutBuffer[2 + iTCPClient_SendPayloadLen + 1] = iTCPClient_SendCalculatedCRC & 0xFF;
			TCPClient_OutBuffer[2 + iTCPClient_SendPayloadLen + 2] = iTCPClient_SendCalculatedCRC >> 8;

			//Set the end of Frame Flag
			TCPClient_OutBuffer[iTCPClient_SendPayloadLen + 5] = 0x9e;

			printf ("sending the following : ");
			for(int i = 0; i <=(iTCPClient_SendPayloadLen + 5); i++)
					printf ("%02X  ",TCPClient_OutBuffer[i]);

			//Set bit to enable sending the data
			bTCPClient_DataReadyToSend = true;
		}


		if(bTCPClient_DataReadyToSend)
		{
			printf ("Got here - data ready to send\n");


			if(bTCPClient_SendTestDataToServer)
			{
				printf ("overwriting with test data\n");
/*
				TCPClient_OutBuffer[0] = 'a';
				TCPClient_OutBuffer[1] = 'b';
				TCPClient_OutBuffer[2] = 'c';
				TCPClient_OutBuffer[3] = 'd';
				TCPClient_OutBuffer[4] = 'e';
				TCPClient_OutBuffer[5] = 'f';
				TCPClient_OutBuffer[6] = 'g';
				TCPClient_OutBuffer[7] = 'h';
				TCPClient_OutBuffer[8] = 'i';
				TCPClient_OutBuffer[9] = '\n';
				TCPClient_OutBuffer[10] = '\0';
				*/

/*
								TCPClient_OutBuffer[0] = 0x01;
								TCPClient_OutBuffer[1] = 0x02;
								TCPClient_OutBuffer[2] = 0x03;
								TCPClient_OutBuffer[3] = 0x04;
								TCPClient_OutBuffer[4] = 0x05;
								TCPClient_OutBuffer[5] = 0x06;
								TCPClient_OutBuffer[6] = 0x07;
								TCPClient_OutBuffer[7] = 0x08;
								TCPClient_OutBuffer[8] = 0x09;
								TCPClient_OutBuffer[9] = '\n';
								TCPClient_OutBuffer[10] = '\0';
*/

				TCPClient_OutBuffer[iTCPClient_SendPayloadLen + 6] = '\n';
				TCPClient_OutBuffer[iTCPClient_SendPayloadLen + 7] = '\0';
			}

			printf ("sending the following : ");
			for(int i = 0; i <=(iTCPClient_SendPayloadLen + 7); i++)
					printf ("%02X  ",TCPClient_OutBuffer[i]);

			printf ("sending chars : ");
				for(int i = 0; i <=(iTCPClient_SendPayloadLen + 7); i++)
						printf ("%c  ",TCPClient_OutBuffer[i]);

				printf ("sending string : ");
					printf ("%s  ",TCPClient_OutBuffer);

			printf("\nwriting : %d bytes\n",(iTCPClient_SendPayloadLen + 7));

			  //n = write(sockfd,buffer,strlen(buffer));
			// Write the contents of TCPClient_OutBuffer on the TCPClient port
			iTCPClient_Ret = write(sockfd,TCPClient_OutBuffer,(iTCPClient_SendPayloadLen + 7));// Send the command on the TCPClient port

			//iTCPClient_Ret = send(sockfd,TCPClient_OutBuffer,(iTCPClient_SendPayloadLen + 7),0);// Send the command on the TCPClient port

			if (iTCPClient_Ret<=0) {                                                           // If the writting operation failed ...
				//printf ("Error while writing data\n");                              // ... display a message ...
				printf("\nError while writing data due to : [%s]\n",(char*)strerror(errno));
	                                 // Send failure...maybe due to packet collision so trigger a read operation next cycle
			}
			else
			{
				bTCPClient_DataReadyToSend = false;

				//Reset bits for next time

				if(bTCPClient_SendControllerUpdatesToServer)
				{
					printf("Got here - bTCPClient_ReqToSendControllerUpdatesToServer = false \n");
					printf("Got here - bTCPClient_SendControllerUpdatesToServer = false \n");
					bTCPClient_ReqToSendControllerUpdatesToServer				= false;
					bTCPClient_SendControllerUpdatesToServer	= false;


					//Start the resend delay timer
					bTCPClient_ControllerUpdatesSentToServer	= true;
					TCPClient_startTime = monotonic_clock::now();
				}

				 else if(bTCPClient_SendCheckForConnectionStatus)
				 {
//								 printf("Got here - bTCPClient_ReqToSendCheckForConnStatus = false \n");
//								 printf("Got here - bTCPClient_SendCheckForConnectionStatus = false \n");
					 bTCPClient_ReqToSendCheckForConnStatus		= false;
					 bTCPClient_SendCheckForConnectionStatus				= false;
				 }

				 else if(bTCPClient_SendDataReceivedACK)
				 {
					 printf("Got here - bTCPClient_ReqToSendDataReceivedACK = false \n");
					 printf("Got here - bTCPClient_SendDataReceivedACK = false \n");
					 bTCPClient_ReqToSendDataReceivedACK			= false;
					 bTCPClient_SendDataReceivedACK		= false;
				 }
				 else if(bTCPClient_SendTestDataToServer)
				 {
					 printf("Got here - bTCPClient_ReqToSendDataReceivedACK = false \n");
					 printf("Got here - bTCPClient_SendTestDataToServer = false \n");
					 bTCPSendDataToServerTEST			= false;
					 bTCPClient_SendTestDataToServer		= false;

					 bTCPSentDataToServerTEST = true;
					 TCPClient_TeststartTime = monotonic_clock::now();
				 }

				printf ("Write operation is successful \n");

			}

		}
		std::cout << "Func_TCPClientHandleAndSendData exiting" << std::endl;
}


//************************************************************************************************************************//

//										** uint16_t Func_TCPClientHandleReceivedData (uint16_t iNumBytesRead) **

//************************************************************************************************************************//
//Notes: This function decodes the received data and reports if the data is valid

// PACKET FRAMING FORMAT - total packet length minus flags len+4
				//U8 sflag;   //0x7e start of packet end of packet flag from HDLC
				//U8 cmd;     //what to do.
				//U8 len;     // payload length
				//U8 payload[len];  // could be zero len
				//U16 crc;
				//U8 eflag;   //end of frame flag


	// Received Data Command List:
	//=====================================================
	// 0x01 - Received status updates from another controller (includes TCPClient Comm connection status of the other controller  BBB handles timing for all controller connection statuses.
	// 0x02 - Received deviation alert
	// 0x03 - Received maintenance alert
	// 0x04 - Unused
	// .
	// .
	// 0x99 - Received ACK from last comm request to another controller

	// Send Data Command List:
	//=====================================================
	// 0x01 - Send status updates to local controller from other remote controller(s)
	// 0x02 - Send request for connection status information from local controller
	// 0x03 - Unused
	// 0x04 - Unused
	// 0x05 - Unused
	// .
	// .
	// 0x99 - Send data recieved ACK


uint16_t Func_TCPClientHandleReceivedData (uint16_t iNumBytesRead)
{

	uint16_t		iTCPClient_ReceivedCRC = 0,
					iTCPClient_FrameStartCount = 0,
					iTCPClient_Receive_CalculatedCRC = 0;


	//Data Received so lets check what we got
	for(uint16_t i = 0; i<=iNumBytesRead; i++)
	{
		//Find start of data packet
		if(TCPClient_InBuffer[i] == 0x7e)
		{

			int iByteCount = 0;

			for(int iTempCount = 0; iTempCount <= iNumBytesRead; iTempCount++)
			{
				iByteCount++;

				if(TCPClient_InBuffer[iTempCount+i] == 0x0A)
					break;
			}
			printf (" \n");
			printf ("Actual Bytes Received = %d \n", iByteCount);

			iTCPClient_FrameStartCount = i;
			chTCPClient_ReceivedCommand = TCPClient_InBuffer[iTCPClient_FrameStartCount+1];
			chTCPClient_ReceivedPayloadLen = TCPClient_InBuffer[iTCPClient_FrameStartCount+2];

			//Convert char to int
			iTCPClient_ReceivedPayloadLen = chTCPClient_ReceivedPayloadLen;

			if(iTCPClient_ReceivedPayloadLen > 0)
			{
				//load the payload buffer
				for(uint8_t iPayloadCount = 0; iPayloadCount <= iTCPClient_ReceivedPayloadLen; iPayloadCount++)
					TCPClient_ReceivedPayloadBuffer[iPayloadCount] = TCPClient_InBuffer[iTCPClient_FrameStartCount + 2 + (iPayloadCount + 1)];
			}

			//load the crc
			for(uint8_t iCRCBufferCount = 0; iCRCBufferCount <= 1; iCRCBufferCount++)
			{
				TCPClient_ReceivedCRCBuffer[iCRCBufferCount] = TCPClient_InBuffer[iTCPClient_FrameStartCount + 2 + iTCPClient_ReceivedPayloadLen + (iCRCBufferCount + 1)];
				printf ("TCPClient_ReceivedCRCBuffer[%d] = %02X \n", iCRCBufferCount, TCPClient_InBuffer[iTCPClient_FrameStartCount + 2 + iTCPClient_ReceivedPayloadLen + (iCRCBufferCount + 1)]);
			}

			//Convert the unsigned char array to unsigned int16
			iTCPClient_ReceivedCRC = TCPClient_ReceivedCRCBuffer[0] | (uint16_t(TCPClient_ReceivedCRCBuffer[1]) <<8);


			printf ("TCPClient_InBuffer[%d] = %02X \n", i, TCPClient_InBuffer[i]);
			printf ("chTCPClient_ReceivedCommand = %02X \n", chTCPClient_ReceivedCommand);
			printf ("chTCPClient_ReceivedPayloadLen = %d \n", iTCPClient_ReceivedPayloadLen);

			for(uint8_t iPayloadCount = 0; iPayloadCount <= (iTCPClient_ReceivedPayloadLen - 1); iPayloadCount++)
				printf ("TCPClient_ReceivedPayloadBuffer[%d] = %02X \n", iPayloadCount, TCPClient_ReceivedPayloadBuffer[iPayloadCount]);

			printf ("iTCPClient_ReceivedCRC = %d \n", iTCPClient_ReceivedCRC);


			// Received Data Command List:
				//=====================================================
				// 0x01 - Received status updates from another controller (includes TCPClient Comm connection status of the other controller  BBB handles timing for all controller connection statuses.
				// 0x02 - Received deviation alert
				// 0x03 - Received maintenance alert
				// 0x04 - Unused
				// .
				// .
				// 0x99 - Received ACK from last comm request to another controller


			iTCPClient_Receive_CalculatedCRC = TCPClientComputeChecksum(reinterpret_cast<uint8_t*> (&TCPClient_InBuffer[iTCPClient_FrameStartCount]), (iTCPClient_ReceivedPayloadLen + 2));

			//Check CRC
			if((iTCPClient_ReceivedCRC == iTCPClient_Receive_CalculatedCRC) && (iByteCount == iNumBytesRead))
			{
				printf ("CRC Check PASSED!! \n");
				bTCPClient_ReceiveDecodeComplete = true;
				//bTCPClient_ReqToSendDataReceivedACK = true;


				//Determine payload action
				if(chTCPClient_ReceivedCommand == 0X01)		//Send Controller phase and i/o data
				{
					iTCPClient_ReceivedLocalControllerNumber = TCPClient_ReceivedPayloadBuffer[0];
					iTCPClient_ReceivedLocalControllerPhase = TCPClient_ReceivedPayloadBuffer[1];
					iTCPClient_ReceivedLocalControllerRecipeNum = TCPClient_ReceivedPayloadBuffer[2];
					iTCPClient_ReceivedLocalControllerBasketNum = TCPClient_ReceivedPayloadBuffer[3];

					bTCPClient_ReqToSendControllerUpdatesToServer = true;
					//TCPClient_ReceivedPayloadBuffer[1 to iTCPClient_ReceivedPayloadLen] == i/o values;
				}

				else if(chTCPClient_ReceivedCommand == 0X03)		//Send Deviation Email Alert
					bEmail_SendDeviationAlert = true;

				else if(chTCPClient_ReceivedCommand == 0X04)		//Send Maintenance Email Alert
					bEmail_SendMaintenanceAlert = true;

				else if(chTCPClient_ReceivedCommand == 0X99)		//Data Received ACK
					bTCPClient_SentDataReceivedByServer = true;

				else											//Do nothing with the data as the command is invalid
					return 0;

				return 1;
			}
			else
			{
				if(iByteCount != iNumBytesRead)
				{
					printf ("Length does not match!! \n");
					printf ("iByteCount used = %d \n", iByteCount);
					printf ("iTCPClient_ReceiveBytesRead used = %d \n", iNumBytesRead);
				}

				if(iTCPClient_ReceivedCRC != iTCPClient_Receive_CalculatedCRC)
				{
					printf ("CRC Check FAILED!! \n");
					printf ("iTCPClient_ReceivedCRC = %d \n", iTCPClient_ReceivedCRC);
					printf ("Calculated CRC = %d \n", iTCPClient_Receive_CalculatedCRC);
				}

				return 0;
			}
		}
	}
	//Oh Snap! We didnt detect a start of packet bit
	return 0;
}



//************************************************************************************************************************//

//								** static uint16_t TCPClientComputeChecksum(uint8_t* data_p, uint16_t length) **

//************************************************************************************************************************//
//Notes: This function creates a CRC

static uint16_t TCPClientComputeChecksum(uint8_t* data_p, uint16_t length){
	uint8_t x;
	uint16_t crc = 0xFFFF;

   while (length--){

//	   printf ("length = %u \n", length);
//	   printf ("*data_p++ = %u \n", *data_p);

       x = crc >> 8 ^ *data_p++;

//       printf ("csX1 = %u \n", x);
//       printf ("crc >> 8 = %u \n", crc >> 8);
//
//       printf ("x>>4 = %u \n", x>>4);


       x ^= x>>4;


//       printf ("csX2 = %u \n", x);
//
//       printf ("crc << 8 = %u \n", crc << 8);
//
//       printf ("x << 12 = %u \n", x << 12);


       crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);

//       printf ("CRC = %u \n", crc);
   }
   return crc;
}


