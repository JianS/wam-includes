/*
 *
 *
 *
 * */

#ifndef OPTITRACKVISION_H
#define OPTITRACKVISION_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <boost/circular_buffer.hpp>
#include <time.h>

#include <NatNetLinux/NatNet.h>
#include <NatNetLinux/CommandListener.h>
#include <NatNetLinux/FrameListener.h>

#include "visionOutputs.h"
#include "visionFilter.h"
#include "ObjectSet.h"

class OptiTrackVision {

public:

	OptiTrackVision(char** add, size_t bl = 1) : BuffLength(bl) {
		//cpOutput = new visionCP (&frameListener->frames());
		startvision(add);
		printf("\nThe vision system is initialized.\n");

		while (!is_receiving()) { usleep(100); }
		printf("\nStart receiving vision data.\n");
	}

	~OptiTrackVision() {}

	void helpAndExit();

	/*!
	 * \brief Creates a socket for communicating commands.
	 *
	 * \param inAddr our local address
	 * \returns socket descriptor bound to PORT_COMMAND and local address
	 */
	static int createCommandSocket( uint32_t inAddr );

	/*!
	 * \brief Creates a socket to read data from the server.
	 *
	 * The socket returned from this function is bound to \c PORT_DATA and
	 * \c INADDR_ANY, and is added to the multicast group given by
	 * \c MULTICAST_ADDRESS.
	 *
	 * \param inAddr our local address
	 * \returns socket bound as described above
	 */
	static int createDataSocket( uint32_t inAddr );

	//! \brief Convert internet address to string.
	void addrToStr( char* str, size_t len, const struct in_addr addr );

	void readOpts( uint32_t& localAddress, uint32_t& serverAddress, int argc, char* argv[] );

	void startvision(char** argvc);

	void stopvision();

/*	boost::circular_buffer< std::pair<MocapFrame, struct timespec> > getLastBuffer();

	MocapFrame getLastFrame();

	struct timespec getLastRecvTime();*/

	bool is_receiving();



protected:
	unsigned char natNetMajor;
	unsigned char natNetMinor;

	int sdCommand;
	int sdData;

	uint32_t localAddress;
	uint32_t serverAddress;

	//boost::circular_buffer<MocapFrame>& frameBuf;

	size_t BuffLength;
	FrameListener* frameListener;

public:
	VisionFilter* visionFilter1;
	visionCP* cpOutput_Hand;
	visionCP* cpOutput_Obj;
	visionPose* poseOutput_Obj_filtered;
	visionPose* poseOutput_Obj_unfiltered;
	visionTimeOutput* tOutput;


private:
	#define MULTICAST_ADDRESS "239.255.42.99"
	#define PORT_COMMAND      1510
	#define PORT_DATA         1511

}; //End of declaration of the class



/******************************************************************/
// Here are the body of the functions


void OptiTrackVision::helpAndExit()
{
   std::cout
      << "Usage:" << std::endl
      << "   simple-example [local address] [server address]" << std::endl
      << "   local address  - Local interface IPv4 address, e.g. 192.168.0.2" << std::endl
      << "   server address - Server IPv4 address, e.g. 192.168.0.3" << std::endl;
   exit(1);
}

/*!
 * \brief Creates a socket for communicating commands.
 *
 * \param inAddr our local address
 * \returns socket descriptor bound to PORT_COMMAND and local address
 */
int OptiTrackVision::createCommandSocket( uint32_t inAddr )
{
   // Asking for a buffer of 1MB = 2^20 bytes. This is what NP does, but this
   // seems far too large on Linux systems where the max is usually something
   // like 256 kB.
   const int rcvBufSize = 0x100000;
   int sd;
   int tmp;
   socklen_t len;
   struct sockaddr_in sockAddr;

   sd = socket(AF_INET, SOCK_DGRAM, 0);
   if( sd < 0 )
   {
      std::cerr << "Could not open socket. Error: " << errno << std::endl;
      exit(1);
   }

   // Bind socket
   memset(&sockAddr, 0, sizeof(sockAddr));
   sockAddr.sin_family = AF_INET;
   sockAddr.sin_port = htons(PORT_COMMAND);
   //sockAddr.sin_port = htons(0);
   sockAddr.sin_addr.s_addr = inAddr;
   tmp = bind( sd, (struct sockaddr*)&sockAddr, sizeof(sockAddr) );
   if( tmp < 0 )
   {
      std::cerr << "Could not bind socket. Error: " << errno << std::endl;
      close(sd);
      exit(1);
   }

   int value = 1;
   tmp = setsockopt( sd, SOL_SOCKET, SO_BROADCAST, (char*)&value, sizeof(value) );
   if( tmp < 0 )
   {
      std::cerr << "Could not set socket to broadcast mode. Error: " << errno << std::endl;
      close(sd);
      exit(1);
   }

   setsockopt(sd, SOL_SOCKET, SO_RCVBUF, (char*)&rcvBufSize, sizeof(rcvBufSize));
   getsockopt(sd, SOL_SOCKET, SO_RCVBUF, (char*)&tmp, &len);
   if( tmp != rcvBufSize )
   {
      std::cerr << "Could not set receive buffer size. Asked for "
         << rcvBufSize << "B got " << tmp << "B" << std::endl;

   }

   return sd;
}

/*!
 * \brief Creates a socket to read data from the server.
 *
 * The socket returned from this function is bound to \c PORT_DATA and
 * \c INADDR_ANY, and is added to the multicast group given by
 * \c MULTICAST_ADDRESS.
 *
 * \param inAddr our local address  /lib/x86_64-linux-gnu/libpthread.so.0 : ()+0x7e9a
 *
 * \returns socket bound as described above
 */
int OptiTrackVision::createDataSocket( uint32_t inAddr )
{
   int sd;
   int value;
   int tmp;
   struct sockaddr_in localSock;
   struct ip_mreq group;

   sd = socket(AF_INET, SOCK_DGRAM, 0);
   value = 1;
   tmp = setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, (char*)&value, sizeof(value));
   if( tmp < 0 )
   {
      std::cerr << "ERROR: Could not set socket option." << std::endl;
      close(sd);
      return -1;
   }

    // Bind the socket to a port.
   memset((char*)&localSock, 0, sizeof(localSock));
   localSock.sin_family = AF_INET;
   localSock.sin_port = htons(PORT_DATA);
   localSock.sin_addr.s_addr = INADDR_ANY;
   bind(sd, (struct sockaddr*)&localSock, sizeof(localSock));

   // Connect a local interface address to the multicast interface address.
   group.imr_multiaddr.s_addr = inet_addr(MULTICAST_ADDRESS);
   group.imr_interface.s_addr = inAddr;
   tmp = setsockopt(sd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&group, sizeof(group));
   if( tmp < 0 )
   {
      std::cerr << "ERROR: Could not add the interface to the multicast group." << std::endl;
      close(sd);
      return -1;
   }

   return sd;
}

//! \brief Convert internet address to string.
void OptiTrackVision::addrToStr( char* str, size_t len, const struct in_addr addr )
{
   const char* s = inet_ntoa( addr );
   strncpy( str, s, len );
}

void OptiTrackVision::readOpts( uint32_t& localAddress, uint32_t& serverAddress, int argc, char* argv[] )
{
   if( argc > 1 )
   {
      localAddress = inet_addr(argv[0]);
      serverAddress = inet_addr(argv[1]);
   }
   else
      helpAndExit();
}

void OptiTrackVision::startvision(char** argvc){

	readOpts( localAddress, serverAddress, 2, argvc );

	// Use this socket address to send commands to the server.
	struct sockaddr_in serverCommands;
	memset(&serverCommands, 0, sizeof(serverCommands));
	serverCommands.sin_family = AF_INET;
	serverCommands.sin_port = htons(PORT_COMMAND);
	serverCommands.sin_addr.s_addr = serverAddress;

	sdCommand = createCommandSocket( localAddress );
	sdData = createDataSocket( localAddress );

	CommandListener commandListener(sdCommand);
	commandListener.start();

	// Send a ping packet to the server so that it sends us the NatNet version
	// in its response to commandListener.
	NatNetPacket ping = NatNetPacket::pingPacket();
	ping.send(sdCommand, serverCommands);

	// Wait here for ping response to give us the NatNet version.
	commandListener.getNatNetVersion(natNetMajor, natNetMinor);
	std::cout << "Main thread got version " << static_cast<int>(natNetMajor) << "." << static_cast<int>(natNetMinor) << std::endl;

	// Start up a FrameListener, and get a reference to its output rame buffer.
	frameListener = new FrameListener(sdData, natNetMajor, natNetMinor, BuffLength);
	frameListener->start();

	visionFilter1 = new VisionFilter(frameListener, objectSet.NumOfObjs(), 10, objectSet);
	//visionFilter2 = new VisionFilter(frameListener, 2, 10);

	cpOutput_Hand = new visionCP (visionFilter1, 1);
	cpOutput_Obj = new visionCP (visionFilter1, 2);
	poseOutput_Obj_filtered = new visionPose(visionFilter1, 2, true);
	poseOutput_Obj_unfiltered = new visionPose(visionFilter1, 2, false);
	tOutput = new visionTimeOutput (visionFilter1);

}

void OptiTrackVision::stopvision(){

	//delete(visionFilter);
	//delete(frameListener);

	frameListener->stop();

	int closeflag1 = close(sdData);
	int closeflag2 = close(sdCommand);

	printf("Close the socket. Vision is off. %d, %d\n", closeflag1, closeflag2);

	//delete(&cpOutput);

}

/*boost::circular_buffer< std::pair<MocapFrame, struct timespec> > OptiTrackVision::getLastBuffer(){
	return frameListener->frames();
}

MocapFrame OptiTrackVision::getLastFrame(){
	return frameListener->frames().front().first;
}

struct timespec OptiTrackVision::getLastRecvTime(){
	return frameListener->frames().front().second;
}*/

bool OptiTrackVision::is_receiving(){

	return frameListener->running();

}




#endif
