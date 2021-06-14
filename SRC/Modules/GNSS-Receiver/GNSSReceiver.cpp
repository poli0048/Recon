//This module provides basic support for a UBLOX GNSS receiver connected to the GCS via UART or USB
//This enables the UI to, for instance, indicate the GCS location on the map.
//It also makes it possible to convert software timestamps (steady_clock) to absolute GPS Time timestamps.
//We elect to use the UBX protocol as opposed to NMEA so we can easily access proper GPS time.
//Authors: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 

//External Includes
#include "serial/serial.h"

//Project Includes
#include "GNSSReceiver.hpp"
#include "../../ProgOptions.hpp"
#include "../../Utilities.hpp"

#define UBX_HEADER_LENGTH 6U //Number of bytes that proceed the payload of a raw UBX packet

#define MESSAGE_CLASS_NAV     0x01
#define MESSAGE_CLASS_RXM     0x02
#define MESSAGE_CLASS_INF     0x04
#define MESSAGE_CLASS_ACK     0x05
#define MESSAGE_CLASS_CFG     0x06
#define MESSAGE_CLASS_MON     0x0A
#define MESSAGE_CLASS_AID     0x0B
#define MESSAGE_CLASS_TIM     0x0D
#define MESSAGE_CLASS_ESF     0x10

#define MESSAGE_ID_CFG_ANT    0x13
#define MESSAGE_ID_CFG_CFG    0x09
#define MESSAGE_ID_CFG_ITFM   0x39
#define MESSAGE_ID_CFG_MSG    0x01
#define MESSAGE_ID_CFG_NAV5   0x24
#define MESSAGE_ID_CFG_NAVX5  0x23
#define MESSAGE_ID_CFG_NVS    0x22
#define MESSAGE_ID_CFG_PM2    0x3B
#define MESSAGE_ID_CFG_PRT    0x00
#define MESSAGE_ID_CFG_RATE   0x08
#define MESSAGE_ID_CFG_RST    0x04
#define MESSAGE_ID_CFG_RXM    0x11
#define MESSAGE_ID_CFG_SBAS   0x16
#define MESSAGE_ID_CFG_TMODE2 0x3D
#define MESSAGE_ID_CFG_TP5    0x31
#define MESSAGE_ID_CFG_TP     0x07

#define MESSAGE_ID_ACK_ACK    0x01
#define MESSAGE_ID_ACK_NAK    0x00

//We use POSECEF, STATUS, and TIMEGPS messages because they are supported from UBX6 through UBX9. SOL and SVINFO have been replaced
//We can use SIG to get info on tracked satelites (to get C/N0s)... but we should not require this since it doesn't exist on older receivers.
#define MESSAGE_ID_NAV_POSECEF 0x01
#define MESSAGE_ID_NAV_STATUS  0x03
#define MESSAGE_ID_NAV_TIMEGPS 0x20
#define MESSAGE_ID_NAV_SIG     0x43

#define MESSAGE_ID_MON_HW      0x09

//TODO: Initially, we check that a received packet is the size that the packet advertises. This will catch the vast majority of
//potential segfault problems, but it's not a guarantee. We should also check in each packet decoder that we never try to decode
//fields that don't exist. Currently, if we get a corrupted packet, or if the receiver sends a packet that doesn't conform the the
//UBX protocol, it could result in a segfault.

// ***************************************************************************************************************************
// ********************************************    Local Function Declarations    ********************************************
// ***************************************************************************************************************************

//The UBX protocol uses little-endian representation for multi-byte fields. The following
//provides functions for encoding and decoding these fields.
static void encodeField_U1(std::vector<uint8_t> & buffer, uint8_t  x);
static void encodeField_I1(std::vector<uint8_t> & buffer, int8_t   x);
static void encodeField_X1(std::vector<uint8_t> & buffer, uint8_t  x);
static void encodeField_U2(std::vector<uint8_t> & buffer, uint16_t x);
static void encodeField_I2(std::vector<uint8_t> & buffer, int16_t  x);
static void encodeField_X2(std::vector<uint8_t> & buffer, uint16_t x);
static void encodeField_U4(std::vector<uint8_t> & buffer, uint32_t x);
static void encodeField_I4(std::vector<uint8_t> & buffer, int32_t  x);
static void encodeField_X4(std::vector<uint8_t> & buffer, uint32_t x);
static void encodeField_R4(std::vector<uint8_t> & buffer, float    x);
static void encodeField_R8(std::vector<uint8_t> & buffer, double   x);
static void encodeField_CH(std::vector<uint8_t> & buffer, char     x);

static uint8_t  decodeField_U1(uint8_t * buffer);
static int8_t   decodeField_I1(uint8_t * buffer);
static uint8_t  decodeField_X1(uint8_t * buffer);
static uint16_t decodeField_U2(uint8_t * buffer);
static int16_t  decodeField_I2(uint8_t * buffer);
static uint16_t decodeField_X2(uint8_t * buffer);
static uint32_t decodeField_U4(uint8_t * buffer);
static int32_t  decodeField_I4(uint8_t * buffer);
static uint32_t decodeField_X4(uint8_t * buffer);
static float    decodeField_R4(uint8_t * buffer);
static double   decodeField_R8(uint8_t * buffer);
static char     decodeField_CH(uint8_t * buffer);
	
//Decodes the packets checksum and validates it on the provided packet buffer. Only call after the packet size has been validated
static bool isChecksumGood(uint8_t * rawUBXPacket);

//Encode a packet header - everything before the actual payload
static void encodeHeader(std::vector<uint8_t> & buffer, uint8_t CLASS, uint8_t ID, uint16_t PayloadLength);

//Add the checksum bytes to a packet
static void addChecksum(std::vector<uint8_t> & buffer);

static uint8_t getMessageClass(uint8_t * rawUBXPacket); //Only call after the packet size has been validated
static uint8_t getMessageID(uint8_t * rawUBXPacket);    //Only call after the packet size has been validated

//Manage message configurations
namespace UBXPacket_CFG_MSG {
	static void encodePollRequest(std::vector<uint8_t> & UBXPacket, uint8_t msgClass, uint8_t msgID);
	static void encodeSet        (std::vector<uint8_t> & UBXPacket, uint8_t msgClass, uint8_t msgID, uint8_t *rate); //Set rates for all 6 targets
	static void encodeSet        (std::vector<uint8_t> & UBXPacket, uint8_t msgClass, uint8_t msgID, uint8_t rate);  //Set rate for current target
	static void decodeGet        (uint8_t * rawUBXPacket, uint8_t &msgClass, uint8_t &msgID, std::vector<uint8_t> &rate);
};

//Manage IO port configurations
namespace UBXPacket_CFG_PRT {
	static void encodePollRequest(std::vector<uint8_t> & UBXPacket); //Poll the used IO port
	static void encodePollRequest(std::vector<uint8_t> & UBXPacket, uint8_t PortID); //Poll a single IO port
	static void encodeSet        (std::vector<uint8_t> & UBXPacket, uint8_t PortID, uint16_t txReady, uint32_t mode, uint32_t baudRate,
	                              uint16_t inProtoMask, uint16_t outProtoMask); //Port configuration
	static void decodeGet        (uint8_t * rawUBXPacket, uint8_t &PortID, uint16_t &txReady, uint32_t &mode, uint32_t &baudRate,
	                              uint16_t &inProtoMask, uint16_t &outProtoMask); //Port Configuration
};

//Manage navigation/measurement rate settings
namespace UBXPacket_CFG_RATE {
	static void encodePollRequest(std::vector<uint8_t> & UBXPacket);
	static void encodeSet        (std::vector<uint8_t> & UBXPacket, uint16_t measRate, uint16_t navRate, uint16_t timeRef);
	static void decodeGet        (uint8_t * rawUBXPacket, uint16_t &measRate, uint16_t &navRate, uint16_t &timeRef);
};

//The NAVSolution class combines data from a few different UBX messages (corresponding to the same epoch).
class NAVSolution {
	public:
		NAVSolution() = default;
		~NAVSolution() = default;
		
		bool isDecoded_POSECEF = false;
		bool isDecoded_STATUS  = false;
		bool isDecoded_TIMEGPS = false;
		bool initialized = false; //Set to true when at least one of the above is true (and thus, iTOW is populated)
		std::chrono::time_point<std::chrono::steady_clock> timestamp; //Time when first component is decoded.
		
		//Initialize all fields to prevent compiler warnings about possible use-before-set bugs
		uint32_t iTOW  = 0U;
		int32_t  fTOW  = 0;
		int16_t  week  = 0;
		uint8_t  valid = 0U;
		uint32_t tAcc  = 0U;
		
		int32_t  ecefX = 0;
		int32_t  ecefY = 0;
		int32_t  ecefZ = 0;
		uint32_t pAcc  = 0U;
		
		uint8_t  gpsFix  = 0U;
		uint8_t  flags   = 0U;
		uint8_t  fixStat = 0U;
		uint8_t  flags2  = 0U;
		uint32_t ttff    = 0U;
		
		bool isComplete(void) { return (isDecoded_POSECEF && isDecoded_STATUS && isDecoded_TIMEGPS); }
		void reset(void) {
			isDecoded_POSECEF = false;
			isDecoded_STATUS  = false;
			isDecoded_TIMEGPS = false;
			initialized       = false;
		}
		
		void decode_NAV_POSECEF(uint8_t * rawUBXPacket) {
			if (! initialized)
				SetTimestamp();
			else {
				if (iTOW != decodeField_U4(rawUBXPacket + UBX_HEADER_LENGTH + 0U)) {
					reset();
					SetTimestamp();
				}
			}
			
			iTOW  = decodeField_U4(rawUBXPacket + UBX_HEADER_LENGTH + 0U);
			ecefX = decodeField_I4(rawUBXPacket + UBX_HEADER_LENGTH + 4U);
			ecefY = decodeField_I4(rawUBXPacket + UBX_HEADER_LENGTH + 8U);
			ecefZ = decodeField_I4(rawUBXPacket + UBX_HEADER_LENGTH + 12U);
			pAcc  = decodeField_U4(rawUBXPacket + UBX_HEADER_LENGTH + 16U);
			
			initialized = true;
			isDecoded_POSECEF = true;
		}
		
		void decode_NAV_STATUS(uint8_t * rawUBXPacket) {
			if (! initialized)
				SetTimestamp();
			else {
				if (iTOW != decodeField_U4(rawUBXPacket + UBX_HEADER_LENGTH + 0U)) {
					reset();
					SetTimestamp();
				}
			}
			
			iTOW    = decodeField_U4(rawUBXPacket + UBX_HEADER_LENGTH + 0U);
			gpsFix  = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 4U);
			flags   = decodeField_X1(rawUBXPacket + UBX_HEADER_LENGTH + 5U);
			fixStat = decodeField_X1(rawUBXPacket + UBX_HEADER_LENGTH + 6U);
			flags2  = decodeField_X1(rawUBXPacket + UBX_HEADER_LENGTH + 7U);
			ttff    = decodeField_U4(rawUBXPacket + UBX_HEADER_LENGTH + 8U);
			
			initialized = true;
			isDecoded_STATUS = true;
		}
		
		void decode_NAV_TIMEGPS(uint8_t * rawUBXPacket) {
			if (! initialized)
				SetTimestamp();
			else {
				if (iTOW != decodeField_U4(rawUBXPacket + UBX_HEADER_LENGTH + 0U)) {
					reset();
					SetTimestamp();
				}
			}
			
			iTOW  = decodeField_U4(rawUBXPacket + UBX_HEADER_LENGTH + 0U);
			fTOW  = decodeField_I4(rawUBXPacket + UBX_HEADER_LENGTH + 4U);
			week  = decodeField_I2(rawUBXPacket + UBX_HEADER_LENGTH + 8U);
			valid = decodeField_X1(rawUBXPacket + UBX_HEADER_LENGTH + 11U);
			tAcc  = decodeField_U4(rawUBXPacket + UBX_HEADER_LENGTH + 12U);
			
			initialized = true;
			isDecoded_TIMEGPS = true;
		}
	
	private:
		void SetTimestamp(void) {
			timestamp = std::chrono::steady_clock::now();
			//TODO: Adjust here to account for latency in serial comms
		}
};

//The NAVSig class holds data collected from NAV-SIG messages, which holds info on tracked signals (C/N0's, etc.)
class NAVSig {
	public:
		NAVSig() = default;
		~NAVSig() = default;
		
		uint32_t iTOW;
		uint8_t version, numSigs;
		std::vector<uint8_t>  gnssId;
		std::vector<uint8_t>  svId;
		std::vector<uint8_t>  sigId;
		std::vector<uint8_t>  freqId;
		std::vector<int16_t>  prRes;
		std::vector<uint8_t>  cno;
		std::vector<uint8_t>  qualityInd;
		std::vector<uint8_t>  corrSource;
		std::vector<uint8_t>  ionoModel;
		std::vector<uint16_t> sigFlags;
		
		std::chrono::time_point<std::chrono::steady_clock> timestamp;
		bool isValid = false;
		
		void decode_NAV_SIG(uint8_t * rawUBXPacket) {
			timestamp = std::chrono::steady_clock::now();
			
			iTOW    = decodeField_U4(rawUBXPacket + UBX_HEADER_LENGTH + 0U);
			version = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 4U);
			numSigs = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 5U);
			
			gnssId.resize(numSigs);
			svId.resize(numSigs);
			sigId.resize(numSigs);
			freqId.resize(numSigs);
			prRes.resize(numSigs);
			cno.resize(numSigs);
			qualityInd.resize(numSigs);
			corrSource.resize(numSigs);
			ionoModel.resize(numSigs);
			sigFlags.resize(numSigs);
			
			for (unsigned int n = 0; n < (unsigned int) numSigs; n++) {
				gnssId[n]     = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 8U  + n*16U);
				svId[n]       = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 9U  + n*16U);
				sigId[n]      = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 10U + n*16U);
				freqId[n]     = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 11U + n*16U);
				prRes[n]      = decodeField_I2(rawUBXPacket + UBX_HEADER_LENGTH + 12U + n*16U);
				cno[n]        = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 14U + n*16U);
				qualityInd[n] = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 15U + n*16U);
				corrSource[n] = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 16U + n*16U);
				ionoModel[n]  = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 17U + n*16U);
				sigFlags[n]   = decodeField_X2(rawUBXPacket + UBX_HEADER_LENGTH + 18U + n*16U);
			}
			
			isValid = true;
		}
};


namespace UBXPacket_MON_HW {
	void encodePollRequest(std::vector<uint8_t> & UBXPacket);
	void decodeGet        (uint8_t * rawUBXPacket, uint32_t &pinSel, uint32_t &pinBank, uint32_t &pinDir, uint32_t &pinVal, uint16_t &noisePerMS,
	                       uint16_t &agcCnt, uint8_t &aStatus, uint8_t &aPower, uint8_t &flags, uint32_t &usedMask, std::vector<uint8_t> &VP, uint8_t &jamInd,
	                       uint32_t &pinIrq, uint32_t &pullH, uint32_t &pullL);
};

static bool ReadPacket(serial::Serial * serialDev, std::vector<uint8_t> & UBXPacket);
static bool ReadPacket(serial::Serial * serialDev, std::vector<uint8_t> & UBXPacket, uint8_t MessageClass, uint8_t MessageID);
static bool isAcknowledged(serial::Serial * serialDev, uint8_t MessageClass, uint8_t MessageID);

// ***************************************************************************************************************************
// ********************************************    Public Function Definitions    ********************************************
// ***************************************************************************************************************************
void GNSSReceiver::GNSSManager::ModuleMain(void) {
	std::unique_ptr<serial::Serial> gnssSerialDev;
	TimePoint last_Packet_Timestamp; //Updated each time we get a packet... used to see if anything is connected.
	NAVSolution currentSol;
	NAVSig      currentSigs;
	while (! m_abort) {
		//Grab copies of the relavent options from progOptions so we don't have to hold a lock on progOptions for long
		bool GNSSModuleEnabled = false;
		bool GNSSModuleVerbose = false;
		std::string GNSSReceiverDevicePath;
		int GNSSReceiverBaudRate = -1;
		{
			ProgOptions * opts = ProgOptions::Instance();
			if (opts != nullptr) {
				std::scoped_lock progOptionsLock(opts->OptionsMutex);
				GNSSModuleEnabled      = opts->GNSSModuleEnabled;
				GNSSModuleVerbose      = opts->GNSSModuleVerbose;
				GNSSReceiverDevicePath = opts->GNSSReceiverDevicePath;
				GNSSReceiverBaudRate   = opts->GNSSReceiverBaudRate;
			}
		}
		
		//Update receiver-connected status
		m_receiverConnected = (SecondsElapsed(last_Packet_Timestamp, std::chrono::steady_clock::now()) < 4.0);
		
		//Service any reset request
		if (m_reset) {
			std::cerr << "GNSS receiver module resetting.\r\n";
			gnssSerialDev.reset();
			m_reset = false;
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}
		
		//If not running, just sleep and continue
		if (! GNSSModuleEnabled) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}
		
		//If the serial device is not open. Try to open it and attempt to configure a UBLOX GNSS receiver that might be listening
		if (gnssSerialDev == nullptr) {
			try {
				gnssSerialDev.reset(new serial::Serial(GNSSReceiverDevicePath, GNSSReceiverBaudRate, serial::Timeout::simpleTimeout(1000)));
			}
			catch (...) {
				if (GNSSModuleVerbose)
					std::cerr << "GNSS Config failure (Exception on port open) - will retry in 2 seconds\r\n";
				gnssSerialDev.reset();
				for (int n = 0; n < 20; n++) {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					if (m_abort) return;
				}
				continue;
			}
			if (! gnssSerialDev->isOpen()) {
				if (GNSSModuleVerbose)
					std::cerr << "GNSS Config failure (Port not open) - will retry in 2 seconds\r\n";
				gnssSerialDev.reset();
				for (int n = 0; n < 20; n++) {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					if (m_abort) return;
				}
				continue;
			}
			
			//Flush the buffers on the device
			gnssSerialDev->flush();
			
			//Configure for UBX protocol only
			std::vector<uint8_t> UBXPacket;
			UBXPacket_CFG_PRT::encodePollRequest(UBXPacket);
			gnssSerialDev->write(UBXPacket);
			if (ReadPacket(gnssSerialDev.get(), UBXPacket, MESSAGE_CLASS_CFG, MESSAGE_ID_CFG_PRT)) {
				uint8_t  PortID;
				uint16_t txReady;
				uint32_t mode;
				uint32_t baudRate;
	               uint16_t inProtoMask;
	               uint16_t outProtoMask;
				UBXPacket_CFG_PRT::decodeGet(&(UBXPacket[0]), PortID, txReady, mode, baudRate, inProtoMask, outProtoMask);
				/*std::cerr << "** Current Receiver port config **\r\n";
				std::cerr << "PortID ------: " << (unsigned int) PortID       << "\r\n";
				std::cerr << "txReady -----: " << (unsigned int) txReady      << "\r\n";
				std::cerr << "mode --------: " << (unsigned int) mode         << "\r\n";
				std::cerr << "baudRate ----: " << (unsigned int) baudRate     << "\r\n";
				std::cerr << "inProtoMask -: " << (unsigned int) inProtoMask  << "\r\n";
				std::cerr << "outProtoMask : " << (unsigned int) outProtoMask << "\r\n";*/
				
				//Enable UBX output and disable NMEA output
				outProtoMask = 1U;
				UBXPacket.clear();
				UBXPacket_CFG_PRT::encodeSet(UBXPacket, PortID, txReady, mode, baudRate, inProtoMask, outProtoMask);
				gnssSerialDev->write(UBXPacket);
				if (! isAcknowledged(gnssSerialDev.get(), MESSAGE_CLASS_CFG, MESSAGE_ID_CFG_PRT)) {
					if (GNSSModuleVerbose)
						std::cerr << "GNSS Config failure (No Ack for CFG_PRT) - will retry in 2 seconds\r\n";
					gnssSerialDev.reset();
					for (int n = 0; n < 20; n++) {
						std::this_thread::sleep_for(std::chrono::milliseconds(100));
						if (m_abort) return;
					}
					continue;
				}
			}
			else {
				if (GNSSModuleVerbose)
					std::cerr << "GNSS Config failure (No reply to CFG_PRT poll request) - will retry in 2 seconds\r\n";
				gnssSerialDev.reset();
				for (int n = 0; n < 20; n++) {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					if (m_abort) return;
				}
				continue;
			}
			
			//Request NAV-POSECEF packets
			UBXPacket.clear();
			UBXPacket_CFG_MSG::encodeSet(UBXPacket, MESSAGE_CLASS_NAV, MESSAGE_ID_NAV_POSECEF, 1U);
			gnssSerialDev->write(UBXPacket);
			if (! isAcknowledged(gnssSerialDev.get(), MESSAGE_CLASS_CFG, MESSAGE_ID_CFG_MSG)) {
				if (GNSSModuleVerbose)
					std::cerr << "GNSS Config failure (No Ack for CFG_MSG) - will retry in 2 seconds\r\n";
				gnssSerialDev.reset();
				for (int n = 0; n < 20; n++) {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					if (m_abort) return;
				}
				continue;
			}
			
			//Request NAV-STATUS packets
			UBXPacket.clear();
			UBXPacket_CFG_MSG::encodeSet(UBXPacket, MESSAGE_CLASS_NAV, MESSAGE_ID_NAV_STATUS, 1U);
			gnssSerialDev->write(UBXPacket);
			if (! isAcknowledged(gnssSerialDev.get(), MESSAGE_CLASS_CFG, MESSAGE_ID_CFG_MSG)) {
				if (GNSSModuleVerbose)
					std::cerr << "GNSS Config failure (No Ack for CFG_MSG) - will retry in 2 seconds\r\n";
				gnssSerialDev.reset();
				for (int n = 0; n < 20; n++) {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					if (m_abort) return;
				}
				continue;
			}
			
			//Request NAV-TIMEGPS packets
			UBXPacket.clear();
			UBXPacket_CFG_MSG::encodeSet(UBXPacket, MESSAGE_CLASS_NAV, MESSAGE_ID_NAV_TIMEGPS, 1U);
			gnssSerialDev->write(UBXPacket);
			if (! isAcknowledged(gnssSerialDev.get(), MESSAGE_CLASS_CFG, MESSAGE_ID_CFG_MSG)) {
				if (GNSSModuleVerbose)
					std::cerr << "GNSS Config failure (No Ack for CFG_MSG) - will retry in 2 seconds\r\n";
				gnssSerialDev.reset();
				for (int n = 0; n < 20; n++) {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					if (m_abort) return;
				}
				continue;
			}
			
			//Request NAV-SIG packets, but don't fail if our request isn't acknowledged
			UBXPacket.clear();
			UBXPacket_CFG_MSG::encodeSet(UBXPacket, MESSAGE_CLASS_NAV, MESSAGE_ID_NAV_SIG, 5U); //Once a seconds should be fine
			gnssSerialDev->write(UBXPacket);
			
			//Request packets at 5 Hz
			UBXPacket.clear();
			uint16_t measRate = 200U; //Updates every 200 ms
			uint16_t timeRef  = 1U;   //Align measurements to GPST seconds rather than UTC seconds
			UBXPacket_CFG_RATE::encodeSet(UBXPacket, measRate, uint16_t(1U), timeRef);
			gnssSerialDev->write(UBXPacket);
			if (! isAcknowledged(gnssSerialDev.get(), MESSAGE_CLASS_CFG, MESSAGE_ID_CFG_RATE)) {
				if (GNSSModuleVerbose)
					std::cerr << "GNSS Config failure (No Ack for CFG_RATE) - will retry in 2 seconds\r\n";
				gnssSerialDev.reset();
				for (int n = 0; n < 20; n++) {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					if (m_abort) return;
				}
				continue;
			}
			
			//Poll for Hardware Status
			if (GNSSModuleVerbose) {
				UBXPacket.clear();
				UBXPacket_MON_HW::encodePollRequest(UBXPacket);
				gnssSerialDev->write(UBXPacket);
				if (ReadPacket(gnssSerialDev.get(), UBXPacket, MESSAGE_CLASS_MON, MESSAGE_ID_MON_HW)) {
					uint32_t pinSel, pinBank, pinDir, pinVal;
					uint16_t noisePerMS, agcCnt;
					uint8_t aStatus, aPower, flags;
					uint32_t usedMask;
					std::vector<uint8_t> VP;
					uint8_t jamInd;
					uint32_t pinIrq, pullH, pullL;
					UBXPacket_MON_HW::decodeGet(&(UBXPacket[0]), pinSel, pinBank, pinDir, pinVal, noisePerMS, agcCnt, aStatus, aPower,
						                       flags, usedMask, VP, jamInd, pinIrq, pullH, pullL);
					std::cerr << "pinSel ----: " << std::bitset<32>(pinSel) << "\r\n";
					std::cerr << "pinBank ---: " << std::bitset<32>(pinBank) << "\r\n";
					std::cerr << "pinDir ----: " << std::bitset<32>(pinDir) << "\r\n";
					std::cerr << "pinVal ----: " << std::bitset<32>(pinVal) << "\r\n";
					std::cerr << "noisePerMS : " << (unsigned int) noisePerMS << "\r\n";
					std::cerr << "agcCnt ----: " << (unsigned int) agcCnt << "\r\n";
					std::cerr << "aStatus ---: " << (unsigned int) aStatus << "\r\n";
					std::cerr << "aPower ----: " << (unsigned int) aPower << "\r\n";
					std::cerr << "flags -----: " << std::bitset<8>(flags) << "\r\n";
					std::cerr << "usedMask --: " << std::bitset<32>(usedMask) << "\r\n";
					std::cerr << "jamInd ----: " << (unsigned int) jamInd << "\r\n";
					std::cerr << "pinIrq ----: " << std::bitset<32>(pinIrq) << "\r\n";
					std::cerr << "pullH -----: " << std::bitset<32>(pullH) << "\r\n";
					std::cerr << "pullL -----: " << std::bitset<32>(pullL) << "\r\n";
				}
				else
					std::cerr << "Warning: GNSS receiver did not respond to MON-HW poll request.\r\n";
			}
		}
		
		//We made it through configuration... let's make sure the port is still open
		if (! gnssSerialDev->isOpen()) {
			if (GNSSModuleVerbose)
				std::cerr << "GNSS error (port unexpectedly closed) - will re-init in 1 second\r\n";
			gnssSerialDev.reset();
			for (int n = 0; n < 10; n++) {
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				if (m_abort) return;
			}
			continue;
		}
		
		//The port is open and the receiver is configured. receive a packet and process it
		std::vector<uint8_t> UBXPacket;
		if (ReadPacket(gnssSerialDev.get(), UBXPacket)) {
			last_Packet_Timestamp = std::chrono::steady_clock::now();
			if ((getMessageClass(&(UBXPacket[0])) == MESSAGE_CLASS_NAV) && (getMessageID(&(UBXPacket[0])) == MESSAGE_ID_NAV_POSECEF))
				currentSol.decode_NAV_POSECEF(&(UBXPacket[0]));
			else if ((getMessageClass(&(UBXPacket[0])) == MESSAGE_CLASS_NAV) && (getMessageID(&(UBXPacket[0])) == MESSAGE_ID_NAV_STATUS))
				currentSol.decode_NAV_STATUS(&(UBXPacket[0]));
			else if ((getMessageClass(&(UBXPacket[0])) == MESSAGE_CLASS_NAV) && (getMessageID(&(UBXPacket[0])) == MESSAGE_ID_NAV_TIMEGPS))
				currentSol.decode_NAV_TIMEGPS(&(UBXPacket[0]));
			else if ((getMessageClass(&(UBXPacket[0])) == MESSAGE_CLASS_NAV) && (getMessageID(&(UBXPacket[0])) == MESSAGE_ID_NAV_SIG)) {
				currentSigs.decode_NAV_SIG(&(UBXPacket[0]));
				std::cerr << "C/N0s: ";
				for (size_t n = 0U; n < currentSigs.cno.size(); n++) {
					std::cerr << (unsigned int) currentSigs.cno[n];
					if (n + 1 < currentSigs.cno.size())
						std::cerr << ", ";
					else
						std::cerr << "\r\n";
				}
			}
		}
		
		//If our current NAV Solution is complete, process it and reset it
		if (currentSol.isComplete()) {
			if (currentSol.flags & uint8_t(1)) {
				//Position and velocity are valid
				m_pos_ECEF << double(currentSol.ecefX)/100.0, double(currentSol.ecefY)/100.0, double(currentSol.ecefZ)/100.0;
				m_lastSolution_Timestamp = currentSol.timestamp;
				m_validSolutionReceived = true;
			}
			
			currentSol.reset();
		}
	}
}

// ***************************************************************************************************************************
// ********************************************    Local Function Definitions    *********************************************
// ***************************************************************************************************************************
static bool isChecksumGood(uint8_t * rawUBXPacket) {
	uint16_t payloadLength = decodeField_U2(rawUBXPacket + 4);
	uint8_t CK_A = 0U;
	uint8_t CK_B = 0U;
	
	for (unsigned int I=2U; I<(payloadLength + 6U); I++) {
		CK_A += (uint8_t) rawUBXPacket[I];
		CK_B += CK_A;
	}
	
	uint8_t shipped_CK_A = decodeField_U1(rawUBXPacket + payloadLength + 6U);
	uint8_t shipped_CK_B = decodeField_U1(rawUBXPacket + payloadLength + 7U);
	return ((shipped_CK_A == CK_A) && (shipped_CK_B == CK_B));
}

static uint8_t getMessageClass(uint8_t * rawUBXPacket) {
	return(decodeField_U1(rawUBXPacket + 2U));
}

static uint8_t getMessageID(uint8_t * rawUBXPacket) {
	return(decodeField_U1(rawUBXPacket + 3U));
}

static void encodeHeader(std::vector<uint8_t> & buffer, uint8_t CLASS, uint8_t ID, uint16_t PayloadLength) {
	buffer.push_back(0xB5);
	buffer.push_back(0x62);
	encodeField_U1(buffer, CLASS);
	encodeField_U1(buffer, ID);
	encodeField_U2(buffer, PayloadLength);
}

static void addChecksum(std::vector<uint8_t> & buffer) {
	uint8_t CK_A = 0U;
	uint8_t CK_B = 0U;
	
	unsigned int charCounter = 0;
	for (auto iter = buffer.begin(); iter != buffer.end(); iter++) {
		if (charCounter >= 2U) {
			CK_A += (uint8_t)(*iter);
			CK_B += CK_A;
		}
		charCounter++;
	}
	
	encodeField_U1(buffer, CK_A);
	encodeField_U1(buffer, CK_B);
}

static void encodeField_U1(std::vector<uint8_t> & buffer, uint8_t x) {
	buffer.push_back(x);
}

static void encodeField_I1(std::vector<uint8_t> & buffer, int8_t x) {
	buffer.push_back((uint8_t) x);
}

static void encodeField_X1(std::vector<uint8_t> & buffer, uint8_t x) {
	buffer.push_back(x);
}

static void encodeField_U2(std::vector<uint8_t> & buffer, uint16_t x) {
	buffer.push_back((uint8_t) x);   x = x >> 8;
	buffer.push_back((uint8_t) x);
}

static void encodeField_I2(std::vector<uint8_t> & buffer, int16_t x) {
	encodeField_U2(buffer, (uint16_t) x);
}

static void encodeField_X2(std::vector<uint8_t> & buffer, uint16_t x) {
	encodeField_U2(buffer, x);
}

static void encodeField_U4(std::vector<uint8_t> & buffer, uint32_t x) {
	buffer.push_back((uint8_t) x);   x = x >> 8;
	buffer.push_back((uint8_t) x);   x = x >> 8;
	buffer.push_back((uint8_t) x);   x = x >> 8;
	buffer.push_back((uint8_t) x);
}

static void encodeField_I4(std::vector<uint8_t> & buffer, int32_t x) {
	encodeField_U4(buffer, (uint32_t) x);
}

static void encodeField_X4(std::vector<uint8_t> & buffer, uint32_t x) {
	encodeField_U4(buffer, x);
}

static void encodeField_R4(std::vector<uint8_t> & buffer, float x) {
	uint32_t bitPattern = *(uint32_t*)(&x);
	encodeField_U4(buffer, bitPattern);
}

static void encodeField_R8(std::vector<uint8_t> & buffer, double x) {
	uint64_t bitPattern = *(uint64_t*)(&x);
	buffer.push_back((uint8_t) bitPattern);   bitPattern = bitPattern >> 8;
	buffer.push_back((uint8_t) bitPattern);   bitPattern = bitPattern >> 8;
	buffer.push_back((uint8_t) bitPattern);   bitPattern = bitPattern >> 8;
	buffer.push_back((uint8_t) bitPattern);   bitPattern = bitPattern >> 8;
	buffer.push_back((uint8_t) bitPattern);   bitPattern = bitPattern >> 8;
	buffer.push_back((uint8_t) bitPattern);   bitPattern = bitPattern >> 8;
	buffer.push_back((uint8_t) bitPattern);   bitPattern = bitPattern >> 8;
	buffer.push_back((uint8_t) bitPattern);
}

static void encodeField_CH(std::vector<uint8_t> & buffer, char x) {
	buffer.push_back((uint8_t) x);
}

static uint8_t decodeField_U1(uint8_t * buffer) {
	return(buffer[0]);
}

static int8_t decodeField_I1(uint8_t * buffer) {
	return((int8_t) buffer[0]);
}

static uint8_t decodeField_X1(uint8_t * buffer) {
	return(buffer[0]);
}

static uint16_t decodeField_U2(uint8_t * buffer) {
	uint16_t value;
	value  =  (uint16_t) buffer[0];
	value += ((uint16_t) buffer[1]) << 8;
	return(value);
}

static int16_t decodeField_I2(uint8_t * buffer) {
	return((int16_t) decodeField_U2(buffer));
}

static uint16_t decodeField_X2(uint8_t * buffer) {
	return(decodeField_U2(buffer));
}

static uint32_t decodeField_U4(uint8_t * buffer) {
	uint32_t value;
	value  =  (uint32_t) buffer[0];
	value += ((uint32_t) buffer[1]) << 8;
	value += ((uint32_t) buffer[2]) << 16;
	value += ((uint32_t) buffer[3]) << 24;
	return(value);
}

static int32_t decodeField_I4(uint8_t * buffer) {
	return((int32_t) decodeField_U4(buffer));
}

static uint32_t decodeField_X4(uint8_t * buffer) {
	return(decodeField_U4(buffer));
}

static float decodeField_R4(uint8_t * buffer) {
	uint32_t bitPattern = decodeField_X4(buffer);
	return(*((float*)(&bitPattern)));
}

static double decodeField_R8(uint8_t * buffer) {
	uint64_t bitPattern;
	bitPattern  =  (uint64_t) buffer[0];
	bitPattern += ((uint64_t) buffer[1]) << 8;
	bitPattern += ((uint64_t) buffer[2]) << 16;
	bitPattern += ((uint64_t) buffer[3]) << 24;
	bitPattern += ((uint64_t) buffer[4]) << 32;
	bitPattern += ((uint64_t) buffer[5]) << 40;
	bitPattern += ((uint64_t) buffer[6]) << 48;
	bitPattern += ((uint64_t) buffer[7]) << 56;
	return(*((double*)(&bitPattern)));
}

static char decodeField_CH(uint8_t * buffer) {
	return((char) buffer[0]);
}

static void UBXPacket_CFG_MSG::encodePollRequest(std::vector<uint8_t> & UBXPacket, uint8_t msgClass, uint8_t msgID) {
	encodeHeader  (UBXPacket, MESSAGE_CLASS_CFG, MESSAGE_ID_CFG_MSG, 2U);
	encodeField_U1(UBXPacket, msgClass);
	encodeField_U1(UBXPacket, msgID);
	addChecksum   (UBXPacket);
}

static void UBXPacket_CFG_MSG::encodeSet(std::vector<uint8_t> & UBXPacket, uint8_t msgClass, uint8_t msgID, uint8_t *rate) {
	//rate must point to an array of 6 consecutive uint8_t items (one for each target)
	encodeHeader  (UBXPacket, MESSAGE_CLASS_CFG, MESSAGE_ID_CFG_MSG, 8U);
	encodeField_U1(UBXPacket, msgClass);
	encodeField_U1(UBXPacket, msgID);
	encodeField_U1(UBXPacket, rate[0]);
	encodeField_U1(UBXPacket, rate[1]);
	encodeField_U1(UBXPacket, rate[2]);
	encodeField_U1(UBXPacket, rate[3]);
	encodeField_U1(UBXPacket, rate[4]);
	encodeField_U1(UBXPacket, rate[5]);
	addChecksum   (UBXPacket);
}

static void UBXPacket_CFG_MSG::encodeSet(std::vector<uint8_t> & UBXPacket, uint8_t msgClass, uint8_t msgID, uint8_t rate) {
	encodeHeader  (UBXPacket, MESSAGE_CLASS_CFG, MESSAGE_ID_CFG_MSG, 3U);
	encodeField_U1(UBXPacket, msgClass);
	encodeField_U1(UBXPacket, msgID);
	encodeField_U1(UBXPacket, rate);
	addChecksum   (UBXPacket);
}

static void UBXPacket_CFG_MSG::decodeGet(uint8_t * rawUBXPacket, uint8_t &msgClass, uint8_t &msgID, std::vector<uint8_t> &rate) {
	msgClass = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 0U);
	msgID    = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 1U);
	rate.resize(6);
	rate[0] = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 2U);
	rate[1] = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 3U);
	rate[2] = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 4U);
	rate[3] = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 5U);
	rate[4] = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 6U);
	rate[5] = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 7U);
}

static void UBXPacket_CFG_PRT::encodePollRequest(std::vector<uint8_t> & UBXPacket) {
	encodeHeader  (UBXPacket, MESSAGE_CLASS_CFG, MESSAGE_ID_CFG_PRT, 0U);
	addChecksum   (UBXPacket);
}

static void UBXPacket_CFG_PRT::encodePollRequest(std::vector<uint8_t> & UBXPacket, uint8_t PortID) {
	encodeHeader  (UBXPacket, MESSAGE_CLASS_CFG, MESSAGE_ID_CFG_PRT, 1U);
	encodeField_U1(UBXPacket, PortID);
	addChecksum   (UBXPacket);
}

static void UBXPacket_CFG_PRT::encodeSet(std::vector<uint8_t> & UBXPacket, uint8_t PortID, uint16_t txReady, uint32_t mode,
                                         uint32_t baudRate, uint16_t inProtoMask, uint16_t outProtoMask) {
	encodeHeader  (UBXPacket, MESSAGE_CLASS_CFG, MESSAGE_ID_CFG_PRT, 20U);
	encodeField_U1(UBXPacket, PortID);
	encodeField_U1(UBXPacket, 0U); //Field "reserved0"
	encodeField_X2(UBXPacket, txReady);
	encodeField_X4(UBXPacket, mode);
	encodeField_U4(UBXPacket, baudRate);
	encodeField_X2(UBXPacket, inProtoMask);
	encodeField_X2(UBXPacket, outProtoMask);
	encodeField_U2(UBXPacket, 0U); //Field "reserved4"
	encodeField_U2(UBXPacket, 0U); //Field "reserved5"
	addChecksum   (UBXPacket);
}

static void UBXPacket_CFG_PRT::decodeGet(uint8_t * rawUBXPacket, uint8_t &PortID, uint16_t &txReady, uint32_t &mode,
                                         uint32_t &baudRate, uint16_t &inProtoMask, uint16_t &outProtoMask) {
	PortID       = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 0U);
	txReady      = decodeField_X2(rawUBXPacket + UBX_HEADER_LENGTH + 2U);
	mode         = decodeField_X4(rawUBXPacket + UBX_HEADER_LENGTH + 4U);
	baudRate     = decodeField_U4(rawUBXPacket + UBX_HEADER_LENGTH + 8U);
	inProtoMask  = decodeField_X2(rawUBXPacket + UBX_HEADER_LENGTH + 12U);
	outProtoMask = decodeField_X2(rawUBXPacket + UBX_HEADER_LENGTH + 14U);
}

static void UBXPacket_CFG_RATE::encodePollRequest(std::vector<uint8_t> & UBXPacket) {
	encodeHeader  (UBXPacket, MESSAGE_CLASS_CFG, MESSAGE_ID_CFG_RATE, 0U);
	addChecksum   (UBXPacket);
}

static void UBXPacket_CFG_RATE::encodeSet(std::vector<uint8_t> & UBXPacket, uint16_t measRate, uint16_t navRate, uint16_t timeRef) {
	encodeHeader  (UBXPacket, MESSAGE_CLASS_CFG, MESSAGE_ID_CFG_RATE, 6U);
	encodeField_U2(UBXPacket, measRate);
	encodeField_U2(UBXPacket, navRate);
	encodeField_U2(UBXPacket, timeRef);
	addChecksum   (UBXPacket);
}

static void UBXPacket_CFG_RATE::decodeGet(uint8_t * rawUBXPacket, uint16_t &measRate, uint16_t &navRate, uint16_t &timeRef) {
	measRate = decodeField_U2(rawUBXPacket + UBX_HEADER_LENGTH + 0U);
	navRate  = decodeField_U2(rawUBXPacket + UBX_HEADER_LENGTH + 2U);
	timeRef  = decodeField_U2(rawUBXPacket + UBX_HEADER_LENGTH + 4U);
}

void UBXPacket_MON_HW::encodePollRequest(std::vector<uint8_t> & UBXPacket) {
	encodeHeader  (UBXPacket, MESSAGE_CLASS_MON, MESSAGE_ID_MON_HW, 0U);
	addChecksum   (UBXPacket);
}

void UBXPacket_MON_HW::decodeGet(uint8_t * rawUBXPacket, uint32_t &pinSel, uint32_t &pinBank, uint32_t &pinDir, uint32_t &pinVal, uint16_t &noisePerMS,
                                 uint16_t &agcCnt, uint8_t &aStatus, uint8_t &aPower, uint8_t &flags, uint32_t &usedMask, std::vector<uint8_t> &VP,
                                 uint8_t &jamInd, uint32_t &pinIrq, uint32_t &pullH, uint32_t &pullL) {
	pinSel     = decodeField_X4(rawUBXPacket + UBX_HEADER_LENGTH + 0U);
	pinBank    = decodeField_X4(rawUBXPacket + UBX_HEADER_LENGTH + 4U);
	pinDir     = decodeField_X4(rawUBXPacket + UBX_HEADER_LENGTH + 8U);
	pinVal     = decodeField_X4(rawUBXPacket + UBX_HEADER_LENGTH + 12U);
	noisePerMS = decodeField_U2(rawUBXPacket + UBX_HEADER_LENGTH + 16U);
	agcCnt     = decodeField_U2(rawUBXPacket + UBX_HEADER_LENGTH + 18U);
	aStatus    = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 20U);
	aPower     = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 21U);
	flags      = decodeField_X1(rawUBXPacket + UBX_HEADER_LENGTH + 22U);
	usedMask   = decodeField_X4(rawUBXPacket + UBX_HEADER_LENGTH + 24U);
	
	VP.resize(25U);
	for (unsigned int n = 0U; n < 25U; n++)
		VP[n] = decodeField_X1(rawUBXPacket + UBX_HEADER_LENGTH + 28U + n);
	
	jamInd     = decodeField_U1(rawUBXPacket + UBX_HEADER_LENGTH + 53U);
	pinIrq     = decodeField_X4(rawUBXPacket + UBX_HEADER_LENGTH + 56U);
	pullH      = decodeField_X4(rawUBXPacket + UBX_HEADER_LENGTH + 60U);
	pullL      = decodeField_X4(rawUBXPacket + UBX_HEADER_LENGTH + 64U);
}

//Read a full packet - returns true on success (and checksum pass) and false on timeout
//This will skip data until it sees the proper sync fields, thereby providing auto-resyncronization for the bytestream
static bool ReadPacket(serial::Serial * serialDev, std::vector<uint8_t> & UBXPacket) {
	std::chrono::time_point<std::chrono::steady_clock> entryTime = std::chrono::steady_clock::now();
	UBXPacket.clear();
	while (true) {
		if (SecondsElapsed(entryTime, std::chrono::steady_clock::now()) >= 1.0)
			return false;
		
		try { serialDev->read(UBXPacket, 1U); }
		catch (...) { return false; }
		if (UBXPacket.empty())
			return false;
		if (UBXPacket[0] != 181U) {
			UBXPacket.clear();
			continue;
		}
		
		try { serialDev->read(UBXPacket, 1U); }
		catch (...) { return false; }
		if (UBXPacket.size() < 2U)
			return false;
		if (UBXPacket[1] != 98U) {
			UBXPacket.clear();
			continue;
		}
		break;
	}
	
	try { serialDev->read(UBXPacket, 6U); }
	catch (...) { return false; }
	if (UBXPacket.size() < 8U)
		return false;
	uint16_t payloadLength = decodeField_U2(&(UBXPacket[0]) + 4);
	try { serialDev->read(UBXPacket, (size_t) payloadLength); }
	catch (...) { return false; }
	if (UBXPacket.size() < size_t(payloadLength + 8U))
		return false;
	return isChecksumGood(&(UBXPacket[0]));
}

//Read packets until we timeout or get one with the given class and ID
static bool ReadPacket(serial::Serial * serialDev, std::vector<uint8_t> & UBXPacket, uint8_t MessageClass, uint8_t MessageID) {
	std::chrono::time_point<std::chrono::steady_clock> entryTime = std::chrono::steady_clock::now();
	while (true) {
		if (SecondsElapsed(entryTime, std::chrono::steady_clock::now()) >= 2.0)
			return false;
		
		if (! ReadPacket(serialDev, UBXPacket))
			return false;
		
		if ((getMessageClass(&(UBXPacket[0])) == MessageClass) && (getMessageID(&(UBXPacket[0])) == MessageID))
			return true;
	}
}

//Look for an acknowledgement packet in response to a message with the given class and ID. If no such packet is found before
//timeout, or if a NAK is received instead, we return false.
static bool isAcknowledged(serial::Serial * serialDev, uint8_t MessageClass, uint8_t MessageID) {
	std::chrono::time_point<std::chrono::steady_clock> entryTime = std::chrono::steady_clock::now();
	std::vector<uint8_t> UBXPacket;
	while (true) {
		if (SecondsElapsed(entryTime, std::chrono::steady_clock::now()) >= 2.0)
			return false;
		
		if (! ReadPacket(serialDev, UBXPacket))
			return false;
		
		if (getMessageClass(&(UBXPacket[0])) != MESSAGE_CLASS_ACK)
			continue;
		
		if ((UBXPacket[6] != MessageClass) || (UBXPacket[7] != MessageID))
			continue;
		
		if (getMessageID(&(UBXPacket[0])) == MESSAGE_ID_ACK_NAK)
			std::cerr << "Warning: NAK received for source message class=" << (unsigned int) MessageClass << ", ID=" << (unsigned int) MessageID << "\r\n";
		return (getMessageID(&(UBXPacket[0])) == MESSAGE_ID_ACK_ACK);
	}
}


