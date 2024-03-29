//DroneComms provides data structures and serialization/deserialization support for socket communications between server and client
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <vector>
#include <string>
#include <cstdint>

//External Includes
#include <opencv2/opencv.hpp>

//Project Includes
//#include "Drone.hpp"
#include "DroneDataStructures.h"
namespace DroneInterface {
	//Packet for holding binary, serialized packet data
	class Packet {
		public:
			std::vector<uint8_t> m_data; //Buffer containing the full serialized packet
			
			Packet()  = default;
			~Packet() = default;
			void Clear(void);
			
			//Utilities for stream parsing
			bool IsFinished(void);
			bool BytesNeeded(uint32_t & ByteCount); //Get num bytes needed to finish packet (returns false if more bytes needed before we can answer)
			void ForwardScanForSync(void); //Search the buffer for the sync field - if we find it, throw out everything before it from m_data
			
			//Packet construction utilities
			void AddHeader(uint32_t Size, uint8_t PID); //Take total packet size and PID and add sync, size, and PID fields to m_data
			void AddHash(void); //Based on current contents of m_data (which should be fully populated except for the hash field) compute and add hash field
			
			//Packet decoding utilities
			bool GetPID(uint8_t & PID) const; //Returns false if not enough data to decode PID
			bool CheckHash(void) const; //Returns true if m_data passes hash check and false otherwise
			bool CheckHashSizeAndPID(uint8_t PID) const; //Returns true if PID matches, size matches advertised size, and hash is good.
			
		private:
			//These fields used only for IsFinished() and BytesNeeded()
			bool     M_highLevelFieldsValid = false;
			uint32_t m_size; //Only valid when M_highLevelFieldsValid = true
			uint8_t  m_PID;  //Only valid when M_highLevelFieldsValid = true
	};
	
	//The classes that follow implement packets specified in the ICD. For precise definitions of each field, refer to the ICD
	class Packet_CoreTelemetry {
		public:
			uint8_t IsFlying;  //=1 if FCS.isFlying is true, =0 if false
			double  Latitude;  //WGS84 Latitude (Degrees)
			double  Longitude; //WGS84 Longitude (Degrees)
			double  Altitude;  //WGS84 Altitude (m)
			double  HAG;       //Height above takeoff location
			float   V_N;       //North velocity (m/s)
			float   V_E;       //East velocity (m/s)
			float   V_D;       //Down velocity (m/s)
			double  Yaw;       //Vehicle Yaw (Degrees) - DJI definition
			double  Pitch;     //Vehicle Pitch (Degrees) - DJI definition
			double  Roll;      //Vehicle Roll (Degrees) - DJI definition
			
			Packet_CoreTelemetry()  = default;
			~Packet_CoreTelemetry() = default;
			bool operator==(Packet_CoreTelemetry const & Other) const; //If switching to C++20, default this
			
			void Serialize(Packet & TargetPacket) const; //Populate Packet from fields
			bool Deserialize(Packet const & SourcePacket); //Populate fields from Packet
	};

	class Packet_ExtendedTelemetry {
		public:
			uint16_t GNSSSatCount; //Equals FCS.satelliteCount
			uint8_t  GNSSSignal;   //From FCS.GPSSignalLevel (−1: None, ≥ 0: equals signal level)
			uint8_t  MaxHeight;    //=1 if FCS.hasReachedMaxFlightHeight is true. =0 if false
			uint8_t  MaxDist;      //=1 if FCS.hasReachedMaxFlightRadius if true. =0 if false
			uint8_t  BatLevel;     //Equals DJIBatteryState.chargeRemainingInPercent
			uint8_t  BatWarning;   //0=No Warning, 1=Warning, 2=Serious Warning
			uint8_t  WindLevel;    //Wind level. (−1: FCS.windWarning = Unknown, >= 0:equals warning level)
			uint8_t  DJICam;       //0:No Cam, 1:Cam present but feed off, 2:Cam present and feed on
			uint8_t  FlightMode;   //Based on FCS.flightMode
			uint16_t MissionID;    //Mission ID for current waypoint mission
			std::string DroneSerial;
			
			Packet_ExtendedTelemetry()  = default;
			~Packet_ExtendedTelemetry() = default;
			bool operator==(Packet_ExtendedTelemetry const & Other) const; //If switching to C++20, default this
			
			void Serialize(Packet & TargetPacket) const; //Populate Packet from fields
			bool Deserialize(Packet const & SourcePacket); //Populate fields from Packet
	};
	
	class Packet_Image {
		public:
			float   TargetFPS;
			cv::Mat Frame;
			
			Packet_Image()  = default;
			~Packet_Image() = default;
			bool operator==(Packet_Image const & Other) const; //If switching to C++20, default this
			
			void Serialize(Packet & TargetPacket) const; //Populate Packet from fields
			bool Deserialize(Packet const & SourcePacket); //Populate fields from Packet
	};
	
	class Packet_CompressedImage {
		public:
			float   TargetFPS;
			cv::Mat Frame;
			
			Packet_CompressedImage()  = default;
			~Packet_CompressedImage() = default;
			bool operator==(Packet_CompressedImage const & Other) const; //If switching to C++20, default this
			
			void Serialize(Packet & TargetPacket) const; //Populate Packet from fields
			bool Deserialize(Packet const & SourcePacket); //Populate fields from Packet
	};

	class Packet_Acknowledgment {
		public:
			uint8_t Positive;
			uint8_t SourcePID;
			
			Packet_Acknowledgment()  = default;
			~Packet_Acknowledgment() = default;
			bool operator==(Packet_Acknowledgment const & Other) const; //If switching to C++20, default this
			
			void Serialize(Packet & TargetPacket) const; //Populate Packet from fields
			bool Deserialize(Packet const & SourcePacket); //Populate fields from Packet
	};

	class Packet_MessageString {
		public:
			uint8_t     Type;
			std::string Message;
			
			Packet_MessageString()  = default;
			~Packet_MessageString() = default;
			bool operator==(Packet_MessageString const & Other) const; //If switching to C++20, default this
			
			void Serialize(Packet & TargetPacket) const; //Populate Packet from fields
			bool Deserialize(Packet const & SourcePacket); //Populate fields from Packet
	};

	class Packet_EmergencyCommand {
		public:
			uint8_t Action;
			
			Packet_EmergencyCommand()  = default;
			~Packet_EmergencyCommand() = default;
			bool operator==(Packet_EmergencyCommand const & Other) const; //If switching to C++20, default this
			
			void Serialize(Packet & TargetPacket) const; //Populate Packet from fields
			bool Deserialize(Packet const & SourcePacket); //Populate fields from Packet
	};
	
	class Packet_CameraControl {
		public:
			uint8_t Action;
			float   TargetFPS;
			
			Packet_CameraControl()  = default;
			~Packet_CameraControl() = default;
			bool operator==(Packet_CameraControl const & Other) const; //If switching to C++20, default this
			
			void Serialize(Packet & TargetPacket) const; //Populate Packet from fields
			bool Deserialize(Packet const & SourcePacket); //Populate fields from Packet
	};
	
	//Note: With this packet we use the existing Waypoint struct from Drone.hpp instead of essentially duplicating it.
	//The meanings of the fields of a Waypoint object are defined in Drone.hpp. Since we use radians there and the ICD
	//uses degrees for angle fields (to match the DJI API) we must convert when serializing and deserializing.
	class Packet_ExecuteWaypointMission {
		public:
			uint8_t LandAtEnd;
			uint8_t CurvedFlight;
			std::vector<Waypoint> Waypoints;
			
			Packet_ExecuteWaypointMission()  = default;
			~Packet_ExecuteWaypointMission() = default;
			bool operator==(Packet_ExecuteWaypointMission const & Other) const; //If switching to C++20, default this
			
			void Serialize(Packet & TargetPacket) const; //Populate Packet from fields
			bool Deserialize(Packet const & SourcePacket); //Populate fields from Packet
	};
	
	class Packet_VirtualStickCommand {
		public:
			uint8_t Mode;
			float   Yaw;
			float   V_x;
			float   V_y;
			float   HAG;
			float   timeout;
			
			Packet_VirtualStickCommand()  = default;
			~Packet_VirtualStickCommand() = default;
			bool operator==(Packet_VirtualStickCommand const & Other) const; //If switching to C++20, default this
			
			void Serialize(Packet & TargetPacket) const; //Populate Packet from fields
			bool Deserialize(Packet const & SourcePacket); //Populate fields from Packet
	};
	
	//Stream Operators - used to print packet contents in human-readable form
	std::ostream & operator<<(std::ostream & Str, Packet_CoreTelemetry          const & v);
	std::ostream & operator<<(std::ostream & Str, Packet_ExtendedTelemetry      const & v);
	std::ostream & operator<<(std::ostream & Str, Packet_Image                  const & v);
	std::ostream & operator<<(std::ostream & Str, Packet_Acknowledgment         const & v);
	std::ostream & operator<<(std::ostream & Str, Packet_MessageString          const & v);
	std::ostream & operator<<(std::ostream & Str, Packet_EmergencyCommand       const & v);
	std::ostream & operator<<(std::ostream & Str, Packet_CameraControl          const & v);
	std::ostream & operator<<(std::ostream & Str, Packet_ExecuteWaypointMission const & v);
	std::ostream & operator<<(std::ostream & Str, Packet_VirtualStickCommand    const & v);
	
}



