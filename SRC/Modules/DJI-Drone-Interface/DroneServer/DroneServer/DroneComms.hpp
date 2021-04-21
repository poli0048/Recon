#pragma once

//System Includes
#include <vector>
#include <string>
#include <cstdint>

//External Includes
#include <opencv2/opencv.hpp>

//Project Includes
//#include "Drone.hpp" // Circular dependency
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
			uint8_t IsFlying;
			double  Latitude;
			double  Longitude;
			double  Altitude;
			double  HAG;
			float   V_N;
			float   V_E;
			float   V_D;
			double  Yaw;
			double  Pitch;
			double  Roll;
			
			Packet_CoreTelemetry()  = default;
			~Packet_CoreTelemetry() = default;
			bool operator==(Packet_CoreTelemetry const & Other) const; //If switching to C++20, default this
			
			void Serialize(Packet & TargetPacket) const; //Populate Packet from fields
			bool Deserialize(Packet const & SourcePacket); //Populate fields from Packet
	};

	class Packet_ExtendedTelemetry {
		public:
			uint16_t GNSSSatCount;
			uint8_t  GNSSSignal;
			uint8_t  MaxHeight;
			uint8_t  MaxDist;
			uint8_t  BatLevel;
			uint8_t  BatWarning;
			uint8_t  WindLevel;
			uint8_t  DJICam;
			uint8_t  FlightMode;
			uint16_t MissionID;
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
	//std::ostream & operator<<(std::ostream & Str, Packet_ExecuteWaypointMission const & v);
	std::ostream & operator<<(std::ostream & Str, Packet_VirtualStickCommand    const & v);
	
}