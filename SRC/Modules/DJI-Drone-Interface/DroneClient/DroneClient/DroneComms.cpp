//Project Includes
#include "DroneComms.hpp"

#define PI 3.14159265358979

// ****************************************************************************************************************************************
// ***************************************************   Standard-type Field encoders   ***************************************************
// ****************************************************************************************************************************************
static void encodeField_uint8 (std::vector<uint8_t> & Buffer, uint8_t  x) { Buffer.push_back(x); }

static void encodeField_uint16 (std::vector<uint8_t> & Buffer, uint16_t x) {
	Buffer.push_back((uint8_t) (x >> 8 ));
	Buffer.push_back((uint8_t)  x       );
}

static void encodeField_uint32 (std::vector<uint8_t> & Buffer, uint32_t x) {
	Buffer.push_back((uint8_t) (x >> 24));
	Buffer.push_back((uint8_t) (x >> 16));
	Buffer.push_back((uint8_t) (x >> 8 ));
	Buffer.push_back((uint8_t)  x       );
}

static void encodeField_uint64 (std::vector<uint8_t> & Buffer, uint64_t x) {
	Buffer.push_back((uint8_t) (x >> 56));
	Buffer.push_back((uint8_t) (x >> 48));
	Buffer.push_back((uint8_t) (x >> 40));
	Buffer.push_back((uint8_t) (x >> 32));
	Buffer.push_back((uint8_t) (x >> 24));
	Buffer.push_back((uint8_t) (x >> 16));
	Buffer.push_back((uint8_t) (x >> 8 ));
	Buffer.push_back((uint8_t)  x       );
}

static void encodeField_int8   (std::vector<uint8_t> & Buffer, int8_t  x) {encodeField_uint8 (Buffer, (uint8_t)  x);}
static void encodeField_int16  (std::vector<uint8_t> & Buffer, int16_t x) {encodeField_uint16(Buffer, (uint16_t) x);}
static void encodeField_int32  (std::vector<uint8_t> & Buffer, int32_t x) {encodeField_uint32(Buffer, (uint32_t) x);}
static void encodeField_int64  (std::vector<uint8_t> & Buffer, int64_t x) {encodeField_uint64(Buffer, (uint64_t) x);}
static void encodeField_float32(std::vector<uint8_t> & Buffer, float  x)  {encodeField_uint32(Buffer, reinterpret_cast<uint32_t &>(x));}
static void encodeField_float64(std::vector<uint8_t> & Buffer, double x)  {encodeField_uint64(Buffer, reinterpret_cast<uint64_t &>(x));}

//Note: Technically, the memory encoding of floating-point values is separate from machine "Endianness" but it is usually the case
//that floating point "Endianness" matches integer endianness and this code will work. However, if our integers are decoding correctly and
//all of our floats are not then we might be dealing with an obnoxious mixed-endianness platform/Architecture and we will need to encode
//and decode floats differently on that platform.

// ****************************************************************************************************************************************
// ***************************************************   Compound-type Field encoders   ***************************************************
// ****************************************************************************************************************************************
static void encodeField_String (std::vector<uint8_t> & Buffer, const std::string & x) {
	encodeField_uint32(Buffer, (uint32_t) x.size());
	for (char item : x)
		Buffer.push_back((uint8_t) item);
}

static void encodeField_Image (std::vector<uint8_t> & Buffer, const Image * x) {
	encodeField_uint16(Buffer, (uint16_t) x->rows);
	encodeField_uint16(Buffer, (uint16_t) x->cols);
    
    int buffer_length = x->rows * x->cols * x->size_pixel;
    
    for (int i = 0; i < buffer_length; i += x->size_pixel) {
        if (x->bitmap == nullptr) {
            encodeField_uint8(Buffer, (uint8_t) 255);
            encodeField_uint8(Buffer, (uint8_t) 0);
            encodeField_uint8(Buffer, (uint8_t) 0);
        }
        encodeField_uint8(Buffer, (uint8_t) x->bitmap[i]);
        encodeField_uint8(Buffer, (uint8_t) x->bitmap[i + 1]);
        encodeField_uint8(Buffer, (uint8_t) x->bitmap[i + 2]);
    }
}


// ****************************************************************************************************************************************
// ***************************************************   Standard-type Field decoders   ***************************************************
// ****************************************************************************************************************************************
static uint8_t decodeField_uint8 (std::vector<uint8_t>::const_iterator & Iter) { return(*Iter++); }

static uint16_t decodeField_uint16 (std::vector<uint8_t>::const_iterator & Iter) {
	uint16_t value;
	value  = (uint16_t) *Iter++;  value <<= 8;
	value += (uint16_t) *Iter++;
	return(value);
}

static uint32_t decodeField_uint32 (std::vector<uint8_t>::const_iterator & Iter) {
	uint32_t value;
     value  = (uint32_t) *Iter++;  value <<= 8;
     value += (uint32_t) *Iter++;  value <<= 8;
     value += (uint32_t) *Iter++;  value <<= 8;
     value += (uint32_t) *Iter++;
     return(value);
}

static uint64_t decodeField_uint64 (std::vector<uint8_t>::const_iterator & Iter) {
	uint64_t value;
     value  = (uint64_t) *Iter++;  value <<= 8;
     value += (uint64_t) *Iter++;  value <<= 8;
     value += (uint64_t) *Iter++;  value <<= 8;
     value += (uint64_t) *Iter++;  value <<= 8;
     value += (uint64_t) *Iter++;  value <<= 8;
     value += (uint64_t) *Iter++;  value <<= 8;
     value += (uint64_t) *Iter++;  value <<= 8;
     value += (uint64_t) *Iter++;
     return(value);
}

static int8_t   decodeField_int8   (std::vector<uint8_t>::const_iterator & Iter) {return((int8_t) *Iter++);}
static int16_t  decodeField_int16  (std::vector<uint8_t>::const_iterator & Iter) {return((int16_t) decodeField_uint16(Iter));}
static int32_t  decodeField_int32  (std::vector<uint8_t>::const_iterator & Iter) {return((int32_t) decodeField_uint32(Iter));}
static int64_t  decodeField_int64  (std::vector<uint8_t>::const_iterator & Iter) {return((int64_t) decodeField_uint64(Iter));}

static float decodeField_float32 (std::vector<uint8_t>::const_iterator & Iter) {
	uint32_t bitPattern = decodeField_uint32(Iter);
	return reinterpret_cast<float &>(bitPattern);
}

static double decodeField_float64 (std::vector<uint8_t>::const_iterator & Iter) {
	uint64_t bitPattern = decodeField_uint64(Iter);
	return reinterpret_cast<double &>(bitPattern);
}

//Note: See comment about floats in field encoder section above.

// ****************************************************************************************************************************************
// ***************************************************   Compound-type Field decoders   ***************************************************
// ****************************************************************************************************************************************
//MaxBytes is the maximum number of bytes that can belong to the full string object (size field included). This is a reference argument that will be
//decremented by the number of decoded bytes. If the string field advertises a length that would make the full string object exceed this number of
//bytes, then we replace the advertised length with the largest safe value.
static std::string decodeField_String (std::vector<uint8_t>::const_iterator & Iter, unsigned int & MaxBytes) {
	if (MaxBytes < 4U) {
		fprintf(stderr,"Warning in decodeField_String: Not enough bytes left for string. Aborting decode.\r\n");
		Iter += MaxBytes;
		MaxBytes = 0U;
		return std::string();
	}
	
	unsigned int size = (unsigned int) decodeField_uint32(Iter);
	unsigned int bytesForObject = size + 4U;
	if (bytesForObject > MaxBytes) {
		fprintf(stderr,"Warning in decodeField_String: Not enough bytes left for string. Aborting decode.\r\n");
		Iter += MaxBytes;
		MaxBytes = 0U;
		return std::string();
	}
	
	std::string S;
	for (unsigned int n = 0U; n < size; n++)
		S.push_back((char) *Iter++);
	MaxBytes -= bytesForObject;
	return(S);
}

namespace DroneInterface {
	// ****************************************************************************************************************************************
	// ******************************************************   Packet Implementation   *******************************************************
	// ****************************************************************************************************************************************
	void Packet::Clear(void) {
		m_data.clear();
		M_highLevelFieldsValid = false;
	}
	
	bool Packet::IsFinished(void) {
		if (m_data.size() < 7U)
			return false;
		if (! M_highLevelFieldsValid) {
			auto iter = m_data.cbegin() + 2;
			m_size = decodeField_uint32(iter);
			m_PID  = decodeField_uint8(iter);
			M_highLevelFieldsValid = true;
		}
		return (m_data.size() >= (size_t) m_size);
	}

	bool Packet::BytesNeeded(uint32_t & ByteCount) {
		if (m_data.size() < 7U)
			return false;
		if (! M_highLevelFieldsValid) {
			auto iter = m_data.cbegin() + 2;
			m_size = decodeField_uint32(iter);
			m_PID  = decodeField_uint8(iter);
			M_highLevelFieldsValid = true;
		}
		ByteCount = m_size - uint32_t(m_data.size());
		return true;
	}

	//Take total packet size and PID and add sync, size, and PID fields to m_data
	void Packet::AddHeader(uint32_t Size, uint8_t PID) {
		encodeField_uint16(m_data, (uint16_t) 55975U);
		encodeField_uint32(m_data, Size);
		encodeField_uint8 (m_data, PID);
	}

	//Based on current contents of m_data (which should be fully populated except for the hash field) compute and add hash field
	void Packet::AddHash(void) {
		uint8_t hashA = 0U;
		uint8_t hashB = 0U;
		for (size_t n = 0U; n < m_data.size(); n++) {
			hashA += m_data[n];
			hashB += hashA;
		}
		encodeField_uint8(m_data, hashA);
		encodeField_uint8(m_data, hashB);
	}

    void Packet::GetCharBuffer(unsigned char* buf, unsigned int &length) {
        std::copy(m_data.begin(), m_data.end(), buf);
        length = m_data.size();
    }

	//Returns false if not enough data to decode PID
	bool Packet::GetPID(uint8_t & PID) const {
		if (m_data.size() < 7U)
			return(false); //Not enough data in buffer yet
		auto iter = m_data.cbegin() + 6U;
		PID = decodeField_uint8(iter);
		return true;
	}

	//Returns true if m_data has correct size and passes hash check and false otherwise
	bool Packet::CheckHash(void) const {
		if (m_data.size() < 9U)
			return false; //Packet cannot be valid because it is below min size
		auto iter = m_data.cbegin() + 2U;
		uint32_t size = decodeField_uint32(iter);
		if (size_t(size) != m_data.size())
			return false; //Packet cannot be valid because it's size is different than advertised
		uint8_t hashA = 0U;
		uint8_t hashB = 0U;
		for (size_t n = 0U; n < m_data.size() - 2U; n++) {
			hashA += m_data[n];
			hashB += hashA;
		}
		return ((hashA == m_data[m_data.size() - 2U]) && (hashB == m_data[m_data.size() - 1U]));
	}

	//Returns true if PID matches, size matches advertised size, and hash is good.
	bool Packet::CheckHashSizeAndPID(uint8_t PID) const {
		if (! CheckHash())
			return false;
		auto iter = m_data.cbegin() + 6U;
		return (decodeField_uint8(iter) == PID);
	}


	// ****************************************************************************************************************************************
	// ************************************************   Packet_CoreTelemetry Implementation   ***********************************************
	// ****************************************************************************************************************************************
	bool Packet_CoreTelemetry::operator==(Packet_CoreTelemetry const & Other) const {
		return (this->IsFlying  == Other.IsFlying)  &&
		       (this->Latitude  == Other.Latitude)  &&
		       (this->Longitude == Other.Longitude) &&
		       (this->Altitude  == Other.Altitude)  &&
		       (this->HAG       == Other.HAG)       &&
		       (this->V_N       == Other.V_N)       &&
		       (this->V_E       == Other.V_E)       &&
		       (this->V_D       == Other.V_D)       &&
		       (this->Yaw       == Other.Yaw)       &&
		       (this->Pitch     == Other.Pitch)     &&
		       (this->Roll      == Other.Roll);
	}
	
	void Packet_CoreTelemetry::Serialize(Packet & TargetPacket) const {
		TargetPacket.Clear();
		TargetPacket.AddHeader(uint32_t(9U + 69U), uint8_t(0U));
		encodeField_uint8  (TargetPacket.m_data, IsFlying);
		encodeField_float64(TargetPacket.m_data, Latitude);
		encodeField_float64(TargetPacket.m_data, Longitude);
		encodeField_float64(TargetPacket.m_data, Altitude);
		encodeField_float64(TargetPacket.m_data, HAG);
		encodeField_float32(TargetPacket.m_data, V_N);
		encodeField_float32(TargetPacket.m_data, V_E);
		encodeField_float32(TargetPacket.m_data, V_D);
		encodeField_float64(TargetPacket.m_data, Yaw);
		encodeField_float64(TargetPacket.m_data, Pitch);
		encodeField_float64(TargetPacket.m_data, Roll);
		TargetPacket.AddHash();
	}

	bool Packet_CoreTelemetry::Deserialize(Packet const & SourcePacket) {
		if (! SourcePacket.CheckHashSizeAndPID((uint8_t) 0U))
			return false;
		if (SourcePacket.m_data.size() != 9U + 69U)
			return false;
		
		auto iter = SourcePacket.m_data.cbegin() + 7U; //Const iterater to begining of payload
		IsFlying  = decodeField_uint8(iter);
		Latitude  = decodeField_float64(iter);
		Longitude = decodeField_float64(iter);
		Altitude  = decodeField_float64(iter);
		HAG       = decodeField_float64(iter);
		V_N       = decodeField_float32(iter);
		V_E       = decodeField_float32(iter);
		V_D       = decodeField_float32(iter);
		Yaw       = decodeField_float64(iter);
		Pitch     = decodeField_float64(iter);
		Roll      = decodeField_float64(iter);
		return true;
	}

	// ****************************************************************************************************************************************
	// **********************************************   Packet_ExtendedTelemetry Implementation   *********************************************
	// ****************************************************************************************************************************************
	bool Packet_ExtendedTelemetry::operator==(Packet_ExtendedTelemetry const & Other) const {
		return (this->GNSSSatCount == Other.GNSSSatCount) &&
		       (this->GNSSSignal   == Other.GNSSSignal)   &&
		       (this->MaxHeight    == Other.MaxHeight)    &&
		       (this->MaxDist      == Other.MaxDist)      &&
		       (this->BatLevel     == Other.BatLevel)     &&
		       (this->BatWarning   == Other.BatWarning)   &&
		       (this->WindLevel    == Other.WindLevel)    &&
		       (this->DJICam       == Other.DJICam)       &&
		       (this->FlightMode   == Other.FlightMode)   &&
		       (this->MissionID    == Other.MissionID)    &&
		       (this->DroneSerial  == Other.DroneSerial);
	}
	
	void Packet_ExtendedTelemetry::Serialize(Packet & TargetPacket) const {
		TargetPacket.Clear();
		TargetPacket.AddHeader(uint32_t(9U + 12U + 4U + DroneSerial.size()), uint8_t(1U));
		encodeField_uint16(TargetPacket.m_data, GNSSSatCount);
		encodeField_uint8 (TargetPacket.m_data, GNSSSignal);
		encodeField_uint8 (TargetPacket.m_data, MaxHeight);
		encodeField_uint8 (TargetPacket.m_data, MaxDist);
		encodeField_uint8 (TargetPacket.m_data, BatLevel);
		encodeField_uint8 (TargetPacket.m_data, BatWarning);
		encodeField_uint8 (TargetPacket.m_data, WindLevel);
		encodeField_uint8 (TargetPacket.m_data, DJICam);
		encodeField_uint8 (TargetPacket.m_data, FlightMode);
		encodeField_uint16(TargetPacket.m_data, MissionID);
		encodeField_String(TargetPacket.m_data, DroneSerial);
		TargetPacket.AddHash();
	}

	bool Packet_ExtendedTelemetry::Deserialize(Packet const & SourcePacket) {
		if (! SourcePacket.CheckHashSizeAndPID((uint8_t) 1U))
			return false;
		if (SourcePacket.m_data.size() < 9U + 16U)
			return false;
		
		auto iter = SourcePacket.m_data.cbegin() + 7U; //Const iterater to begining of payload
		GNSSSatCount = decodeField_uint16(iter);
		GNSSSignal   = decodeField_uint8(iter);
		MaxHeight    = decodeField_uint8(iter);
		MaxDist      = decodeField_uint8(iter);
		BatLevel     = decodeField_uint8(iter);
		BatWarning   = decodeField_uint8(iter);
		WindLevel    = decodeField_uint8(iter);
		DJICam       = decodeField_uint8(iter);
		FlightMode   = decodeField_uint8(iter);
		MissionID    = decodeField_uint16(iter);
		
		unsigned int MaxBytesForStr = (unsigned int) SourcePacket.m_data.size() - 9U - 12U;
		DroneSerial = decodeField_String(iter, MaxBytesForStr);
		return true;
	}


	// ****************************************************************************************************************************************
	// ****************************************************   Packet_Image Implementation   ***************************************************
	// ****************************************************************************************************************************************

	void Packet_Image::Serialize(Packet & TargetPacket) const {
		TargetPacket.Clear();
		TargetPacket.AddHeader(uint32_t(9U + 4U + 4U + (unsigned int)(Frame->rows * Frame->cols * 3)), uint8_t(2U));
		encodeField_float32(TargetPacket.m_data, TargetFPS);
		encodeField_Image  (TargetPacket.m_data, Frame);
		TargetPacket.AddHash();
	}

	// ****************************************************************************************************************************************
	// ************************************************   Packet_Acknowledgment Implementation   **********************************************
	// ****************************************************************************************************************************************
	bool Packet_Acknowledgment::operator==(Packet_Acknowledgment const & Other) const {
		return (this->Positive  == Other.Positive) &&
		       (this->SourcePID == Other.SourcePID);
	}
	
	void Packet_Acknowledgment::Serialize(Packet & TargetPacket) const {
		TargetPacket.Clear();
		TargetPacket.AddHeader(uint32_t(9U + 2U), uint8_t(3U));
		encodeField_uint8(TargetPacket.m_data, Positive);
		encodeField_uint8(TargetPacket.m_data, SourcePID);
		TargetPacket.AddHash();
	}

	bool Packet_Acknowledgment::Deserialize(Packet const & SourcePacket) {
		if (! SourcePacket.CheckHashSizeAndPID((uint8_t) 3U))
			return false;
		if (SourcePacket.m_data.size() != 9U + 2U)
			return false;
		
		auto iter = SourcePacket.m_data.cbegin() + 7U; //Const iterater to begining of payload
		Positive  = decodeField_uint8(iter);
		SourcePID = decodeField_uint8(iter);
		return true;
	}


	// ****************************************************************************************************************************************
	// ************************************************   Packet_MessageString Implementation   ***********************************************
	// ****************************************************************************************************************************************
	bool Packet_MessageString::operator==(Packet_MessageString const & Other) const {
		return (this->Type    == Other.Type) &&
		       (this->Message == Other.Message);
	}
	
	void Packet_MessageString::Serialize(Packet & TargetPacket) const {
		TargetPacket.Clear();
		TargetPacket.AddHeader(uint32_t(9U + 1U + 4U + Message.size()), uint8_t(4U));
		encodeField_uint8 (TargetPacket.m_data, Type);
		encodeField_String(TargetPacket.m_data, Message);
		TargetPacket.AddHash();
	}

	bool Packet_MessageString::Deserialize(Packet const & SourcePacket) {
		if (! SourcePacket.CheckHashSizeAndPID((uint8_t) 4U))
			return false;
		if (SourcePacket.m_data.size() < 9U + 5U)
			return false;
		
		auto iter = SourcePacket.m_data.cbegin() + 7U; //Const iterater to begining of payload
		Type = decodeField_uint8(iter);
		
		unsigned int MaxBytesForStr = (unsigned int) SourcePacket.m_data.size() - 9U - 1U;
		Message = decodeField_String(iter, MaxBytesForStr);
		return true;
	}


	// ****************************************************************************************************************************************
	// **********************************************   Packet_EmergencyCommand Implementation   **********************************************
	// ****************************************************************************************************************************************
	bool Packet_EmergencyCommand::operator==(Packet_EmergencyCommand const & Other) const {
		return (this->Action == Other.Action);
	}
	
	void Packet_EmergencyCommand::Serialize(Packet & TargetPacket) const {
		TargetPacket.Clear();
		TargetPacket.AddHeader(uint32_t(9U + 1U), uint8_t(255U));
		encodeField_uint8(TargetPacket.m_data, Action);
		TargetPacket.AddHash();
	}

	bool Packet_EmergencyCommand::Deserialize(Packet const & SourcePacket) {
		if (! SourcePacket.CheckHashSizeAndPID((uint8_t) 255U))
            return false;
		if (SourcePacket.m_data.size() != 9U + 1U)
            return false;
		
		auto iter = SourcePacket.m_data.cbegin() + 7U; //Const iterater to begining of payload
		Action = decodeField_uint8(iter);
        return true;
	}
	
	
	// ****************************************************************************************************************************************
	// ************************************************   Packet_CameraControl Implementation   ***********************************************
	// ****************************************************************************************************************************************
	bool Packet_CameraControl::operator==(Packet_CameraControl const & Other) const {
		return (this->Action    == Other.Action) &&
		       (this->TargetFPS == Other.TargetFPS);
	}
	
	void Packet_CameraControl::Serialize(Packet & TargetPacket) const {
		TargetPacket.Clear();
		TargetPacket.AddHeader(uint32_t(9U + 5U), uint8_t(254U));
		encodeField_uint8  (TargetPacket.m_data, Action);
		encodeField_float32(TargetPacket.m_data, TargetFPS);
		TargetPacket.AddHash();
	}
	
	bool Packet_CameraControl::Deserialize(Packet const & SourcePacket) {
		if (! SourcePacket.CheckHashSizeAndPID((uint8_t) 254U))
			return false;
		if (SourcePacket.m_data.size() != 9U + 5U)
			return false;
		
		auto iter = SourcePacket.m_data.cbegin() + 7U; //Const iterater to begining of payload
		Action    = decodeField_uint8(iter);
		TargetFPS = decodeField_float32(iter);
		return true;
	}
	
	
	// ****************************************************************************************************************************************
	// ********************************************   Packet_ExecuteWaypointMission Implementation   ******************************************
	// ****************************************************************************************************************************************
	bool Packet_ExecuteWaypointMission::operator==(Packet_ExecuteWaypointMission const & Other) const {
		if ((this->LandAtEnd != Other.LandAtEnd) || (this->CurvedFlight != Other.CurvedFlight))
			return false;
		if (this->Waypoints.size() != Other.Waypoints.size())
			return false;
		for (size_t n = 0U; n < this->Waypoints.size(); n++) {
			if (! (this->Waypoints[n] == Other.Waypoints[n]))
				return false;
		}
		return true;
	}
	
	void Packet_ExecuteWaypointMission::Serialize(Packet & TargetPacket) const {
		TargetPacket.Clear();
		TargetPacket.AddHeader(uint32_t(9U + 2U + 40U*((unsigned int) Waypoints.size())), uint8_t(253U));
		encodeField_uint8(TargetPacket.m_data, LandAtEnd);
		encodeField_uint8(TargetPacket.m_data, CurvedFlight);
		for (auto const & waypoint : Waypoints) {
			encodeField_float64(TargetPacket.m_data, waypoint.Latitude*180.0/PI);
			encodeField_float64(TargetPacket.m_data, waypoint.Longitude*180.0/PI);
			encodeField_float64(TargetPacket.m_data, waypoint.Altitude);
			encodeField_float32(TargetPacket.m_data, waypoint.CornerRadius);
			encodeField_float32(TargetPacket.m_data, waypoint.Speed);
			encodeField_float32(TargetPacket.m_data, waypoint.LoiterTime);
			encodeField_float32(TargetPacket.m_data, waypoint.GimbalPitch*180.0/PI);
		}
		TargetPacket.AddHash();
	}
	
	bool Packet_ExecuteWaypointMission::Deserialize(Packet const & SourcePacket) {
		if (! SourcePacket.CheckHashSizeAndPID((uint8_t) 253U))
			return false;
		if (SourcePacket.m_data.size() < 9U + 2U + 40U)
			return false;
		
		auto iter    = SourcePacket.m_data.cbegin() + 7U; //Const iterater to begining of payload
		LandAtEnd    = decodeField_uint8(iter);
		CurvedFlight = decodeField_uint8(iter);
		
		Waypoints.clear();
		unsigned int waypointBytes = (unsigned int) SourcePacket.m_data.size() - 9U - 2U;
		if (waypointBytes % 40U != 0U) {
			return false;
		}
		unsigned int numWaypoints = waypointBytes / 40U;
		Waypoints.reserve(numWaypoints);
		for (unsigned int n = 0U; n < numWaypoints; n++) {
			Waypoints.emplace_back();
			Waypoints.back().Latitude     = decodeField_float64(iter)*PI/180.0;
			Waypoints.back().Longitude    = decodeField_float64(iter)*PI/180.0;
			Waypoints.back().Altitude     = decodeField_float64(iter);
			Waypoints.back().CornerRadius = decodeField_float32(iter);
			Waypoints.back().Speed        = decodeField_float32(iter);
			Waypoints.back().LoiterTime   = decodeField_float32(iter);
			Waypoints.back().GimbalPitch  = decodeField_float32(iter)*PI/180.0;
		}
		return true;
	}
	
	
	// ****************************************************************************************************************************************
	// *********************************************   Packet_VirtualStickCommand Implementation   ********************************************
	// ****************************************************************************************************************************************
	bool Packet_VirtualStickCommand::operator==(Packet_VirtualStickCommand const & Other) const {
		return (this->Mode    == Other.Mode) &&
		       (this->Yaw     == Other.Yaw)  &&
		       (this->V_x     == Other.V_x)  &&
		       (this->V_y     == Other.V_y)  &&
		       (this->HAG     == Other.HAG)  &&
		       (this->timeout == Other.timeout);
	}
	
	void Packet_VirtualStickCommand::Serialize(Packet & TargetPacket) const {
		TargetPacket.Clear();
		TargetPacket.AddHeader(uint32_t(9U + 21U), uint8_t(252U));
		encodeField_uint8  (TargetPacket.m_data, Mode);
		encodeField_float32(TargetPacket.m_data, Yaw);
		encodeField_float32(TargetPacket.m_data, V_x);
		encodeField_float32(TargetPacket.m_data, V_y);
		encodeField_float32(TargetPacket.m_data, HAG);
		encodeField_float32(TargetPacket.m_data, timeout);
		TargetPacket.AddHash();
	}
	
	bool Packet_VirtualStickCommand::Deserialize(Packet const & SourcePacket) {
		if (! SourcePacket.CheckHashSizeAndPID((uint8_t) 252U))
			return false;
		if (SourcePacket.m_data.size() != 9U + 21U)
			return false;
		
		auto iter = SourcePacket.m_data.cbegin() + 7U; //Const iterater to begining of payload
		Mode    = decodeField_uint8(iter);
		Yaw     = decodeField_float32(iter);
		V_x     = decodeField_float32(iter);
		V_y     = decodeField_float32(iter);
		HAG     = decodeField_float32(iter);
		timeout = decodeField_float32(iter);
		return true;
	}
}
