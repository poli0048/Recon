// System Includes
#include <iostream>
#include <condition_variable>
#include <iostream>
#include <mutex>

// C Includes
#include <signal.h>

// External Includes
#include <tacopie/tacopie>

//Project Includes
#include "Drone.hpp"
#include "../../Utilities.hpp"
#include "../../UI/VehicleControlWidget.hpp"
#include "../Guidance/Guidance.hpp"
#include "DroneUtils.hpp"
#include "../GNSS-Receiver/GNSSReceiver.hpp"

#define PI 3.14159265358979

namespace DroneInterface {
	RealDrone::RealDrone(const std::shared_ptr<tacopie::tcp_client> & client) {
		m_client = client.get();
		m_TimestampOfLastFPSReport = std::chrono::steady_clock::now();
		
		//Set disconnection handler
		client->set_on_disconnection_handler(std::bind(&RealDrone::DisconnectHandler, this));
		
		//Kick off reading from socket
		client->async_read({ 1024, bind(&RealDrone::DataReceivedHandler, this, client, std::placeholders::_1) });
		
		m_isConnected = true;
	}
	
	RealDrone::~RealDrone() {
		std::scoped_lock lock(m_mutex_A, m_mutex_B);
		if (m_packet_fragment != nullptr)
			delete m_packet_fragment;
	}
	
	void RealDrone::DisconnectHandler(void) {
		std::cerr << "Drone socket disconnected.\r\n";
		m_isConnected = false;
	}
	
	void RealDrone::LoadTestWaypointMission(WaypointMission & testMission){

	    testMission.Waypoints.clear();
	    testMission.Waypoints.emplace_back();
	    testMission.Waypoints.back().Latitude     =  44.237308 * PI/180.0; // radians
	    testMission.Waypoints.back().Longitude    = -95.307433  * PI/180.0; // radians
	    testMission.Waypoints.back().RelAltitude  =  30.5; // m
	    testMission.Waypoints.back().CornerRadius =  0.0f;  // m (Not used since we aren't using curved trajectories)
	    testMission.Waypoints.back().Speed        =  16.0f;  // m/s
	    testMission.Waypoints.back().LoiterTime   = std::nanf("");
	    testMission.Waypoints.back().GimbalPitch  = std::nanf("");

	    testMission.Waypoints.emplace_back();
	    testMission.Waypoints.back().Latitude     =  44.237308 * PI/180.0; // radians
	    testMission.Waypoints.back().Longitude    = -95.309309 * PI/180.0; // radians
	    testMission.Waypoints.back().RelAltitude  =  30.5; // m
	    testMission.Waypoints.back().CornerRadius = 0.2f;   // m (Not used since we aren't using curved trajectories)
	    testMission.Waypoints.back().Speed        = 9.5f;   // m/s
	    testMission.Waypoints.back().LoiterTime   = std::nanf("");
	    testMission.Waypoints.back().GimbalPitch  = std::nanf("");

	    testMission.Waypoints.emplace_back();
	    testMission.Waypoints.back().Latitude     =  44.237819 * PI/180.0; // radians
	    testMission.Waypoints.back().Longitude    = -95.309309 * PI/180.0; // radians
	    testMission.Waypoints.back().RelAltitude  =  30.5; // m
	    testMission.Waypoints.back().CornerRadius = 0.2f;   // m (Not used since we aren't using curved trajectories)
	    testMission.Waypoints.back().Speed        = 9.5f;   // m/s
	    testMission.Waypoints.back().LoiterTime   = std::nanf("");
	    testMission.Waypoints.back().GimbalPitch  = std::nanf("");

	    testMission.Waypoints.emplace_back();
	    testMission.Waypoints.back().Latitude     =  44.237819 * PI/180.0; // radians
	    testMission.Waypoints.back().Longitude    = -95.307433 * PI/180.0; // radians
	    testMission.Waypoints.back().RelAltitude  =  30.5; // m
	    testMission.Waypoints.back().CornerRadius = 0.2f;   // m (Not used since we aren't using curved trajectories)
	    testMission.Waypoints.back().Speed        = 9.5f;   // m/s
	    testMission.Waypoints.back().LoiterTime   = std::nanf("");
	    testMission.Waypoints.back().GimbalPitch  = std::nanf("");

	    testMission.Waypoints.emplace_back();
	    testMission.Waypoints.back().Latitude     =  44.238344 * PI/180.0; // radians
	    testMission.Waypoints.back().Longitude    = -95.307433 * PI/180.0; // radians
	    testMission.Waypoints.back().RelAltitude  =  30.5; // m
	    testMission.Waypoints.back().CornerRadius = 0.2f;   // m (Not used since we aren't using curved trajectories)
	    testMission.Waypoints.back().Speed        = 9.5f;   // m/s
	    testMission.Waypoints.back().LoiterTime   = std::nanf("");
	    testMission.Waypoints.back().GimbalPitch  = std::nanf("");

	    testMission.Waypoints.emplace_back();
	    testMission.Waypoints.back().Latitude     =  44.238344 * PI/180.0; // radians
	    testMission.Waypoints.back().Longitude    = -95.309309 * PI/180.0; // radians
	    testMission.Waypoints.back().RelAltitude  =  30.5; // m
	    testMission.Waypoints.back().CornerRadius = 0.2f; // m (Not used since we aren't using curved trajectories)
	    testMission.Waypoints.back().Speed        = 9.5f; // m/s
	    testMission.Waypoints.back().LoiterTime   = std::nanf("");
	    testMission.Waypoints.back().GimbalPitch  = std::nanf("");

	    testMission.LandAtLastWaypoint = false;
	    testMission.CurvedTrajectory = false;

	    //m_flightMode = 1;
	}

	void RealDrone::SendTestVirtualStickPacketA(){
		DroneInterface::Packet packet;
		DroneInterface::Packet_VirtualStickCommand PacketA;
		/*
		PacketA.Mode    = 0U;    //Mode A (V_x is V_North, and V_y is V_East)
		PacketA.Yaw     = 31.0f; //degrees, relative to true north (positive yaw is clockwise rotation)
		PacketA.V_x     = 1.2f;  //m/s
		PacketA.V_y     = -0.7f; //m/s
		PacketA.HAG     = 39.5f; //m
		PacketA.timeout = 5.0f;  //s
		*/
		// Diabolical Case. Testing clamp functions
		PacketA.Mode    = 0U;    //Mode A (V_x is V_North, and V_y is V_East)
		PacketA.Yaw     = 200.0f; //degrees, relative to true north (positive yaw is clockwise rotation)
		PacketA.V_x     = 30.0f;  //m/s
		PacketA.V_y     = -20.0f; //m/s
		PacketA.HAG     = -10.0f; //m
		PacketA.timeout = 5.0f;  //s

		PacketA.Serialize(packet);

		m_mutex_A.lock();
		tacopie::tcp_client * TCPClient = m_client;
		m_mutex_A.unlock();
		RealDrone::SendPacket(packet, TCPClient);
	}
	
	void RealDrone::SendTestVirtualStickPacketB(){
		DroneInterface::Packet packet;
		DroneInterface::Packet_VirtualStickCommand PacketB;

		PacketB.Mode    = 1U;    //Mode B
		PacketB.Yaw     = 31.0f; //degrees, relative to true north (positive yaw is clockwise rotation)
		PacketB.V_x     = 1.2f;  //m/s
		PacketB.V_y     = -0.7f; //m/s
		PacketB.HAG     = 39.5f; //m
		PacketB.timeout = 5.0f;  //s

		PacketB.Serialize(packet);

		m_mutex_A.lock();
		tacopie::tcp_client * TCPClient = m_client;
		m_mutex_A.unlock();
		RealDrone::SendPacket(packet, TCPClient);
	}

	void RealDrone::DataReceivedHandler(const std::shared_ptr<tacopie::tcp_client> & client, const tacopie::tcp_client::read_result & res) {
		//If this callback is called after disconnection, return without starting a new read (breaking read loop)
		if (! m_isConnected) {
			std::cerr << "DataReceivedHandler() called after disconnection. Stopping read loop.\r\n";
			return;
		}
		
		std::scoped_lock lock_A(m_mutex_A);
		if (res.success) {
			//Copy all the received data to our buffer
			m_buffer.insert(m_buffer.end(), res.buffer.begin(), res.buffer.end());
			
			//Process received data
			while (! m_buffer.empty()) {
				uint32_t bytes_needed;
				if (m_packet_fragment->BytesNeeded(bytes_needed)) {
					if (bytes_needed >= (uint32_t) m_buffer.size()) {
						//All bytes in buffer belong to our current packet
						m_packet_fragment->m_data.insert(m_packet_fragment->m_data.end(), m_buffer.begin(), m_buffer.end());
						m_buffer.clear();
					}
					else {
						//Some bytes in buffer belong to our current packet
						m_packet_fragment->m_data.insert(m_packet_fragment->m_data.end(), m_buffer.begin(), m_buffer.begin() + bytes_needed);
						m_buffer.erase(m_buffer.begin(), m_buffer.begin() + bytes_needed);
					}
				}
				else {
					//We don't have enough data in our packet fragment to detirmine how many bytes are needed to finish the packet.
					//Just move 1 byte in this case and try again
					m_packet_fragment->m_data.push_back(m_buffer[0]);
					m_buffer.erase(m_buffer.begin());
				}
				
				if (m_packet_fragment->IsFinished()) {
					if (ProcessFullReceivedPacket())
						m_packet_fragment->Clear();
					else
						m_packet_fragment->ForwardScanForSync(); //If the packet failed to decode we may be out of sync - attempt resynchronization
				}
			}
			
			//If we have been instructed to take possession of another object, transfer our state to it here and set up the client to
			//call the target objects receive handler instead of our own. Otherwise, trigger the next read.
			std::scoped_lock lock_B(m_mutex_B);
			if ((m_possessionTarget != nullptr) && TransferStateToTargetObject()) {
				client->set_on_disconnection_handler(std::bind(&RealDrone::DisconnectHandler, m_possessionTarget));
				client->async_read({ 1024, bind(&RealDrone::DataReceivedHandler, m_possessionTarget, client, std::placeholders::_1) });
			}
			else
				client->async_read({ 1024, bind(&RealDrone::DataReceivedHandler, this, client, std::placeholders::_1) });
		}
		else {
			std::cout << "Client disconnected" << std::endl;
			client->disconnect();
		}
	}
	
	//Transfer state to another RealDrone Object on the next opportunity, leaving this object dead
	void RealDrone::Possess(RealDrone * Target) {
		std::scoped_lock lock(m_mutex_B);
		m_possessionTarget = Target;
	}
	
	//Returns true if state has been transferred to another object. Can safely be destroyed if dead.
	bool RealDrone::IsDead(void) {
		std::scoped_lock lock(m_mutex_B);
		return m_isDead;
	}
	
	bool RealDrone::TransferStateToTargetObject(void) {
		if (m_possessionTarget != nullptr) {
			if (m_possessionTarget->m_isConnected) {
				std::cerr << "Can't take possession of target because it has active socket connection.\r\n";
				return false;
			}
			
			std::cerr << "Taking possession of target drone object.\r\n";
			//Locks are already held by the caller on this objects A and B mutexes.
			std::scoped_lock lock_TargetObs(m_possessionTarget->m_mutex_A, m_possessionTarget->m_mutex_B);
			
			m_possessionTarget->m_client = this->m_client;
			m_possessionTarget->m_buffer = this->m_buffer;
			delete m_possessionTarget->m_packet_fragment;
			m_possessionTarget->m_packet_fragment = this->m_packet_fragment;
			this->m_packet_fragment = nullptr; //Make sure when the dead drone is destroyed that we don't kill the packet fragment
			
			//Transfer Core Telemetry data
			if (this->m_packet_ct_received) {
				if ((! m_possessionTarget->m_packet_ct_received) || (this->m_PacketTimestamp_ct > m_possessionTarget->m_PacketTimestamp_ct)) {
					m_possessionTarget->m_packet_ct          = this->m_packet_ct;
					m_possessionTarget->m_PacketTimestamp_ct = this->m_PacketTimestamp_ct;
					m_possessionTarget->m_packet_ct_received = this->m_packet_ct_received;
				}
			}
			
			//Transfer Extended Telemetry data
			if (this->m_packet_et_received) {
				if ((! m_possessionTarget->m_packet_et_received) || (this->m_PacketTimestamp_et > m_possessionTarget->m_PacketTimestamp_et)) {
					m_possessionTarget->m_packet_et          = this->m_packet_et;
					m_possessionTarget->m_PacketTimestamp_et = this->m_PacketTimestamp_et;
					m_possessionTarget->m_packet_et_received = this->m_packet_et_received;
				}
			}
			
			//Transfer Imagery data
			if (this->m_frame_num >= 0) {
				if ((m_possessionTarget->m_frame_num < 0) || (this->m_PacketTimestamp_imagery > m_possessionTarget->m_PacketTimestamp_imagery)) {
					m_possessionTarget->m_PacketTimestamp_imagery = this->m_PacketTimestamp_imagery;
					m_possessionTarget->m_frame_num = std::max(m_possessionTarget->m_frame_num + 1, this->m_frame_num);
					m_possessionTarget->m_MostRecentFrame = this->m_MostRecentFrame;
					m_possessionTarget->m_receivedImageTimestamps = this->m_receivedImageTimestamps;
					m_possessionTarget->m_TimestampOfLastFPSReport = this->m_TimestampOfLastFPSReport;
				}
			}
			
			//Transfer Image callbacks - this is a bit sketchy since both objects may have issued the same callback handle to two different requests.
			//We will try to merge and skip any items that conflict. This shouldn't be a problem in practice since this is primarily used to support
			//re-activating a drone object if the connection is lost and re-established, in which case typically only the old object will have callbacks.
			for (auto const & kv : this->m_ImageryCallbacks) {
				if (m_possessionTarget->m_ImageryCallbacks.count(kv.first) > 0U)
					std::cerr << "Warning in RealDrone::TransferStateToTargetObject(): Dropping Image callback due to conflicting handle.\r\n";
				else
					m_possessionTarget->m_ImageryCallbacks[kv.first] = kv.second;
			}
			
			//Transfer waypoint mission data
			if (! this->m_currentWaypointMission.empty())
				m_possessionTarget->m_currentWaypointMission = this->m_currentWaypointMission;
			
			//Make sure the target of posession doesn't have it's own target
			m_possessionTarget->m_possessionTarget = nullptr;
			m_possessionTarget->m_isDead = false;
			
			//Mark this object as dead and the possessed object as having an active connection
			m_isDead = true;
			m_possessionTarget->m_isConnected = true;
			return true;
		}
		else
			return false;
	}
	
	bool RealDrone::ProcessFullReceivedPacket(void) {
		uint8_t PID;
		m_packet_fragment->GetPID(PID);

		switch (PID) {
			case 0U: {
				std::scoped_lock lock(m_mutex_B);
				if (this->m_packet_ct.Deserialize(*m_packet_fragment)) {
					//std::cout << this->m_packet_ct;
					this->m_packet_ct_received = true;
					this->m_PacketTimestamp_ct = std::chrono::steady_clock::now();
					return true;
				}
				else {
					std::cerr << "Error: Tried to deserialize invalid Core Telemetry packet." << std::endl;
					return false;
				}
			}
			case 1U: {
				std::scoped_lock lock(m_mutex_B);
				if (this->m_packet_et.Deserialize(*m_packet_fragment)) {
					//std::cout << this->m_packet_et;
					this->m_packet_et_received = true;
					this->m_PacketTimestamp_et = std::chrono::steady_clock::now();
					return true;
				}
				else {
					std::cerr << "Error: Tried to deserialize invalid Extended Telemetry packet." << std::endl;
					return false;
				}
			}
			case 2U: {
				if (this->m_packet_img.Deserialize(*m_packet_fragment)) {
					std::scoped_lock lock(m_mutex_B);
					this->m_MostRecentFrame = this->m_packet_img.Frame;
					this->m_frame_num++;
					this->m_PacketTimestamp_imagery = std::chrono::steady_clock::now();
					AddImageTimestampToLogAndFPSReport(this->m_PacketTimestamp_imagery);
					for (auto const & kv : m_ImageryCallbacks)
						kv.second(m_MostRecentFrame, m_PacketTimestamp_imagery);
					return true;
				}
				else {
					std::cerr << "Error: Tried to deserialize invalid Image packet." << std::endl;
					return false;
				}
			}
			case 3U: {
				if (this->m_packet_ack.Deserialize(*m_packet_fragment)) {
					if ((this->m_packet_ack.Positive == uint8_t(1)) && (this->m_packet_ack.SourcePID == uint8_t(252)))
						return true; //Don't spam terminal with positive acks from virtualStick commands
					else {
						std::cout << this->m_packet_ack;
						return true;
					}
				}
				else {
					std::cerr << "Error: Tried to deserialize invalid Acknowledgment packet." << std::endl;
					return false;
				}
			}
			case 4U: {
				if (this->m_packet_ms.Deserialize(*m_packet_fragment)) {
					std::cout << this->m_packet_ms;
					return true;
				}
				else {
					std::cerr << "Error: Tried to deserialize invalid Message String packet." << std::endl;
					return false;
				}
			}
			case 5U: {
				if (this->m_packet_compressedImg.Deserialize(*m_packet_fragment)) {
					std::scoped_lock lock(m_mutex_B);
					this->m_MostRecentFrame = this->m_packet_compressedImg.Frame;
					this->m_frame_num++;
					this->m_PacketTimestamp_imagery = std::chrono::steady_clock::now();
					AddImageTimestampToLogAndFPSReport(this->m_PacketTimestamp_imagery);
					for (auto const & kv : m_ImageryCallbacks)
						kv.second(m_MostRecentFrame, m_PacketTimestamp_imagery);
					return true;
				}
				else {
					std::cerr << "Error: Tried to deserialize invalid Compressed Image packet." << std::endl;
					return false;
				}
			}
			default: {
				std::cerr << "Error: Unrecognized PID - failed to decode packet from drone." << std::endl;
				return false;
			}
		}
	}
	
	void RealDrone::AddImageTimestampToLogAndFPSReport(TimePoint Timestamp) {
		//A lock should already be held on m_mutex_B by the caller of this function
		
		m_receivedImageTimestamps.push_back(Timestamp);
		
		//If desired, print out the effective frame rate over the last few seconds
		//Just replace true with false to turn this off - this is really just for development so this should suffice.
		if (true) {
			double timeAverageInterval = 10.0; //Time interval to estimate average frame rate over (seconds)
			double reportingInterval   = 2.0;  //Minimum amount of time between FPS printout to stderr (seconds)
			
			if (SecondsElapsed(m_TimestampOfLastFPSReport, std::chrono::steady_clock::now()) >= reportingInterval) {
				double imageCount = 0.0;
				for (auto riter = m_receivedImageTimestamps.rbegin(); riter != m_receivedImageTimestamps.rend(); riter++) {
					if (SecondsElapsed(*riter, Timestamp) < timeAverageInterval)
						imageCount += 1.0;
					else
						break;
				}
				std::cerr << "Average framerate over last " << timeAverageInterval << " seconds: " << imageCount/timeAverageInterval << " fps\r\n";
				m_TimestampOfLastFPSReport = std::chrono::steady_clock::now();
			}
		}
	}
	
	bool RealDrone::Ready(void) {
		std::scoped_lock lock(m_mutex_B);
		return (m_packet_ct_received && m_packet_et_received);
	}
	
	//Get drone serial number as a string (should be available on construction)
	std::string RealDrone::GetDroneSerial(void) {
		std::scoped_lock lock(m_mutex_B);
		return this->m_packet_et.DroneSerial; 
	}
	
	//Lat & Lon (radians) and WGS84 Altitude (m)
	bool RealDrone::GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex_B);
		Latitude  = this->m_packet_ct.Latitude  * (PI/180.0);
		Longitude  = this->m_packet_ct.Longitude * (PI/180.0);
		
		//If a GNSS receiver is connected to the GCS, use that and barometric relative altitude to compute absolute drone altitude.
		//If not, rely on the drones estimate of absolute altitude (which is often very poor for DJI drones). Note: We don't need
		//to issue a warning here about poor altitude accuracy since the drone manager handles this and posts a single warning
		//(instead of 1 per drone) when altitude data is unaided by a GCS-connected GNSS receiver.
		double GCS_GroundAlt = 0.0;
		if (GNSSReceiver::GNSSManager::Instance().GetGroundAlt(GCS_GroundAlt)) {
			//std::cerr << "Aiding drone alt.\r\n";
			Altitude = GCS_GroundAlt + this->m_packet_ct.HAG;
		}
		else
			Altitude  = this->m_packet_ct.Altitude;
		
		Timestamp = this->m_PacketTimestamp_ct;
		return this->m_packet_ct_received;
	}
	
	//NED velocity vector (m/s)
	bool RealDrone::GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex_B);
		Eigen::Vector3d V_NED(this->m_packet_ct.V_N, this->m_packet_ct.V_E, this->m_packet_ct.V_D);
		//Sanitize velocity based on max vehicle speed of 28 m/s (shouldn't need this but in case velocity is bad, we don't want to cause problems elsewhere)
		if (V_NED.norm() > 28.0) {
			std::cerr << "Warning: Sanitizing unreasonable velocity vector. Speed = " << V_NED.norm() << " m/s\r\n";
			V_NED.normalize();
			V_NED *= 28.0;
		}
		V_North   = this->m_packet_ct.V_N;
		V_East    = this->m_packet_ct.V_E;
		V_Down    = this->m_packet_ct.V_D;
		Timestamp = this->m_PacketTimestamp_ct;
		return this->m_packet_ct_received;
	}
	
	//Yaw, Pitch, Roll (radians) using DJI definitions
	bool RealDrone::GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex_B);
		Yaw       = this->m_packet_ct.Yaw   * (PI/180.0);
		Pitch     = this->m_packet_ct.Pitch * (PI/180.0);
		Roll      = this->m_packet_ct.Roll  * (PI/180.0);
		Timestamp = this->m_PacketTimestamp_ct;
		return this->m_packet_ct_received;
	}
	
	//Barometric height above ground (m) - Drone altitude minus takeoff altitude
	bool RealDrone::GetHAG(double & HAG, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex_B);
		HAG = this->m_packet_ct.HAG;
		Timestamp = this->m_PacketTimestamp_ct;
		return this->m_packet_ct_received;
	}
	
	//Drone Battery level (0 = Empty, 1 = Full)
	bool RealDrone::GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex_B);
		BattLevel = double(this->m_packet_et.BatLevel) / 100.0;
		Timestamp = this->m_PacketTimestamp_et;
		return this->m_packet_et_received;
	}
	
	//Whether the drone has hit height or radius limits
	bool RealDrone::GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex_B);
		MaxHAG = (this->m_packet_et.MaxHeight == uint8_t(1));
		MaxDistFromHome = (this->m_packet_et.MaxDist == uint8_t(1));
		Timestamp = this->m_PacketTimestamp_et;
		return this->m_packet_et_received;
	}
	
	//Wind & other vehicle warnings as strings
	bool RealDrone::GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex_B);
		
		//Interpret battery warning field
		if (this->m_packet_et.BatWarning == uint8_t(1))
			ActiveWarnings.push_back("Battery Low"s);
		else if (this->m_packet_et.BatWarning == uint8_t(2))
			ActiveWarnings.push_back("Battery Critically Low"s);
		else if (this->m_packet_et.BatWarning != uint8_t(0))
			ActiveWarnings.push_back("Battery State Invalid"s);
		
		//Interpret wind warning field
		if ((this->m_packet_et.BatWarning < int8_t(0)) || (this->m_packet_et.BatWarning > int8_t(2)))
			ActiveWarnings.push_back("Wind Warning: Unknown Condition"s);
		else if (this->m_packet_et.BatWarning == int8_t(1))
			ActiveWarnings.push_back("Wind Warning: Level 1"s);
		else if (this->m_packet_et.BatWarning == int8_t(2))
			ActiveWarnings.push_back("Wind Warning: Level 2 (Serious)"s);
		
		Timestamp = this->m_PacketTimestamp_et;
		return this->m_packet_et_received;
	}
	
	//GNSS status (-1 for none, 0-5: DJI definitions)
	bool RealDrone::GetGNSSStatus(unsigned int & SatCount, int & SignalLevel, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex_B);
		SatCount = this->m_packet_et.GNSSSatCount;
		SignalLevel = this->m_packet_et.GNSSSignal;
		Timestamp = this->m_PacketTimestamp_et;
		return this->m_packet_et_received;
	}
	
	//Returns true if recognized DJI camera is present - Should be available on construction
	bool RealDrone::IsDJICamConnected(void) {
		std::scoped_lock lock(m_mutex_B);
		return ((this->m_packet_et.DJICam == 1) || (this->m_packet_et.DJICam == 2));
	}

	//True if receiving imagery from drone, false otherwise (valid on construction... initially returns false)
	bool RealDrone::IsCamImageFeedOn(void) {
		std::scoped_lock lock(m_mutex_B);
		return (this->m_packet_et.DJICam == 2);
	}

	//Start sending frames of live video (as close as possible to the given framerate (frame / s))
	void RealDrone::StartDJICamImageFeed(double TargetFPS) { 
		this->SendPacket_CameraControl(1, TargetFPS);
	}
	
	//Stop sending frames of live video
	void RealDrone::StopDJICamImageFeed(void) { 
		this->SendPacket_CameraControl(0, 0);
	}
	
	bool RealDrone::GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex_B);
		if (this->m_frame_num < 0)
			return false;
		else {
			Frame = this->m_MostRecentFrame;
			FrameNumber = (unsigned int) this->m_frame_num;
			Timestamp = this->m_PacketTimestamp_imagery;
			return true;
		}
	}
	
	//Register callback for new frames
	int RealDrone::RegisterCallback(std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)> Callback) {
		std::scoped_lock lock(m_mutex_B);
		int token = 0;
		while (m_ImageryCallbacks.count(token) > 0U)
			token++;
		m_ImageryCallbacks[token] = Callback;
		return token;
	}

	//Unregister callback for new frames (input is token returned by RegisterCallback()
	void RealDrone::UnRegisterCallback(int Handle) {
		std::scoped_lock lock(m_mutex_B);
		m_ImageryCallbacks.erase(Handle);
	}
	
	//Populate Result with whether or not the drone is currently flying (in any mode)
	bool RealDrone::IsCurrentlyFlying(bool & Result, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex_B);
		Result = (this->m_packet_ct.IsFlying == uint8_t(1));
		Timestamp = this->m_PacketTimestamp_ct;
		return this->m_packet_ct_received;
	}

	//Get flight mode as a human-readable string
	bool RealDrone::GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex_B);
		if (this->m_packet_et_received) {
			switch (this->m_packet_et.FlightMode) {
				case uint8_t(0):  FlightModeStr = "Manual"s;                 break;
				case uint8_t(1):  FlightModeStr = "Atti"s;                   break;
				case uint8_t(2):  FlightModeStr = "AttiCourseLock"s;         break;
				case uint8_t(3):  FlightModeStr = "GPSAtti"s;                break;
				case uint8_t(4):  FlightModeStr = "GPSCourseLock"s;          break;
				case uint8_t(5):  FlightModeStr = "GPSHomeLock"s;            break;
				case uint8_t(6):  FlightModeStr = "GPSHotPoint"s;            break;
				case uint8_t(7):  FlightModeStr = "AssistedTakeoff"s;        break;
				case uint8_t(8):  FlightModeStr = "AutoTakeoff"s;            break;
				case uint8_t(9):  FlightModeStr = "AutoLanding"s;            break;
				case uint8_t(10): FlightModeStr = "GPSWaypoint"s;            break;
				case uint8_t(11): FlightModeStr = "GoHome"s;                 break;
				case uint8_t(12): FlightModeStr = "Joystick"s;               break;
				case uint8_t(13): FlightModeStr = "GPSAttiWristband"s;       break;
				case uint8_t(14): FlightModeStr = "Draw"s;                   break;
				case uint8_t(15): FlightModeStr = "GPSFollowMe"s;            break;
				case uint8_t(16): FlightModeStr = "ActiveTrack"s;            break;
				case uint8_t(17): FlightModeStr = "TapFly"s;                 break;
				case uint8_t(18): FlightModeStr = "GPSSport"s;               break;
				case uint8_t(19): FlightModeStr = "GPSNovice"s;              break;
				case uint8_t(20): FlightModeStr = "Unknown"s;                break;
				case uint8_t(21): FlightModeStr = "ConfirmLanding"s;         break;
				case uint8_t(22): FlightModeStr = "TerrainFollow"s;          break;
				case uint8_t(23): FlightModeStr = "Tripod"s;                 break;
				case uint8_t(24): FlightModeStr = "ActiveTrackSpotlight"s;   break;
				case uint8_t(25): FlightModeStr = "MotorsJustStarted"s;      break;
				default:          FlightModeStr = "Unknown / Unrecognized"s; break;
			}
			Timestamp = this->m_PacketTimestamp_et;
			return true;
		}
		else
			return false;
	}
	
	//Stop current mission, if running. Then load, verify, and start new waypoint mission.
	void RealDrone::ExecuteWaypointMission(WaypointMission & Mission) {
		m_mutex_B.lock();
		m_currentWaypointMission = Mission;
		m_mutex_B.unlock();
		this->SendPacket_ExecuteWaypointMission(Mission.LandAtLastWaypoint, Mission.CurvedTrajectory, Mission.Waypoints);
	}
	
	//Populate Result with whether or not a waypoint mission is currently being executed
	bool RealDrone::IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex_B);
		Result = (this->m_packet_et.FlightMode == uint8_t(10));
		Timestamp = this->m_PacketTimestamp_et;
		return this->m_packet_et_received;
	}

	//Populate arg with current mission (returns false if not flying waypoint mission)
	bool RealDrone::GetCurrentWaypointMission(WaypointMission & Mission) {
		std::scoped_lock lock(m_mutex_B);
		if (this->m_packet_et.FlightMode == uint8_t(10)) {
			Mission = m_currentWaypointMission;
			return true;
		}
		else
			return false;
	}

	//Put in virtualStick Mode and send command (stop mission if running)
	void RealDrone::IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command) { 
		this->SendPacket_VirtualStickCommand(0, Command.Yaw, Command.V_North, Command.V_East, Command.HAG, Command.timeout);
	}
	
	//Put in virtualStick Mode and send command (stop mission if running)
	void RealDrone::IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command) { 
		this->SendPacket_VirtualStickCommand(1, Command.Yaw, Command.V_Forward, Command.V_Right, Command.HAG, Command.timeout);
	}
	
	//Stop any running missions an leave virtualStick mode (if in it) and hover in place (P mode)
	void RealDrone::Hover(void) {
		this->SendPacket_EmergencyCommand(0);
	}
	
	//Initiate landing sequence immediately at current vehicle location
	void RealDrone::LandNow(void) {
		this->SendPacket_EmergencyCommand(1);
	}
	
	//Initiate a Return-To-Home sequence that lands the vehicle at it's take-off location
	void RealDrone::GoHomeAndLand(void) {
		this->SendPacket_EmergencyCommand(2);
	}
	
	void RealDrone::StartSampleWaypointMission(int NumWaypoints, bool CurvedTrajectories, bool LandAtEnd,
	                                           Eigen::Vector2d const & StartOffset_EN, double HAG) {
		std::cerr << "Starting sample waypoint mission.\r\n";
		
		//Tell the vehicle control widget and the guidance module to stop commanding this drone.
		std::string droneSerial = GetDroneSerial();
		//VehicleControlWidget::Instance().StopCommandingDrone(droneSerial); //Initiated by vehicle control widget, so it does this
		Guidance::GuidanceEngine::Instance().RemoveLowFlier(droneSerial);
		
		//Get drone's current position and the ground altitude
		DroneInterface::Drone::TimePoint Timestamp;
		double Latitude, Longitude, Altitude;
		if (! this->GetPosition(Latitude, Longitude, Altitude, Timestamp)) {
			std::cerr << "Error in StartSampleWaypointMission(): Failed to get drone's current position. Aborting.\r\n";
			return;
		}
		
		Eigen::Matrix3d C_ECEF_ENU = latLon_2_C_ECEF_ENU(Latitude, Longitude);
		Eigen::Vector3d currentPos_ECEF = LLA2ECEF(Eigen::Vector3d(Latitude, Longitude, Altitude));
		Eigen::Vector3d StartOffset_ENU(StartOffset_EN(0), StartOffset_EN(1), 0.0);
		Eigen::Vector3d FirstWaypoint_ECEF = currentPos_ECEF + C_ECEF_ENU.transpose()*StartOffset_ENU;
		Eigen::Vector3d FirstWaypoint_LLA = ECEF2LLA(FirstWaypoint_ECEF);
		Eigen::Vector2d FirstWaypoint_LL(FirstWaypoint_LLA(0), FirstWaypoint_LLA(1));
		
		DroneInterface::WaypointMission mission = CreateSampleWaypointMission(NumWaypoints, CurvedTrajectories, LandAtEnd, FirstWaypoint_LL, HAG);
		
		this->ExecuteWaypointMission(mission);
	}
	
	void RealDrone::SendPacket(DroneInterface::Packet & packet, tacopie::tcp_client * TCPClient) {
		std::vector<char> ch_data(packet.m_data.begin(), packet.m_data.end()); //Is this copy necessary?
		try {
			TCPClient->async_write({ ch_data, nullptr });
		}
		catch (...) {
			std::cerr << "Error in RealDrone::SendPacket(): writing to socket failed.\r\n";
		}
	}
	
	void RealDrone::SendPacket_EmergencyCommand(uint8_t Action) {
		DroneInterface::Packet packet;
		DroneInterface::Packet_EmergencyCommand packet_ec;

		packet_ec.Action = Action;
		
		packet_ec.Serialize(packet);
		
		m_mutex_A.lock();
		tacopie::tcp_client * TCPClient = m_client;
		m_mutex_A.unlock();
		RealDrone::SendPacket(packet, TCPClient);
	}
	
	void RealDrone::SendPacket_CameraControl(uint8_t Action, double TargetFPS) {
		DroneInterface::Packet packet;
		DroneInterface::Packet_CameraControl packet_cc;

		packet_cc.Action = Action;
		packet_cc.TargetFPS = TargetFPS;

		packet_cc.Serialize(packet);
		
		m_mutex_A.lock();
		tacopie::tcp_client * TCPClient = m_client;
		m_mutex_A.unlock();
		RealDrone::SendPacket(packet, TCPClient);
	}
	
	void RealDrone::SendPacket_ExecuteWaypointMission(uint8_t LandAtEnd, uint8_t CurvedFlight, std::vector<Waypoint> Waypoints) {
		DroneInterface::Packet packet;
		DroneInterface::Packet_ExecuteWaypointMission packet_ewm;

		packet_ewm.LandAtEnd = LandAtEnd;
		packet_ewm.CurvedFlight = CurvedFlight;
		packet_ewm.Waypoints = Waypoints;

		packet_ewm.Serialize(packet);
		
		m_mutex_A.lock();
		tacopie::tcp_client * TCPClient = m_client;
		m_mutex_A.unlock();
		RealDrone::SendPacket(packet, TCPClient);
	}
	
	void RealDrone::SendPacket_VirtualStickCommand(uint8_t Mode, float Yaw, float V_x, float V_y, float HAG, float timeout) {
		DroneInterface::Packet packet;
		DroneInterface::Packet_VirtualStickCommand packet_vsc;

		packet_vsc.Mode = Mode;
		packet_vsc.Yaw = Yaw;
		packet_vsc.V_x = V_x;
		packet_vsc.V_y = V_y;
		packet_vsc.HAG = HAG;
		packet_vsc.timeout = timeout;

		packet_vsc.Serialize(packet);
		
		m_mutex_A.lock();
		tacopie::tcp_client * TCPClient = m_client;
		m_mutex_A.unlock();
		RealDrone::SendPacket(packet, TCPClient);
	}
}




