// System Includes
#include <iostream>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <algorithm>

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
	
	void RealDrone::LoadTestWaypointMission(WaypointMission & testMission) {
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
				//Print a warning. Note: Don't try to disconnect or cleanup the old connection. Every call we have tried for that
				//results in some kind on double-free exception. Hopefully it will be cleaned up by Tacopie, but even if it is orphaned
				//until the program closes it isn't that big of a deal.
				std::cerr << "Warning: Possession target already has active socket connection. It will be orphaned during possession.\r\n";
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

	//Get a log of the most recent packets, their types, and whether they were decoded successfully or not
	void RealDrone::GetMostRecentPacketLog(std::vector<std::tuple<TimePoint, int, bool>> & Log, bool ReverseOrdering) {
		Log.clear();
		std::scoped_lock lock(m_mutex_B);
		if (m_packetLog_CircBuf.empty())
			return;
		else
			Log.reserve(m_packetLog_CircBuf.size());

		int oldestIndex = m_packetLog_LastUsedIndex + 1;
		if (oldestIndex >= (int) m_packetLog_CircBuf.size())
			oldestIndex = 0;

		if (ReverseOrdering) {
			int index = m_packetLog_LastUsedIndex;
			while (index != oldestIndex) {
				Log.push_back(m_packetLog_CircBuf[index]);
				index--;
				if (index < 0)
					index = int(m_packetLog_CircBuf.size()) - 1;
			}
			Log.push_back(m_packetLog_CircBuf[index]);
		}
		else {
			int index = oldestIndex;
			while (index != m_packetLog_LastUsedIndex) {
				Log.push_back(m_packetLog_CircBuf[index]);
				index++;
				if (index >= (int) m_packetLog_CircBuf.size())
					index = 0;
			}
			Log.push_back(m_packetLog_CircBuf[index]);
		}
	}

	//Get a distribution of the time delta between consecutive core and extended telemetry packets (from first connection to now)
	//Distributions are passed back as vectors of doubles, adding to 1 (unless they are all 0), indicating the density falling
	//within a set of consecutive bins. Bins cover 0.1 seconds, with bin 0 representing deltaTs between 0 and 0.1 seconds.
	//Bin 1 covers deltaTs between 0.1 and 0.2 seconds, etc. The histograms are truncated at a reasonable value and any
	//deltaTs beyond the last bin are coalesced into the final bin.
	void RealDrone::GetTelemetryDeltaTDistributions(std::vector<double> & CoreTelemDist, std::vector<double> & ExtendedTelemDist) {
		std::scoped_lock lock(m_mutex_B);
		if (CoreTelemDist.size() != m_deltaTDist_coreTelem.size())
			CoreTelemDist = std::vector<double>(m_deltaTDist_coreTelem.size(), 0.0);
		if (ExtendedTelemDist.size() != m_deltaTDist_extendedTelem.size())
			ExtendedTelemDist = std::vector<double>(m_deltaTDist_extendedTelem.size(), 0.0);

		int64_t sampleCount = 0;
		for (int tally : m_deltaTDist_coreTelem)
			sampleCount += tally;
		if (sampleCount > 0) {
			for (size_t n = 0U; n < m_deltaTDist_coreTelem.size(); n++)
				CoreTelemDist[n] = double(m_deltaTDist_coreTelem[n])/double(sampleCount);
		}

		sampleCount = 0;
		for (int tally : m_deltaTDist_extendedTelem)
			sampleCount += tally;
		if (sampleCount > 0) {
			for (size_t n = 0U; n < m_deltaTDist_extendedTelem.size(); n++)
				ExtendedTelemDist[n] = double(m_deltaTDist_extendedTelem[n])/double(sampleCount);
		}
	}

	//m_mutex_B should be locked externally
	void RealDrone::AddReceivedPacketToLog(TimePoint const & T, int PID, bool DecodeSuccess) {
		if (m_packetLog_CircBuf.size() < 300U) {
			//Grow buffer - don't go into circular mode yet
			m_packetLog_LastUsedIndex = (int) m_packetLog_CircBuf.size();
			m_packetLog_CircBuf.push_back(std::make_tuple(T, PID, DecodeSuccess));
		}
		else {
			//Buffer is full - go into circular mode
			m_packetLog_LastUsedIndex++;
			if (m_packetLog_LastUsedIndex >= (int) m_packetLog_CircBuf.size())
				m_packetLog_LastUsedIndex = 0;
			m_packetLog_CircBuf[m_packetLog_LastUsedIndex] = std::make_tuple(T, PID, DecodeSuccess);
		}
	}
	
	bool RealDrone::ProcessFullReceivedPacket(void) {
		uint8_t PID;
		m_packet_fragment->GetPID(PID);

		switch (PID) {
			case 0U: {
				m_mutex_B.lock();
				bool isFlying_prevState = (this->m_packet_ct_received) && (this->m_packet_ct.IsFlying > 0U);
				if (this->m_packet_ct.Deserialize(*m_packet_fragment)) {
					bool isFlying_currentState = (this->m_packet_ct.IsFlying > 0U);
					if ((! isFlying_prevState) && isFlying_currentState) {
						std::cout << "Latching takeoff position.\r\n";
						m_mutex_B.unlock();
						TimePoint Timestamp;
						GetPosition(m_takeoffLat, m_takeoffLon, m_takeoffAlt, Timestamp);
						m_mutex_B.lock();
					}

					//std::cout << this->m_packet_ct;
					if (this->m_packet_ct_received) {
						//Before updating our core telemetry data and timestamp, record the deltaT from the last packet
						if (m_deltaTDist_coreTelem.size() < 50U)
							m_deltaTDist_coreTelem = std::vector<int>(50U, 0);
						double deltaT = SecondsElapsed(this->m_PacketTimestamp_ct);
						int histBin = (int) std::clamp(std::floor(deltaT*10.0), 0.0, 49.0);
						m_deltaTDist_coreTelem[histBin]++;
					}
					this->m_packet_ct_received = true;
					this->m_PacketTimestamp_ct = std::chrono::steady_clock::now();
					AddReceivedPacketToLog(this->m_PacketTimestamp_ct, (int) PID, true);
					m_mutex_B.unlock();
					return true;
				}
				else {
					std::cerr << "Error: Tried to deserialize invalid Core Telemetry packet." << std::endl;
					AddReceivedPacketToLog(std::chrono::steady_clock::now(), (int) PID, false);
					m_mutex_B.unlock();
					return false;
				}
			}
			case 1U: {
				std::scoped_lock lock(m_mutex_B);
				if (this->m_packet_et.Deserialize(*m_packet_fragment)) {
					//std::cout << this->m_packet_et;
					if (this->m_packet_et_received) {
						//Before updating our extended telemetry data and timestamp, record the deltaT from the last packet
						if (m_deltaTDist_extendedTelem.size() < 50U)
							m_deltaTDist_extendedTelem = std::vector<int>(50U, 0);
						double deltaT = SecondsElapsed(this->m_PacketTimestamp_et);
						int histBin = (int) std::clamp(std::floor(deltaT*10.0), 0.0, 49.0);
						m_deltaTDist_extendedTelem[histBin]++;
					}
					this->m_packet_et_received = true;
					this->m_PacketTimestamp_et = std::chrono::steady_clock::now();
					AddReceivedPacketToLog(this->m_PacketTimestamp_et, (int) PID, true);
					return true;
				}
				else {
					std::cerr << "Error: Tried to deserialize invalid Extended Telemetry packet." << std::endl;
					AddReceivedPacketToLog(std::chrono::steady_clock::now(), (int) PID, false);
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
					AddReceivedPacketToLog(this->m_PacketTimestamp_imagery, (int) PID, true);
					return true;
				}
				else {
					std::scoped_lock lock(m_mutex_B);
					std::cerr << "Error: Tried to deserialize invalid Image packet." << std::endl;
					AddReceivedPacketToLog(std::chrono::steady_clock::now(), (int) PID, false);
					return false;
				}
			}
			case 3U: {
				if (this->m_packet_ack.Deserialize(*m_packet_fragment)) {
					m_mutex_B.lock();
					AddReceivedPacketToLog(std::chrono::steady_clock::now(), (int) PID, true);
					m_mutex_B.unlock();
					if ((this->m_packet_ack.Positive == uint8_t(1)) && (this->m_packet_ack.SourcePID == uint8_t(252)))
						return true; //Don't spam terminal with positive acks from virtualStick commands
					else {
						std::cout << this->m_packet_ack;
						return true;
					}
				}
				else {
					std::scoped_lock lock(m_mutex_B);
					std::cerr << "Error: Tried to deserialize invalid Acknowledgment packet." << std::endl;
					AddReceivedPacketToLog(std::chrono::steady_clock::now(), (int) PID, false);
					return false;
				}
			}
			case 4U: {
				if (this->m_packet_ms.Deserialize(*m_packet_fragment)) {
					std::scoped_lock lock(m_mutex_B);
					std::cout << this->m_packet_ms;
					AddReceivedPacketToLog(std::chrono::steady_clock::now(), (int) PID, true);
					return true;
				}
				else {
					std::scoped_lock lock(m_mutex_B);
					std::cerr << "Error: Tried to deserialize invalid Message String packet." << std::endl;
					AddReceivedPacketToLog(std::chrono::steady_clock::now(), (int) PID, false);
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
					AddReceivedPacketToLog(this->m_PacketTimestamp_imagery, (int) PID, true);
					return true;
				}
				else {
					std::scoped_lock lock(m_mutex_B);
					std::cerr << "Error: Tried to deserialize invalid Compressed Image packet." << std::endl;
					AddReceivedPacketToLog(std::chrono::steady_clock::now(), (int) PID, false);
					return false;
				}
			}
			default: {
				std::scoped_lock lock(m_mutex_B);
				std::cerr << "Error: Unrecognized PID - failed to decode packet from drone." << std::endl;
				AddReceivedPacketToLog(std::chrono::steady_clock::now(), -1, false);
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
		WaypointMission sanitizedMission = Mission;
		SanitizeMissionForRealDrone(sanitizedMission);

		//Debug
		std::cerr << "ExecuteWaypointMission on mission:\r\n" << sanitizedMission << "\r\n";

		m_mutex_B.lock();
		m_currentWaypointMission = sanitizedMission;
		m_mutex_B.unlock();
		this->SendPacket_ExecuteWaypointMission(sanitizedMission.LandAtLastWaypoint, sanitizedMission.CurvedTrajectory, sanitizedMission.Waypoints);
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
	
	bool RealDrone::GetTakeoffPosition(double & Latitude, double & Longitude, double & Altitude) {
		std::scoped_lock lock(m_mutex_B);
		if (std::isnan(m_takeoffLat))
			return false;
		Latitude  = m_takeoffLat;
		Longitude = m_takeoffLon;
		Altitude  = m_takeoffAlt;
		return true;
	}

	void RealDrone::StartSampleWaypointMission(int NumWaypoints, bool CurvedTrajectories, bool LandAtEnd,
	                                           Eigen::Vector2d const & StartOffset_EN, double HAG) {
		std::cerr << "Starting sample waypoint mission.\r\n";
		
		//Tell the vehicle control widget and the guidance module to stop commanding this drone.
		std::string droneSerial = GetDroneSerial();
		//VehicleControlWidget::Instance().StopCommandingDrone(droneSerial); //Initiated by vehicle control widget, so it does this
		Guidance::GuidanceEngine::Instance().RemoveDroneFromMission(droneSerial);
		
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

	//Modify mission in place (if needed) to meet DJI rules - in particular, there is apparently a secret rule that consecutive waypoints
	//within a mission must not be within 0.5 meters (3D) of each other (also between the first and last waypoint). Rather than doings it's
	//best or adjusting the mission the DJI flight controller will just fail and reject a mission that doesn't meet this criteria.
	//This function will nudge or remove redundant waypoints to try and get a mission as close as possible to the requested one that
	//the flight controller will (hopefully) not reject.
	//Technically the drone interface iOS App is responsible for sanitizing missions when needed, but it currently doesn't do much in
	//this regard and doing what we can server-side allows us to get more feedback about what's going on. Additionally, if the mission has
	//rounded corners, we must sanitize the corner radii to meet various rules.
	void RealDrone::SanitizeMissionForRealDrone(WaypointMission & Mission) {
		//Set a minimum waypoint separation (in meters). This should be greater than 0.5, since we don't know if the flight controller
		//estimates distance the same way that we do.
		double minWpSep = 1.0; //Min waypoint separation (m)

		int wpIndex             = 0;
		int waypointAdjustments = 0;
		int waypointDeletions   = 0;
		while (wpIndex < (int) Mission.Waypoints.size()) {
			int nextWpIndex = (wpIndex + 1U < Mission.Waypoints.size()) ? wpIndex + 1U : 0U;
			double dist3D = DistBetweenWaypoints3D(Mission.Waypoints[wpIndex], Mission.Waypoints[nextWpIndex]);
			if (dist3D < minWpSep) {
				//We have an issue and intervention is needed.
				Waypoint & WPA(Mission.Waypoints[wpIndex]);
				Waypoint & WPB(Mission.Waypoints[nextWpIndex]);
				if (nextWpIndex > wpIndex) {
					//Two ordinary consecutive waypoints are too close. Try to nudge or remove the second waypoint
					//to respect the min separation requirement. Only adjust the second waypoint so we know that
					//adjustments to later waypoints don't mess up the separation of earlier ones.
					Eigen::Vector3d WPA_ECEF = LLA2ECEF(Eigen::Vector3d(WPA.Latitude, WPA.Longitude, 0.0));
					Eigen::Vector3d WPB_ECEF = LLA2ECEF(Eigen::Vector3d(WPB.Latitude, WPB.Longitude, 0.0));
					Eigen::Vector3d V_ECEF   = WPB_ECEF - WPA_ECEF;
					V_ECEF.normalize();
					if (V_ECEF.norm() < 0.5) {
						//WPA and WPB have the same 2D positions to machine precision. Trim the second one.
						Mission.Waypoints.erase(Mission.Waypoints.begin() + nextWpIndex);
						waypointDeletions++;
					}
					else {
						//We can nudge the second one in the direction of V to respect the min separation requirement.
						//This ensures that all adjustments of consecutive waypoints are 2D adjustments (we don't mess with altitude).
						WPB_ECEF = WPA_ECEF + minWpSep*V_ECEF;
						Eigen::Vector3d WPB_LLA = ECEF2LLA(WPB_ECEF);
						WPB.Latitude  = WPB_LLA(0);
						WPB.Longitude = WPB_LLA(1);
						wpIndex++;
						waypointAdjustments++;
					}
				}
				else {
					//The first and last waypoints are too close (not sure why on Earth this would be a problem but I don't work for DJI)
					//If we adjust the 2D position of either waypoint we might violate the separation requirement between those
					//waypoints and their other adjacent waypoints. Instead, adjust the vertical positions to meet sep requirement.
					double meanRelAlt = 0.5*WPA.RelAltitude + 0.5*WPB.RelAltitude;
					if (WPA.RelAltitude < WPB.RelAltitude) {
						WPA.RelAltitude = meanRelAlt - 0.5*minWpSep;
						WPB.RelAltitude = meanRelAlt + 0.5*minWpSep;
					}
					else {
						WPA.RelAltitude = meanRelAlt + 0.5*minWpSep;
						WPB.RelAltitude = meanRelAlt - 0.5*minWpSep;
					}
					//We might want to make sure both rel altitudes are positive, but there may be cases where you want
					//to fly beneath your takeoff altitude and this would mess that up. Also, unless we have reason to
					//believe that the flight controller rejects missions based on this, we have no good reason to do
					//farther manipulation here.
					wpIndex++;
					waypointAdjustments++;
				}
			}
			else
				wpIndex++;
		}
		if (waypointDeletions + waypointAdjustments > 0) {
			std::cerr << "Warning: Adjustments made during waypoint mission sanitization.\r\n";
			if (waypointDeletions > 0)
				std::cerr << waypointDeletions << " waypoints deleted due to redundancy.\r\n";
			if (waypointAdjustments > 0)
				std::cerr << waypointAdjustments << " waypoints nudged to respect min separation requirement.\r\n";
		}

		if (Mission.CurvedTrajectory) {
			//Make sure the corner radii meet DJI rules. In particular, the first and last waypoints must have radii set to 0.2 m.
			//All other waypoints must satisfy that for 2 consecutive waypoints A and B, with corner radii RA, RB, RA + RB < dist(A, B).
			for (size_t n = 0U; n < Mission.Waypoints.size(); n++) {
				if ((n == 0U) || (n + 1U == Mission.Waypoints.size()))
					Mission.Waypoints[n].CornerRadius = 0.2;
				else {
					//This is an interior waypoint. Make sure the radii is less than half the distance to the previous and less than
					//half the distance to the next waypoint. This will ensure that RA + RB < dist(A, B) for all consecutive A and B.
					Waypoint & WPA(Mission.Waypoints[n - 1U]);
					Waypoint & WPB(Mission.Waypoints[n]);
					Waypoint & WPC(Mission.Waypoints[n + 1U]);

					double distAB = DistBetweenWaypoints3D(WPA, WPB);
					double distBC = DistBetweenWaypoints3D(WPB, WPC);
					double maxCornerRadius = std::clamp(std::min(distAB, distBC)/2.0 - 0.1, 0.2, 1000.0);
					Mission.Waypoints[n].CornerRadius = std::clamp(Mission.Waypoints[n].CornerRadius, 0.2f, (float) maxCornerRadius);
				}
			}
		}
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
	
	void RealDrone::SendPacket_ExecuteWaypointMission(uint8_t LandAtEnd, uint8_t CurvedFlight, std::vector<Waypoint> const & Waypoints) {
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




