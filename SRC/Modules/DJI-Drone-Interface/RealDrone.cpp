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

#define PI 3.14159265358979

namespace DroneInterface {
	RealDrone::RealDrone(tacopie::tcp_client & client) {
		m_client = &client;
		m_TimestampOfLastFPSReport = std::chrono::steady_clock::now();
	}
	
	RealDrone::~RealDrone() {
		m_client->disconnect(true);
	}

	void RealDrone::DataReceivedHandler(const std::shared_ptr<tacopie::tcp_client> & client, const tacopie::tcp_client::read_result & res) {
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
			
			client->async_read({ 1024, bind(&RealDrone::DataReceivedHandler, this, client, std::placeholders::_1) });
		}
		else {
			std::cout << "Client disconnected" << std::endl;
			client->disconnect();
		}
	}
	
	bool RealDrone::ProcessFullReceivedPacket(void) {
		uint8_t PID;
		m_packet_fragment->GetPID(PID);

		switch (PID) {
			case 0U: {
				std::scoped_lock lock(m_mutex);
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
				std::scoped_lock lock(m_mutex);
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
					std::scoped_lock lock(m_mutex);
					this->m_MostRecentFrame = this->m_packet_img.Frame;
					this->m_frame_num++;
					this->m_PacketTimestamp_imagery = std::chrono::steady_clock::now();
					AddImageTimestampToLogAndFPSReport(this->m_PacketTimestamp_imagery);
					for (auto const & kv : m_ImageryCallbacks)
						kv.second(m_MostRecentFrame, m_PacketTimestamp_imagery);

					//cv::imshow("frame", this->m_packet_img.Frame);
					//cv::waitKey(0);
					return true;
				}
				else {
					std::cerr << "Error: Tried to deserialize invalid Image packet." << std::endl;
					return false;
				}
			}
			case 3U: {
				if (this->m_packet_ack.Deserialize(*m_packet_fragment)) {
					std::cout << this->m_packet_ack;
					return true;
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
					std::scoped_lock lock(m_mutex);
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
		std::scoped_lock lock(m_mutex);
		return (m_packet_ct_received && m_packet_et_received);
	}
	
	//Get drone serial number as a string (should be available on construction)
	std::string RealDrone::GetDroneSerial(void) {
		std::scoped_lock lock(m_mutex);
		return this->m_packet_et.DroneSerial; 
	}
	
	//Lat & Lon (radians) and WGS84 Altitude (m)
	bool RealDrone::GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		Latitude  = this->m_packet_ct.Latitude  * (PI/180.0);
		Latitude  = this->m_packet_ct.Longitude * (PI/180.0);
		Altitude  = this->m_packet_ct.Altitude;
		Timestamp = this->m_PacketTimestamp_ct;
		return this->m_packet_ct_received;
	}
	
	//NED velocity vector (m/s)
	bool RealDrone::GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		V_North   = this->m_packet_ct.V_N;
		V_East    = this->m_packet_ct.V_E;
		V_Down    = this->m_packet_ct.V_D;
		Timestamp = this->m_PacketTimestamp_ct;
		return this->m_packet_ct_received;
	}
	
	//Yaw, Pitch, Roll (radians) using DJI definitions
	bool RealDrone::GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		Yaw       = this->m_packet_ct.Yaw   * (PI/180.0);
		Pitch     = this->m_packet_ct.Pitch * (PI/180.0);
		Roll      = this->m_packet_ct.Roll  * (PI/180.0);
		Timestamp = this->m_PacketTimestamp_ct;
		return this->m_packet_ct_received;
	}
	
	//Barometric height above ground (m) - Drone altitude minus takeoff altitude
	bool RealDrone::GetHAG(double & HAG, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		HAG = this->m_packet_ct.HAG;
		Timestamp = this->m_PacketTimestamp_ct;
		return this->m_packet_ct_received;
	}
	
	//Drone Battery level (0 = Empty, 1 = Full)
	bool RealDrone::GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		BattLevel = double(this->m_packet_et.BatLevel) / 100.0;
		Timestamp = this->m_PacketTimestamp_et;
		return this->m_packet_et_received;
	}
	
	//Whether the drone has hit height or radius limits
	bool RealDrone::GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		MaxHAG = (this->m_packet_et.MaxHeight == uint8_t(1));
		MaxDistFromHome = (this->m_packet_et.MaxDist == uint8_t(1));
		Timestamp = this->m_PacketTimestamp_et;
		return this->m_packet_et_received;
	}
	
	//Wind & other vehicle warnings as strings
	bool RealDrone::GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		
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
		std::scoped_lock lock(m_mutex);
		SatCount = this->m_packet_et.GNSSSatCount;
		SignalLevel = this->m_packet_et.GNSSSignal;
		Timestamp = this->m_PacketTimestamp_et;
		return this->m_packet_et_received;
	}
	
	//Returns true if recognized DJI camera is present - Should be available on construction
	bool RealDrone::IsDJICamConnected(void) {
		std::scoped_lock lock(m_mutex);
		return ((this->m_packet_et.DJICam == 1) || (this->m_packet_et.DJICam == 2));
	}

	//True if receiving imagery from drone, false otherwise (valid on construction... initially returns false)
	bool RealDrone::IsCamImageFeedOn(void) {
		std::scoped_lock lock(m_mutex);
		return (this->m_packet_et.DJICam == 2);
	}

	//Start sending frames of live video (as close as possible to the given framerate (frame / s))
	void RealDrone::StartDJICamImageFeed(double TargetFPS) { 
		std::scoped_lock lock(m_mutex);
		this->SendPacket_CameraControl(1, TargetFPS);
	}
	
	//Stop sending frames of live video
	void RealDrone::StopDJICamImageFeed(void) { 
		std::scoped_lock lock(m_mutex);
		this->SendPacket_CameraControl(0, 0);
	}
	
	bool RealDrone::GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
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
		std::scoped_lock lock(m_mutex);
		int token = 0;
		while (m_ImageryCallbacks.count(token) > 0U)
			token++;
		m_ImageryCallbacks[token] = Callback;
		return token;
	}

	//Unregister callback for new frames (input is token returned by RegisterCallback()
	void RealDrone::UnRegisterCallback(int Handle) {
		std::scoped_lock lock(m_mutex);
		m_ImageryCallbacks.erase(Handle);
	}
	
	//Populate Result with whether or not the drone is currently flying (in any mode)
	bool RealDrone::IsCurrentlyFlying(bool & Result, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		Result = (this->m_packet_ct.IsFlying == uint8_t(1));
		return this->m_packet_ct_received;
	}

	//Get flight mode as a human-readable string
	bool RealDrone::GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
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
		std::scoped_lock lock(m_mutex);
		//The iOS App should handle any necessary state change, like cancelling a current mission. In any event we certainly don't want to issue an RTL command
		/*bool isExecuting;
		TimePoint timestamp;
		this->IsCurrentlyExecutingWaypointMission(isExecuting, timestamp);
		if (isExecuting) {
			this->SendPacket_EmergencyCommand(2); // RTH Command
		}*/
		m_currentWaypointMission = Mission;
		this->SendPacket_ExecuteWaypointMission(Mission.LandAtLastWaypoint, Mission.CurvedTrajectory, Mission.Waypoints);
	}
	
	//Populate Result with whether or not a waypoint mission is currently being executed
	bool RealDrone::IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		Result = (this->m_packet_et.FlightMode == uint8_t(10));
		return this->m_packet_et_received;
	}

	//Populate arg with current mission (returns false if not flying waypoint mission)
	bool RealDrone::GetCurrentWaypointMission(WaypointMission & Mission) {
		std::scoped_lock lock(m_mutex);
		if (this->m_packet_et.FlightMode == uint8_t(10)) {
			Mission = m_currentWaypointMission;
			return true;
		}
		else
			return false;
	}

	//Put in virtualStick Mode and send command (stop mission if running)
	void RealDrone::IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command) { 
		std::scoped_lock lock(m_mutex);
		this->SendPacket_VirtualStickCommand(0, Command.Yaw, Command.V_North, Command.V_East, Command.HAG, Command.timeout);
	}
	
	//Put in virtualStick Mode and send command (stop mission if running)
	void RealDrone::IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command) { 
		std::scoped_lock lock(m_mutex);
		this->SendPacket_VirtualStickCommand(1, Command.Yaw, Command.V_Forward, Command.V_Right, Command.HAG, Command.timeout);
	}
	
	//Stop any running missions an leave virtualStick mode (if in it) and hover in place (P mode)
	void RealDrone::Hover(void) {
		std::scoped_lock lock(m_mutex);
		this->SendPacket_EmergencyCommand(0);
	}
	
	//Initiate landing sequence immediately at current vehicle location
	void RealDrone::LandNow(void) {
		std::scoped_lock lock(m_mutex);
		this->SendPacket_EmergencyCommand(1);
	}
	
	//Initiate a Return-To-Home sequence that lands the vehicle at it's take-off location
	void RealDrone::GoHomeAndLand(void) {
		std::scoped_lock lock(m_mutex);
		this->SendPacket_EmergencyCommand(2);
	}
	
	void RealDrone::SendPacket(DroneInterface::Packet &packet) {
		std::vector<char> ch_data(packet.m_data.begin(), packet.m_data.end()); //Is this copy necessary?
		m_client->async_write({ ch_data, nullptr });
	}
	
	void RealDrone::SendPacket_EmergencyCommand(uint8_t Action) {
		DroneInterface::Packet packet;
		DroneInterface::Packet_EmergencyCommand packet_ec;

		packet_ec.Action = Action;
		
		packet_ec.Serialize(packet);
		this->SendPacket(packet);
	}
	
	void RealDrone::SendPacket_CameraControl(uint8_t Action, double TargetFPS) {
		DroneInterface::Packet packet;
		DroneInterface::Packet_CameraControl packet_cc;

		packet_cc.Action = Action;
		packet_cc.TargetFPS = TargetFPS;

		packet_cc.Serialize(packet);
		this->SendPacket(packet);
	}
	
	void RealDrone::SendPacket_ExecuteWaypointMission(uint8_t LandAtEnd, uint8_t CurvedFlight, std::vector<Waypoint> Waypoints) {
		DroneInterface::Packet packet;
		DroneInterface::Packet_ExecuteWaypointMission packet_ewm;

		packet_ewm.LandAtEnd = LandAtEnd;
		packet_ewm.CurvedFlight = CurvedFlight;
		packet_ewm.Waypoints = Waypoints;

		packet_ewm.Serialize(packet);
		this->SendPacket(packet);

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
		this->SendPacket(packet);
	}
}




