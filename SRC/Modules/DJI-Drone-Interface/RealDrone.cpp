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

namespace DroneInterface {

	RealDrone::RealDrone() : m_abort(false) {
		if (m_packet_et_received) {
			m_serial = m_packet_et.DroneSerial;
		}
		m_thread = std::thread(&RealDrone::DroneMain, this);
	}

	RealDrone::RealDrone(tacopie::tcp_client &client) : m_abort(false) {
		if (m_packet_et_received) {
			m_serial = m_packet_et.DroneSerial;
		}
		m_client = &client;
		m_thread = std::thread(&RealDrone::DroneMain, this);
	}
	
	RealDrone::~RealDrone() {
		m_abort = true;
		if (m_thread.joinable())
			m_thread.join();
	}

	void RealDrone:: DisconnectClient(void){
		m_client->disconnect(true);
	}

	void RealDrone::DataReceivedHandler(const std::shared_ptr<tacopie::tcp_client>& client, const tacopie::tcp_client::read_result& res){
		std::scoped_lock lock(m_mutex);
		if (res.success) {
			uint32_t bytes_needed;
			m_packet_fragment->BytesNeeded(bytes_needed); //Fragile: Implicit assumption is that this will always succeed/
			// ECHAI: Will need to revise this fragility at some point in the future.

			if (bytes_needed >= (uint32_t)res.buffer.size()) {
				m_packet_fragment->m_data.insert(m_packet_fragment->m_data.end(), res.buffer.begin(), res.buffer.end());
			}
			else {
				m_packet_fragment->m_data.insert(m_packet_fragment->m_data.end(), res.buffer.begin(), res.buffer.begin() + bytes_needed);
			}

			if (m_packet_fragment->IsFinished()) {
				uint8_t PID;
				m_packet_fragment->GetPID(PID);
					
				switch (PID) {
					case 0U: {
						if (this->m_packet_ct.Deserialize(*m_packet_fragment)) {
							std::cout << this->m_packet_ct;
							this->m_packet_ct_received = true;
						}
						else {
							std::cerr << "Error: Tried to deserialize invalid Core Telemetry packet." << std::endl;
						}
						break;
					}
					case 1U: {
						if (this->m_packet_et.Deserialize(*m_packet_fragment)) {
							std::cout << this->m_packet_et;
							this->m_packet_et_received = true;
						}
						else {
							std::cerr << "Error: Tried to deserialize invalid Extended Telemetry packet." << std::endl;
						}
						break;
					}
					case 2U: {
						if (this->m_packet_img.Deserialize(*m_packet_fragment)) {
							std::cout << this->m_packet_img;
							this->m_packet_img_received = true;

							cv::imshow("frame", this->m_packet_img.Frame);
							cv::waitKey(0);
						}
						else {
							std::cerr << "Error: Tried to deserialize invalid Image packet." << std::endl;
						}
						break;
					}
					case 3U: {
						if (this->m_packet_ack.Deserialize(*m_packet_fragment)) {
							std::cout << this->m_packet_ack;
							this->m_packet_ack_received = true;
						}
						else {
							std::cerr << "Error: Tried to deserialize invalid Acknowledgment packet." << std::endl;
						}
						break;
					}
					case 4U: {
						if (this->m_packet_ms.Deserialize(*m_packet_fragment)) {
							std::cout << this->m_packet_ms;
							this->m_packet_ms_received = true;

							//SendPacket_EmergencyCommand(1);
							//std::cout << "SENT EMERGENCY COMMAND PACKET" << std::endl;

							//SendPacket_CameraControl(0, 30);
							//std::cout << "SENT CAMERA CONTROL PACKET" << std::endl;

							//SendPacket_ExecuteWaypointMission(1, 0, )
							//std::cout << "SENT EXECUTE WAYPOINT MISSION PACKET" << std::endl;

							//SendPacket_VirtualStickCommand(0, 34.44, 12.2, 3.4, 500, 12);
							//std::cout << "SENT VIRTUAL STICK COMAND PACKET" << std::endl;
						}
						else {
							std::cerr << "Error: Tried to deserialize invalid Message String packet." << std::endl;
						}
						break;
					}
				}
				m_packet_fragment->Clear();
			}
			client->async_read({ 1024, bind(&RealDrone::DataReceivedHandler, this, client, std::placeholders::_1) });
		}
		else {
			std::cout << "Client disconnected" << std::endl;
			client->disconnect();
		}
	}
	
	//Get drone serial number as a string (should be available on construction)
	std::string RealDrone::GetDroneSerial(void) {
		std::scoped_lock lock(m_mutex);
		return this->m_packet_et.DroneSerial; 
	}
	
	//Lat & Lon (radians) and WGS84 Altitude (m)
	bool RealDrone::GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		Latitude = this->m_packet_ct.Latitude;
		Latitude = this->m_packet_ct.Longitude;
		Altitude = this->m_packet_ct.Altitude;
		Timestamp = std::chrono::steady_clock::now();
		return this->m_packet_ct_received;
	}
	
	//NED velocity vector (m/s)
	bool RealDrone::GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		V_North = this->m_packet_ct.V_N;
		V_East = this->m_packet_ct.V_E;
		V_Down = this->m_packet_ct.V_D;
		Timestamp = std::chrono::steady_clock::now();
		return this->m_packet_ct_received;
	}
	
	//Yaw, Pitch, Roll (radians) using DJI definitions
	bool RealDrone::GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		Yaw = this->m_packet_ct.Yaw;
		Pitch = this->m_packet_ct.Pitch;
		Roll = this->m_packet_ct.Roll;
		Timestamp = std::chrono::steady_clock::now();
		return this->m_packet_ct_received;
	}
	
	//Barometric height above ground (m) - Drone altitude minus takeoff altitude
	bool RealDrone::GetHAG(double & HAG, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		HAG = this->m_packet_ct.HAG;
		Timestamp = std::chrono::steady_clock::now();
		return this->m_packet_ct_received;
	}
	
	//Drone Battery level (0 = Empty, 1 = Full)
	bool RealDrone::GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		BattLevel = this->m_packet_et.BatLevel / 100;
		Timestamp = std::chrono::steady_clock::now();
		return this->m_packet_et_received;
	}
	
	//Whether the drone has hit height or radius limits
	bool RealDrone::GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		MaxHAG = this->m_packet_et.MaxHeight;
		MaxDistFromHome = this->m_packet_et.MaxDist;
		Timestamp = std::chrono::steady_clock::now();
		return this->m_packet_et_received;
	}
	
	//Wind & other vehicle warnings as strings
	bool RealDrone::GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		ActiveWarnings.push_back("BatWarning: " + this->m_packet_et.BatWarning);
		ActiveWarnings.push_back("WindLevel: " + this->m_packet_et.WindLevel);
		if (this->m_packet_ms.Type == 2) {
			ActiveWarnings.push_back("Messages: " + this->m_packet_ms.Message);
		}
		Timestamp = std::chrono::steady_clock::now();
		return this->m_packet_ms_received;
	}
	
	//GNSS status (-1 for none, 0-5: DJI definitions)
	bool RealDrone::GetGNSSStatus(unsigned int & SatCount, int & SignalLevel, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		SatCount = this->m_packet_et.GNSSSatCount;
		SignalLevel = this->m_packet_et.GNSSSignal;
		Timestamp = std::chrono::steady_clock::now();
		return this->m_packet_et_received;
	}
	
	//Returns true if recognized DJI camera is present - Should be available on construction
	bool RealDrone::IsDJICamConnected(void) {
		std::scoped_lock lock(m_mutex);
		return this->m_packet_et.DJICam == 0 || this->m_packet_et.DJICam == 1;
	}

	//True if receiving imagery from drone, false otherwise (valid on construction... initially returns false)
	bool RealDrone::IsCamImageFeedOn(void) { return false; }

	void RealDrone::SendPacket(DroneInterface::Packet &packet) {
		//std::scoped_lock lock(m_mutex);
		std::vector<char> ch_data(packet.m_data.begin(), packet.m_data.end());
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


	//Start sending frames of live video (as close as possible to the given framerate (frame / s))
	void RealDrone::StartDJICamImageFeed(double TargetFPS) { 
		this->SendPacket_CameraControl(1, TargetFPS);
	}
	
	//Stop sending frames of live video
	void RealDrone::StopDJICamImageFeed(void) { 
		this->SendPacket_CameraControl(0, 0);
	}
	
	bool RealDrone::GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		Frame = this->m_packet_img.Frame;
		FrameNumber = this->m_frame_num++;
		Timestamp = std::chrono::steady_clock::now();
		return this->m_packet_img_received;
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
	bool RealDrone::IsCurrentlyFlying(bool & Result, TimePoint & Timestamp) { return false; }

	//Get flight mode as a human-readable string
	bool RealDrone::GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		FlightModeStr = "Flight Mode: " + this->m_packet_et.FlightMode;
		Timestamp = std::chrono::steady_clock::now();
		return this->m_packet_et_received;
	}
	
	//Stop current mission, if running. Then load, verify, and start new waypoint mission.
	void RealDrone::ExecuteWaypointMission(WaypointMission & Mission) {
		bool isExecuting;
		TimePoint timestamp;
		this->IsCurrentlyExecutingWaypointMission(isExecuting, timestamp);
		if (isExecuting) {
			this->SendPacket_EmergencyCommand(2); // RTH Command
		}

		this->SendPacket_ExecuteWaypointMission(Mission.LandAtLastWaypoint, Mission.CurvedTrajectory, Mission.Waypoints);
	}
	
	//Populate Result with whether or not a waypoint mission is currently being executed
	bool RealDrone::IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) { return false; }

	//Populate arg with current mission (returns false if not flying waypoint mission)
	bool RealDrone::GetCurrentWaypointMission(WaypointMission & Mission) { return false; }


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
	
	//Function for internal thread managing drone object
	void RealDrone::DroneMain(void) {
	}
}




