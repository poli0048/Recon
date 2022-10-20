//The drone interface module provides the software interface to DJI drones, connected over network sockets
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved.
#pragma once

//System Includes
#include <vector>
#include <string>
#include <chrono>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <filesystem>
#include <algorithm>

//External Includes
#include "../../../handycpp/Handy.hpp" //Provides std::filesystem and Handy::File
#include <opencv2/opencv.hpp>
#include <tacopie/tacopie>

//Project Includes
#include "../../EigenAliases.h"
#include "DroneComms.hpp"
#include "DroneDataStructures.h"
namespace DroneInterface {

    //Abstract class for drones - This means that an object of this type cannot actually exist... only objects of a derived type. We have two derived types:
    //RealDrone and SimulatedDrone
    class Drone {
    public:
        using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
        Drone() = default;
        virtual ~Drone() = default;

        //The drone should not be advertised until this returns true and other methods may give incorrect results if called before the drone is ready.
        //Once a drone advertises that is is ready, it will stay ready, so this doesn't need to be checked constantly.
        virtual bool Ready(void) = 0;

        //Basic hardware info (available when Ready() returns true)
        virtual std::string GetDroneSerial(void) = 0;

        //Telemetry - all methods return true if at least one valid reading has been received (regardless of age) and false otherwise
        virtual bool GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) = 0; //Lat & Lon (radians) and WGS84 Altitude (m)
        virtual bool GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp) = 0; //NED velocity vector (m/s)
        virtual bool GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp) = 0; //Yaw, Pitch, Roll (radians) using DJI definitions
        virtual bool GetHAG(double & HAG, TimePoint & Timestamp) = 0; //Barometric height above ground (m) - Drone altitude minus takeoff altitude

        //Drone state and warnings
        virtual bool GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp) = 0; //Drone Battery level (0 = Empty, 1 = Full)
        virtual bool GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp) = 0; //Whether the drone has hit height or radius limits
        virtual bool GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp) = 0; //Wind & other vehicle warnings as strings
        virtual bool GetGNSSStatus(unsigned int & SatCount, int & SignalLevel, TimePoint & Timestamp) = 0; //GNSS status (-1 for none, 0-5: DJI definitions)

        //Drone live video access - The mobile client should decode the live video feed and extract every N'th frame and send it to the server.
        //N should be detirmined when StartDJICamImageFeed() is called in order to achieve the given frame rate as closely as possible.
        //There is a frame counter that starts at 0 and increments each time a new frame is received by the server.
        //If it is important that no frames ever be missed (e.g. for feeding into LSTM), you should use the callback system.
        virtual bool IsDJICamConnected(void) = 0; //Should be available when drone is ready
        virtual bool IsCamImageFeedOn(void) = 0; //True if receiving imagery from drone, false otherwise (valid when drone is ready)
        virtual void StartDJICamImageFeed(double TargetFPS) = 0; //Start sending frames of live video (as close as possible to the given framerate (frame / s))
        virtual void StopDJICamImageFeed(void) = 0; //Stop sending frames of live video
        virtual bool GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp) = 0;
        virtual int  RegisterCallback(std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)> Callback) = 0; //Regester callback for new frames
        virtual void UnRegisterCallback(int Handle) = 0; //Unregister callback for new frames (input is token returned by RegisterCallback()

        //Drone Command & Control. If a method returns bool, it should return True if the requested info is known (even if very old) and set the
        //Timestamp argument to the instant of validity of the most recently received value. If the requested info is unknown, these return False.
        //There is a method provided to get an ID for the currently running waypoint mission (if one is running). An unfortunate quirk of the DJI
        //SDK design though is that it doesn't look like you can choose your own IDs when creating a mission, so to use this to see if a new mission
        //is running you need to get the mission ID, start your new mission, and then check the ID again after some time to see if the ID has changed.
        virtual bool IsCurrentlyFlying(bool & Result, TimePoint & Timestamp) = 0;
        virtual bool GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp) = 0; //Get flight mode as a human-readable string
        virtual void ExecuteWaypointMission(WaypointMission & Mission) = 0; //Stop current mission, if running. Then load, verify, and start new waypoint mission.
        virtual bool IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) = 0;
        virtual bool GetCurrentWaypointMission(WaypointMission & Mission) = 0; //Populate arg with current mission (returns false if not flying waypoint mission)
        virtual void IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command) = 0; //Put in virtualStick Mode and send command (stop mission if running)
        virtual void IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command) = 0; //Put in virtualStick Mode and send command (stop mission if running)

        //If a Hover command is issued when the drone is not flying (on ground), it should take off and hover at a safe height above ground.
        virtual void Hover(void) = 0; //Stop any running missions and leave virtualStick mode (if in it) and hover in place (P mode)
        virtual void LandNow(void) = 0; //Initiate landing sequence immediately at current vehicle location
        virtual void GoHomeAndLand(void) = 0; //Initiate a Return-To-Home sequence that lands the vehicle at it's take-off location

        //Other functions
        virtual bool GetTakeoffPosition(double & Latitude, double & Longitude, double & Altitude) = 0; //Get position of drones (most recent) takeoff point
        
        //Dev and Testing Methods
        virtual void StartSampleWaypointMission(int NumWaypoints, bool CurvedTrajectories, bool LandAtEnd, Eigen::Vector2d const & StartOffset_EN, double HAG) = 0;
    };

	//The RealDrone class provides an interface to interact with a single real drone
	class RealDrone : public Drone {
	public:
		RealDrone() = delete;
		RealDrone(const std::shared_ptr<tacopie::tcp_client> & client);
		~RealDrone();

		bool Ready(void) override;

		std::string GetDroneSerial(void) override;

		bool GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) override;
		bool GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp)       override;
		bool GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp)           override;
		bool GetHAG(double & HAG, TimePoint & Timestamp)                                                  override;

		bool GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp)                   override;
		bool GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp)  override;
		bool GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp) override;
		bool GetGNSSStatus(unsigned int & SatCount, int & SignalLevel, TimePoint & Timestamp)    override;

		bool IsDJICamConnected(void)                                                                            override;
		bool IsCamImageFeedOn(void)                                                                             override;
		void StartDJICamImageFeed(double TargetFPS)                                                             override;
		void StopDJICamImageFeed(void)                                                                          override;
		bool GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp)             override;
		int  RegisterCallback(std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)> Callback) override;
		void UnRegisterCallback(int Handle)                                                                     override;

		bool IsCurrentlyFlying(bool & Result, TimePoint & Timestamp)                   override;
		bool GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp)         override;
		void ExecuteWaypointMission(WaypointMission & Mission)                         override;
		bool IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) override;
		bool GetCurrentWaypointMission(WaypointMission & Mission)                      override;
		void IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command)       override;
		void IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command)       override;

		void Hover(void)         override;
		void LandNow(void)       override;
		void GoHomeAndLand(void) override;

		bool GetTakeoffPosition(double & Latitude, double & Longitude, double & Altitude) override;

		//Dev and Testing Methods
		void StartSampleWaypointMission(int NumWaypoints, bool CurvedTrajectories, bool LandAtEnd, Eigen::Vector2d const & StartOffset_EN, double HAG) override;

		//RealDrone-specific methods
		void DisconnectHandler(void);
		void DataReceivedHandler(const std::shared_ptr<tacopie::tcp_client>& client, const tacopie::tcp_client::read_result& res);
		void Possess(RealDrone * Target); //Transfer state to another RealDrone Object on the next opportunity, leaving this object dead
		bool IsDead(void); //Returns true if state has been transferred to another object. Can safely be destroyed if dead.

		// test functions
		void LoadTestWaypointMission(WaypointMission & testMission);
		void SendTestVirtualStickPacketA();
		void SendTestVirtualStickPacketB();

	private:
		static void SendPacket(Packet & packet, tacopie::tcp_client * TCPClient);
		static void SanitizeMissionForRealDrone(WaypointMission & Mission); //Modify mission in place (if needed) to meet DJI rules

		void SendPacket_EmergencyCommand(uint8_t Action);
		void SendPacket_CameraControl(uint8_t Action, double TargetFPS);
		void SendPacket_ExecuteWaypointMission(uint8_t LandAtEnd, uint8_t CurvedFlight, std::vector<Waypoint> const & Waypoints);
		void SendPacket_VirtualStickCommand(uint8_t Mode, float Yaw, float V_x, float V_y, float HAG, float timeout);

		bool ProcessFullReceivedPacket(void); //Process a full packet. Returns true on success and false on failure (likily hash check fail)
		void AddImageTimestampToLogAndFPSReport(TimePoint Timestamp);
		bool TransferStateToTargetObject(void); //Used for possession
		
		//Note: There are two mutexes in this class, protecting resources in two groups. To avoid deadlocks, it is critical that whenever both
		//locks are needed at the same time, they be locked in a single call to lock(), through the construction of a single scoped_lock, or they
		//need to be locked in the order A, B (that is: A first, followed by B).
		
		std::mutex               m_mutex_A;             //All fields in this block are protected by this mutex
		tacopie::tcp_client *    m_client;
		std::vector<uint8_t>     m_buffer;
		Packet *                 m_packet_fragment = new Packet();
		
		std::mutex               m_mutex_B;                   //All fields in this block are protected by this mutex
		double                   m_takeoffLat = std::nan(""); //Latched on each takeoff event
		double                   m_takeoffLon = std::nan(""); //Latched on each takeoff event
		double                   m_takeoffAlt = std::nan(""); //Latched on each takeoff event
		Packet_CoreTelemetry     m_packet_ct;                 //Data is retrieved from this packet in access methods
		Packet_ExtendedTelemetry m_packet_et;                 //Data is retrieved from this packet in access methods
		TimePoint                m_PacketTimestamp_ct;
		TimePoint                m_PacketTimestamp_et;
		TimePoint                m_PacketTimestamp_imagery;
		bool                     m_packet_ct_received = false;
		bool                     m_packet_et_received = false;
		int                      m_frame_num = -1;
		cv::Mat                  m_MostRecentFrame;
		std::unordered_map<int, std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)>> m_ImageryCallbacks;
		std::vector<TimePoint>   m_receivedImageTimestamps; //Log of timestamps for received imagery
		TimePoint                m_TimestampOfLastFPSReport;
		RealDrone *              m_possessionTarget = nullptr; //Nullptr if no possession requested
		bool                     m_isDead = false; //Set to true after possessing another object. Can be destroyed safely.
		WaypointMission          m_currentWaypointMission;
		
		std::atomic<bool> m_isConnected;
		
		//These fields are used exclusively for deserialization in ProcessFullReceivedPacket. They are not mutex-protected.
		Packet_Image m_packet_img;                     //Only used in ProcessFullReceivedPacket
		Packet_CompressedImage m_packet_compressedImg; //Only used in ProcessFullReceivedPacket
		Packet_Acknowledgment m_packet_ack;            //Only used in ProcessFullReceivedPacket
		Packet_MessageString m_packet_ms;              //Only used in ProcessFullReceivedPacket
	};

	//The SimulatedDrone class provides an interface to interact with a single virtual/simulated drone.
	//Imagery is pulled from a video file and dispatched either at real-time speed or as fast as possible, depending on the configuration.
	//Simulated drones should pretty much work in all supported modes and their dynamics are based on the DJI Inspire 2.
	//The one notable exception is that we don't simulate the drones smart RTL functionality and they won't auto-land on critical battery.
	//The battery level will just sit at 0 and the drone will continue to fly normally. This was deliberate to avoid practical limitations
	//from hindering the testing and development of guidance algorithms.
	class SimulatedDrone : public Drone {
		public:
			SimulatedDrone();
			SimulatedDrone(std::string Serial, Eigen::Vector3d const & Position_LLA); //Position is Lat (rad), Lon (rad), Alt (m)
			~SimulatedDrone();

			bool Ready(void) override;

			std::string GetDroneSerial(void) override;

			bool GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) override;
			bool GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp)       override;
			bool GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp)           override;
			bool GetHAG(double & HAG, TimePoint & Timestamp)                                                  override;

			bool GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp)                   override;
			bool GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp)  override;
			bool GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp) override;
			bool GetGNSSStatus(unsigned int & SatCount, int & SignalLevel, TimePoint & Timestamp)    override;

			bool IsDJICamConnected(void)                                                                            override;
			bool IsCamImageFeedOn(void)                                                                             override;
			void StartDJICamImageFeed(double TargetFPS)                                                             override;
			void StopDJICamImageFeed(void)                                                                          override;
			bool GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp)             override;
			int  RegisterCallback(std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)> Callback) override;
			void UnRegisterCallback(int Handle)                                                                     override;

			bool IsCurrentlyFlying(bool & Result, TimePoint & Timestamp)                   override;
			bool GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp)         override;
			void ExecuteWaypointMission(WaypointMission & Mission)                         override;
			bool IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) override;
			bool GetCurrentWaypointMission(WaypointMission & Mission)                      override;
			void IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command)       override;
			void IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command)       override;

			void Hover(void)         override;
			void LandNow(void)       override;
			void GoHomeAndLand(void) override;

			bool GetTakeoffPosition(double & Latitude, double & Longitude, double & Altitude) override;

			//Dev and Testing Methods
			void StartSampleWaypointMission(int NumWaypoints, bool CurvedTrajectories, bool LandAtEnd, Eigen::Vector2d const & StartOffset_EN, double HAG) override;

			//SimulatedDrone-specific methods
			void SetRealTime(bool Realtime); //True: Imagery will be provided at close-to-real-time rate. False: Imagery is provided as fast as possible
			void SetSourceVideoFile(std::filesystem::path const & VideoPath); //Should be set before calling StartDJICamImageFeed()
			std::filesystem::path GetSourceVideoFile(void);
			bool GetReferenceFrame(double SecondsIntoVideo, cv::Mat & Frame); //Get a single frame - will fail if the video feed is running

			static bool ResizeTo720p(cv::Mat & Frame); //Make sure the frame is 720p... resize if needed.
			static bool Resize_4K_to_720p(cv::Mat & Frame); //Drop a 4K m_frame down to 720p

		private:
			std::unordered_map<int, std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)>> m_ImageryCallbacks;

			std::thread       m_MainThread;
			std::atomic<bool> m_abort;
			std::mutex        m_mutex; //Lock in each public method for thread safety

			//When the video feed is running, a thread is launched to do nothing but read and decode the necessary frames.
			std::thread       m_VideoProcessingThread;
			std::atomic<bool> m_VideoProcessingThreadAbort;
			std::mutex        m_NextFrameMutex;
			cv::Mat           m_NextFrame;                     //Protected by m_NextFrameMutex
			bool              m_NextFrameReady = false;        //Protected by m_NextFrameMutex
			bool              m_VideoFileReadFinished = false; //Protected by m_NextFrameMutex

			//Additional State Data
			std::string m_serial;
			double m_Lat, m_Lon, m_Alt; //Lat (rad), Lon (rad), alt (m)
			double m_V_North, m_V_East, m_V_Down; //NED velocity
			double m_yaw, m_pitch, m_roll; //Radians (DJI definitions)
			double m_groundAlt; //(m)

			double m_HomeLat, m_HomeLon;
			double m_takeoffLat, m_takeoffLon, m_takeoffAlt; //Latched on each takeoff event
			double m_battLevel;
			int m_targetWaypoint = -1; //Next waypoint (when in waypoint mission mode)
			//We use m_waypointMissionState to track a state machine that governs the drones behavior in a waypoint mission
			//States: 0=takeoff, 1=goto waypoint (P2P), 2=pause at waypoint (P2P), 3=turning at waypoint (P2P),
			//        4=goto waypoint (curved), 5=turn through waypoint (curved)
			int m_waypointMissionState = -1;
			TimePoint m_arrivalAtWaypoint_Timestamp;
			double m_turningSpeedThroughWaypoint;
			VirtualStickCommand_ModeA m_LastVSCommand_ModeA;
			VirtualStickCommand_ModeB m_LastVSCommand_ModeB;
			TimePoint m_LastVSCommand_ModeA_Timestamp;
			TimePoint m_LastVSCommand_ModeB_Timestamp;
			TimePoint m_LastPoseUpdate;

			bool m_realtime = false;
			std::filesystem::path m_videoPath;
			double m_targetFPS = -1.0;
			bool m_imageFeedActive = false;
			cv::Mat m_Frame;                     //Most recent frame
			unsigned int m_FrameNumber = 0U;     //Frame number of most recent frame (increments on each *used* frame)
			TimePoint m_FrameTimestamp;          //Timestamp of most recent frame
			TimePoint m_VideoFeedStartTimestamp; //Timestamp of start of video feed
			int m_flightMode = 0;                //-1=Other, 0=On Ground, 1=P, 2=Waypoint, 3=VirtualStick_A, 4=VirtualStick_B, 5=Takeoff, 6=Landing, 7=RTH
			int m_flightMode_LastPass = 0;       //State on last call to UpdateDronePose()
			WaypointMission m_LastMission;       //A copy of the last waypoint mission uploaded to the drone

			void   DroneMain(void);
			void   UpdateDronePose(void);
			void   UpdateDrone2DPositionBasedOnVelocity(double deltaT, Eigen::Matrix3d const & C_ENU_ECEF);
			void   UpdateDroneVertChannelBasedOnTargetHAG(double deltaT, double TargetHAG, double climbRate, double descentRate);
			double UpdateDroneOrientationBasedOnYawTarget(double deltaT, double TargetYaw, double turnRate);
			void   Update2DVelocityBasedOnTarget(double deltaT, Eigen::Vector2d const & V_Target_EN, double max2DAcc, double max2DDec, double max2DSpeed);
			void   ComputeTarget2DVelocityBasedOnTargetPosAndSpeed(double TargetLat, double TargetLon, double TargetMoveSpeed,
			                                                       Eigen::Vector2d & V_Target_EN, Eigen::Matrix3d const & C_ECEF_ENU);
	};

}



