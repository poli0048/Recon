//This module provides basic support for a UBLOX GNSS receiver connected to the GCS
//This enables the UI to, for instance, indicate the GCS location on the map.
//We elect to use the UBX protocol as opposed to NMEA so we can easily access proper GPS time
//Authors: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved.
#pragma once

//System Includes
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <iostream>
#include <iomanip>

//External Includes
#include "../../HandyImGuiInclude.hpp"

//Project Includes
#include "../../EigenAliases.h"
#include "../../Maps/MapUtils.hpp"
#include "../../Utilities.hpp"

namespace GNSSReceiver {
	//Singleton class for the GNSS receiver module. This singleton tries to connect to a receiver on a given serial port.
	//It then makes available the received data. This entire module can be enable/disabled and configured through the main settings system
	class GNSSManager {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
		private:
			std::thread       m_managerThread;
			std::atomic<bool> m_receiverConnected;
			std::atomic<bool> m_reset;
			std::atomic<bool> m_abort;
			std::mutex        m_mutex; //Protects non-atomic fields
			
			//These fields are only updated when the NAV solution is valid
			Eigen::Vector3d m_pos_ECEF;       //ECEF position (m)
			Eigen::Vector3d m_pos_LLA;        //LLA position (rad, rad, m)
			double          m_pos_3DAccuracy; //Receiver-provided 3D position accuracy estimate (m) - no precise definition found
			double          m_pos_2DAccuracy; //Receiver-provided 2D position accuracy estimate (m) - no precise definition found
			double          m_pos_VAccuracy;  //Receiver-provided vertical position accuracy estimate (m) - no precise definition found
			uint32_t        m_time_GPSWeek;   //GPS Week number
			double          m_time_GPSTOW;    //GPS TOW
			TimePoint       m_lastSolution_Timestamp; //Updated only when the NAV solution is valid
			bool            m_validSolutionReceived;  //Set to true on the first valid NAV solution
			
			double m_AveragedAlt;         //Running average of altitude (used for GetGroundAlt()). Updated on each valid NAV solution
			double m_AveragedAltAccuracy; //Updated on each valid NAV solution
			
			//These fields are only updated when the connected receiver supports UBX-NAV-SIG messages (not available on older receivers)
			int m_satCount_GPS      = -1; //Number of usable GPS     sats being tracked
			int m_satCount_SBAS     = -1; //Number of usable SBAS    sats being tracked
			int m_satCount_Galileo  = -1; //Number of usable Galileo sats being tracked
			int m_satCount_BeiDou   = -1; //Number of usable BeiDou  sats being tracked
			int m_satCount_IMES     = -1; //Number of usable IMES    sats being tracked
			int m_satCount_QZSS     = -1; //Number of usable QZSS    sats being tracked
			int m_satCount_GLONASS  = -1; //Number of usable GLONASS sats being tracked
			int m_satsWithCodeLock  = -1; //Number of sats with code lock
			int m_satsWithPhaseLock = -1; //Number of sats with carrier phase lock
			std::unordered_map<std::tuple<uint8_t,uint8_t>,uint8_t> m_CN0s; //<GNSSID,SVID> -> C/N0 map
			TimePoint m_lastNAV_SIG_Timestamp;
			
			//This vector holds a log of GPS times and timepoints for us to use in lining up the time bases
			std::vector<std::tuple<uint32_t, double, TimePoint>> m_timeCorrespondenceLog;
			
			void ModuleMain(void);
			
		public:
			static GNSSManager & Instance() { static GNSSManager Obj; return Obj; }
			
			//Constructors and Destructors
			GNSSManager() : m_receiverConnected(false), m_reset(false), m_abort(false),
			                m_pos_ECEF(std::nan(""), std::nan(""), std::nan("")), m_pos_3DAccuracy(std::nan("")), m_pos_2DAccuracy(std::nan("")),
			                m_pos_VAccuracy(std::nan("")), m_time_GPSWeek(0U), m_time_GPSTOW(std::nan("")),
			                m_validSolutionReceived(false), m_AveragedAlt(std::nan("")), m_AveragedAltAccuracy(std::nan("")) {
				m_managerThread = std::thread(&GNSSManager::ModuleMain, this);
			}
			~GNSSManager() { Shutdown(); }
			
			inline void Shutdown(void) {
				m_abort = true;
				if (m_managerThread.joinable())
					m_managerThread.join();
			}
			
			//Returns true if a GNSS receiver is currently connected
			inline bool IsConnected(void) { return m_receiverConnected; }
			
			inline void Reset(void) { m_reset = true; }
			
			//Get the most recent position. Returns false if not available
			inline bool GetPosition_ECEF(Eigen::Vector3d & Pos_ECEF, TimePoint & Timestamp);
			inline bool GetPosition_LLA (Eigen::Vector3d & Pos_LLA,  TimePoint & Timestamp);
			inline bool GetPosition_NM  (Eigen::Vector2d & Pos_NM,   TimePoint & Timestamp); //Note: This is only 2D position
			
			//Get position accuracy estimates (2D, vertical, and 3D... all in meters)
			inline bool GetPositionAccuracy(double & HorizontalAcc, double & VerticalAcc, double & ThreeDAcc, TimePoint & Timestamp);
			
			inline bool GetGPSTime(uint32_t & Week, double & TOW, TimePoint & Timestamp);
			
			inline bool GetSigInfo(int & SatCount_GPS,  int & SatCount_SBAS, int & SatCount_Galileo, int & SatCount_BeiDou,
			                       int & SatCount_IMES, int & SatCount_QZSS, int & SatCount_GLONASS, int & SatsWithCodeLock,
			                       int & SatsWithPhaseLock, std::unordered_map<std::tuple<uint8_t,uint8_t>,uint8_t> & CN0s, TimePoint & Timestamp);
			
			//Get the altitude of the receiver (assumed to be at approximately ground level). Altitude is height above the WGS84 ellipsoid, in meters.
			//This field is averaged over time and will remain valid if the receiver is disconnected. This is primarily available for drone objects
			//to compute their own absolute altitude based on barometric relative altitude in cases where their own estimates of absolute altitude
			//may be inaccurate or unlreiable (cough... DJI... cough).
			inline bool GetGroundAlt(double Altitude);
			
			//Get the absolute GPS time corresponding to a steady_clock timestamp. Only possible if we got a valid GNSS fix (returns false otherwise)
			inline bool TimestampToGPSTime(TimePoint const & Timestamp, uint32_t & GPS_Week, double & GPS_TOW);
	};
	
	inline bool GNSSManager::GetPosition_ECEF(Eigen::Vector3d & Pos_ECEF, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		if (! m_validSolutionReceived)
			return false;
		Pos_ECEF  = m_pos_ECEF;
		Timestamp = m_lastSolution_Timestamp;
		return true;
	}
	
	inline bool GNSSManager::GetPosition_LLA(Eigen::Vector3d & Pos_LLA, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		if (! m_validSolutionReceived)
			return false;
		Pos_LLA = m_pos_LLA;
		Timestamp = m_lastSolution_Timestamp;
		return true;
	}
	
	inline bool GNSSManager::GetPosition_NM(Eigen::Vector2d & Pos_NM, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		if (! m_validSolutionReceived)
			return false;
		Pos_NM = LatLonToNM(Eigen::Vector2d(m_pos_LLA(0), m_pos_LLA(1)));
		Timestamp = m_lastSolution_Timestamp;
		return true;
	}
	
	inline bool GNSSManager::GetPositionAccuracy(double & HorizontalAcc, double & VerticalAcc, double & ThreeDAcc, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		if (! m_validSolutionReceived)
			return false;
		HorizontalAcc = m_pos_2DAccuracy;
		VerticalAcc   = m_pos_VAccuracy;
		ThreeDAcc     = m_pos_3DAccuracy;
		Timestamp     = m_lastSolution_Timestamp;
		return true;
	}
	
	inline bool GNSSManager::GetGPSTime(uint32_t & Week, double & TOW, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		if (! m_validSolutionReceived)
			return false;
		Week      = m_time_GPSWeek;
		TOW       = m_time_GPSTOW;
		Timestamp = m_lastSolution_Timestamp;
		return true;
	}
	
	inline bool GNSSManager::GetSigInfo(int & SatCount_GPS,  int & SatCount_SBAS, int & SatCount_Galileo, int & SatCount_BeiDou,
	                                    int & SatCount_IMES, int & SatCount_QZSS, int & SatCount_GLONASS, int & SatsWithCodeLock,
	                                    int & SatsWithPhaseLock, std::unordered_map<std::tuple<uint8_t,uint8_t>,uint8_t> & CN0s, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		if (m_satCount_GPS < 0)
			return false;
		SatCount_GPS      = m_satCount_GPS;
		SatCount_SBAS     = m_satCount_SBAS;
		SatCount_Galileo  = m_satCount_Galileo;
		SatCount_BeiDou   = m_satCount_BeiDou;
		SatCount_IMES     = m_satCount_IMES;
		SatCount_QZSS     = m_satCount_QZSS;
		SatCount_GLONASS  = m_satCount_GLONASS;
		SatsWithCodeLock  = m_satsWithCodeLock;
		SatsWithPhaseLock = m_satsWithPhaseLock;
		CN0s              = m_CN0s;
		Timestamp         = m_lastNAV_SIG_Timestamp;
		return true;
	}
	
	inline bool GNSSManager::GetGroundAlt(double Altitude) {
		std::scoped_lock lock(m_mutex);
		if (std::isnan(m_AveragedAlt))
			return false;
		else {
			Altitude = m_AveragedAlt;
			return true;
		}
	}

	inline bool GNSSManager::TimestampToGPSTime(TimePoint const & Timestamp, uint32_t & GPS_Week, double & GPS_TOW) {
		std::scoped_lock lock(m_mutex);
		if (m_timeCorrespondenceLog.empty())
			return false;
		
		//std::cerr << (unsigned int) m_timeCorrespondenceLog.size() << " time correspondences.\r\n";
		std::vector<double> deltas;
		deltas.reserve(m_timeCorrespondenceLog.size());
		for (auto const & item : m_timeCorrespondenceLog) {
			double timePointSecondsFromT0Epoch = SecondsSinceT0Epoch(std::get<2>(item));
			double GPST = 604800.0*double(std::get<0>(item)) + std::get<1>(item); //This cuts us down to about 1 microsecond resolution, but good enough for us
			double delta = GPST - timePointSecondsFromT0Epoch;
			deltas.push_back(delta);
			//std::cerr << std::fixed << std::setprecision(6) << delta << "\r\n";
		}
		
		//Compute a robust average of the deltas... this neglects clock drift on the local computer over the course of a mission,
		//but this is negligable for our purposes (and insignificant compared to other error sources, like variable serial comms latancy)
		std::sort(deltas.begin(), deltas.end());
		size_t startIndex = std::min((size_t) std::round(0.3*double(deltas.size() - 1U)), deltas.size() - size_t(1U));
		size_t endIndex   = std::min((size_t) std::round(0.7*double(deltas.size() - 1U)), deltas.size() - size_t(1U));
		unsigned int tally = 0U;
		double sum = 0.0;
		for (size_t index = startIndex; index <= endIndex; index++) {
			sum += deltas[index];
			tally++;
		}
		double robustAverageDelta = sum / double(tally);
		
		//Now compute the GPS Time corresponding to the given timestamp
		double tp_GPST = robustAverageDelta + SecondsSinceT0Epoch(Timestamp);
		GPS_Week = (uint32_t) std::floor(tp_GPST/604800.0);
		GPS_TOW  = tp_GPST - 604800.0*double(GPS_Week);
		return true;
	}
}




