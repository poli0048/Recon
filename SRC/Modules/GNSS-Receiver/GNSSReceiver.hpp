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

//Project Includes
#include "../../EigenAliases.h"
#include "../../Maps/MapUtils.hpp"

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
			Eigen::Vector3d m_pos_ECEF; // ECEF position (m)
			TimePoint       m_lastSolution_Timestamp; //Updated only when the NAV solution is valid
			bool            m_validSolutionReceived;  //Set to true on the first valid NAV solution
			
			void ModuleMain(void);
			
		public:
			static GNSSManager & Instance() { static GNSSManager Obj; return Obj; }
			
			//Constructors and Destructors
			GNSSManager() : m_managerThread(&GNSSManager::ModuleMain, this), m_receiverConnected(false), m_reset(false), m_abort(false),
			                                m_pos_ECEF(std::nan(""), std::nan(""), std::nan("")),
			                                m_validSolutionReceived(false) { }
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
		Pos_LLA = ECEF2LLA(m_pos_ECEF);
		Timestamp = m_lastSolution_Timestamp;
		return true;
	}
	
	inline bool GNSSManager::GetPosition_NM(Eigen::Vector2d & Pos_NM, TimePoint & Timestamp) {
		std::scoped_lock lock(m_mutex);
		if (! m_validSolutionReceived)
			return false;
		Eigen::Vector3d Pos_LLA = ECEF2LLA(m_pos_ECEF);
		Pos_NM = LatLonToNM(Eigen::Vector2d(Pos_LLA(0), Pos_LLA(1)));
		Timestamp = m_lastSolution_Timestamp;
		return true;
	}

	inline bool GNSSManager::TimestampToGPSTime(TimePoint const & Timestamp, uint32_t & GPS_Week, double & GPS_TOW) {
		std::scoped_lock lock(m_mutex);
		return false;
	}
}




