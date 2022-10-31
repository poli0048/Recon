//The drone interface module provides the software interface to DJI drones, connected over network sockets
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved.
#pragma once

//System Includes
#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <mutex>

//Project Includes
#include "Drone.hpp"
#include "../../UI/MapWidget.hpp"
#include "../GNSS-Receiver/GNSSReceiver.hpp"

namespace DroneInterface {
	//The DroneManager is a singleton class; it provides the main drone interface to other software components.
	//Drones are identified by their serial numbers. A vector of serial numbers for currently connected drones
	//can be retrieved by the drone manager and individual drones can be accessed by serial number. Once a drone
	//object is created for a drone with a specific serial, the drone object should remain in existence for the
	//lifetime of the program, even if the connection to it is lost. If the connection is re-established, the
	//new connection should be attached to the existing drone object. This way, a valid pointer to a drone object
	//provided by the drone manager will never be invalidated in the future.

	//DroneManager is the analagous engine to ShadowDetectionEngine and ShadowPropagationEngine. 
	//Need to store the pointers to the drone object, similar to the Callbacks to the InstantaneousShadowMaps
	
	// 7.5.2021: Copied over Parthiv's code
	// In Parthiv's original code, he basically copies over the tcp_server example from the tacopie project
	// In this example, a tcp_server is started. This server listens for any tcp connection requests, creates a tcp_client
	// The handle for this unique client is stored in m_droneRealVector
	// This server and the clients are active until the function exits. Therefore in the original code, a mutex lock
	// is created, and it is released via SIGINT signal. This created a problem because the related variable and signal_handler 
	// had to be created as GLOBAL variables: SUPER BAD!!!!! Especially when integrating with Recon
	// The solution to this is to create a while loop that terminates when m_abort is set to true

	// Following the Start, Stop, Constructor, deconstructor formats from ShadowDetection and ShadowPropagation
	// Start(): set m_running flag to true
	// Stop():  set m_running flag to false
	// ModuleMain(): This is called in the Constructor onto a separate thread. Starts tcp_server. Enters while loop and exits when m_abort set to true
	// Constructor: starts ModuleMain, and consequently the tcp_server, on the thread m_DroneManagerThread
	// Deconstructor: sets m_abort to true, waits for ModuleMain to close and cleans up the thread
	class DroneManager {
		private:
			std::mutex m_mutex;
			std::thread m_managerThread;
			std::atomic<bool> m_abort;
			
			std::unordered_map<std::string, std::unique_ptr<SimulatedDrone>> m_simulatedDrones; //Serial -> SimDronePtr
			std::vector<RealDrone *> m_droneRealVector; // For storing the handles for the tcp_client, one unique client per drone
			std::vector<RealDrone *> m_droneRealHoldingPool; //Stores real drones that aren't advertising themselves as ready yet
			tacopie::tcp_server m_server;
			
			int m_port;
			int m_MessageToken;
			
			void ManagerMain(void) {
				while (true) {
					//Sleep 1 second - check the abort flag every 0.1 seconds
					for (int n = 0; n < 10; n++) {
						if (m_abort)
							return;
						std::this_thread::sleep_for(std::chrono::milliseconds(100));
					}
					
					std::scoped_lock lock(m_mutex);
					
					//See if there are any drone objects in the holding pool that are now ready or dead
					size_t n = 0U;
					while (n < m_droneRealHoldingPool.size()) {
						RealDrone * drone = m_droneRealHoldingPool[n];
						std::string serial = drone->GetDroneSerial();
						if (drone->IsDead()) {
							//This drone has taken possession of an existing drone - we can delete this object
							std::cerr << "Drone in holding pool has taken possession of existing drone. Destroying dead drone.\r\n";
							delete drone;
							m_droneRealHoldingPool.erase(m_droneRealHoldingPool.begin() + n);
						}
						else if (drone->Ready()) {
							//See if there is an existing object with the same serial number to merge with
							RealDrone * mergeTarget = nullptr;
							for (RealDrone * existingDrone : m_droneRealVector) {
								if (existingDrone->GetDroneSerial() == serial) {
									std::cerr << "Target for possession set for drone with serial " << serial << " in holding pool.\r\n";
									mergeTarget = existingDrone;
									break;
								}
							}
							
							if (mergeTarget != nullptr) {
								drone->Possess(mergeTarget);
								n++;
							}
							else {
								//There is no existing drone with the same serial number as the new drone
								std::cerr << "No existing drone with serial " << serial << ". Moving drone from holding pool.\r\n";
								m_droneRealVector.push_back(drone);
								m_droneRealHoldingPool.erase(m_droneRealHoldingPool.begin() + n);
							}
						}
						else
							n++;
					}
					
					//If we have drones connected but don't have a GNSS receiver connected to the GCS, warn user about poor altitude.
					std::string warningString;
					if (m_droneRealVector.size() > 0U) {
						Eigen::Vector3d Pos_LLA;
						GNSSReceiver::GNSSManager::TimePoint Timestamp;
						if (! GNSSReceiver::GNSSManager::Instance().GetPosition_LLA(Pos_LLA, Timestamp)) {
							warningString = "Warning: No GNSS receiver connected to GCS. DJI drone altitude will be unaided (high error)."s;
							warningString += "\nRecommend connecting GNSS receiver to GCS or disabling watchdog module (this option is less good)"s;
						}
					}
					MapWidget::Instance().m_messageBoxOverlay.AddMessage(warningString, m_MessageToken);
				}
			}
			
		public:
			static DroneManager & Instance() { static DroneManager Obj; return Obj; }
			
			//Constructor
			DroneManager() : m_abort(false) {
				std::scoped_lock lock(m_mutex);
				
				// If Recon was shutdown and then restarted quickly, port 3001 may not be released yet
				// 	Restarting Recon too quickly afterwards will fail due to a binding failure
				// The time to release the port is set by TIME_WAIT, which is dictated by the TCP protcol
				// An option is to simply wait this out
				// Another option, is to keep checking for a free port, and increasing port number until we find an empty port
				//     This second option would mean that we would need an option to set the port on the iOS DJI Interface Client
				// Another option is to have the remote client close the connection instead. This would eliminate the TIME_WAIT requirement
				m_port = 3001;
				m_MessageToken = MapWidget::Instance().m_messageBoxOverlay.GetAvailableToken();
				m_server.start("0.0.0.0", m_port, [this](const std::shared_ptr<tacopie::tcp_client> & client) -> bool {
					std::string hostname = client->get_host();
					uint32_t    port     = client->get_port();
					std::cerr << "New client from " << hostname << " on port " << (unsigned int) port << "\r\n";
					
					RealDrone* droneReal = new RealDrone(client);
					{
						std::scoped_lock lock(m_mutex);
						m_droneRealHoldingPool.push_back(droneReal);
					}
					
					//client->async_read({ 1024, bind(&RealDrone::DataReceivedHandler, droneReal, client, std::placeholders::_1) });

					return true;
				});
				std::cerr << std::endl;
				std::cerr << "Starting Drone Manager Server on port " << m_port << std::endl;
				
				m_managerThread = std::thread(&DroneManager::ManagerMain, this);
			}

			// Destructor
			~DroneManager() {
				m_abort = true; //Signal the manager thread to stop
				
				//Join the manager thread
				if (m_managerThread.joinable())
					m_managerThread.join();
				
				std::scoped_lock lock(m_mutex);
				// Destroy RealDrone objects (and close TCP client sockets)
				std::cerr << "Removing " << m_droneRealVector.size() << " active client drones ... ... ... ";
				for (RealDrone * drone : m_droneRealVector)
					delete drone; //Destroy drone objects (their destructors will disconnect their respective sockets)
				m_droneRealVector.clear();
				std::cerr<< "Done" << std::endl;
				
				// Shutdown TCP server
				m_server.stop(true, true);
				std::cerr << "Closed Drone Manager Server. Please wait until port " << m_port <<" released to start Recon again" << std::endl;
			}

			inline std::vector<std::string> GetConnectedDroneSerialNumbers(void) {
				std::scoped_lock lock(m_mutex);
				
				std::vector<std::string> droneSerialVector;
				for (int i = 0; i < (int) m_droneRealVector.size(); i++)
					droneSerialVector.push_back(m_droneRealVector[i]->GetDroneSerial());
				std::vector<std::string> simDroneSerials;
				for (auto const & kv : m_simulatedDrones)
					simDroneSerials.push_back(kv.first);
				std::sort(simDroneSerials.begin(), simDroneSerials.end());
				droneSerialVector.insert(droneSerialVector.end(), simDroneSerials.begin(), simDroneSerials.end());
				
				return droneSerialVector;
			}
			
			inline Drone * GetDrone(std::string const & Serial) {
				std::scoped_lock lock(m_mutex);
				
				if (m_simulatedDrones.count(Serial) > 0U)
					return m_simulatedDrones.at(Serial).get();
				
				//Search for the provided serial in our real drone vector
				for (RealDrone * drone : m_droneRealVector) {
					if (drone->GetDroneSerial() == Serial)
						return drone; //Upcast to Drone and return
				}
				
				return nullptr; //Default if not a recognized simulated drone serial and we can't find the requested drone
			}
			
			//Simulated drones are initialized in a not-flying state at ground level. Hence, the provided
			//initial location should be at ground level. Position is Lat (rad), Lon (rad), Alt (m).
			inline SimulatedDrone * AddSimulatedDrone(std::string Serial, Eigen::Vector3d const & Position_LLA) {
				std::scoped_lock lock(m_mutex);
				
				if (Serial.empty()) {
					std::cerr << "Can't add sim drone with empty serial number.\r\n";
					return nullptr;
				}
				if (m_simulatedDrones.count(Serial) > 0U) {
					std::cerr << "Failed to add sim drone because another sim drone exists with the same serial number.\r\n";
					return nullptr;
				}
				for (RealDrone * drone : m_droneRealVector) {
					if (drone->GetDroneSerial() == Serial) {
						std::cerr << "Failed to add sim drone because a real drone has the same serial number.\r\n";
						return nullptr;
					}
				}
				SimulatedDrone * simDrone = new SimulatedDrone(Serial, Position_LLA);
				m_simulatedDrones[Serial].reset(simDrone);
				return simDrone;
			}
			
			inline void ClearSimulatedDrones(void) {
				std::scoped_lock lock(m_mutex);
				m_simulatedDrones.clear();
			}
			
			inline unsigned int NumSimulatedDrones(void) {
				std::scoped_lock lock(m_mutex);
				return (unsigned int) m_simulatedDrones.size();
			}
	};
}




