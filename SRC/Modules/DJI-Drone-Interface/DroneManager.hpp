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

namespace DroneInterface {
	//The DroneManager is a singleton class; it provides the main drone interface to other software components
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
			std::thread       m_DroneManagerThread;
			bool              m_running;
			std::atomic<bool> m_abort;
			std::mutex        m_mutex;

			std::unique_ptr<SimulatedDrone> m_droneSim;
			std::vector<RealDrone*> m_droneRealVector; // For storing the handles for the tcp_client, one unique client per drone
			tacopie::tcp_server m_server;
		
			inline void ModuleMain(void);
			int m_port;
			
		public:
			static DroneManager & Instance() { static DroneManager Obj; return Obj; }
			

			DroneManager() : m_running(false), m_abort(false){
				
				std::scoped_lock lock(m_mutex);
				
				// If Recon was shutdown and then restarted quickly, port 3001 may not be released yet
				// 	Restarting Recon too quickly afterwards will fail due to a binding failure
				// The time to release the port is set by TIME_WAIT, which is dictated by the TCP protcol
				// An option is to simply wait this out
				// Another option, is to keep checking for a free port, and increasing port number until we find an empty port
				//     This second option would mean that we would need an option to set the port on the iOS DJI Interface Client
				// Another option is to have the remote client close the connection instead. This would eliminate the TIME_WAIT requirement
				m_port = 3001;
				m_server.start("0.0.0.0", m_port, [this](const std::shared_ptr<tacopie::tcp_client>& client) -> bool {
					std::string hostname = client->get_host();
					std::uint32_t port = client->get_port();

					RealDrone* droneReal = new RealDrone(*client);
					m_droneRealVector.push_back(droneReal);

					std::cout << "New client from " + hostname + " on port " + std::to_string(port) << std::endl;
					client->async_read({ 1024, bind(&RealDrone::DataReceivedHandler, droneReal, client, std::placeholders::_1) });

					return true;
				});
				std::cout << std::endl;
				std::cout << "Starting Drone Manager Server on port " << m_port << std::endl;
				
			}

			// Main shutdowns of Drone Manager here
			~DroneManager() {
					
				Stop();
				
				// Grab finished threads
				if (m_DroneManagerThread.joinable())
					m_DroneManagerThread.join();

				// Close tcp clients
				std::cout << "Removing " << m_droneRealVector.size() << " active client drones ... ... ... ";
				while (!m_droneRealVector.empty()){
					
					m_droneRealVector.back()->DisconnectClient();
					m_droneRealVector.pop_back();
				}
				std::cout<< "Done" << std::endl;
				m_server.stop(true, true);
				std::cout << "Closed Drone Manager Server. Please wait until port " << m_port <<" released to start Recon again" << std::endl;
				
			}
			
			inline void Start(void);	//Start or restart continuous monitoring of drones
			inline void Stop(void);		//Stop managing drones
			inline bool IsRunning(void);	//Returns true if running, false if stopped

			inline std::vector<std::string> GetConnectedDroneSerialNumbers(void) const;
			inline Drone * GetDrone(std::string const & Serial);
			

	};
	inline void DroneManager::Start(void){
		std::scoped_lock lock(m_mutex);
		m_running = true;
	}
	inline void DroneManager::Stop(void){
		std::scoped_lock lock(m_mutex);
		m_running = false;
	}
	inline bool DroneManager::IsRunning(void) {
		std::scoped_lock lock(m_mutex);
		return m_running;
	}
	inline std::vector<std::string> DroneManager::GetConnectedDroneSerialNumbers(void) const {
		std::vector<std::string> droneSerialVector;
		for (int i = 0; i < m_droneRealVector.size(); i++) {
			droneSerialVector.push_back(m_droneRealVector.at(i)->GetDroneSerial());
		}
		return droneSerialVector;
	}
	
	inline Drone * DroneManager::GetDrone(std::string const & Serial) {
		if (Serial == "Simulation"s) {
			//Lazily  create simulated drone so we don't make one unless it's requested
			if (m_droneSim == nullptr)
				m_droneSim.reset(new SimulatedDrone);
			return m_droneSim.get(); //Upcast to Drone and return
		}
		else
			return nullptr;
	}
	
	inline void DroneManager::ModuleMain(void){
		return;
	}
}




