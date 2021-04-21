#pragma once

//System Includes
#include <vector>
#include <string>
#include <memory>

//Project Includes
#include "Drone.hpp"

std::condition_variable cond_var;

void sigint_handler(int) {
	cond_var.notify_all();
}

namespace DroneInterface {
	//The DroneManager is a singleton class; it provides the main drone interface to other software components
	//Drones are identified by their serial numbers. A vector of serial numbers for currently connected drones
	//can be retrieved by the drone manager and individual drones can be accessed by serial number. Once a drone
	//object is created for a drone with a specific serial, the drone object should remain in existence for the
	//lifetime of the program, even if the connection to it is lost. If the connection is re-established, the
	//new connection should be attached to the existing drone object. This way, a valid pointer to a drone object
	//provided by the drone manager will never be invalidated in the future.
	class DroneManager {
		public:
			static DroneManager & Instance() { static DroneManager Obj; return Obj; }
			
			DroneManager();
			~DroneManager() = default;
			
			inline std::vector<std::string> GetConnectedDroneSerialNumbers(void) const;
			inline Drone * GetDrone(std::string const & Serial);
		
		private:
			std::unique_ptr<SimulatedDrone> m_droneSim;
			std::vector<RealDrone*> m_droneRealVector;
			tacopie::tcp_server m_server;
	};

	// When there's a new connection, grab client object, make Drone and give it tcp_client <-- private field in new Drone object

	DroneManager::DroneManager() {
		m_server.start("0.0.0.0", 3001, [this](const std::shared_ptr<tacopie::tcp_client>& client) -> bool {
			std::string hostname = client->get_host();
			std::uint32_t port = client->get_port();

			RealDrone* droneReal = new RealDrone(*client);
			m_droneRealVector.push_back(droneReal);

			std::cout << "New client from " + hostname + " on port " + std::to_string(port) << std::endl;
			client->async_read({ 1024, bind(&RealDrone::DataReceivedHandler, droneReal, client, std::placeholders::_1) });

			return true;
		});

		signal(SIGINT, &sigint_handler);
		std::mutex mtx;
		std::unique_lock<std::mutex> lock(mtx);

		cond_var.wait(lock);
	}
	
	inline std::vector<std::string> DroneManager::GetConnectedDroneSerialNumbers(void) const {
		std::vector<std::string> droneSerialVector;
		for (int i = 0; i < m_droneRealVector.size(); i++) {
			droneSerialVector.push_back(m_droneRealVector.at(i)->GetDroneSerial());
		}
		return droneSerialVector;
	}
	
	inline Drone * DroneManager::GetDrone(std::string const & Serial) {
		if (Serial == "Simulation") {
			//Lazily create simulated drone so we don't make one unless it's requested
			if (m_droneSim == nullptr)
				m_droneSim.reset(new SimulatedDrone);
			return m_droneSim.get(); //Upcast to Drone and return
		}
		else
			return nullptr;
	}
}