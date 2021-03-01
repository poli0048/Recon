//The drone interface module provides the software interface to DJI drones, connected over network sockets
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved. 
#pragma once

//System Includes
#include <vector>
#include <string>

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
	class DroneManager {
		public:
			static DroneManager & Instance() { static DroneManager Obj; return Obj; }
			
			DroneManager();
			~DroneManager() = default;
			
			std::vector<std::string> GetConnectedDroneSerialNumbers(void);
			Drone * GetDrone(std::string const & Serial) const;
			
	};



}


