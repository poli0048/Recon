// System Includes
#include <iostream>
#include <condition_variable>
#include <iostream>
#include <mutex>

// C Includes
#include <signal.h>

// TCP Includes
#include <tacopie/tacopie>

// Project Includes
#include "Drone.hpp"
#include "DroneManager.hpp"
#include "DroneComms.hpp"

using namespace std;

#ifdef _WIN32
#include <Winsock2.h>
#endif /* _WIN32 */

int main(int argc, char* argv[]) {

#ifdef _WIN32
    //! Windows netword DLL init
    WORD version = MAKEWORD(2, 2);
    WSADATA data;

    if (WSAStartup(version, &data) != 0) {
        cerr << "WSAStartup() failure" << endl;
        return -1;
    }
#endif /* _WIN32 */

    vector<string> serialNumbers = DroneInterface::DroneManager::Instance().GetConnectedDroneSerialNumbers();
    
#ifdef _WIN32
    WSACleanup();
#endif /* _WIN32 */

    return 0;
}