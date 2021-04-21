// System Includes
#include <condition_variable>
#include <iostream>
#include <mutex>

// C Includes
#include <signal.h>

// TCP Includes
#include <tacopie/tacopie>

using namespace std;

#ifdef _WIN32
#include <Winsock2.h>
#endif /* _WIN32 */

condition_variable cv;

void
signint_handler(int) {
    cv.notify_all();
}

void
on_new_message(tacopie::tcp_client& client, const tacopie::tcp_client::read_result& res) {
    if (res.success) {
        cout << "Client recv data" << endl;
        client.async_write({ res.buffer, nullptr });
        client.async_read({ 1024, bind(&on_new_message, ref(client), placeholders::_1) });
    }
    else {
        cout << "Client disconnected" << endl;
        client.disconnect();
    }
}

int
main(void) {
#ifdef _WIN32
    //! Windows netword DLL init
    WORD version = MAKEWORD(2, 2);
    WSADATA data;

    if (WSAStartup(version, &data) != 0) {
        cerr << "WSAStartup() failure" << endl;
        return -1;
    }
#endif /* _WIN32 */

    tacopie::tcp_client client;
    client.connect("127.0.0.1", 3001);
    client.async_read({ 1024, bind(&on_new_message, ref(client), placeholders::_1) });

    signal(SIGINT, &signint_handler);

    mutex mtx;
    unique_lock<mutex> lock(mtx);
    cv.wait(lock);

#ifdef _WIN32
    WSACleanup();
#endif /* _WIN32 */

    return 0;
}