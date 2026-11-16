#include <iostream>
#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <bitset>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <atomic>
#include <timeapi.h>

std::atomic<bool> running{true};

void signal_handler(int signal)
{
    if (signal == SIGINT)
    {
        running.store(false, std::memory_order_release);
    }
}

int main()
{

    WSADATA wsa;

    std::signal(SIGINT, signal_handler);
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        std::cerr << "WSAStartup failed" << "\n";
        return 1;
    }

    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET)
    {
        std::cerr << "socket failed: " << WSAGetLastError() << "\n";
        WSACleanup();
        return 1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5005);                           // Pico's port
    inet_pton(AF_INET, "192.168.137.124", &addr.sin_addr); // Pico's IP

    // For UDP, connect() only sets the default destination for send().
    if (connect(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == SOCKET_ERROR)
    {

        std::cerr << "connect failed: " << WSAGetLastError() << "\n";
        closesocket(sock);
        WSACleanup();
        return 1;
    }

    constexpr int sleepTime = 1;
    constexpr uint16_t exitByte = 0x80; // bit 7: sender is quitting

    timeBeginPeriod(1);

    uint16_t value{0};
    uint16_t lastValue{0xFF};

    auto sendValue = [&](uint16_t v)
    {
        char buf[2] = {static_cast<char>(v & 0xFF),         // low byte first
                       static_cast<char>((v >> 8) & 0xFF)}; // high byte second
        send(sock, buf, 2, 0);
    };
    while (running.load(std::memory_order_acquire))
    {
        /*
            bit 0 (W): rotate all wheels clockwise at base speed
            bit 1 (S): rotate all wheels counter clockwise at base speed
            bit 2 (A): increase speed of left wheels
            bit 3 (D): increase speed of right wheels
            bit 4 (Shift): increase base speed; 0 sets it to normal
            bit 5 (left mouse button): set servo one to 180 degree
            bit 6 (right mouse button): set servo two to 180 degree
            bit 7 (0x80): sender is quitting, stop everything
            bit 8 (c): going side ways
            bit 9 (control): decrease the radius

        */
        const bool w = GetAsyncKeyState('W') < 0;
        const bool s = GetAsyncKeyState('S') < 0;
        const bool a = GetAsyncKeyState('A') < 0;
        const bool d = GetAsyncKeyState('D') < 0;
        const bool c = GetAsyncKeyState('C') < 0;

        const bool shift = GetAsyncKeyState(VK_SHIFT) < 0;

        const bool lmb = GetAsyncKeyState(VK_LBUTTON) < 0;
        const bool rmb = GetAsyncKeyState(VK_RBUTTON) < 0;

        const bool control = GetAsyncKeyState(VK_CONTROL) < 0;

        value = static_cast<uint16_t>(w | (s << 1) | (a << 2) | (d << 3) | (shift << 4) | (lmb << 5) | (rmb << 6) | (c << 8) | (control << 9));
        if (value != lastValue)
        {
            std::cout << std::bitset<8>(value) << "\n";
            lastValue = value;
            for (int n = 0; n < 5; ++n)
            {
                sendValue(value);
                Sleep(sleepTime);
            }
        }
        Sleep(sleepTime);
    }

    for (int i = 0; i < 5; ++i)
    {
        sendValue(exitByte);
        std::cout << std::bitset<8>(exitByte) << "\n";
        Sleep(sleepTime);
    }

    closesocket(sock);
    WSACleanup();
    timeEndPeriod(1);
    return 0;
}