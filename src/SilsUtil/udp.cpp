#include <winsock2.h>
#include <string>

static SOCKET sock;
static struct sockaddr_in addr;
static WSAData wsaData;


void initUdpClient(std::string ip, int port) {
    WSAStartup(MAKEWORD(2, 0), &wsaData);
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.S_un.S_addr = inet_addr(ip.c_str());

}

void sendUdpString(std::string s) {
    sendto(sock, s.c_str(), s.length(), 0, (struct sockaddr*)&addr, sizeof(addr));
}

void finalizeUdpClient() {
    closesocket(sock);
    WSACleanup();
}

