#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>



static int sock;
static struct sockaddr_in addr;

void initUdpClient(std::string ip, int port) {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip.c_str());        
}

void sendUdpString(std::string s) {
    sendto(sock, s.c_str(), s.length(), 0, (struct sockaddr*)&addr, sizeof(addr));
}

void finalizeUdpClient() {
    close(sock);
}
