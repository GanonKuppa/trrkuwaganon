#pragma once

void initUdpClient(std::string ip, int port);
void sendUdpString(std::string s);
void finalizeUdpClient();