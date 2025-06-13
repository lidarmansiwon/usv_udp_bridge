// receiver.cpp
#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 5005

struct NavigationMsg {
    float x, y, psi, u, v, r, w;
};

int main() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Socket creation failed");
        return -1;
    }

    struct sockaddr_in recv_addr;
    memset(&recv_addr, 0, sizeof(recv_addr));

    recv_addr.sin_family = AF_INET;
    recv_addr.sin_port = htons(PORT);
    recv_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd, (struct sockaddr*)&recv_addr, sizeof(recv_addr)) < 0) {
        perror("Bind failed");
        close(sockfd);
        return -1;
    }

    NavigationMsg nav_msg;

    while (true) {
        ssize_t received = recv(sockfd, &nav_msg, sizeof(nav_msg), 0);
        if (received > 0) {
            std::cout << "Received Navigation Data:" << std::endl;
            std::cout << "x: " << nav_msg.x << ", y: " << nav_msg.y << ", psi: " << nav_msg.psi << std::endl;
            std::cout << "u: " << nav_msg.u << ", v: " << nav_msg.v << ", r: " << nav_msg.r << ", w: " << nav_msg.w << std::endl;
        } else {
            perror("Receive failed");
            break;
        }
    }

    close(sockfd);
    return 0;
}
