#include <stdio.h>
#include <stdlib.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <netinet/in.h>

int main()
{
    struct ifaddrs *ifaddr, *ifa;
    char addr[INET6_ADDRSTRLEN];

    if (getifaddrs(&ifaddr) == -1)
    {
        perror("getifaddrs");
        return 1;
    }

    printf("Network interfaces detected:\n");
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr == NULL)
            continue; // Skip null addresses

        int family = ifa->ifa_addr->sa_family;

        printf("Interface: %s\n", ifa->ifa_name);
        if (family == AF_INET)
        {
            struct sockaddr_in *sa = (struct sockaddr_in *)ifa->ifa_addr;
            inet_ntop(AF_INET, &sa->sin_addr, addr, sizeof(addr));
            printf("  IPv4 Address: %s\n", addr);
        }
        else if (family == AF_INET6)
        {
            struct sockaddr_in6 *sa = (struct sockaddr_in6 *)ifa->ifa_addr;
            inet_ntop(AF_INET6, &sa->sin6_addr, addr, sizeof(addr));
            printf("  IPv6 Address: %s\n", addr);
        }
    }

    freeifaddrs(ifaddr);
    return 0;
}