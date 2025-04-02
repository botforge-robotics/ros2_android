#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dlfcn.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <errno.h>
#include <net/if.h>

// Function to run ifconfig and parse output
int parse_ifconfig(struct ifaddrs **ifap)
{
    FILE *fp = popen("ifconfig", "r");
    if (!fp)
    {
        perror("popen failed");
        return -1;
    }

    struct ifaddrs *wlan_head = NULL, *wlan_prev = NULL;
    struct ifaddrs *other_head = NULL, *other_prev = NULL;
    char line[256], iface_name[32], inet_addr[32];
    struct ifaddrs *current_iface = NULL;  // Track current interface being processed
    int is_wlan = 0;  // Flag to track if current interface is wlan

    while (fgets(line, sizeof(line), fp))
    {
        // New interface section starts
        if (sscanf(line, "%31[^:]:", iface_name) == 1 && strstr(line, "flags"))
        {
            struct ifaddrs *iface = calloc(1, sizeof(struct ifaddrs));
            iface->ifa_name = strdup(iface_name);
            iface->ifa_flags = IFF_UP | IFF_RUNNING;

            is_wlan = (strncmp(iface_name, "wlan", 4) == 0);
            current_iface = iface;

            if (is_wlan)
            {
                if (wlan_prev)
                    wlan_prev->ifa_next = iface;
                else
                    wlan_head = iface;
                wlan_prev = iface;
            }
            else
            {
                if (other_prev)
                    other_prev->ifa_next = iface;
                else
                    other_head = iface;
                other_prev = iface;
            }
        }
        // Parse inet address for current interface
        else if (current_iface && strstr(line, "inet "))
        {
            if (sscanf(line, " inet %31s", inet_addr) == 1)
            {
                struct sockaddr_in *addr = calloc(1, sizeof(struct sockaddr_in));
                addr->sin_family = AF_INET;
                inet_pton(AF_INET, inet_addr, &addr->sin_addr);
                current_iface->ifa_addr = (struct sockaddr *)addr;
            }
        }
    }

    pclose(fp);

    // Connect wlan interfaces to other interfaces
    if (wlan_prev)
        wlan_prev->ifa_next = other_head;

    // Return wlan_head if exists, otherwise other_head
    *ifap = wlan_head ? wlan_head : other_head;

    return 0;
}

// Overriding getifaddrs()
int getifaddrs(struct ifaddrs **ifap)
{
    return parse_ifconfig(ifap);
}

// Free allocated memory
void freeifaddrs(struct ifaddrs *ifa)
{
    while (ifa)
    {
        struct ifaddrs *next = ifa->ifa_next;
        free(ifa->ifa_addr);
        free(ifa->ifa_name);
        free(ifa);
        ifa = next;
    }
}