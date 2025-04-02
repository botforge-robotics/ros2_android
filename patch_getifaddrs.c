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
    //printf("[DEBUG] Running parse_ifconfig()\n");
    FILE *fp = popen("ifconfig", "r");
    if (!fp)
    {
        perror("popen failed");
        return -1;
    }

    struct ifaddrs *head = NULL, *prev = NULL;
    char line[256], iface_name[32], inet_addr[32];

    while (fgets(line, sizeof(line), fp))
    {
        //printf("[DEBUG] Read line: %s", line);

        if (sscanf(line, "%31[^:]:", iface_name) == 1 && strstr(line, "flags"))
        { // Parse interface name
            //printf("[DEBUG] Detected interface: %s\n", iface_name);
            struct ifaddrs *iface = calloc(1, sizeof(struct ifaddrs));
            iface->ifa_name = strdup(iface_name);
            iface->ifa_flags = IFF_UP | IFF_RUNNING;

            if (prev)
                prev->ifa_next = iface;
            else
                head = iface;
            prev = iface;
        }
        else if (strstr(line, "inet ") && prev)
        { // Parse IPv4 address
            if (sscanf(line, " inet %31s", inet_addr) == 1)
            {
                //printf("[DEBUG] Found IPv4 address: %s for interface %s\n", inet_addr, prev->ifa_name);
                struct sockaddr_in *addr = calloc(1, sizeof(struct sockaddr_in));
                addr->sin_family = AF_INET;
                inet_pton(AF_INET, inet_addr, &addr->sin_addr);
                prev->ifa_addr = (struct sockaddr *)addr;
            }
        }
    }

    pclose(fp); // Close file pointer

    *ifap = head;
    //printf("[DEBUG] parse_ifconfig() completed successfully\n");
    return 0;
}

// Overriding getifaddrs()
int getifaddrs(struct ifaddrs **ifap)
{
    //printf("[DEBUG] getifaddrs() called\n");
    return parse_ifconfig(ifap);
}

// Free allocated memory (needed for compatibility)
void freeifaddrs(struct ifaddrs *ifa)
{
    //printf("[DEBUG] freeifaddrs() called\n");
    while (ifa)
    {
        struct ifaddrs *next = ifa->ifa_next;
        //printf("[DEBUG] Freeing interface: %s\n", ifa->ifa_name);
        free(ifa->ifa_addr);
        free(ifa->ifa_name);
        free(ifa);
        ifa = next;
    }
    //printf("[DEBUG] freeifaddrs() completed\n");
}