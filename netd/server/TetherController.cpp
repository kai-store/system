/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <netinet/in.h>
#include <arpa/inet.h>

#define LOG_TAG "TetherController"
#include <cutils/log.h>
#include <cutils/properties.h>
#include <logwrap/logwrap.h>

#include "Fwmark.h"
#include "NetdConstants.h"
#include "Permission.h"
#include "TetherController.h"

#define IP4_CFG_IP_FORWARD          "/proc/sys/net/ipv4/ip_forward"
#define IP6_CFG_ALL_PROXY_NDP       "/proc/sys/net/ipv6/conf/all/proxy_ndp"
#define IP6_CFG_ALL_FORWARDING      "/proc/sys/net/ipv6/conf/all/forwarding"
#define IP6_IFACE_CFG_ACCEPT_RA     "/proc/sys/net/ipv6/conf/%s/accept_ra"
#define PROC_PATH_SIZE              255

namespace {

static const char BP_TOOLS_MODE[] = "bp-tools";
static const char IPV4_FORWARDING_PROC_FILE[] = "/proc/sys/net/ipv4/ip_forward";
static const char IPV6_FORWARDING_PROC_FILE[] = "/proc/sys/net/ipv6/conf/all/forwarding";

bool writeToFile(const char* filename, const char* value) {
    int fd = open(filename, O_WRONLY);
    if (fd < 0) {
        ALOGE("Failed to open %s: %s", filename, strerror(errno));
        return false;
    }

    const ssize_t len = strlen(value);
    if (write(fd, value, len) != len) {
        ALOGE("Failed to write %s to %s: %s", value, filename, strerror(errno));
        close(fd);
        return false;
    }
    close(fd);
    return true;
}

bool inBpToolsMode() {
    // In BP tools mode, do not disable IP forwarding
    char bootmode[PROPERTY_VALUE_MAX] = {0};
    property_get("ro.bootmode", bootmode, "unknown");
    return !strcmp(BP_TOOLS_MODE, bootmode);
}

}  // namespace

TetherController::TetherController() {
    mInterfaces = new InterfaceCollection();
    mDnsNetId = 0;
    mDnsForwarders = new NetAddressCollection();
    mDaemonFd = -1;
    mDaemonPid = 0;

#ifdef NETD_SUPPORT_USB_IPV6_TETHERING
    mV6Interface[0] = '\0';
    mRadvdStarted = false;
    mRadvdPid = 0;
    mV6networkaddr[0] = '\0';
    mV6TetheredInterface[0] = '\0';
	mDhcp6sPid = 0;
#endif

    if (inBpToolsMode()) {
        enableForwarding(BP_TOOLS_MODE);
    } else {
        setIpFwdEnabled();
    }
}

TetherController::~TetherController() {
    InterfaceCollection::iterator it;

    for (it = mInterfaces->begin(); it != mInterfaces->end(); ++it) {
        free(*it);
    }
    mInterfaces->clear();

    mDnsForwarders->clear();
    mForwardingRequests.clear();
}

bool TetherController::setIpFwdEnabled() {
    bool success = true;
    const char* value = mForwardingRequests.empty() ? "0" : "1";
    ALOGD("Setting IP forward enable = %s", value);
    success &= writeToFile(IPV4_FORWARDING_PROC_FILE, value);
    success &= writeToFile(IPV6_FORWARDING_PROC_FILE, value);
    return success;
}

bool TetherController::enableForwarding(const char* requester) {
    // Don't return an error if this requester already requested forwarding. Only return errors for
    // things that the caller caller needs to care about, such as "couldn't write to the file to
    // enable forwarding".
    mForwardingRequests.insert(requester);
    return setIpFwdEnabled();
}

bool TetherController::disableForwarding(const char* requester) {
    mForwardingRequests.erase(requester);
    return setIpFwdEnabled();
}

size_t TetherController::forwardingRequestCount() {
    return mForwardingRequests.size();
}

#define TETHER_START_CONST_ARG        8

int TetherController::startTethering(int num_addrs, struct in_addr* addrs) {
    if (mDaemonPid != 0) {
        ALOGE("Tethering already started");
        errno = EBUSY;
        return -1;
    }

    ALOGD("Starting tethering services");

    pid_t pid;
    int pipefd[2];

    if (pipe(pipefd) < 0) {
        ALOGE("pipe failed (%s)", strerror(errno));
        return -1;
    }

    /*
     * TODO: Create a monitoring thread to handle and restart
     * the daemon if it exits prematurely
     */
    if ((pid = fork()) < 0) {
        ALOGE("fork failed (%s)", strerror(errno));
        close(pipefd[0]);
        close(pipefd[1]);
        return -1;
    }

    if (!pid) {
        close(pipefd[1]);
        if (pipefd[0] != STDIN_FILENO) {
            if (dup2(pipefd[0], STDIN_FILENO) != STDIN_FILENO) {
                ALOGE("dup2 failed (%s)", strerror(errno));
                return -1;
            }
            close(pipefd[0]);
        }

        int num_processed_args = TETHER_START_CONST_ARG + (num_addrs/2) + 1;
        char **args = (char **)malloc(sizeof(char *) * num_processed_args);
        args[num_processed_args - 1] = NULL;
        args[0] = (char *)"/system/bin/dnsmasq";
        args[1] = (char *)"--keep-in-foreground";
        args[2] = (char *)"--no-resolv";
        args[3] = (char *)"--no-poll";
        args[4] = (char *)"--dhcp-authoritative";
        // TODO: pipe through metered status from ConnService
        args[5] = (char *)"--dhcp-option-force=43,ANDROID_METERED";
        args[6] = (char *)"--pid-file";
        args[7] = (char *)"";

        int nextArg = TETHER_START_CONST_ARG;
        for (int addrIndex=0; addrIndex < num_addrs;) {
            char *start = strdup(inet_ntoa(addrs[addrIndex++]));
            char *end = strdup(inet_ntoa(addrs[addrIndex++]));
            asprintf(&(args[nextArg++]),"--dhcp-range=%s,%s,1h", start, end);
            free(start);
            free(end);
        }

        if (execv(args[0], args)) {
            ALOGE("execl failed (%s)", strerror(errno));
        }
        ALOGE("Should never get here!");
        _exit(-1);
    } else {
        close(pipefd[0]);
        mDaemonPid = pid;
        mDaemonFd = pipefd[1];
        applyDnsInterfaces();
        ALOGD("Tethering services running");
    }

    return 0;
}

int TetherController::stopTethering() {

    if (mDaemonPid == 0) {
        ALOGE("Tethering already stopped");
        return 0;
    }

    ALOGD("Stopping tethering services");

    kill(mDaemonPid, SIGTERM);
    waitpid(mDaemonPid, NULL, 0);
    mDaemonPid = 0;
    close(mDaemonFd);
    mDaemonFd = -1;
#ifdef NETD_SUPPORT_USB_IPV6_TETHERING
    //stop radvd
    stopRadvd();

    memset(mV6networkaddr, 0, sizeof(mV6networkaddr));
    mV6Interface[0] = '\0';
#endif
    ALOGD("Tethering services stopped");
    return 0;
}

bool TetherController::isTetheringStarted() {
    return (mDaemonPid == 0 ? false : true);
}

#define MAX_CMD_SIZE 1024

int TetherController::setDnsForwarders(unsigned netId, char **servers, int numServers) {
    int i;
    char daemonCmd[MAX_CMD_SIZE];

    Fwmark fwmark;
    fwmark.netId = netId;
    fwmark.explicitlySelected = true;
    fwmark.protectedFromVpn = true;
    fwmark.permission = PERMISSION_SYSTEM;

    snprintf(daemonCmd, sizeof(daemonCmd), "update_dns:0x%x", fwmark.intValue);
    int cmdLen = strlen(daemonCmd);

    mDnsForwarders->clear();
    for (i = 0; i < numServers; i++) {
        ALOGD("setDnsForwarders(0x%x %d = '%s')", fwmark.intValue, i, servers[i]);

        struct in_addr a;

        if (!inet_aton(servers[i], &a)) {
            ALOGE("Failed to parse DNS server '%s'", servers[i]);
            mDnsForwarders->clear();
            return -1;
        }

        cmdLen += (strlen(servers[i]) + 1);
        if (cmdLen + 1 >= MAX_CMD_SIZE) {
            ALOGD("Too many DNS servers listed");
            break;
        }

        strcat(daemonCmd, ":");
        strcat(daemonCmd, servers[i]);
        mDnsForwarders->push_back(a);
    }

    mDnsNetId = netId;
    if (mDaemonFd != -1) {
        ALOGD("Sending update msg to dnsmasq [%s]", daemonCmd);
        if (write(mDaemonFd, daemonCmd, strlen(daemonCmd) +1) < 0) {
            ALOGE("Failed to send update command to dnsmasq (%s)", strerror(errno));
            mDnsForwarders->clear();
            return -1;
        }
    }
    return 0;
}

unsigned TetherController::getDnsNetId() {
    return mDnsNetId;
}

NetAddressCollection *TetherController::getDnsForwarders() {
    return mDnsForwarders;
}

int TetherController::applyDnsInterfaces() {
    char daemonCmd[MAX_CMD_SIZE];

    strcpy(daemonCmd, "update_ifaces");
    int cmdLen = strlen(daemonCmd);
    InterfaceCollection::iterator it;
    bool haveInterfaces = false;

    for (it = mInterfaces->begin(); it != mInterfaces->end(); ++it) {
        cmdLen += (strlen(*it) + 1);
        if (cmdLen + 1 >= MAX_CMD_SIZE) {
            ALOGD("Too many DNS ifaces listed");
            break;
        }

        strcat(daemonCmd, ":");
        strcat(daemonCmd, *it);
        haveInterfaces = true;
    }

    if ((mDaemonFd != -1) && haveInterfaces) {
        ALOGD("Sending update msg to dnsmasq [%s]", daemonCmd);
        if (write(mDaemonFd, daemonCmd, strlen(daemonCmd) +1) < 0) {
            ALOGE("Failed to send update command to dnsmasq (%s)", strerror(errno));
            return -1;
        }
    }
    return 0;
}

int TetherController::tetherInterface(const char *interface) {
    ALOGD("tetherInterface(%s)", interface);
    if (!isIfaceName(interface)) {
        errno = ENOENT;
        return -1;
    }
    mInterfaces->push_back(strdup(interface));

    if (applyDnsInterfaces()) {
        InterfaceCollection::iterator it;
        for (it = mInterfaces->begin(); it != mInterfaces->end(); ++it) {
            if (!strcmp(interface, *it)) {
                free(*it);
                mInterfaces->erase(it);
                break;
            }
        }
        return -1;
    } else {
        return 0;
    }
}

int TetherController::untetherInterface(const char *interface) {
    InterfaceCollection::iterator it;

    ALOGD("untetherInterface(%s)", interface);

    for (it = mInterfaces->begin(); it != mInterfaces->end(); ++it) {
        if (!strcmp(interface, *it)) {
            free(*it);
            mInterfaces->erase(it);

            return applyDnsInterfaces();
        }
    }
    errno = ENOENT;
    return -1;
}

InterfaceCollection *TetherController::getTetheredInterfaceList() {
    return mInterfaces;
}




#ifdef NETD_SUPPORT_USB_IPV6_TETHERING

//add for ipv6 USB tethering
static int get_v6network_addr_of_interface(char *interface, char *network_addr, bool need_match) {
    char rawaddrstr[INET6_ADDRSTRLEN], addrstr[INET6_ADDRSTRLEN];
    unsigned int prefixlen;
    int i, j;
    char ifname[64];  // Currently, IFNAMSIZ = 16.
    FILE *f = fopen("/proc/net/if_inet6", "r");
    if (!f) {
        return -errno;
    }

    // Format:
    // 20010db8000a0001fc446aa4b5b347ed 03 40 00 01    wlan0
    while (fscanf(f, "%32s %*02x %02x %*02x %*02x %63s\n",
                  rawaddrstr, &prefixlen, ifname) == 3) {
        // Is this the interface we're looking for?
        if (need_match && strcmp(interface, ifname)) {
            continue;
        }

        if (strcmp(ifname, "rndis0") == 0 ||strcmp(ifname, "lo") == 0 ||
            strcmp(ifname, "wlan0") == 0) {
            continue;
        }

        // Put the colons back into the address.
        for (i = 0, j = 0; i < 32; i++, j++) {
            addrstr[j] = rawaddrstr[i];
            if (i % 4 == 3) {
                addrstr[++j] = ':';
            }
        }
        addrstr[j - 1] = '\0';

        // Don't delete the link-local address as well, or it will disable IPv6
        // on the interface.
        if (strncmp(addrstr, "fe80:", 5) == 0) {
            continue;
        }

        int addrlen = strlen(addrstr);
        for(i=0, j=0; i<addrlen; i++) {
            if(addrstr[i] != ':')	j += 4;
            if(j<=(int)prefixlen)
                network_addr[i] = addrstr[i];

            if(j>=(int)prefixlen) break;
        }

        ALOGE("111address %s/%d on %s", addrstr, prefixlen, network_addr);

        addrlen = strlen(network_addr);
        for(i=addrlen; i>=0; i--) {
            if(network_addr[i] == ':') {
                if ((i+2 < INET6_ADDRSTRLEN) && strncmp(network_addr+i+1, "0000", 4) == 0) {
                    network_addr[i+1] = ':';
                    network_addr[i+2] = '\0';
                    continue;
                }
                else {
                    break;
                }
            }
        }

        addrlen = strlen(network_addr);
        //to deal with 2001:0e80:c210:000e:0002:0000:01c5:5822/64
        if(network_addr[addrlen -1] != ':') // last char is not ':'
            snprintf(network_addr+addrlen, 7, "::/%d", prefixlen);
        else
            snprintf(network_addr+addrlen, 5, "/%d", prefixlen);

        ALOGE("address %s/%d on %s of %s", addrstr, prefixlen, network_addr, ifname);

        if(!need_match) strcpy(interface, ifname);
        fclose(f);
        return 0;

    }

    fclose(f);

    return -1;
}

static int get_v6_addr_of_interface(const char *interface, char *v6_addr) {
    char rawaddrstr[INET6_ADDRSTRLEN], addrstr[INET6_ADDRSTRLEN];
    unsigned int prefixlen;
    int i, j;
    char ifname[64];  // Currently, IFNAMSIZ = 16.
    FILE *f = fopen("/proc/net/if_inet6", "r");
    if (!f) {
        return -errno;
    }

    // Format:
    // 20010db8000a0001fc446aa4b5b347ed 03 40 00 01    wlan0
    while (fscanf(f, "%32s %*02x %02x %*02x %*02x %63s\n",
                  rawaddrstr, &prefixlen, ifname) == 3) {
        // Is this the interface we're looking for?
        if (strcmp(interface, ifname)) {
            continue;
        }

        // Put the colons back into the address.
        for (i = 0, j = 0; i < 32; i++, j++) {
            addrstr[j] = rawaddrstr[i];
            if (i % 4 == 3) {
                addrstr[++j] = ':';
            }
        }
        addrstr[j - 1] = '\0';

        // Don't delete the link-local address as well, or it will disable IPv6
        // on the interface.
        if (strncmp(addrstr, "fe80:", 5) == 0) {
            continue;
        }

        int addrlen = strlen(addrstr);

        snprintf(addrstr+addrlen, 5, "/%d", prefixlen);

        ALOGE("v6 address of  %s is %s", interface, addrstr);
        strcpy(v6_addr, addrstr);

        fclose(f);
        return 0;

    }

    fclose(f);

    return -1;
}


static int runCmd(int argc, const char **argv) {
    int ret = 0;

    ret = android_fork_execvp(argc, (char **)argv, NULL, false, false);

    std::string full_cmd = argv[0];
    argc--; argv++;
    /*
     * HACK: Sometimes runCmd() is called with a ridcously large value (32)
     * and it works because the argv[] contains a NULL after the last
     * true argv. So here we use the NULL argv[] to terminate when the argc
     * is horribly wrong, and argc for the normal cases.
     */
    for (; argc && argv[0]; argc--, argv++) {
        full_cmd += " ";
        full_cmd += argv[0];
    }
    ALOGD("runCmd(%s) res=%d", full_cmd.c_str(), ret);

    return ret;
}


static const char RADVD_CONF_FILE[]    = "/data/misc/wifi/radvd.conf";
//static const char RADVD_BIN_FILE[]    = "/system/bin/radvd";

#define RADVD_PID_FILE "/data/misc/wifi/radvd.pid"

#define USB_INTERFACE "rndis0"
#define SOFTAP_INTERFACE "wlan0"

int TetherController::addV6RadvdIface(const char *iface) {

    ALOGD("addV6RadvdIface:%s", iface?iface:"null");

    if(!iface)  return -1;

    if(strlen(iface) >= sizeof(mV6Interface)) {
        ALOGE("Invalid interface %s , too long\n", iface);
        return -1;
    }

    memset(mV6Interface, 0, sizeof(mV6Interface));
    strcpy(mV6Interface, iface);

    char *iface2 = (char*)iface;
    return startRadvd(iface2, false);
}

int TetherController::rmV6RadvdIface(const char *iface) {

    ALOGD("rmV6RadvdIface: %s", iface?iface:"null");

    if(!iface)  return -1;

    stopRadvd();

    memset(mV6networkaddr, 0, sizeof(mV6networkaddr));
    mV6Interface[0] = '\0';

    return 0;
}

int TetherController::startRadvd(char *up_interface, bool idle_check){
    int fd;
    char *wbuf = NULL;
    int ret = 0;
    bool usb_tethered = false, wifi_tethered = false;

    char networkaddr[64];

    ALOGD("startRadvd");

    if(!up_interface) {
        ALOGE("NULL interface");
        return -1;
    }

    if (mRadvdStarted) {
        ALOGE("Radvd is already running");

        //check if the ipv6 address of up_interface is changed.
        memset(networkaddr, 0, sizeof(networkaddr));
        ret = get_v6network_addr_of_interface(up_interface, networkaddr, true);
        if(strcmp(networkaddr, mV6networkaddr) == 0) {
            ALOGE("radvd for interface %s has been already configed, just return\n", up_interface);
            return 0;
        }

        ALOGE("interface %s has not ipv6 addr or its network addr is changed, stop radvd first\n", up_interface);
        stopRadvd();
    }

    InterfaceCollection::iterator it;
    for (it = mInterfaces->begin(); it != mInterfaces->end(); ++it) {
        if (!strcmp(USB_INTERFACE, *it)){
            ALOGD("startRadvd has usb tethering");
            usb_tethered = true;
        }

        if(!strcmp(SOFTAP_INTERFACE, *it)){
            ALOGD("startRadvd has wifi tethering");
            wifi_tethered = true;
        }

    }

    if(!usb_tethered && !wifi_tethered) {
        ALOGE("startRadvd usb/wifi tethering is not opend just return\n");
        return -1;
    }

    if(usb_tethered && wifi_tethered) {
        ALOGE("startRadvd  cannot support usb/wifi ipv6 tethering at the same time, just for usb in this situation\n");
        wifi_tethered = false;
    }

    memset(networkaddr, 0, sizeof(networkaddr));
    ret = get_v6network_addr_of_interface(up_interface, networkaddr, !idle_check);
    if(ret < 0) {
        ALOGE("interface %s has not ipv6 addr\n", up_interface);
        return -1;
    }

    if(strcmp(networkaddr, mV6networkaddr) == 0) {
        ALOGE("radvd for interface %s has been already configed\n", up_interface);
        return -1;
    }

    memset(mV6networkaddr, 0, sizeof(mV6networkaddr));
    strcpy(mV6networkaddr, networkaddr);

    memset(networkaddr, 0, sizeof(networkaddr));
    ret = get_v6_addr_of_interface(up_interface, networkaddr);//including prefix
    if(ret < 0) {
        ALOGE("interface %s has not ipv6 addr\n", up_interface);
        return -1;
    }

    //Get dns
    char dns1[PROPERTY_VALUE_MAX] = {'\0'};
    char dns2[PROPERTY_VALUE_MAX] = {'\0'};
    char default_dns[64] = {'\0'};;

    char prop1[32];
    memset(prop1, 0, sizeof(prop1));

    snprintf(prop1, 32, "net.%s.ipv6_dns1", up_interface);

    strcpy(default_dns, networkaddr);

    int i = strlen(default_dns);
    for(; i>0; i--) {
        if(default_dns[i] == '/') {
            default_dns[i] = '\0';
            break;
        }
    }

    property_get(prop1, dns1, default_dns);
    ALOGD("get dns1 :%s from %s", dns1, prop1);

    char prop2[32];
    memset(prop2, 0, sizeof(prop2));

    snprintf(prop2, 32, "net.%s.ipv6_dns2", up_interface);

    property_get(prop2, dns2, default_dns);
    ALOGD("get dns :%s from %s", dns2, prop2);

    if(usb_tethered) {
        asprintf(&wbuf, "interface %s\n{\n\tAdvSendAdvert on;\n"
                "\tMinRtrAdvInterval 3;\n\tMaxRtrAdvInterval 10;\n"
                "\tAdvManagedFlag off;\n\tAdvOtherConfigFlag on;\n"
                "\tprefix %s\n"
                "\t{\n\t\tAdvOnLink off;\n\t\tAdvAutonomous on;\n\t\tAdvValidLifetime 50;\n"
                "\t\tAdvPreferredLifetime 10;\n\t\tAdvRouterAddr off;\n"
                "\t};\n\tRDNSS %s %s\n"
                "\t{\n"
                "\t\tAdvRDNSSLifetime 8000;\n"
                "\t};"
                "\n};\n",
                "rndis0", networkaddr, dns1, dns2);

        strcpy(mV6TetheredInterface, "rndis0");

    }

    if(wifi_tethered) {
        asprintf(&wbuf, "interface %s\n{\n\tAdvSendAdvert on;\n"
                "\tMinRtrAdvInterval 3;\n\tMaxRtrAdvInterval 10;\n"
                "\tAdvManagedFlag off;\n\tAdvOtherConfigFlag on;\n"
                "\tprefix %s\n"
                "\t{\n\t\tAdvOnLink off;\n\t\tAdvAutonomous on;\n\t\tAdvValidLifetime 50;\n"
                "\t\tAdvPreferredLifetime 10;\n\t\tAdvRouterAddr off;\n"
                "\t};\n\tRDNSS %s %s\n"
                "\t{\n"
                "\t\tAdvRDNSSLifetime 8000;\n"
                "\t};"
                "\n};\n",
                "wlan0", networkaddr, dns1, dns2);

        strcpy(mV6TetheredInterface, "wlan0");

    }

    //set config file
    fd = open(RADVD_CONF_FILE, O_CREAT | O_TRUNC | O_WRONLY | O_NOFOLLOW, 0660);
    if (fd < 0) {
        ALOGE("Cannot update \"%s\": %s", RADVD_CONF_FILE, strerror(errno));
        free(wbuf);
        return -1;
    }

    if(wbuf) {
        if (write(fd, wbuf, strlen(wbuf)) < 0) {
            ALOGE("Cannot write to \"%s\": %s", RADVD_CONF_FILE, strerror(errno));
            ret = -1;
        }
        free(wbuf);
    }

    /* Note: apparently open can fail to set permissions correctly at times */
    if (fchmod(fd, 0660) < 0) {
        ALOGE("Error changing permissions of %s to 0660: %s",
                RADVD_CONF_FILE, strerror(errno));
        close(fd);
        unlink(RADVD_CONF_FILE);
        return -1;
    }

#if 0
    if (fchown(fd, AID_SYSTEM, AID_WIFI) < 0) {
        ALOGE("Error changing group ownership of %s to %d: %s",
                RADVD_CONF_FILE, AID_WIFI, strerror(errno));
        close(fd);
        unlink(RADVD_CONF_FILE);
        return -1;
    }
#endif


    close(fd);

    //start radvd

    pid_t pid = 1;

    if ((pid = fork()) < 0) {
        ALOGE("fork failed (%s)", strerror(errno));
        return -1;
    }

    if (!pid) {
        //gid_t groups [] = { AID_NET_ADMIN, AID_NET_RAW, AID_INET };

        //setgroups(sizeof(groups)/sizeof(groups[0]), groups);
        //setresgid(AID_SYSTEM, AID_SYSTEM, AID_SYSTEM);
        //setresuid(AID_SYSTEM, AID_SYSTEM, AID_SYSTEM);

        //if (execl(RADVD_BIN_FILE, RADVD_BIN_FILE,
        //    "-C", RADVD_CONF_FILE, (char *) NULL)) {
        //    ALOGE("execl failed (%s)", strerror(errno));
        //}
        //ALOGE("Radvd failed to start");
        //return -1;

        char **args = (char **)malloc(sizeof(char *) * 7);
        args[6] = NULL;
        args[0] = (char *)"/system/bin/radvd";
        args[1] = (char *)"-C";
        args[2] = (char *)RADVD_CONF_FILE; //"/data/misc/wifi/radvd.conf";
        args[3] = (char *)"-p";
        args[4] = (char *)RADVD_PID_FILE;
        args[5] = (char *)"-n";


        if (execv(args[0], args)) {
            ALOGE("execl radvd failed (%s)", strerror(errno));
        }
        ALOGE("Should never get here!");
        _exit(-1);

    } else {
        mRadvdPid = pid;
        mRadvdStarted = true;
        ALOGD("Radvd started successfully, mRadvdPid:%d", mRadvdPid);
    }

    startDhcp6s(dns1, dns2);

    //set IP rule
    applyIpV6Rule();

    return 0;
}

int TetherController::stopRadvd(void) {

    ALOGD("stopRadvd");

    if (!mRadvdStarted) {
        ALOGE("Radvd is not running");
        return -1;
    }

    ALOGD("Stopping the Radvd service...");

#if 0
    kill(mRadvdPid, SIGTERM);
    waitpid(mRadvdPid, NULL, 0);
#endif

    FILE *f = fopen(RADVD_PID_FILE, "r");
    if(f) {
        int radvd_pid = 0;
        if(fscanf(f, "%d\n", &radvd_pid) == 1) {
            ALOGD("Pid from radvd.pid: %d, kill it", radvd_pid);
            kill(radvd_pid, SIGTERM);
            waitpid(radvd_pid, NULL, 0);
            mRadvdPid = 0;
        }
        fclose(f);//close
    }

    //if get pid from RADVD_PID_FILE fail, use orig pid, and kill it
    if(mRadvdPid) {
        ALOGD("mRadvdPid: %d, kill it", mRadvdPid);
        kill(mRadvdPid, SIGTERM);
        waitpid(mRadvdPid, NULL, 0);
    }

    //clear ip rule
    clearIpV6Rule();

    mRadvdStarted = false;
    mV6Interface[0] = '\0';
    mRadvdPid = 0;
    mV6TetheredInterface[0] = '\0';
    ALOGD("Radvd stopped successfully");

    stopDhcp6s();

    return 0;
}

static const char DHCP6S_CONF_FILE[]    = "/data/misc/wifi/dhcp6s.conf";

#define DHCP6S_PID_FILE "/data/misc/wifi/dhcp6s.pid"

int TetherController::startDhcp6s(char *dns1, char *dns2){
    int fd;
    char *wbuf = NULL, *iface = NULL;
    int ret = 0;

    char networkaddr[64] = {'\0'};;

    if(!dns1 || !dns2) return -1;

    ALOGD("startDhcp6s");

    strcpy(networkaddr, mV6networkaddr);

    int i = strlen(networkaddr);
    for(; i>0; i--) {
        if(networkaddr[i] == '/') {
            networkaddr[i] = '\0';
            break;
        }
    }

    ALOGD("startDhcp6s: network prefix: %s", networkaddr);

    if(strcmp(dns1, dns2))
        asprintf(&wbuf, "option domain-name-servers %s;\n"
                "option domain-name-servers %s;\n"
                "interface %s {\n"
                "address-pool pool1 3600;\n"
                "};\n"
                "pool pool1 {\n"
                "range %s6 to %s8 ;\n"
                "};\n",
                dns1, dns2, mV6TetheredInterface, networkaddr, networkaddr);
    else
        asprintf(&wbuf, "option domain-name-servers %s;\n"
                "interface %s {\n"
                "address-pool pool1 3600;\n"
                "};\n"
                "pool pool1 {\n"
                "range %s6 to %s8 ;\n"
                "};\n",
                dns1, mV6TetheredInterface, networkaddr, networkaddr);

    //set config file
    fd = open(DHCP6S_CONF_FILE, O_CREAT | O_TRUNC | O_WRONLY | O_NOFOLLOW, 0660);
    if (fd < 0) {
        ALOGE("Cannot update \"%s\": %s", DHCP6S_CONF_FILE, strerror(errno));
        free(wbuf);
        return -1;
    }

    if(wbuf) {
        if (write(fd, wbuf, strlen(wbuf)) < 0) {
            ALOGE("Cannot write to \"%s\": %s", DHCP6S_CONF_FILE, strerror(errno));
            ret = -1;
        }
        free(wbuf);
    }

    close(fd);

    asprintf(&iface, "%s", mV6TetheredInterface);

    //start dhcp6s

    pid_t pid = 1;

    if ((pid = fork()) < 0) {
        if(iface)
            free(iface);
        ALOGE("fork failed (%s)", strerror(errno));
        return -1;
    }

    if (!pid) {

        char **args = (char **)malloc(sizeof(char *) * 8);
        args[7] = NULL;
        args[0] = (char *)"/system/bin/dhcp6s";
        args[1] = (char *)"-c";
        args[2] = (char *)DHCP6S_CONF_FILE; //"/data/misc/dhcp6s.conf";
        args[3] = (char *)"-P";
        args[4] = (char *)DHCP6S_PID_FILE;
        args[5] = (char *)"-f";
        args[6] = (char *)iface;


        if (execv(args[0], args)) {
            ALOGE("execl dhcp6s failed (%s)", strerror(errno));
        }
        ALOGE("Should never get here!");
        _exit(-1);

    } else {
        mDhcp6sPid = pid;
        ALOGD("dhcp6s started successfully, mDhcp6sPid:%d", mDhcp6sPid);
    }

    free(iface);

    return 0;
}


int TetherController::stopDhcp6s(void){

    ALOGD("stopDhcp6s");

    if(mDhcp6sPid) {
        ALOGD("mDhcp6sPid: %d, kill it", mDhcp6sPid);
        kill(mDhcp6sPid, SIGTERM);
        waitpid(mDhcp6sPid, NULL, 0);
    }

    mDhcp6sPid = 0;
    ALOGD("dhcp6s stopped successfully");

    return 0;
}


int TetherController::applyIpV6Rule(void){

    //set ip rule
    const char *cmd3[] = {
            IP_PATH,
            "-6",
            "route",
            "add",
            mV6networkaddr,
            "dev",
            mV6TetheredInterface,
            "table",
            "75"
    };

    runCmd(ARRAY_SIZE(cmd3), cmd3);

    const char *cmd4[] = {
            IP_PATH,
            "-6",
            "route",
            "add",
            "fe80::/64",
            "dev",
            mV6TetheredInterface,
            "table",
            "75"
    };

    runCmd(ARRAY_SIZE(cmd4), cmd4);

    const char *default_routecmd[] = {
            IP_PATH,
            "-6",
            "route",
            "add",
            "::/0",
            "dev",
            mV6TetheredInterface,
            "table",
            "75"
    };

    runCmd(ARRAY_SIZE(default_routecmd), default_routecmd);

    const char *cmd5[] = {
            IP_PATH,
            "-6",
            "rule",
            "add",
            "iif",
            mV6Interface,
            "table",
            "75"
    };

    runCmd(ARRAY_SIZE(cmd5), cmd5);


    const char *cmd6[] = {
            IP_PATH,
            "-6",
            "route",
            "flush",
            "cache"
    };

    runCmd(ARRAY_SIZE(cmd6), cmd6);


    const char *cmd7[] = {
            IPTABLES_PATH,
            "-A",
            "FORWARD",
            "-p",
            "udp",
            "--dport",
            "3544",
            "-j",
            "DROP"
    };

    runCmd(ARRAY_SIZE(cmd7), cmd7);

    const char *cmd8[] = {
            IPTABLES_PATH,
            "-A",
            "FORWARD",
            "-p",
            "udp",
            "--sport",
            "3544",
            "-j",
            "DROP"
    };

    runCmd(ARRAY_SIZE(cmd8), cmd8);


    //to add rule for oif is mV6TetheredInterface, such as rndis0
    const char *cmd9[] = {
            IP_PATH,
            "-6",
            "rule",
            "add",
            "oif",
            mV6TetheredInterface,
            "table",
            "75"
    };

    runCmd(ARRAY_SIZE(cmd9), cmd9);

    return 0;
}


int TetherController::clearIpV6Rule(void){

    const char *cmd[] = {
            IP_PATH,
            "-6",
            "route",
            "del",
            mV6networkaddr,
            "dev",
            mV6TetheredInterface,
            "table",
            "75"
    };

    runCmd(ARRAY_SIZE(cmd), cmd);

    const char *cmd1[] = {
            IP_PATH,
            "-6",
            "route",
            "del",
            "fe80::/64",
            "dev",
            mV6TetheredInterface,
            "table",
            "75"
    };

    runCmd(ARRAY_SIZE(cmd1), cmd1);

    const char *del_routecmd[] = {
            IP_PATH,
            "-6",
            "route",
            "del",
            "::/0",
            "dev",
            mV6TetheredInterface,
            "table",
            "75"
    };

    runCmd(ARRAY_SIZE(del_routecmd), del_routecmd);

    const char *cmd2[] = {
            IP_PATH,
            "-6",
            "rule",
            "del",
            "iif",
            mV6Interface,
            "table",
            "75"
    };

    runCmd(ARRAY_SIZE(cmd2), cmd2);

    const char *flush_cmd[] = {
            IP_PATH,
            "-6",
            "route",
            "flush",
            "cache"
    };

    runCmd(ARRAY_SIZE(flush_cmd), flush_cmd);

    const char *del_udpcmd[] = {
            IPTABLES_PATH,
            "-D",
            "FORWARD",
            "-p",
            "udp",
            "--dport",
            "3544",
            "-j",
            "DROP"
    };

    runCmd(ARRAY_SIZE(del_udpcmd), del_udpcmd);

    const char *del_udpcmd1[] = {
            IPTABLES_PATH,
            "-D",
            "FORWARD",
            "-p",
            "udp",
            "--sport",
            "3544",
            "-j",
            "DROP"
    };

    runCmd(ARRAY_SIZE(del_udpcmd1), del_udpcmd1);

    return 0;
}
//add for ipv6 USB tethering END
#endif
