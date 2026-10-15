/*
  lwIP options for the M4 network service (the M4 image and its SITL process).
  Feature options copy libraries/AP_Networking/config/lwipopts.h; memory is static
  (no malloc), sized by the Network Baseline for the reduced Lua set.
  The PPP capture hook (a pcap file on the M7's SD card) is not carried to the M4.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* features, as AP_Networking's lwipopts.h */
#define LWIP_IPV4                       1
#define LWIP_IPV6                       0
#define NO_SYS                          0
#define LWIP_SOCKET                     1
#define LWIP_NETCONN                    1
#define LWIP_NETIF_API                  1
#define SO_REUSE                        1
#define LWIP_IGMP                       1
#define LWIP_ICMP                       1
#define LWIP_SNMP                       1
#define LWIP_DNS                        0
#define LWIP_MDNS_RESPONDER             0
#define LWIP_NUM_NETIF_CLIENT_DATA      0
#define LWIP_HAVE_LOOPIF                1
#define LWIP_NETIF_LOOPBACK             1
#define LWIP_LOOPBACK_MAX_PBUFS         2
#define TCP_LISTEN_BACKLOG              1
#define LWIP_COMPAT_SOCKETS             0
#define LWIP_SO_RCVTIMEO                1
#define LWIP_SO_RCVBUF                  1
#define LWIP_SO_LINGER                  1
#define LWIP_TCPIP_CORE_LOCKING         1
#define LWIP_NETCONN_SEM_PER_THREAD     0
#define LWIP_NETIF_LINK_CALLBACK        1
#define LWIP_NETIF_STATUS_CALLBACK      1
#define LWIP_NETIF_EXT_STATUS_CALLBACK  1
#define LWIP_TIMEVAL_PRIVATE            0
#define LWIP_FD_SET_PRIVATE             0
#define LCP_ECHOINTERVAL                1
#define LCP_MAXECHOFAILS                5
#define MEM_ALIGNMENT                   4U
#define LWIP_TCP                        1
#define TCP_TTL                         255
#define LWIP_ALTCP                      1
#define TCP_QUEUE_OOSEQ                 1
#define TCP_MSS                         1024
#define TCP_MAXRTX                      12
#define TCP_SYNMAXRTX                   4
#define LWIP_ARP                        1
#define ARP_TABLE_SIZE                  10
#define ARP_QUEUEING                    0
#define ARP_PROXYARP_SUPPORT            1
#define IP_FORWARD                      1
#define PBUF_LINK_HLEN                  16
#define IP_REASSEMBLY                   1
#define IP_FRAG                         1
#define ICMP_TTL                        255
#define LWIP_DHCP                       1
#define DHCP_DOES_ARP_CHECK             1
#define LWIP_AUTOIP                     1
#define LWIP_DHCP_AUTOIP_COOP           1
#define LWIP_UDP                        1
#define LWIP_UDPLITE                    1
#define UDP_TTL                         255
#define LWIP_RAW                        0
#define LWIP_STATS                      0
#define LWIP_STATS_DISPLAY              0
#define LWIP_NETBIOS_RESPOND_NAME_QUERY 0
#define SYS_LIGHTWEIGHT_PROT            1
#define PPP_SUPPORT                     1
#define NUM_PPP                         1
#define PPPOE_SUPPORT                   0
#define PPPOS_SUPPORT                   1
#define PAP_SUPPORT                     0
#define CHAP_SUPPORT                    0
#define MSCHAP_SUPPORT                  0
#define CBCP_SUPPORT                    0
#define CCP_SUPPORT                     0
#define VJ_SUPPORT                      0
#define MD5_SUPPORT                     0
#define LWIP_NO_CTYPE_H                 1

/* static memory: no malloc on the M4 */
#define MEM_LIBC_MALLOC                 0
#define MEMP_MEM_MALLOC                 0
#ifndef AP_NETWORKING_M4_MEM_SIZE
#define AP_NETWORKING_M4_MEM_SIZE       18752   /* CubeOrangePlus-M4LuaNet; CubeRedPrimary-M4Lua 20652 */
#endif
#define MEM_SIZE                        AP_NETWORKING_M4_MEM_SIZE
#define MEMP_NUM_SYS_TIMEOUT            (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 4)
#define MEMP_NUM_PBUF                   16
#define MEMP_NUM_RAW_PCB                1
#define MEMP_NUM_UDP_PCB                16
#define MEMP_NUM_TCP_PCB                10
#define MEMP_NUM_TCP_PCB_LISTEN         6
#define MEMP_NUM_NETBUF                 16
#define MEMP_NUM_NETCONN                20
#define MEMP_NUM_TCPIP_MSG_API          8
#define MEMP_NUM_TCPIP_MSG_INPKT        16
#define PBUF_POOL_SIZE                  176
#define PBUF_POOL_BUFSIZE               256
#define TCP_WND                         2048
#define TCP_SND_BUF                     2048
#define TCP_SND_QUEUELEN                (4 * TCP_SND_BUF / TCP_MSS)
#define TCP_SNDLOWAT                    512
#define TCP_OVERSIZE                    0
#define TCP_OOSEQ_MAX_PBUFS             4
#define MEMP_NUM_TCP_SEG                32
#define DEFAULT_ACCEPTMBOX_SIZE         4
#define TCPIP_MBOX_SIZE                 16
#define DEFAULT_TCP_RECVMBOX_SIZE       4   /* every TCP netconn, accepted ones included */
#define DEFAULT_UDP_RECVMBOX_SIZE       8
#define DEFAULT_RAW_RECVMBOX_SIZE       4
#define IP_REASS_MAX_PBUFS              24
#define MEMP_NUM_REASSDATA              IP_REASS_MAX_PBUFS

/* threads: os_thread_start adds NORMALPRIO on ChibiOS and ignores the priority on POSIX */
#define TCPIP_THREAD_NAME               "tcpip"
#define TCPIP_THREAD_STACKSIZE          2048
#define TCPIP_THREAD_PRIO               3

/* hooks: errno per thread, the routing hook (PPP gateway routes), the core-lock check, asserts */
void net_m4_set_errno(int err);
#define set_errno(err)                  net_m4_set_errno(err)
struct ip4_addr;
struct netif;
struct netif *net_m4_routing_hook(const struct ip4_addr *dest);
#define LWIP_HOOK_IP4_ROUTE             net_m4_routing_hook
void sys_check_core_locking(void);
#define LWIP_ASSERT_CORE_LOCKED()       sys_check_core_locking()
void net_m4_assert(const char *file, int line);
#define LWIP_PLATFORM_ASSERT(x)         net_m4_assert(__FILE__, __LINE__)
#define LWIP_PLATFORM_DIAG(x)           do { } while (0)

#ifdef __cplusplus
}
#endif
