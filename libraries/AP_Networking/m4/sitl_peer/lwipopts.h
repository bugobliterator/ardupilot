/*
  lwIP options for m4_net_peer, the host-side PPP peer of the SITL M4 network process:
  no OS, raw API only, heap from the C library.
 */
#pragma once

#define NO_SYS                          1
#define SYS_LIGHTWEIGHT_PROT            0
#define LWIP_SOCKET                     0
#define LWIP_NETCONN                    0
#define MEM_LIBC_MALLOC                 1
#define MEMP_MEM_MALLOC                 1
#define MEM_ALIGNMENT                   8
#define LWIP_IPV4                       1
#define LWIP_IPV6                       0
#define LWIP_ARP                        0
#define LWIP_ICMP                       1
#define LWIP_RAW                        0
#define LWIP_DHCP                       0
#define LWIP_AUTOIP                     0
#define LWIP_DNS                        0
#define LWIP_IGMP                       1
#define LWIP_TCP                        1
#define LWIP_UDP                        1
#define TCP_MSS                         1024
#define TCP_WND                         8192
#define TCP_SND_BUF                     8192
#define TCP_SND_QUEUELEN                32
#define MEMP_NUM_TCP_SEG                64
#define LWIP_STATS                      0
#define PPP_SUPPORT                     1
#define PPPOS_SUPPORT                   1
#define PPP_IPV4_SUPPORT                1
#define PPP_IPV6_SUPPORT                0
#define NUM_PPP                         1
#define PAP_SUPPORT                     0
#define CHAP_SUPPORT                    0
#define MSCHAP_SUPPORT                  0
#define CCP_SUPPORT                     0
#define VJ_SUPPORT                      0
#define LCP_ECHOINTERVAL                0

// the host's <arpa/inet.h> provides htons() and friends
#define LWIP_DONT_PROVIDE_BYTEORDER_FUNCTIONS 1

// datagrams to the multicast groups of udp-group-to-host relays go to the host
struct pbuf;
struct netif;
#ifdef __cplusplus
extern "C" {
#endif
int m4_net_peer_ip4_input(struct pbuf *p, struct netif *inp);
#ifdef __cplusplus
}
#endif
#define LWIP_HOOK_IP4_INPUT(pbuf, input_netif) m4_net_peer_ip4_input((pbuf), (input_netif))
