#include "lwip/opt.h"
#include "lwip/netif.h"

// lwIP's LWIP_HOOK_IP4_ROUTE (lwipopts.h): no gateway route, lwIP's own netif choice
extern "C" struct netif *net_m4_routing_hook(const struct ip4_addr *)
{
    return nullptr;
}
