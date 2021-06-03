/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Siddharth Bharat Purohit
 */

#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <string.h>

#include "TCPClient.h"
#include <AP_HAL/AP_HAL.h>

#include "lwip/opt.h"

#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include "lwip/tcpip.h"
#include "netif/tapif.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)

#include <SITL/SITL.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

extern struct netif netif;

#define LWIP_PORT_INIT_IPADDR(addr)   IP4_ADDR((addr), 192,168,20,200)
#define LWIP_PORT_INIT_GW(addr)       IP4_ADDR((addr), 192,168,20,1)
#define LWIP_PORT_INIT_NETMASK(addr)  IP4_ADDR((addr), 255,255,255,0)

static uint8_t HW_ADDR[] = LWIP_MAC_ADDR_BASE;
void TCPClient::init()
{
    err_t result;

    result = sys_sem_new(&init_sem, 0);
    if (result != ERR_OK) {
        AP_HAL::panic("Failed to initialise sys sem");
    }
    /* Initializes the thing.*/
    tcpip_init(init_iface, this);
    sys_sem_wait(&init_sem);
    sys_sem_free(&init_sem);
}

void TCPClient::init_iface(void *arg)
{
    TCPClient* tcpclient = (TCPClient*)arg;

    /* TCP/IP parameters, runtime or compile time.*/
    memcpy(netif.hwaddr, HW_ADDR, sizeof(HW_ADDR));
    LWIP_PORT_INIT_IPADDR(&tcpclient->ipaddr);
    LWIP_PORT_INIT_GW(&tcpclient->gw);
    LWIP_PORT_INIT_NETMASK(&tcpclient->netmask);
#if LWIP_NETIF_HOSTNAME
    netif.hostname = NULL;
#endif

#if LWIP_NETIF_HOSTNAME
    if (netif.hostname == NULL) {
        netif.hostname = LWIP_NETIF_HOSTNAME_STRING;
    }
#endif

    /* Add interface. */
    netif_add(&netif, &tcpclient->ipaddr, &tcpclient->netmask, &tcpclient->gw, 
                                 NULL, tapif_init, tcpip_input);

    netif_set_default(&netif);

    netif_set_up(&netif);

    autoip_start(&netif);

    sys_sem_signal(&tcpclient->init_sem);
}

void TCPClient::thread_loop()
{
}

#endif // #if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)