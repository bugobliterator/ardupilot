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

#include "HTMLClient.h"
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)

#include <SITL/SITL.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

extern struct netif netif;

void HTMLClient::init()
{
    err_t err;

    setvbuf(stdout, NULL,_IONBF, 0);
    lwip_init();

    ip4_addr_set_zero(&gw);
    ip4_addr_set_zero(&ipaddr);
    ip4_addr_set_zero(&netmask);

    init_default_netif(&ipaddr, &netmask, &gw);
    
    netif_set_status_callback(netif_default, status_callback);
    netif_set_link_callback(netif_default, link_callback);

    //AUTOIP
    autoip_set_struct(netif_default, &netif_autoip);

    dhcp_set_struct(netif_default, &netif_dhcp);

    netif_set_up(netif_default);

    err = dhcp_start(netif_default);
    if (err != ERR_OK) {
        printf("DHCP start Failed.");
    }

    err = autoip_start(netif_default);
    if (err != ERR_OK) {
        printf("AUTOIP start Failed.");
    }
    
    netif.flags &= ~(NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP); /* no ARP */
    netif.flags |= NETIF_FLAG_ETHERNET; /* but pure ethernet */
}

void HTMLClient::status_callback(struct netif *state_netif)
{
  if (netif_is_up(state_netif)) {
    printf("status_callback==UP, local interface IP is %s\n", ip4addr_ntoa(netif_ip4_addr(state_netif)));
  } else {
    printf("status_callback==DOWN\n");
  }
}

void HTMLClient::link_callback(struct netif *state_netif)
{
  if (netif_is_link_up(state_netif)) {
    printf("link_callback==UP\n");
  } else {
    printf("link_callback==DOWN\n");
  }
}

void HTMLClient::thread_loop()
{
    default_netif_poll();
}

#endif // #if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)