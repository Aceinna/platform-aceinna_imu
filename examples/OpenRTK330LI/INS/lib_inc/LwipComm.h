#ifndef _LWIPCOMM_H
#define _LWIPCOMM_H

#include <stdio.h>
#include "lwipopts.h"
#include "lwip/ip_addr.h"
#include "UserConfiguration.h"


/*Static IP ADDRESS*/
#define IP_ADDR0   (uint8_t) gUserConfiguration.staticIp[0]
#define IP_ADDR1   (uint8_t) gUserConfiguration.staticIp[1]
#define IP_ADDR2   (uint8_t) gUserConfiguration.staticIp[2]
#define IP_ADDR3   (uint8_t) gUserConfiguration.staticIp[3]

/*NETMASK*/
#define NETMASK_ADDR0   (uint8_t) gUserConfiguration.netmask[0]
#define NETMASK_ADDR1   (uint8_t) gUserConfiguration.netmask[1]
#define NETMASK_ADDR2   (uint8_t) gUserConfiguration.netmask[2]
#define NETMASK_ADDR3   (uint8_t) gUserConfiguration.netmask[3]

/*Gateway Address*/
#define GW_ADDR0   (uint8_t) gUserConfiguration.gateway[0]
#define GW_ADDR1   (uint8_t) gUserConfiguration.gateway[1]
#define GW_ADDR2   (uint8_t) gUserConfiguration.gateway[2]
#define GW_ADDR3   (uint8_t) gUserConfiguration.gateway[3]

#define ETH_LINK_DOWN 0
#define ETH_LINK_UP 1

#define MAX_DHCP_TRIES 5

typedef enum
{
    DHCP_STATE_OFF                  = 0,
    DHCP_STATE_START                = 1,
    DHCP_STATE_WAIT_ADDRESS         = 2,        
    DHCP_STATE_ADDRESS_ASSIGNED     = 3,
    DHCP_STATE_TIMEOUT              = 4,
    DHCP_STATE_LINK_DOWN            = 5
} dhcp_state_enum_t;

extern struct netif gnetif;
extern uint8_t eth_link_state;
extern uint8_t eth_dhcp_state;

void ethernet_init(void);

uint8_t dhcp_supplied_address(const struct netif *netif);
void User_notification(struct netif *netif);
void ethernetif_notify_conn_changed(struct netif *netif);
uint8_t get_eth_link_state(void);
void netif_ethernet_config_changed(void);
void netif_ntrip_config_changed(void);
void netif_set_static_ip(struct netif *netif);
void dhcp_link_down(void);
void DHCP_thread(void const *argument);
uint8_t dns_get_ip_by_hostname(uint8_t *hostname, ip_addr_t* addr);


#endif
