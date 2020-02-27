#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__
#include "lwip/err.h"
#include "lwip/netif.h"
#include "sys_arch.h"
#include "stm32f4xx_hal.h"

#define IFNAME0 'a'
#define IFNAME1 'c'

#define HOSTNAME "OpenRTK330"

#define INTERFACE_THREAD_STACK_SIZE            ( 512 )

extern ETH_HandleTypeDef EthHandle;           
	
extern sys_sem_t s_xSemaphore;

void KSZ8041NL_reset_port_init(void);
void KSZ8041NL_reset(void);

void ethernetif_input(void const *argument);
err_t ethernetif_init(struct netif *netif);

void ethernetif_link_state_check(struct netif *netif);
void ethernetif_update_config(struct netif *netif);


#endif
