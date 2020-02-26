#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#define NO_SYS                         0

#define SYS_LIGHTWEIGHT_PROT           1

/* ---------- Memory options ---------- */
#define MEM_ALIGNMENT                  4
#define MEM_SIZE                       (10*1024)
#define MEMP_NUM_PBUF                  16
#define MEMP_NUM_UDP_PCB               4
#define MEMP_NUM_TCP_PCB               14
#define MEMP_NUM_TCP_PCB_LISTEN        1
#define MEMP_NUM_TCP_SEG               16
#define MEMP_NUM_SYS_TIMEOUT           8

/* ---------- Pbuf ---------- */
#define PBUF_POOL_SIZE                 12
#define PBUF_POOL_BUFSIZE              512

/* ---------- TCP ---------- */
#define LWIP_TCP                       1
#define TCP_TTL                        255

#define TCP_QUEUE_OOSEQ                0

#define TCP_MSS                        (1500 - 40)
#define TCP_SND_BUF                    (4*TCP_MSS)
#define TCP_SND_QUEUELEN               (2*TCP_SND_BUF/TCP_MSS)
#define TCP_WND                        (2*TCP_MSS)

/* ---------- ICMP ---------- */
#define LWIP_ICMP                      1

/* ---------- DNS ----------- */
#define LWIP_DNS                       1

/* ---------- DHCP ---------- */
#define LWIP_DHCP                      1
#define LWIP_NETIF_HOSTNAME            1

/* ---------- UDP ----------- */
#define LWIP_UDP                       1
#define UDP_TTL                        255
#define LWIP_UDPLITE                   1

/* ---------- Statistics options ---------- */
#define LWIP_STATS                     0
#define LWIP_PROVIDE_ERRNO             1

/* ---------- link callback options ---------- */
#define LWIP_NETIF_LINK_CALLBACK       1

/*
   --------------------------------------
   ---------- Checksum options ----------
   --------------------------------------
*/
#define CHECKSUM_BY_HARDWARE

#ifdef CHECKSUM_BY_HARDWARE
#define CHECKSUM_GEN_IP                0
#define CHECKSUM_GEN_UDP               0
#define CHECKSUM_GEN_TCP               0
#define CHECKSUM_CHECK_IP              0
#define CHECKSUM_CHECK_UDP             0
#define CHECKSUM_CHECK_TCP             0
#define CHECKSUM_GEN_ICMP              0
#else
#define CHECKSUM_GEN_IP                1
#define CHECKSUM_GEN_UDP               1
#define CHECKSUM_GEN_TCP               1
#define CHECKSUM_CHECK_IP              1
#define CHECKSUM_CHECK_UDP             1
#define CHECKSUM_CHECK_TCP             1
#define CHECKSUM_GEN_ICMP              1
#endif

/*
   -------------------------------------------
   ---------- SequentialAPI----------
   -------------------------------------------
*/
#define LWIP_NETCONN                   1

/*
   ----------------------------------
   ---------- Socket API----------
   ----------------------------------
*/
#define LWIP_SOCKET                    1

/*
   ----------------------------------
   ---------- RCV Time Out ----------
   ----------------------------------
*/
#define LWIP_SO_RCVTIMEO               1

/*
   ---------------------------------
   ---------- OS options ----------
   ---------------------------------
*/
#define TCPIP_THREAD_NAME              "TCP/IP"
#define TCPIP_THREAD_STACKSIZE         1*1024
#define TCPIP_MBOX_SIZE                20
#define DEFAULT_UDP_RECVMBOX_SIZE      20
#define DEFAULT_TCP_RECVMBOX_SIZE      20
#define DEFAULT_ACCEPTMBOX_SIZE        20
#define DEFAULT_THREAD_STACKSIZE       1024
#define TCPIP_THREAD_PRIO              osPriorityAboveNormal

/*
   --------------------------------------
   ---------- Lwip DEBUG----------
   --------------------------------------
*/
//#define LWIP_DEBUG

#endif /* __LWIPOPTS_H__ */
