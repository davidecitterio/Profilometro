/**
  ******************************************************************************
  * @author  Davide Citterio
  * @date    6/11/17 
  * @brief   Header file for tcp_echoclient.c
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TCP_ECHOCLIENT_H__
#define __TCP_ECHOCLIENT_H__

#define DEST_IP_ADDR0   (uint8_t)192
#define DEST_IP_ADDR1   (uint8_t)168
#define DEST_IP_ADDR2   (uint8_t)2
#define DEST_IP_ADDR3   (uint8_t)1

#define LOCAL_IP_ADDR0   (uint8_t)192
#define LOCAL_IP_ADDR1   (uint8_t)168
#define LOCAL_IP_ADDR2   (uint8_t)2
#define LOCAL_IP_ADDR3   (uint8_t)5

#define DEST_PORT       (uint16_t)8888
#define LOCAL_PORT      (uint16_t)80
#define LISTEN_INCOMING_PORT      (uint16_t)8887
   
/*NETMASK*/
#define NETMASK_ADDR0   (uint8_t) 255
#define NETMASK_ADDR1   (uint8_t) 255
#define NETMASK_ADDR2   (uint8_t) 255
#define NETMASK_ADDR3   (uint8_t) 0

/*Gateway Address*/
#define GW_ADDR0   (uint8_t) 192
#define GW_ADDR1   (uint8_t) 168
#define GW_ADDR2   (uint8_t) 2
#define GW_ADDR3   (uint8_t) 3

#define ALIVE_MICRO_PORT	 (uint16_t)8889
#define ALIVE_SERVER_PORT	 (uint16_t)8889



/* Includes ------------------------------------------------------------------*/

#include "lwip/tcp.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void tcp_client_connect(void);
void tcp_alive_connect(void);
void tcp_send(char * _data);
void tcp_start_listen(void);


void start_measuring(void);
void stop_measuring(void);

extern int config_mode;
extern int num_of_points;

#endif /* __TCP_ECHOCLIENT_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
