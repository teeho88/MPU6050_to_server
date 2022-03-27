#ifndef AT_ESP8266_H
#define AT_ESP_8266_H

#include "main.h"
#include "stdio.h"
#include "string.h"

enum WIFI_MODE{
	STA     = 1,
	AP      = 2,
	STA_AP  = 3
};

enum TRANS_MODE{
	SINGLE_CON   = 0,
	MULTI_CON    = 1,
};

enum TRANSPARENT_PROTOCOL{
	TCP_TRANS    = 0,
	UDP_TRANS    = 1,
};

#define IP "192.168.1.111"
#define PORT 80
#define SSID "SSID"
#define PASS "PASS_WORDS"

void ESP_SendCommand(char* cmd);
void ESP_Init(UART_HandleTypeDef *huart, uint8_t mode, uint8_t *inBuff, uint16_t len);
void ESP_SetIP(uint8_t mode, char *ip);
void ESP_WifiConnect(char *ssid, char *pass);
void ESP_SoftAPCreate(char *ssid, char *pass);
HAL_StatusTypeDef ESP_CheckWifiConnect(void);	
void ESP_CreateServer(void);
void ESP_SendData(char *data);
void ESP_TCP_CreateTransparentMode(char *ip, uint16_t server_port);
void ESP_UDP_CreateTransparentMode(char *ip, uint16_t server_port);
void ESP_TransparentSend(char *data);
void ESP_CloseTransparent(void);
	
	
	
	
#endif

