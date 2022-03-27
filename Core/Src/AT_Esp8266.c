#include "AT_Esp8266.h"

static UART_HandleTypeDef MyUart;
uint8_t transMode = SINGLE_CON;
uint8_t TransparentProtocol = UDP_TRANS;
char *respond;
uint16_t lenBuffer;
uint8_t clientID = 0;

void ESP_SendCommand(char *cmd)
{
	int len = strlen(cmd);
	char str[100] = {0};
	sprintf(str,"%s\r\n",cmd);
	HAL_UART_Transmit(&MyUart,(uint8_t*)str, len + 2, 100);
}

void ESP_SetIP(uint8_t mode, char *ip)
{
	char cmd[50] = {0};
	if(mode == STA)
	{
		sprintf(cmd,"AT+CIPSTA=\"%s\"",ip);
	}
	if(mode == AP)
	{
		sprintf(cmd,"AT+CIPAP=\"%s\"",ip);
	}
	ESP_SendCommand(cmd);
	HAL_Delay(50);
}

void ESP_Init(UART_HandleTypeDef *huart, uint8_t mode, uint8_t *inBuff, uint16_t len)
{
	respond  = (char*)inBuff;
	lenBuffer = len;
	memcpy(&MyUart,huart,sizeof(*huart));
	ESP_CloseTransparent();  // close transparent if it is running 
	char cmd[50] = {0};
	sprintf(cmd,"AT+CWMODE=%d",mode);
	ESP_SendCommand(cmd);
	HAL_Delay(50);
	switch(mode)
	{
		case STA: ESP_SetIP(STA,IP);
							break;
		case AP:  ESP_SetIP(AP, IP);
							break;
		case STA_AP: ESP_SetIP(STA,IP);
								 break;
		default: break;
	}
}

void ESP_WifiConnect(char *ssid, char *pass)
{
	char cmd[100] = {0};
	sprintf(cmd,"AT+CWJAP=\"%s\",\"%s\"",ssid,pass);
	while(1)
	{
	  ESP_SendCommand(cmd);
		HAL_Delay(500);
		if(strstr(respond,"CONNECTED")) break;
	}
}

void ESP_SoftAPCreate(char *ssid, char *pass)
{
	char cmd[100] = {0};
	sprintf(cmd,"AT+CWSAP=\"%s\",\"%s\",1,2",ssid,pass);
  ESP_SendCommand(cmd);
	HAL_Delay(50);
}

HAL_StatusTypeDef ESP_CheckWifiConnect(void)
{
	ESP_SendCommand("AT+CWJAP?");
	HAL_Delay(1000);
	if(strstr(respond,"+CWJAP:")) return HAL_OK;
	else if(strstr(respond,"No AP"))
	{
		ESP_SendCommand("AT+CWJAP?");
		HAL_Delay(1000);
		if(strstr(respond,"No AP")) return HAL_ERROR;
	}
	return HAL_BUSY;
}

void ESP_TCP_CreateServer(void)
{
	ESP_SendCommand("AT+CIPMUX=1");
	transMode = MULTI_CON;
	HAL_Delay(50);
	char cmd[50] = {0};
	sprintf(cmd,"AT+CIPSERVER=1,%d",PORT);
	memset(respond,0,lenBuffer);
	ESP_SendCommand(cmd);
	HAL_Delay(50);
	while(!strstr(respond,"OK"))
	{
		ESP_SendCommand(cmd);
		HAL_Delay(50);
	}
}

void ESP_SendData(char *data)
{
	int len = strlen(data);
	char cmd[50] = {0};
	if(transMode == MULTI_CON) sprintf(cmd,"AT+CIPSEND=%d,%d",clientID,len);
	else sprintf(cmd,"AT+CIPSEND=%d",len);
	ESP_SendCommand(cmd);
	HAL_Delay(3);
	HAL_UART_Transmit(&MyUart,(uint8_t*)data, len, 100);
}

void ESP_CloseServer(void)
{
	ESP_SendCommand("AT+CIPSERVER=0");
}

void ESP_CloseConnect(void)
{
	if(transMode == SINGLE_CON) ESP_SendCommand("AT+CIPCLOSE");
	else
	{
		char cmd[50] = {0};
		sprintf(cmd,"AT+CIPCLOSE=%d",clientID);
		ESP_SendCommand(cmd);
	}
}

void ESP_UDP_CreateTransparentMode(char *ip, uint16_t server_port)
{
	memset(respond,0,lenBuffer);
	ESP_SendCommand("AT+CIPMUX=0");
	transMode = SINGLE_CON;
	HAL_Delay(50);
	ESP_SendCommand("AT+CIPMODE=1");
	HAL_Delay(50);
	char cmd[100] = {0};
	sprintf(cmd,"AT+CIPSTART=\"UDP\",\"%s\",%d,%d,0",ip,server_port,PORT);
	ESP_SendCommand(cmd);
	HAL_Delay(50);
	while(!strstr(respond,"CONNECT"))
	{
		ESP_SendCommand(cmd);
		HAL_Delay(50);
	}
	ESP_SendCommand("AT+CIPSEND");
	HAL_Delay(50);
	while(!strstr(respond,">"))
	{
		ESP_SendCommand("AT+CIPSEND");
		HAL_Delay(50);
	}
	TransparentProtocol = UDP_TRANS;
}

void ESP_TransparentSend(char *data)
{
	int len = strlen(data);
	HAL_UART_Transmit(&MyUart,(uint8_t*)data, len, 100);
}

void ESP_CloseTransparent(void)
{
	char str[3] = "+++";
	for(int i = 0; i<3; i++)
	{
		HAL_UART_Transmit(&MyUart,(uint8_t*)str, 3, 100);
		HAL_Delay(50);
		ESP_CloseConnect();
		HAL_Delay(50);
	}
}

void ESP_TCP_CreateTransparentMode(char *ip, uint16_t server_port)
{
	memset(respond,0,lenBuffer);
	ESP_SendCommand("AT+CIPMUX=0");
	transMode = SINGLE_CON;
	HAL_Delay(50);
	ESP_SendCommand("AT+CIPMODE=1");
	HAL_Delay(50);
	char cmd[100] = {0};
	sprintf(cmd,"AT+CIPSTART=\"TCP\",\"%s\",%d",ip,server_port);
	ESP_SendCommand(cmd);
	HAL_Delay(50);
	while(!strstr(respond,"CONNECT"))
	{
		ESP_SendCommand(cmd);
		HAL_Delay(50);
	}
	ESP_SendCommand("AT+CIPSEND");
	HAL_Delay(50);
	while(!strstr(respond,">"))
	{
		ESP_SendCommand("AT+CIPSEND");
		HAL_Delay(50);
	}
	TransparentProtocol = TCP_TRANS;
}


