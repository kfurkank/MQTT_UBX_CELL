
/** ============================================================================
*   File:   transport_interface.c
*   Dated:  03-02-2021
*
*   Description: This file include Definition of all components and task
**  ============================================================================*/

/** ============================================================================
*   Include files
**  ============================================================================*/
#include "transport_interface.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
/** ============================================================================
*   Local function Declaration
**  ============================================================================*/
void ConvertDataBinaryToHex(uint8_t srcBuffer[], uint8_t destBuffer[], size_t size);

/** ============================================================================
*   Local constant declarations
**  ============================================================================*/
#define MQTT_TRANSMIT_BUFFER_SIZE 2048

/** ============================================================================
*   Local Variable definition
**  ============================================================================*/

/** ============================================================================
*   Global Variable definition
**  ============================================================================*/

/** ============================================================================
*   Local function Implimentation
**  ============================================================================*/
/** ============================================================================
*       void ConvertDataBinaryToHex(uint8_t *pBuffer, size_t size)
*
*       Author: Dilawar Ali       
*       Dated:  04-02-2021
*
*       Description: This function convert MQTT binary data to hexadecimal format
*
** ============================================================================= */
void ConvertDataBinaryToHex(uint8_t srcBuffer[], uint8_t destBuffer[], size_t size)
{
    uint32_t index = 0;

    for (int i = 0; i < size; i++)
    {
        sprintf((char *)&destBuffer[index], "%02x", srcBuffer[i]);
        index += 2;
    }
}


/** ============================================================================
*       void ConvertDataBinaryToHex(uint8_t *pBuffer, size_t size)
*
*       Author: Dilawar Ali       
*       Dated:  04-02-2021
*
*       Description: This function convert MQTT binary data to hexadecimal format
*
** ============================================================================= */
void ConvertDataHexToBinary(uint8_t srcBuffer[], uint8_t destBuffer[], size_t size)
{
    uint8_t hex[3];
    uint32_t index = 0;

    for (int i = 0; i < size; i += 2)
    {
        sprintf((char *)hex, "%c%c", srcBuffer[i], srcBuffer[i + 1]);
        sscanf((const char *)hex, "%hhx", (uint8_t *)&destBuffer[index]);
        index++;
    }
}

/** ============================================================================
*   Global function Implimentation
**  ============================================================================*/
/** ============================================================================
*       int32_t MqttDataReceiveFromCell(NetworkContext_t *pNetworkContext, void *pBuffer, size_t bytesToRecv)
*
*       Author: Dilawar Ali       
*       Dated:  04-02-2021
*
*       Description: This function reads data from TCP socket of MQTT server
*
** ============================================================================= */
int32_t MqttDataReceiveFromCell(NetworkContext_t *pNetworkContext, void *pBuffer, size_t bytesToRecv)
{
    int32_t ret = -1;
    uint8_t mqttReceivebuffer[MQTT_TRANSMIT_BUFFER_SIZE];
    uint8_t retryCount = 5;

    while (retryCount > 1)
    {
       ret = ReadDataFromTcpSocket(mqttReceivebuffer, bytesToRecv);
       ESP_LOGI("[Transport]", "Rx Count: %d", ret);

       if((uint32_t)ret == (uint32_t)bytesToRecv)
       {
           break;
       }
       vTaskDelay(300 / portTICK_RATE_MS);

       retryCount--;
    }
    
    ConvertDataHexToBinary(mqttReceivebuffer, (uint8_t *)pBuffer, (bytesToRecv * 2));


    return ret;
}

/** ============================================================================
*       int32_t MqttDataSendFromCell(NetworkContext_t *pNetworkContext, void *pBuffer, bytesToSend)
)
*
*       Author: Dilawar Ali       
*       Dated:  04-02-2021
*
*       Description: This function sends data from TCP socket of MQTT server
*
** ============================================================================= */
int32_t MqttDataSendFromCell(NetworkContext_t *pNetworkContext, const void *pBuffer, size_t bytesToSend)
{
#define TCP_MAX_SIZE 90
    int32_t ret = -1;
    uint8_t mqttTransmitbuffer[MQTT_TRANSMIT_BUFFER_SIZE];
    uint8_t TmpCopyBuff[256];

    uint8_t *inputData = (uint8_t *)pBuffer;
    int32_t RemainingTcpBytes = bytesToSend;
    uint32_t CurrentIndex = 0;
    uint32_t CopyBytes = 0;

    while(RemainingTcpBytes > 0)
    {
    	(void)memset((void *)mqttTransmitbuffer, 0x00, sizeof(mqttTransmitbuffer));
    	(void)memset((void *)TmpCopyBuff, 0x00, sizeof(TmpCopyBuff));

    	if(RemainingTcpBytes > TCP_MAX_SIZE)
    	{
    		CopyBytes = TCP_MAX_SIZE;
    	}
    	else
    	{
    		CopyBytes = RemainingTcpBytes;
    	}

    	memcpy(TmpCopyBuff, &inputData[CurrentIndex], CopyBytes);

    	ConvertDataBinaryToHex(TmpCopyBuff, mqttTransmitbuffer, CopyBytes);
    	ret = WriteDataToTcpSocket(mqttTransmitbuffer, CopyBytes);

    	RemainingTcpBytes =- CopyBytes;
    	CurrentIndex =+ CopyBytes;
    }

    return ret;
}
