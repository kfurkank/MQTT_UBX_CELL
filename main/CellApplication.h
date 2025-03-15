#ifndef CellApplication_H
#define CellApplication_H
/** ============================================================================
*   File:   CellApplication.h
*   Author: Dilawar Ali
*   Dated:  02-02-2021
*
*   Description: This file control all the Cellular related tasks
**  ============================================================================*/

/** ============================================================================
*   Include files
**  ============================================================================*/
#include "SysDef.h"

/** ============================================================================
*   Global constant declarations
**  ============================================================================*/
#define MQTT_CLIENT_ID                  "MQTT_ESP32_SARA-G3_06022021"
#define MQTT_TOPIC_SiZE                 128
#define MQTT_SERVER_PORT                8883
#define MQTT_BUFFER_SIZE                1024
#define MQTT_EXAMPLE_TOPIC              "/ESP32/SARAG312345"
//#define MQTT_SERVER_ADDRESS             "broker.hivemq.com"
#define MQTT_SERVER_ADDRESS             "tppm.iot.turkcell.com.tr"
#define MQTT_USER_NAME                  "tppm/device_tpTestDevice1"
#define MQTT_PASSWD						"PvOx47qW)y"
#define MQTT_EXAMPLE_PAYLOAD            "Hello World MQTT from ESP32"
#define MQTT_BROKER_ADDRESS_SiZE        128
#define MQTT_PUBLISH_PAYLOAD_SIZE       1024
#define MQTT_CLIENT_IDENTIFIER_SIZE     64
#define MQTT_CONNACK_RECV_TIMEOUT_MS    2000
#define MQTT_KEEP_ALIVE_TIMEOUT_SECONDS 60


#define CELL_AT_RESPONSE_BUFF_SIZE 1024
typedef enum
{
  CELL_STATE_INIT = 0,
  CELL_STATE_NETWORK_REGISTERATION,
  CELL_STATE_PSD_ACTIVATION,
  CELL_STATE_TCP_SOCK_CONNECT,
  CELL_STATE_TCP_SOCK_DISCONNECT,
  CELL_STATE_MQTT_INIT,
  CELL_STATE_MQTT_CONNECT,
  CELL_STATE_MQTT_DATA_PUBLISH,
  CELL_STATE_MQTT_TOPIC_SUBCRIBE,
  CELL_STATE_MQTT_RECEIVE_DATA,
  CELL_STATE_MQTT_IDLE,
  CELL_STATE_MQTT_DISCONNECT,

} CELLULAR_STATE_ENUM;

typedef struct CellData
{
  CELLULAR_STATE_ENUM currentCellState;
  uint8_t atResponseBuffer[CELL_AT_RESPONSE_BUFF_SIZE];
}CellData_t;

typedef enum
{
	CA_CERTIFICATE = 0,
	DEVICE_CERTIFICATE = 1,
	DEVICE_PRIVATE_KEY = 2,

}CertificateType;

/** ============================================================================
*   external variables
**  ============================================================================*/

/** ============================================================================
*   Global function Declarations
**  ============================================================================*/
void CellTask(void *arg);

/** ============================================================================
*       int32_t SendATCAndWaitForOK(uint8_t ATCommand[], uint8_t AtResponse[], uint8_t timeout)
*
*       Author: Dilawar Ali       
*       Dated:  19-05-2019
*
*       Description: This function send the AT command to module and wait for the OK response from module
*
** ============================================================================= */
int32_t SendATCAndWaitForOK(uint8_t AtCommand[], uint8_t AtResponse[], uint8_t timeout);

/** ============================================================================
*       int32_t WriteDataToTcpSocket(uint8_t data[], uint32_t size)
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function writes data to TCP socket of MQTT server
*
** ============================================================================= */
int32_t WriteDataToTcpSocket(uint8_t data[], uint32_t size);

/** ============================================================================
*       int32_t ReadDataFromTcpSocket(uint8_t data[], uint32_t size)
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function reads data from TCP socket of MQTT server
*
** ============================================================================= */
int32_t ReadDataFromTcpSocket(uint8_t data[], uint32_t size);


/** ============================================================================
*       int32_t ReadDataFromTcpSocket(uint8_t data[], uint32_t size)
*
*       Author: Dilawar Ali       
*       Dated:  06-02-2021
*
*       Description: This function Increment the pending publish message count
*
** ============================================================================= */
void SignalPublishMessage();

/** ============================================================================
*       void AddQueueElementForMQTTClient()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function add queue element to be posted on serve
*
** ============================================================================= */
void AddQueueElementForMQTTClient(char* TopicName, char* Payload, int length, char qos, bool retain);
void AddExtQueueElementForMQTTClient(char* TopicName, char* Payload, int length, char qos, bool retain);

/** ============================================================================
*       void UpdateMQTTBrookerUrl()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function updates server URL
*
** ============================================================================= */
void UpdateMQTTBrookerUrl(char* UpdatedURL, int32_t URLLength);

/** ============================================================================
*       void UpdateMQTTUserID()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function updates user ID on server
*
** ============================================================================= */
void UpdateMQTTUserID(char* UpdatedID, int32_t IDLength);

#endif
