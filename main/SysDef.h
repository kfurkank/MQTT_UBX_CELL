#ifndef __SYS_DEF_H
#define __SYS_DEF_H

/** ============================================================================
*   Include files
**  ============================================================================*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <time.h>
#include <math.h>
#include "cJSON.h"
#include "nvs_flash.h"

#include <sys/types.h>
#include <sys/param.h>
#include <sys/time.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/rtc.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_sntp.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_https_ota.h"

#include "mqtt_client.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "sdkconfig.h"


#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

/** ============================================================================
*   Global constant declarations
**  ============================================================================*/
#define MQTT_TOPIC_NAME_MAX_LENGTH        64
#define MQTT_MAX_PAYLOAD_LENGTH           128
#define MQTT_MAX_EXT_PAYLOAD_LENGTH		  15000

typedef enum
{
    MSG_SENDER_SYS_TASK=0,
    MSG_SENDER_TIMER_TASK,
    MSG_SENDER_CELLULAR_TASK,
    MQTT_RECV_MSG_HANDLER_CELLULAR_TASK,
    MSG_SENDER_IRQ,
}MSG_SENDR_t;

typedef struct
{
    MSG_SENDR_t sender;
}MsgSenderInfo_t;

typedef enum
{
  
  EMPTY_EVENT_REQ = 0,
  INITIALIZATION_SEQUENCE,
  SEND_DEVICE_INFO_TO_CLOUD,

  
  //Sensor Functionality
  READ_SENSOR_VALUES,
  RECONFIGURE_SENSOR_PARAMETERS,
  SHARE_SENSOR_SETTING_OVER_CLOUD,
  PROCESS_EXTREME_DATA_SENSOR_REQUEST,



  //Wi-Fi Functionality
  INITIALIZE_WIFI_AP_MODE,
  INITIALIZE_WIFI_ST_MODE,

  POST_TEST_DATA_TO_MQTT,

}SYS_MSG_ID_t;

typedef struct
{
	MSG_SENDR_t msgSender;
    SYS_MSG_ID_t msgId;
    uint32_t msgInfo;
    void * ptrData;
}SysMsg_t;


typedef struct
{
    uint8_t qos;  //Quality of Service for message.
    bool retain;  //Whether this is a retained message.
    char pTopicName[MQTT_TOPIC_NAME_MAX_LENGTH]; //Topic name on which the message is published.
    uint16_t topicNameLength; //Length of topic name
    uint8_t pPayload[MQTT_MAX_PAYLOAD_LENGTH];  //Message payload.
    uint16_t payloadLength; //Message payload length.
}MqttQueueMsg_t;

typedef struct
{
    uint8_t qos;  //Quality of Service for message.
    bool retain;  //Whether this is a retained message.
    char pTopicName[MQTT_TOPIC_NAME_MAX_LENGTH]; //Topic name on which the message is published.
    uint16_t topicNameLength; //Length of topic name
    uint8_t pPayload[MQTT_MAX_EXT_PAYLOAD_LENGTH];  //Message payload.
    uint16_t payloadLength; //Message payload length.
}MqttQueueExtMsg_t;


typedef enum
{
  
  INIT_NVS_MODULE = 0,
  INIT_GET_MAC_ADDRESS,
  INIT_GET_FW_VER,
  INIT_SET_WIFI_TP_AP_THEN_STATION,
  INIT_SYNC_TIME_FROM_CELLULAR_DEVICE,
  INIT_READ_MQTT_PARAMS_FROM_NVS,
  INIT_UART_DRIVERS_FOR_CELLULAR,

  INIT_READ_SENSORS_RANGE_FROM_NVS,
  INIT_SPI_DRIVERS_FOR_SENSORS,
  INIT_READ_SENSORS_PROPERTIES,
  INIT_READ_SENSOR_STATUS,
  INIT_SEND_DEVICE_INFO_TO_CLOUD,


  INIT_SEQUENCE_COMPLETE,
  
}INIT_MSG_ID_t;

/** ============================================================================
*   external variables
**  ============================================================================*/
extern QueueHandle_t xSysMailbox;
extern QueueHandle_t xMqttMessageQueue;
extern QueueHandle_t xMqttExtremeMessageQueue;
#endif /* __SYSDEF_H */
