#ifndef CORE_MQTT_CONFIG_H
#define CORE_MQTT_CONFIG_H
/** ============================================================================
*   File:   core_mqtt_config.h
*   Author: Dilawar Ali
*   Dated:  04-02-2021
*
*   Description: This file MQTT config parameters
**  ============================================================================*/

/** ============================================================================
*   Include files
**  ============================================================================*/
 #include "System.h"
 #include "esp_log.h"
/** ============================================================================
*   Global constant declarations
**  ============================================================================*/
#define MQTT_STATE_ARRAY_MAX_COUNT 10u
#define MQTT_PINGRESP_TIMEOUT_MS   500u
#define MQTT_MAX_CONNACK_RECEIVE_RETRY_COUNT 5u

//#define LogError( message ) LogEntry("[MQTT][ERROR]", message)


#define LogError(message, ...) ESP_LOGE("[MQTT][ERROR]", message, ##__VA_ARGS__);

//#define LogDebug(message, ...) ESP_LOGI("[MQTT][Debug]", message, ##__VA_ARGS__);

#define LogWarn( message, ... )  ESP_LOGW("[MQTT][WARN]", message, ##__VA_ARGS__)
#define LogInfo( message, ... )  ESP_LOGI("[MQTT][Info]", message, ##__VA_ARGS__)


/** ===========================================================================
*   external variables
**  ============================================================================*/

/** ============================================================================
*   Global function Declarations
**  ============================================================================*/

#endif