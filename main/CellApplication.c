/** ============================================================================
*   File:   CellApplication.c
*   Author: Dilawar Ali
*   Dated:  02-22-2021
*
*   Description: This file include Definition of all components and task
**  ============================================================================*/

/** ============================================================================
*   Include files
**  ============================================================================*/

#include "CellApplication.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "System.h"
#include "core_mqtt.h"
#include "adxl357.h"

/** ============================================================================
*   Local function Declaration
**  ============================================================================*/
int32_t CellSetHWFCDisable();
int32_t CellCheckModelIMEI();
int32_t RegisterToNetwork();
int32_t ActivatePSD();
int32_t PingGoogleDNS();
int32_t OpenTcpSocket();
int32_t CloseTcpSocket();
void MqttEventCallback ( struct MQTTContext * pContext, struct MQTTPacketInfo * pPacketInfo, struct MQTTDeserializedInfo * pDeserializedInfo );

int32_t MqttInit();
int32_t MqttConnect();
int32_t MqttPublishToTopic( );
int32_t MqttSubscribe(uint8_t *subscribeTopic);
static void prvMQTTProcessResponse( MQTTPacketInfo_t * pxIncomingPacket, uint16_t usPacketId );
static void prvMQTTProcessIncomingPublish( MQTTPublishInfo_t * pxPublishInfo );
int32_t PerformStateAction(CELLULAR_STATE_ENUM state);
int32_t MqttReceiveSubscribedData();
int32_t WriteCertificateToModule(uint8_t *certificate, uint8_t *certificateName, CertificateType crtType);
int32_t ConfigureAllCertificate();

/** ============================================================================
*   Local constant declarations
**  ============================================================================*/
#define ERR_CELLULAR_AT_RESPONSE_RECEIVE_TIMEOUT -2
#define ERR_CELLULAR_UART_READ_FAILED            -3
#define ERR_CELLULAR_UART_WRITE_FAILED           -4
#define ERR_CELLULAR_AT_COMMAND_INVALID          -5
#define ERR_CELLULAR_NETWORK_REGISTRATION_FAILED -6
#define ERR_CELLULAR_PSD_ACTIVATION_FAILED       -7
#define ERR_CELLULAR_TCP_SOCK_FAILED             -8
#define ERR_CELLULAR_TCP_SOCK_WRITE_DATA_SIZE_MISMATCH -9
#define ERR_CELLULAR_TCP_SOCK_DATA_WRITE_FAILED  -10
#define ERR_CELLULAR_TCP_SOCK_DATA_Read_FAILED   -11

typedef struct MQTTData
{
  uint8_t mqttbuf[MQTT_BUFFER_SIZE];
  uint8_t mqttPublishTopic[MQTT_TOPIC_SiZE];
  uint8_t mqttsubscribeTopic[MQTT_TOPIC_SiZE];
  uint8_t mqqttBrokerAddress[MQTT_BROKER_ADDRESS_SiZE];
  uint8_t mqttPublishPayload[MQTT_PUBLISH_PAYLOAD_SIZE];
  uint8_t mqttclientIdentifier[MQTT_CLIENT_IDENTIFIER_SIZE];
  uint16_t mqttSubscribePacketIdentifier;
  uint32_t mqttBrokerPort;
  uint32_t mqttKeepAliveTimeoutSeconds;
  uint32_t mqttConnakRecvTimeoutMs;
  uint32_t mqttpendingPublishMessage;
  NetworkContext_t mqttnetworkContext;
  MQTTContext_t mqttContext;
  MQTTConnectInfo_t mqttConnectInfo;
  TransportInterface_t mqttTransport;
  bool mqttSessionPresent;

}MQTTData_t;


CellData_t cellularData = 
{
  .currentCellState = CELL_STATE_INIT, 
};
CellData_t *cellUserData = &cellularData;

MQTTData_t mqttUserData = 
{
  .mqttPublishTopic = MQTT_EXAMPLE_TOPIC,
  .mqttsubscribeTopic = MQTT_EXAMPLE_TOPIC,
  .mqqttBrokerAddress = MQTT_SERVER_ADDRESS,
  .mqttPublishPayload = MQTT_EXAMPLE_PAYLOAD,
  .mqttclientIdentifier = MQTT_CLIENT_ID,
  .mqttBrokerPort = MQTT_SERVER_PORT,
  .mqttKeepAliveTimeoutSeconds = MQTT_KEEP_ALIVE_TIMEOUT_SECONDS,
  .mqttConnakRecvTimeoutMs = MQTT_CONNACK_RECV_TIMEOUT_MS,
  .mqttConnectInfo.pUserName = MQTT_USER_NAME,
  .mqttConnectInfo.userNameLength = strlen(MQTT_USER_NAME),
  .mqttConnectInfo.pPassword = MQTT_PASSWD,
  .mqttConnectInfo.passwordLength = strlen(MQTT_PASSWD),

};

const MQTTFixedBuffer_t mqttBuffer = {
    .pBuffer = mqttUserData.mqttbuf,
    .size = MQTT_BUFFER_SIZE,
};

/** ============================================================================
*   Local Variable definition
**  ============================================================================*/
MqttQueueExtMsg_t ExtMsgToQueue;
MqttQueueExtMsg_t ExtMsgFromQueue;

uint8_t *deviceCrt = (uint8_t*)"-----BEGIN CERTIFICATE-----\
MIIF5TCCA82gAwIBAgICEAAwDQYJKoZIhvcNAQELBQAwgYwxCzAJBgNVBAYTAnRy\
MQswCQYDVQQIDAJ0cjELMAkGA1UEBwwCdHIxEzARBgNVBAoMCnRyZW5kb3BlYWsx\
EzARBgNVBAsMCnRyZW5kb3BlYWsxEzARBgNVBAMMCnRyZW5kb3BlYWsxJDAiBgkq\
hkiG9w0BCQEWFW1laG1ldEB0cmVuZG9wZWFrLmNvbTAeFw0yMTA0MDIyMTAwMTJa\
Fw0yMjA0MDIyMTAwMTJaMH8xCzAJBgNVBAYTAnRyMQswCQYDVQQIDAJ0cjETMBEG\
A1UECgwKdHJlbmRvcGVhazETMBEGA1UECwwKdHJlbmRvcGVhazETMBEGA1UEAwwK\
dHJlbmRvcGVhazEkMCIGCSqGSIb3DQEJARYVbWVobWV0QHRyZW5kb3BlYWsuY29t\
MIICIjANBgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEAqRPlfIAJqzVM+qaPomRL\
edYBGWfwSsjePT4hReY80KkkcwUZLWQhVfFaGi8coDwcWyAZtyIfa5VwwNQ1J60Y\
Ihk6dLJAlwVHudY5462STO2S285hv5YckeatFaaEzyRDif6TTGq+hPQorOlWoicK\
CWju+PZRXOqBJzoxVCXREgGxK6mO0LsUFgbrpTaW53TQu2jmtakUB8fgUPfWu3od\
Eyv5dpRb9I6tjwXY/u5VZCVN2AMnVZ9Hy7DxjRUSfaT0LusTCLuhRgHbi+zTHz+/\
AEmSt8nMmnZAfoXrNlWN9nSk3Cv/K19MuFj1tdKINK7MFvCsSWGlRMZJj6oUuL3F\
SXyFdyrKfcUMwXHBw7o3P3Q0CK+EEQSThHsv1lTgZ20oz+KomQkYz2daGQ7sfJhZ\
GPXvRv6ZYtzI8zyESVXFk7bPag5guGCI5Nom6lzbEcAwdbSg1rBmKj2LOVUnw+8B\
nQmfPoth1UnDI33pm5SmSIhOXtSiV0blfMvWt39gu8EPRcw75VqtdlV33rpxTG/x\
/xB9gbMQ3Ub06N2apSXxudDaV5mcVOJO49kYoL1bBIv33Eyxckc+I8euGq47VEh4\
WjLYR4PBERV3I6E62ybMgEhURoSbJM8+9lpa6SgybakZRkapUqmlEyBkh2jkLi9I\
RxFgg/njWJwg+L8PndcuAmMCAwEAAaNdMFswHQYDVR0OBBYEFESEieMzAZNlxSNW\
kH2evrEpSznGMB8GA1UdIwQYMBaAFC6m1r/tuX7BT9N8wr6XmPIDkqaCMAwGA1Ud\
EwEB/wQCMAAwCwYDVR0PBAQDAgXgMA0GCSqGSIb3DQEBCwUAA4ICAQCC4/Pw9nwa\
WvPr/zdyTDqaPQA7m8NLhU6Cw40Uz9WE/hNwsrO1Bb708XDOwDfn+UFNCCcQ2As+\
yxSsLQ7bbAzBtVNb47PWAd6WDDfplzluzVQN/uopG8ZOpTzIFLSHUG2jqJV+b0Wd\
addkZxjm3VwcxHJnz8lAooUo2LdYZcwENZOP4uabXYgFsGlNk4JbAL2tWLfNwv9C\
EFvqzeCB/LMOujv71NpVVX/m/qU51DlydEF7atnpjr2wJja0bIeADqrmM55WrO2l\
TTgD2avVef9fvD6Y1vlnfwgM/bAH+9RY6KZnvDfqhUW/41G5m0BEK1D8dipWCybB\
v1MDCDPIB9pPYYy064unSMp+rwDNZbEc5YDwwDvmt1viuxDIA5KsZTV8uKXNC7t/\
bzO8z85R3txatJCHB1mKBDVpCajx1OQyPtZszI6tSAOhr+LjL5vsKk5tSUhvPF5o\
qlYbylSyrxwtX10RpygZgXITSCBj4fPb/cH7wBJxHQu3zcNUP2nYs2//PS3SaR0m\
EBpf7U9zw/2SfbDQtyGvMN1i4VIXMWDfiWTBOsKVIEj2QvXbADIvPv3uvpbbXavX\
3XEgGKMz/eTD7dIq7CVrcgWAPkWhDfK4DLKk1yg3VMZGNqG3kjUobv1Oh0HvZZb5\
RkLR+UhHgOlyR8s1qOt4aadKNpfvI3gprQ==\
-----END CERTIFICATE-----\
";

uint8_t *deviceKey = (uint8_t*)"-----BEGIN RSA PRIVATE KEY-----\
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\
-----END RSA PRIVATE KEY-----\
";

uint8_t *CAcrt = (uint8_t*)"-----BEGIN CERTIFICATE-----\
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\
wiedbZnsbcRjA7pYhVIo\
-----END CERTIFICATE-----\
";


/** ============================================================================
*   Global Variable definition
**  ============================================================================*/


/** ============================================================================
*   Local function Implimentation
**  ============================================================================*/

int32_t PerformStateAction(CELLULAR_STATE_ENUM state)
{
  int32_t ret =-1;

   ESP_LOGI("[Cell][State]", "State execution: %d", state);

  switch (state)
  {
  case CELL_STATE_INIT:
  ret = CellCheckModelIMEI();
  break;

  case CELL_STATE_NETWORK_REGISTERATION:
  ret = RegisterToNetwork();
  break;

  case CELL_STATE_PSD_ACTIVATION:
  ret = ActivatePSD();
  break;

  case CELL_STATE_TCP_SOCK_CONNECT:
  ret = OpenTcpSocket();
  break;

  case CELL_STATE_TCP_SOCK_DISCONNECT:
  ret = CloseTcpSocket();
  break;

  case CELL_STATE_MQTT_INIT:
  ret = MqttInit();
  break;

  case CELL_STATE_MQTT_CONNECT:
  ret = MqttConnect();
  break;

  case CELL_STATE_MQTT_DATA_PUBLISH:
  ret = MqttPublishToTopic();
  break;
  
  case CELL_STATE_MQTT_TOPIC_SUBCRIBE:
  //ret = MqttSubscribe(mqttUserData.mqttsubscribeTopic);
  ret = MqttSubscribe((uint8_t *)TOPICSUB_ATT);
  ret = MqttSubscribe((uint8_t *)TOPICSUB_REQ);
  break;

  case CELL_STATE_MQTT_RECEIVE_DATA:
  ret = MqttReceiveSubscribedData();
  break;

  case CELL_STATE_MQTT_DISCONNECT:
  case CELL_STATE_MQTT_IDLE:


  default:
    break;
  }

  ESP_LOGI("[Cell][State]", "State return: %d", ret);

  return ret;
}


/** ============================================================================
*       int32_t CellSetHWFCDisable()
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function sets the HW FW of cell module disable
*
** ============================================================================= */
int32_t CellSetHWFCDisable(CellData_t *cellData)
{
    int32_t ret = -1;

    ret = SendATCAndWaitForOK((uint8_t *)"AT&k0\r\n", cellUserData->atResponseBuffer, 10);
    ret = SendATCAndWaitForOK((uint8_t *)"AT&w0\r\n", cellUserData->atResponseBuffer, 10);
    ret = SendATCAndWaitForOK((uint8_t *)"AT&y0\r\n", cellUserData->atResponseBuffer, 10);

    return ret;
}

/** ============================================================================
*       int32_t CellCheckModelIMEI()
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function Check Cell module ID and IMEI
*
** ============================================================================= */
int32_t CellCheckModelIMEI()
{
    int32_t ret = -1;

    ret = SendATCAndWaitForOK((uint8_t *)"AT&k0\r\n", cellUserData->atResponseBuffer, 5);
    ESP_LOGI("Cell", "ret: %d", ret);
    vTaskDelay(3000 / portTICK_RATE_MS);
    ret = SendATCAndWaitForOK((uint8_t *)"ATE0\r\n", cellUserData->atResponseBuffer, 5);
    ret = SendATCAndWaitForOK((uint8_t *)"AT+CMEE=2\r\n", cellUserData->atResponseBuffer, 5);
    ESP_LOGI("Cell", "ret: %d", ret);
    ret = SendATCAndWaitForOK((uint8_t *)"AT+CGMM\r\n", cellUserData->atResponseBuffer, 10);
    ret = SendATCAndWaitForOK((uint8_t *)"AT+CGSN\r\n", cellUserData->atResponseBuffer, 10);

    return ret;
}

/** ============================================================================
*       int32_t RegisterToNetwork()
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This functions verify the cell module registration to network
*
** ============================================================================= */
int32_t RegisterToNetwork()
{
    int32_t ret = -1;
    int32_t copsVal = -1;
    int32_t regn = -1;
    int32_t regVal = -1;
    int64_t refrenceTime = GetTime();

    ret = SendATCAndWaitForOK((uint8_t *)"AT+CPIN?\r\n", cellUserData->atResponseBuffer, 10);
    vTaskDelay(3000 / portTICK_RATE_MS);
    
    ret = SendATCAndWaitForOK((uint8_t *)"AT+COPS?\r\n", cellUserData->atResponseBuffer, 10);
    if (ret > 0)
    {
      if (sscanf((const char *)cellUserData->atResponseBuffer, "\r\n+COPS: %i", &copsVal) > 0)
      {
        if(copsVal != 0)
        {
          ret = SendATCAndWaitForOK((uint8_t *)"AT+COPS=0\r\n", cellUserData->atResponseBuffer, 10);
        }
      }
    }

    do
    {
      ret = SendATCAndWaitForOK((uint8_t *)"AT+CREG?\r\n", cellUserData->atResponseBuffer, 10);
      if (ret > 0)
      {
        sscanf((const char *)cellUserData->atResponseBuffer, "\r\n+CREG: %d,%d", &regn, &regVal);
        if (regVal == 1)
        {
          break;
        }
        else
        {
          ret = ERR_CELLULAR_NETWORK_REGISTRATION_FAILED;
        }
      }

      ESP_LOGI("[Cell]", "Reg Status: %d\n", regVal);
    } while ((GetTime() - refrenceTime) < 10);

    SendATCAndWaitForOK((uint8_t *)"AT+COPS?\r\n", cellUserData->atResponseBuffer, 10);

    if (ret < 0)
    {
      ret = ERR_CELLULAR_NETWORK_REGISTRATION_FAILED;
    }
    return ret;
}

/** ============================================================================
*       int32_t ActivatePSD()
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function internet capabilities to the cell module
*
** ============================================================================= */
int32_t ActivatePSD()
{
  int32_t ret = -1;
  uint32_t psdStatus = 0;

  ret = SendATCAndWaitForOK((uint8_t *)"AT+UPSND=0,8\r\n", cellUserData->atResponseBuffer, 10);
  sscanf((const char *)cellUserData->atResponseBuffer, "\r\n+UPSND: 0,8,%d", &psdStatus);
  
  ESP_LOGI("[Cell]", "PSD Status: %d\n", psdStatus);

  if(psdStatus != 1)
  {
    ret = SendATCAndWaitForOK((uint8_t *)"AT+UPSD=0,0,0\r\n", cellUserData->atResponseBuffer, 10);
    if (ret > 0)
    {
      ret = SendATCAndWaitForOK((uint8_t *)"AT+UPSD=0,1,\"internet\"\r\n", cellUserData->atResponseBuffer, 10);
      if (ret > 0)
      {
        ret = SendATCAndWaitForOK((uint8_t *)"AT+UPSDA=0,3\r\n", cellUserData->atResponseBuffer, 60);
      }
    }
  }

  SendATCAndWaitForOK((uint8_t *)"AT+CGDCONT?\r\n", cellUserData->atResponseBuffer, 10);

  if (ret < 0)
  {
    ret = ERR_CELLULAR_PSD_ACTIVATION_FAILED;
  }

  return ret;
}

/** ============================================================================
*       int32_t PingGoogleDNS()
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function sends ping request to google DNS server
*
** ============================================================================= */
int32_t PingGoogleDNS()
{
    int32_t ret = -1;

    ret = SendATCAndWaitForOK((uint8_t *)"AT+UPING=\"8.8.8.8\"\r\n", cellUserData->atResponseBuffer, 60);

    return ret;
}


/** ============================================================================
*       int32_t OpenTcpSocket()
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function open TCP socket to MQTT server
*
** ============================================================================= */
int32_t OpenTcpSocket()
{
  int32_t ret = -1;
  uint8_t command[1024] = {0};

  ConfigureAllCertificate();

  ret = SendATCAndWaitForOK((uint8_t *)"AT+USOCR=6\r\n", cellUserData->atResponseBuffer, 10);
  if (ret > 0)
  {
	ret = SendATCAndWaitForOK((uint8_t *)"AT+USOSEC=0,1,0\r\n", cellUserData->atResponseBuffer, 10);

    sprintf((char *)&command, "AT+USOCO=0,\"%s\",%d\r\n", mqttUserData.mqqttBrokerAddress, mqttUserData.mqttBrokerPort);
    ret = SendATCAndWaitForOK((uint8_t *)command, cellUserData->atResponseBuffer, 30);
    if (ret > 0)
    {
      ret = SendATCAndWaitForOK((uint8_t *)"AT+UDCONF=1,1\r\n", cellUserData->atResponseBuffer, 10);
    }
  }

  if (ret < 0)
  {
    ret = ERR_CELLULAR_TCP_SOCK_FAILED;
  }
  return ret;
}

/** ============================================================================
*       int32_t CloseTcpSocket()
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function close TCP socket of MQTT server
*
** ============================================================================= */
int32_t CloseTcpSocket()
{
  
  SendATCAndWaitForOK((uint8_t *)"AT+USOCL=0\r\n", cellUserData->atResponseBuffer, 10);

  return 0;
}

void MqttEventCallback(struct MQTTContext *pContext, struct MQTTPacketInfo *pPacketInfo, struct MQTTDeserializedInfo *pDeserializedInfo)
{
  ESP_LOGI("[MQTT]", "App callback function got hit");
  /* The MQTT context is not used for this demo. */
  // (void)pContext;

  if ((pPacketInfo->type & 0xF0U) == MQTT_PACKET_TYPE_PUBLISH)
  {
    prvMQTTProcessIncomingPublish(pDeserializedInfo->pPublishInfo);
  }
  else
  {
    prvMQTTProcessResponse(pPacketInfo, pDeserializedInfo->packetIdentifier);
  }
}


/** ============================================================================
*       int32_t MqttInit()
*
*       Author: Dilawar Ali       
*       Dated:  05-02-2021
*
*       Description: This function perform Mqttoperations on cellular connection
*
** ============================================================================= */
int32_t MqttInit()
{
  int32_t ret = -1;
  MQTTStatus_t xResult;
  static uint8_t isMqttInit = 0;

  if (isMqttInit == 0)
  {
    mqttUserData.mqttTransport.pNetworkContext = &mqttUserData.mqttnetworkContext;
    mqttUserData.mqttTransport.send = MqttDataSendFromCell;
    mqttUserData.mqttTransport.recv = MqttDataReceiveFromCell;

    /* Initialize MQTT library. */
    xResult = MQTT_Init(&mqttUserData.mqttContext, &mqttUserData.mqttTransport, GetTimeMS, MqttEventCallback, &mqttBuffer);
    configASSERT(xResult == MQTTSuccess);
    if (xResult == MQTTSuccess)
    {
      isMqttInit = 1;
      ret = 0;
    }
  }
  else
  {
    ret = 0;
  }

  return ret;
}

/** ============================================================================
*       int32_t MqttConnect()
*
*       Author: Dilawar Ali       
*       Dated:  05-02-2021
*
*       Description: This function perform Mqttoperations on cellular connection
*
** ============================================================================= */
int32_t MqttConnect()
{
  int32_t ret = -1;
  MQTTStatus_t xResult;
  
  //(void)memset((void *)&mqttUserData.mqttConnectInfo, 0x00, sizeof(mqttUserData.mqttConnectInfo));

  mqttUserData.mqttConnectInfo.cleanSession = true;

  mqttUserData.mqttConnectInfo.pClientIdentifier = (const char *)mqttUserData.mqttclientIdentifier;
  mqttUserData.mqttConnectInfo.clientIdentifierLength = ( uint16_t ) strlen((const char *) mqttUserData.mqttclientIdentifier );

  mqttUserData.mqttConnectInfo.keepAliveSeconds = mqttUserData.mqttKeepAliveTimeoutSeconds;

  xResult = MQTT_Connect(&mqttUserData.mqttContext,
                         &mqttUserData.mqttConnectInfo,
                         NULL,
                         mqttUserData.mqttConnakRecvTimeoutMs,
                         &mqttUserData.mqttSessionPresent);
  //configASSERT(xResult == MQTTSuccess);

  if(xResult == MQTTSuccess)
  {
    ret = 0;
  }

  return ret;
}

/** ============================================================================
*       int32_t MqttReceiveSubscribedData()
*
*       Author: Dilawar Ali       
*       Dated:  05-02-2021
*
*       Description: This function Get subsribed messages
*
** ============================================================================= */
int32_t MqttReceiveSubscribedData()
{
  int32_t ret = -1;
  MQTTStatus_t xResult;

  xResult = MQTT_ProcessLoop(&mqttUserData.mqttContext, 3000u);
  //configASSERT(xResult == MQTTSuccess);

  if (xResult == MQTTSuccess)
  {
    ret = 0;
  }

  return ret;
}

/** ============================================================================
*       int32_t MqttPublishToTopic()
*
*       Author: Dilawar Ali       
*       Dated:  05-02-2021
*
*       Description: This function perform Mqttoperations on cellular connection
*
** ============================================================================= */
int32_t MqttPublishToTopic()
{
  int32_t ret = 0;
  MQTTStatus_t xResult;
  MqttQueueMsg_t xQueueMQTTPublishInfo;
  MQTTPublishInfo_t xMQTTPublishInfo;
  int8_t isMsgTypeExt = -1;


  (void)memset((void *)&xMQTTPublishInfo, 0x00, sizeof(xMQTTPublishInfo));
  (void)memset((void *)&xQueueMQTTPublishInfo, 0x00, sizeof(xQueueMQTTPublishInfo));
  (void)memset((void *)&ExtMsgFromQueue, 0x00, sizeof(ExtMsgFromQueue));


  if(0 != xQueuePeek(xMqttExtremeMessageQueue, &ExtMsgFromQueue, 10))
  {
	    ESP_LOGI("[Cell][MQTT]", "Topic:%s,%d\r\n",ExtMsgFromQueue.pTopicName, ExtMsgFromQueue.topicNameLength);
	    ESP_LOGI("[Cell][MQTT]", "PAyload:%s,%d\r\n", ExtMsgFromQueue.pPayload, ExtMsgFromQueue.payloadLength);

	    xMQTTPublishInfo.qos = ExtMsgFromQueue.qos;
	    xMQTTPublishInfo.retain = ExtMsgFromQueue.retain;
	    xMQTTPublishInfo.pTopicName = (const char * )ExtMsgFromQueue.pTopicName;
	    xMQTTPublishInfo.topicNameLength = (uint16_t)ExtMsgFromQueue.topicNameLength;
	    xMQTTPublishInfo.pPayload = ExtMsgFromQueue.pPayload;
	    xMQTTPublishInfo.payloadLength = ExtMsgFromQueue.payloadLength;

	    isMsgTypeExt = 1;
  }
  else if( 0 != xQueuePeek(xMqttMessageQueue, &xQueueMQTTPublishInfo, 10))
  {
	    ESP_LOGI("[Cell][MQTT]", "Topic:%s,%d\r\n",xQueueMQTTPublishInfo.pTopicName, xQueueMQTTPublishInfo.topicNameLength);
	    ESP_LOGI("[Cell][MQTT]", "PAyload:%s,%d\r\n", xQueueMQTTPublishInfo.pPayload, xQueueMQTTPublishInfo.payloadLength);

	    xMQTTPublishInfo.qos = xQueueMQTTPublishInfo.qos;
	    xMQTTPublishInfo.retain = xQueueMQTTPublishInfo.retain;
	    xMQTTPublishInfo.pTopicName = (const char * )xQueueMQTTPublishInfo.pTopicName;
	    xMQTTPublishInfo.topicNameLength = (uint16_t)xQueueMQTTPublishInfo.topicNameLength;
	    xMQTTPublishInfo.pPayload = xQueueMQTTPublishInfo.pPayload;
	    xMQTTPublishInfo.payloadLength = xQueueMQTTPublishInfo.payloadLength;
	    isMsgTypeExt = 0;
  }

  if(isMsgTypeExt >= 0)
  {
    // Send PUBLISH packet. Packet ID is not used for a QoS0 publish.
    xResult = MQTT_Publish(&mqttUserData.mqttContext, &xMQTTPublishInfo, 0U);

    if (xResult == MQTTSuccess)
    {
        ESP_LOGI("[Cell][MQTT]", "Publish Success\r\n");


        if(isMsgTypeExt == 1)
        {
        	xQueueReceive( xMqttExtremeMessageQueue, &ExtMsgFromQueue, ( TickType_t ) 10 );
        }
        else
        {
        	xQueueReceive( xMqttMessageQueue, &xQueueMQTTPublishInfo, ( TickType_t ) 10 );
        }
    }
    else
    {
    	ret = -1;
        ESP_LOGI("[Cell][MQTT]", "Publish Failed\r\n");
    }

    //configASSERT(xResult == MQTTSuccess);
  }



  return ret;
}
/** ============================================================================
*       int32_t MqttSubscribe()
*
*       Author: Dilawar Ali       
*       Dated:  05-02-2021
*
*       Description: This function perform Mqtt subscribe on cellular connection
*
** ============================================================================= */
int32_t MqttSubscribe(uint8_t *subscribeTopic)
{
  int32_t ret = -1;

  MQTTStatus_t xResult = MQTTSuccess;
  MQTTSubscribeInfo_t xMQTTSubscription[2];
  mqttUserData.mqttSubscribePacketIdentifier = MQTT_GetPacketId(&mqttUserData.mqttContext);

  (void)memset((void *)&xMQTTSubscription, 0x00, sizeof(xMQTTSubscription));

  /* Subscribe to the ExampleTopic topic filter uses QoS0. */
  xMQTTSubscription[0].qos = MQTTQoS0;
  xMQTTSubscription[0].pTopicFilter = (const char *) subscribeTopic;
  xMQTTSubscription[0].topicFilterLength = strlen((const char *)subscribeTopic);


  xResult = MQTT_Subscribe(&mqttUserData.mqttContext,
                           xMQTTSubscription,
                           1, /* Only subscribing to one topic. */
                           mqttUserData.mqttSubscribePacketIdentifier);

  configASSERT(xResult == MQTTSuccess);

  xResult = MQTT_ProcessLoop(&mqttUserData.mqttContext, 3000u);
    
  if(xResult == MQTTSuccess)
  {
    ret = 0;
  }

  return ret;
}

static void prvMQTTProcessIncomingPublish(MQTTPublishInfo_t *pxPublishInfo)
{

  LogInfo("Incoming Publish Topic Name: %.*s matches subscribed topic.\r\n Incoming Publish Message : %.*s",
          pxPublishInfo->topicNameLength,
          (uint8_t *)pxPublishInfo->pTopicName,
          pxPublishInfo->payloadLength,
          (uint8_t *)pxPublishInfo->pPayload);

  ParseMQTTData((char *)pxPublishInfo->pPayload);
}

static void prvMQTTProcessResponse( MQTTPacketInfo_t * pxIncomingPacket, uint16_t usPacketId )
{
    MQTTStatus_t xResult = MQTTSuccess;
    uint8_t * pucPayload = NULL;
    size_t ulSize = 0;

    switch( pxIncomingPacket->type )
    {
        case MQTT_PACKET_TYPE_SUBACK:

            /* A SUBACK from the broker, containing the server response to our
             * subscription request, has been received.  It contains the status
             * code indicating server approval/rejection for the subscription to
             * the single topic requested. The SUBACK will be parsed to obtain
             * the status code, and this status code will be stored in
             * #xTopicFilterContext. */
            xResult = MQTT_GetSubAckStatusCodes( pxIncomingPacket,
                                                 &pucPayload,
                                                 &ulSize );

            /* MQTT_GetSubAckStatusCodes always returns success if called with
             * packet info from the event callback and non-NULL parameters. */
            //configASSERT( xResult == MQTTSuccess );

            /* This should be the QOS leve, 0 in this case. */
            //xTopicFilterContext.xSubAckStatus = *pucPayload;

            /* Make sure ACK packet identifier matches with Request packet
             * identifier. */
            //configASSERT( mqttUserData.mqttSubscribePacketIdentifier == usPacketId );
            break;

        case MQTT_PACKET_TYPE_UNSUBACK:
            LogInfo( "Unsubscribed from the topic %s.", (uint8_t *)mqttUserData.mqttsubscribeTopic );
            /* Make sure ACK packet identifier matches with Request packet
             * identifier. */
            //configASSERT( mqttUserData.mqttSubscribePacketIdentifier == usPacketId );
            break;

        case MQTT_PACKET_TYPE_PINGRESP:

            /* Nothing to be done from application as library handles
             * PINGRESP with the use of MQTT_ProcessLoop API function. */
            LogWarn ( "PINGRESP should not be handled by the application callback when using MQTT_ProcessLoop.\n" ) ;
            break;

        /* Any other packet type is invalid. */
        default:
            LogWarn( "prvMQTTProcessResponse() called with unknown packet type:(%02X).",
                       pxIncomingPacket->type );
    }
}


/** ============================================================================
*       int32_t GetNewState()
*
*       Author: Dilawar Ali       
*       Dated:  05-02-2021
*
*       Description: This function decide the next operation for cellular module
*
** ============================================================================= */
CELLULAR_STATE_ENUM GetNewState(CELLULAR_STATE_ENUM state, int32_t lastStateExecutionStatus)
{
  CELLULAR_STATE_ENUM nextState = CELL_STATE_INIT;
  if (lastStateExecutionStatus >= 0)
  {
    switch (state)
    {
    case CELL_STATE_INIT:
      nextState = CELL_STATE_NETWORK_REGISTERATION;
      break;

    case CELL_STATE_NETWORK_REGISTERATION:
      nextState = CELL_STATE_PSD_ACTIVATION;
      break;

    case CELL_STATE_PSD_ACTIVATION:
      nextState = CELL_STATE_TCP_SOCK_DISCONNECT;
      break;

    case CELL_STATE_TCP_SOCK_CONNECT:
      nextState = CELL_STATE_MQTT_INIT;
      break;

    case CELL_STATE_TCP_SOCK_DISCONNECT:
      nextState = CELL_STATE_TCP_SOCK_CONNECT;
      break;

    case CELL_STATE_MQTT_INIT:
      nextState = CELL_STATE_MQTT_CONNECT;
      break;

    case CELL_STATE_MQTT_CONNECT:
      nextState = CELL_STATE_MQTT_TOPIC_SUBCRIBE;
      break;

    case CELL_STATE_MQTT_TOPIC_SUBCRIBE:
      nextState = CELL_STATE_MQTT_IDLE;
      break;

    case CELL_STATE_MQTT_DATA_PUBLISH:
    case CELL_STATE_MQTT_RECEIVE_DATA:
    case CELL_STATE_MQTT_IDLE:
    case CELL_STATE_MQTT_DISCONNECT:
    break;

    default:
    break;

    }
  }
  else
  {
    switch (state)
    {
    case CELL_STATE_INIT:
    case CELL_STATE_NETWORK_REGISTERATION:
    case CELL_STATE_PSD_ACTIVATION:
    case CELL_STATE_TCP_SOCK_CONNECT:
    case CELL_STATE_TCP_SOCK_DISCONNECT:
      nextState = CELL_STATE_INIT;
      break;

    case CELL_STATE_MQTT_INIT:
    case CELL_STATE_MQTT_CONNECT:
    case CELL_STATE_MQTT_TOPIC_SUBCRIBE:
    case CELL_STATE_MQTT_DATA_PUBLISH:
    case CELL_STATE_MQTT_RECEIVE_DATA:
      nextState = CELL_STATE_TCP_SOCK_DISCONNECT;
      break;

    case CELL_STATE_MQTT_IDLE:
    case CELL_STATE_MQTT_DISCONNECT:
      break;
    default:
      break;
    }
  }

  return nextState;
}

/** ============================================================================
*   Global function Implimentation
**  ============================================================================*/

void HandleCellState(CELLULAR_STATE_ENUM *state)
{
  int32_t ret = -1;

  while (*state != CELL_STATE_MQTT_IDLE)
  {
    ret = PerformStateAction(*state);
    *state = GetNewState(*state, ret);
  }
}

/** ============================================================================
*       void CellTask(void *arg)
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function Manages the all Cell tasks
*
** ============================================================================= */
void CellTask(void *arg)
{
    int32_t ret;

    //Allow other core to finish initialization
    vTaskDelay(7000 / portTICK_RATE_MS);
    
    //HandleCellState(&cellUserData->currentCellState);
    ESP_LOGI("[Cell]", "Start Cell task");

    while (1)
    {
      if (cellUserData->currentCellState != CELL_STATE_MQTT_IDLE)
      {
        HandleCellState(&cellUserData->currentCellState);
      }
      else
      {
        ESP_LOGI("[Cell]", "Pending publish messages: %d", mqttUserData.mqttpendingPublishMessage);
        vTaskDelay(2000 / portTICK_RATE_MS);
        while (mqttUserData.mqttpendingPublishMessage > 0)
        {
          ret = PerformStateAction(CELL_STATE_MQTT_DATA_PUBLISH);
          if (ret < 0)
          {
            ESP_LOGI("[Cell]", "Publish Failed");
            vTaskDelay(2000 / portTICK_RATE_MS);
            cellUserData->currentCellState = CELL_STATE_TCP_SOCK_DISCONNECT;
            break;
          }
          else
          {
            mqttUserData.mqttpendingPublishMessage--;
          }
        }

        ret = PerformStateAction(CELL_STATE_MQTT_RECEIVE_DATA);
        if (ret < 0)
        {
          ESP_LOGI("[Cell]", "Receive Failed");
          vTaskDelay(2000 / portTICK_RATE_MS);
          cellUserData->currentCellState = CELL_STATE_TCP_SOCK_DISCONNECT;
        }

        ESP_LOGI("[Cell]", "%d: Mqqt in IDLE STATE\n", GetTime());
        //vTaskDelay(10000 / portTICK_RATE_MS);
      }
    }
}

/** ============================================================================
*       int32_t SendATCAndWaitForOK(uint8_t ATCommand[], uint8_t AtResponse[], uint8_t timeout)
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function send the AT command to module and wait for the OK response from module
*
** ============================================================================= */
int32_t SendATCAndWaitForOK(uint8_t AtCommand[], uint8_t AtResponse[], uint8_t timeout)
{
  int32_t retVal = -1;
  uint32_t AtCommandTimeout = timeout;
  uint32_t referenceStartTime = 0;
  uint8_t readBytes = 0, commandLength = 0;
  uint16_t totalReadBytes = 0;
  
    commandLength = strlen((char const*)AtCommand);
    if(commandLength == 0)
    {
      retVal = ERR_CELLULAR_AT_COMMAND_INVALID;
    }
    else
    {
      //sprintf((char *)&AtCommand[commandLength], "\r\n");
      ESP_LOGI("[Cellular]", "Tx: %s", AtCommand);
      if(UartTransmit(AtCommand, commandLength) > 0)
      {
        referenceStartTime = GetTime();
        readBytes = 0;
        retVal = ERR_CELLULAR_AT_RESPONSE_RECEIVE_TIMEOUT;
        do
        {
          readBytes = UartReceive(&AtResponse[totalReadBytes], (UART_BUF_SIZE - totalReadBytes));
          if (readBytes > 0)
          {
            totalReadBytes += readBytes;
            AtResponse[totalReadBytes] = 0;
            
            if(strstr((const char*)AtResponse, "OK\r\n") != NULL)
            {
              retVal = totalReadBytes;
              break;
            }
          }
          
        }while (((GetTime() - referenceStartTime) < AtCommandTimeout));
        
        ESP_LOGI("[Cellular]", "Rx: %s,%d", AtResponse, strlen((const char*)AtResponse));
      }
      else
      {
        retVal = ERR_CELLULAR_UART_WRITE_FAILED;
      }
    }

  return retVal;
}

/** ============================================================================
*       int32_t WriteDataToTcpSocket(uint8_t data[], uint32_t size)
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function writes data to TCP socket of MQTT server
*
** ============================================================================= */
int32_t WriteDataToTcpSocket(uint8_t data[], uint32_t size)
{
  int32_t ret = -1;
  uint8_t command[2048] = {0};

  if(((size * 2) != strlen((const char *)data)) || (size > 512))
  {
    ret = ERR_CELLULAR_TCP_SOCK_WRITE_DATA_SIZE_MISMATCH;
    ESP_LOGE("[Cell]", "MQTT Buffer size overflow from cell buffer\r\n");
  }
  else
  {
	//cellUserData->atResponseBuffer[0]=0;
    sprintf((char *)&command, "AT+USOWR=0,%d,\"%s\"\r\n", size, data);
    ret = SendATCAndWaitForOK((uint8_t *)command, cellUserData->atResponseBuffer, 60);
    if (ret > 0)
    {
      ret = size;
    }
    else
    {
      ret = ERR_CELLULAR_TCP_SOCK_DATA_WRITE_FAILED;
    }
  }

  return ret;
}

/** ============================================================================
*       int32_t ReadDataFromTcpSocket(uint8_t data[], uint32_t size)
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function reads data from TCP socket of MQTT server
*
** ============================================================================= */
int32_t ReadDataFromTcpSocket(uint8_t data[], uint32_t size)
{
  int32_t ret = -1;
  uint8_t command[128] = {0};
  uint8_t *responseStart = 0;

    sprintf((char *)&command, "AT+USORD=0,%d\r\n", size);
    ret = SendATCAndWaitForOK((uint8_t *)command, cellUserData->atResponseBuffer, 10);
    if(ret > 0)
    {
      responseStart = (uint8_t *)strstr((const char*)cellUserData->atResponseBuffer, "\r\n+USORD:");

      if(responseStart != NULL)
      {
        sscanf((const char *)responseStart, "\r\n+USORD: 0,%d,\"%s\"", &ret, data);
      }
    }

    if (ret < 0)
    {
      ret = ERR_CELLULAR_TCP_SOCK_DATA_Read_FAILED;
    }

  return ret;
}

/** ============================================================================
*       int32_t ReadDataFromTcpSocket(uint8_t data[], uint32_t size)
*
*       Author: Dilawar Ali       
*       Dated:  06-02-2021
*
*       Description: This function Increment the pending publish message count
*
** ============================================================================= */
void SignalPublishMessage()
{
  mqttUserData.mqttpendingPublishMessage++;
}

/** ============================================================================
*       void AddQueueElementForMQTTClient()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function add queue element to be posted on serve
*
** ============================================================================= */
void AddQueueElementForMQTTClient(char* TopicName, char* Payload, int length, char qos, bool retain)
{
  MqttQueueMsg_t msg;

  memset(&msg, 0, sizeof(msg));

  msg.qos = qos;
  msg.retain = retain;


  ESP_LOGI("[Cell][Queue]", "Topic:%s,%d\r\n",TopicName, strlen(TopicName));
  ESP_LOGI("[Cell][Queue]", "Payload: %s, %d\r\n", Payload, strlen(Payload));

  if((strlen(TopicName) >= MQTT_TOPIC_NAME_MAX_LENGTH) || ((strlen(TopicName) == 0)))
  {
    LogError("Buffer Overflow Increase the TopicName Field (AddQueueElementForMQTTClient()): Size:%d\r\n",strlen(TopicName));
  }
  else
  {
    msg.topicNameLength = strlen(TopicName);
    memcpy(msg.pTopicName, TopicName, strlen(TopicName));

    if((strlen(Payload) >= MQTT_MAX_PAYLOAD_LENGTH) || (strlen(Payload) == 0))
    {
      LogError("Buffer Overflow Increase the Payload Field (AddQueueElementForMQTTClient()): Size:%d\r\n",strlen(Payload));
    }
    else
    {
      msg.payloadLength = strlen(Payload);
      memcpy(msg.pPayload, Payload, strlen(Payload));

      //Send data via Queue
      //xQueueWrite( xMqttMessageQueue, &msg );
      //xQueueGenericSend( xMqttMessageQueue, ( void * ) &msg, ( TickType_t ) 0, queueSEND_TO_BACK );
      xQueueSendToBack( xMqttMessageQueue, ( void * ) &msg, ( TickType_t ) 10 );

      //Increase Queue element
      SignalPublishMessage();

      ESP_LOGI("[Cell][Queue]", "No of elements in Queue: %d", uxQueueMessagesWaiting( xMqttMessageQueue ));
    }
  }
}

void AddExtQueueElementForMQTTClient(char* TopicName, char* Payload, int length, char qos, bool retain)
{
  memset(&ExtMsgToQueue, 0, sizeof(ExtMsgToQueue));

  ExtMsgToQueue.qos = qos;
  ExtMsgToQueue.retain = retain;


  ESP_LOGI("[Cell][Queue]", "Topic:%s,%d\r\n",TopicName, strlen(TopicName));
  ESP_LOGI("[Cell][Queue]", "Payload: %s, %d\r\n", Payload, strlen(Payload));

  if((strlen(TopicName) >= MQTT_TOPIC_NAME_MAX_LENGTH) || ((strlen(TopicName) == 0)))
  {
    LogError("Buffer Overflow Increase the TopicName Field (AddQueueElementForMQTTClient()): Size:%d\r\n",strlen(TopicName));
  }
  else
  {
    ExtMsgToQueue.topicNameLength = strlen(TopicName);
    memcpy(ExtMsgToQueue.pTopicName, TopicName, strlen(TopicName));

    if((strlen(Payload) >= MQTT_MAX_EXT_PAYLOAD_LENGTH) || (strlen(Payload) == 0))
    {
      LogError("Buffer Overflow Increase the Payload Field (AddQueueElementForMQTTClient()): Size:%d\r\n",strlen(Payload));
    }
    else
    {
      ExtMsgToQueue.payloadLength = strlen(Payload);
      memcpy(ExtMsgToQueue.pPayload, Payload, strlen(Payload));

      //Send data via Queue
      //xQueueWrite( xMqttMessageQueue, &msg );
      //xQueueGenericSend( xMqttMessageQueue, ( void * ) &msg, ( TickType_t ) 0, queueSEND_TO_BACK );
      xQueueSendToBack( xMqttExtremeMessageQueue, ( void * ) &ExtMsgToQueue, ( TickType_t ) 10 );

      //Increase Queue element
      SignalPublishMessage();

      ESP_LOGI("[Cell][Queue]", "No of elements in Queue: %d", uxQueueMessagesWaiting( xMqttExtremeMessageQueue ));
    }
  }
}
/** ============================================================================
*       void UpdateMQTTBrookerUrl()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function updates server URL
*
** ============================================================================= */
void UpdateMQTTBrookerUrl(char* UpdatedURL, int32_t URLLength)
{
  //memcpy(mqttUserData.mqqttBrokerAddress, UpdatedURL, URLLength);
}

/** ============================================================================
*       void UpdateMQTTUserID()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function updates user ID on server
*
** ============================================================================= */
void UpdateMQTTUserID(char* UpdatedID, int32_t IDLength)
{
  //memcpy(mqttUserData.mqttConnectInfo.pUserName, UpdatedID, IDLength);
  //mqttUserData.mqttConnectInfo.userNameLength = IDLength;
}



/** ============================================================================
*       int32_t ConfigureAllCertificate()
*
*       Author: Dilawar Ali
*       Dated:  04-04-2021
*
*       Description: This function Configure the certificate and keys
*
** ============================================================================= */
int32_t ConfigureAllCertificate()
{
	int32_t retVal = -1;

	retVal = WriteCertificateToModule(CAcrt, (uint8_t*)"CA crt", CA_CERTIFICATE);
	retVal = WriteCertificateToModule(deviceCrt, (uint8_t*)"Device crt", DEVICE_CERTIFICATE);
	retVal = WriteCertificateToModule(deviceKey, (uint8_t*)"Device Key", DEVICE_PRIVATE_KEY);

	retVal = SendATCAndWaitForOK((uint8_t *)"AT+USECMNG=3\r\n", cellUserData->atResponseBuffer, 10);
	retVal = SendATCAndWaitForOK((uint8_t *)"AT+USECPRF=0,0,0\r\n", cellUserData->atResponseBuffer, 10);
	//retVal = SendATCAndWaitForOK((uint8_t *)"AT+USECPRF=0,1,3\r\n", cellUserData->atResponseBuffer, 10);


	return retVal;
}



/** ============================================================================
*       int32_t WriteCertificateToModule(uint8_t AtCommand[], uint8_t AtResponse[], uint8_t timeout)
*
*       Author: Dilawar Ali
*       Dated:  02-02-2021
*
*       Description: This function write the certificate
*
** ============================================================================= */
int32_t WriteCertificateToModule(uint8_t *certificate, uint8_t *certificateName, CertificateType crtType)
{
  int32_t retVal = -1;
  uint32_t AtCommandTimeout = 30;
  uint32_t referenceStartTime = 0;
  uint8_t readBytes = 0, commandLength = 0;
  uint16_t totalReadBytes = 0;
  uint8_t AtCommand[128] = {0};
  uint8_t *AtResponse = cellUserData->atResponseBuffer;

  sprintf((char *)AtCommand, "AT+USECMNG=0,%d,\"%s\",%d\r\n", crtType, certificateName, strlen((const char *)certificate));

    commandLength = strlen((char const*)AtCommand);
    if(commandLength == 0)
    {
      retVal = ERR_CELLULAR_AT_COMMAND_INVALID;
    }
    else
    {
      //sprintf((char *)&AtCommand[commandLength], "\r\n");
      ESP_LOGI("[Cellular][Certificate]", "Tx: %s", AtCommand);
      if(UartTransmit(AtCommand, commandLength) > 0)
      {
        referenceStartTime = GetTime();
        readBytes = 0;
        retVal = ERR_CELLULAR_AT_RESPONSE_RECEIVE_TIMEOUT;
        do
        {
          readBytes = UartReceive(&AtResponse[totalReadBytes], (UART_BUF_SIZE - totalReadBytes));
          if (readBytes > 0)
          {
            totalReadBytes += readBytes;
            AtResponse[totalReadBytes] = 0;

            if(strstr((const char*)AtResponse, ">") != NULL)
            {
              retVal = totalReadBytes;
              if(UartTransmit(certificate, strlen((const char *)certificate)) > 0)
              {
            	  referenceStartTime = GetTime();
				  readBytes = 0;
				  retVal = ERR_CELLULAR_AT_RESPONSE_RECEIVE_TIMEOUT;
				  do
				  {
					readBytes = UartReceive(&AtResponse[totalReadBytes], (UART_BUF_SIZE - totalReadBytes));
					if (readBytes > 0)
					{
					  totalReadBytes += readBytes;
					  AtResponse[totalReadBytes] = 0;

					  if(strstr((const char*)AtResponse, "OK\r\n") != NULL)
					  {
						retVal = totalReadBytes;
						break;
					  }
					}

				  }while (((GetTime() - referenceStartTime) < AtCommandTimeout));
              }
              break;
            }
          }

        }while (((GetTime() - referenceStartTime) < AtCommandTimeout));

        ESP_LOGI("[Cellular][Certificate]", "Rx: %s,%d", AtResponse, strlen((const char*)AtResponse));
      }
      else
      {
        retVal = ERR_CELLULAR_UART_WRITE_FAILED;
      }
    }

  return retVal;
}

