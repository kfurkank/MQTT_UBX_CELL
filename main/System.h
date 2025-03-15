#ifndef SYSTEM_H
#define SYSTEM_H
/** ============================================================================
*   File:   System.h
*   Author: Dilawar Ali
*   Dated:  02-02-2021
*
*   Description: This file control all the system related tasks
**  ============================================================================*/

/** ============================================================================
*   Include files
**  ============================================================================*/
#include "SysDef.h"

/** ============================================================================
*   Global constant declarations
**  ============================================================================*/
#define UART_NUMBER   (UART_NUM_2)
#define UART_PIN_TXD  (GPIO_NUM_17)
#define UART_PIN_RXD  (GPIO_NUM_16)
#define UART_PIN_RTS  (UART_PIN_NO_CHANGE)
#define UART_PIN_CTS  (UART_PIN_NO_CHANGE)
#define UART_BUF_SIZE (512)

/** ============================================================================
*   external variables
**  ============================================================================*/

/** ============================================================================
*   Global function Declarations
**  ============================================================================*/
/** ============================================================================
*       void SystemTask(void *arg)
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function Manages the system peripherals and all tasks
*
** ============================================================================= */
void SystemTask(void *arg);

/** ============================================================================
*       int32_t UartTransmit(uint8_t *Data, uint32_t size)
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function sends data to UART
*
** ============================================================================= */
int32_t UartTransmit(uint8_t *data, uint32_t size);

/** ============================================================================
*       int32_t UartTransmit(uint8_t *Data, uint32_t size)
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function receive data from UART
*
** ============================================================================= */
int32_t UartReceive (uint8_t *data, uint32_t size);

/** ============================================================================
*       uint32_t GetTime ()
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function get time from RTC
*
** ============================================================================= */
uint32_t GetTime ();

/** ============================================================================
*       uint32_t GetTimeMS ()
*
*       Author: Dilawar Ali       
*       Dated:  05-02-2021
*
*       Description: This function get time in ms from RTC
*
** ============================================================================= */
uint32_t GetTimeMS();

/** ============================================================================
*       void LogEntry (unint8_t *tag, uint8_t *message)
*
*       Author: Dilawar Ali       
*       Dated:  05-02-2021
*
*       Description: This function get time from RTC
*
** ============================================================================= */
void LogEntry (void *message);

/** ============================================================================
*       void UartInit()
*
*       Author: Dilawar Ali       
*       Dated:  06-02-2021
*
*       Description: This function initialize the UART
*
** ============================================================================= */
void UartInit();

/** ============================================================================
*       void InitializationRoutine()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function initialize the required modules
*
** ============================================================================= */
void InitializationRoutine(INIT_MSG_ID_t EventType);

/** ============================================================================
*       void PeriodicInitializationHandler(uint32_t timeElapsed)
*
*       Author: Tayyab Tahir
*       Dated:  21-03-2021
*
*       Description: This function Post's periodic messages to initialize system
*
** ============================================================================= */
void PeriodicInitializationHandler(uint32_t timeElapsed);

/** ============================================================================
*       void RequestExtremeSensorData(MSG_SENDR_t SenderTask)
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function sends reads sensor data signal
*
** ============================================================================= */
void RequestExtremeSensorData(MSG_SENDR_t SenderTask);

/** ============================================================================
*       void NotifySensorReconfiguration(MSG_SENDR_t SenderTask)
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function posts sensor reconfiguration signal
*
** ============================================================================= */
void NotifySensorReconfiguration(MSG_SENDR_t SenderTask);

/** ============================================================================
*       void NotifySensorSettingToBeSentToCloud()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function posts shares sensors settings to cloud signal
*
** ============================================================================= */
void NotifySensorSettingToBeSentToCloud(MSG_SENDR_t SenderTask);

#endif
