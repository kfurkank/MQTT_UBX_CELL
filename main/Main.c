
/** ============================================================================
*   File:   Main.c
*   Dated:  02-02-2021
*
*   Description: This file include Definition of all components and task
**  ============================================================================*/

/** ============================================================================
*   Include files
**  ============================================================================*/
#include "Main.h"
#include "System.h"
#include "CellApplication.h"
#include "TimerTask.h"
#include "adxl357.h"
#include "fft.h"
/** ============================================================================
*   Local function Declaration
**  ============================================================================*/

/** ============================================================================
*   Local constant declarations
**  ============================================================================*/
#define SYSTEM_TASK_STACK_SIZE          (4*1024)
#define Cell_TASK_STACK_SIZE            (8*1024)
#define TIMER_TASK_STACK_SIZE           (3*1024)


// Tasks priority
#define TASK_PRIORITY_3                 ( ( UBaseType_t ) 3U )

// Queue Element size
#define SYS_MAILBOX_MSG_COUNT           20u
#define MQTT_MSG_QUEUE_COUNT            20u
#define MQTT_EXT_MSG_QUEUE_COUNT        5u

/** ============================================================================
*   Local Variable definition
**  ============================================================================*/
// Structure that will hold the TCB of the task being created.
StaticTask_t SystemTaskBuffer;
StaticTask_t CellTaskBuffer;
StaticTask_t TimerTaskBuffer;

// Task Stack definition
StackType_t SystemStack[ SYSTEM_TASK_STACK_SIZE ];
StackType_t CellStack[ Cell_TASK_STACK_SIZE ];
StackType_t TimerStack[ TIMER_TASK_STACK_SIZE ];

/** ============================================================================
*   Global Variable definition
**  ============================================================================*/

// Task defined handles
TaskHandle_t SystemTaskHandle = NULL;
TaskHandle_t CellTaskHandle = NULL;
TaskHandle_t TimerTaskHandle = NULL;

// Queue defined handles
QueueHandle_t xSysMailbox;
QueueHandle_t xMqttMessageQueue;
QueueHandle_t xMqttExtremeMessageQueue;

/** ============================================================================
*   Local function Implimentation
**  ============================================================================*/
/** ============================================================================
*       void app_main(void)
*
*       Author: Dilawar Ali       
*       Dated:  12-07-2020
*
*       Description: This function Initialize all the tasks and components
*
** ============================================================================= */
void app_main()
{
    //Allow other core to finish initialization
    vTaskDelay(pdMS_TO_TICKS(1000));
 
    //Initialize Queue
    CreateQueueForMailbox();
    //Initialize Queue
    CreateQueueForMqttMessages();
    CreateQueueForMqttExtremeMessages();

    //Initialize System Task
    CreateSystemTask();
    //Initialize Timer Task
    CreateTimerTask();

    //Initialize System and drivers
    RegisterPeriodicFunction(&PeriodicInitializationHandler);
}

/** ============================================================================
*   Global function Implimentation
**  ============================================================================*/

/** ============================================================================
*       void CreateSystemTask()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function creates task
*
** ============================================================================= */
void CreateSystemTask()
{
        //Create all the Tasks
    SystemTaskHandle = xTaskCreateStatic(
                SystemTask,       // Function that implements the task.
                "SYS_TASK",      // Text name for the task.
                SYSTEM_TASK_STACK_SIZE,      // Stack size in bytes, not words.
                ( void * ) 1,    // Parameter passed into the task.
                TASK_PRIORITY_3,// Priority at which the task is created.
                SystemStack,          // Array to use as the task's stack.
                &SystemTaskBuffer );  // Variable to hold the task's data structure.
}

/** ============================================================================
*       void CreateCellularTask()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function creates task
*
** ============================================================================= */
void CreateCellularTask()
{
    CellTaskHandle = xTaskCreateStatic(
            CellTask,       // Function that implements the task.
            "Cell_TASK",   // Text name for the task.
            Cell_TASK_STACK_SIZE,      // Stack size in bytes, not words.
            ( void * ) 1,    // Parameter passed into the task.
            TASK_PRIORITY_3,// Priority at which the task is created.
            CellStack,          // Array to use as the task's stack.
            &CellTaskBuffer );  // Variable to hold the task's data structure.
}

/** ============================================================================
*       void CreateTimerTask()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function creates task
*
** ============================================================================= */
void CreateTimerTask()
{
    TimerTaskHandle = xTaskCreateStatic(
    		TimerTask,       // Function that implements the task.
            "Timer_TASK",   // Text name for the task.
            TIMER_TASK_STACK_SIZE,      // Stack size in bytes, not words.
            ( void * ) 1,    // Parameter passed into the task.
            TASK_PRIORITY_3,// Priority at which the task is created.
            TimerStack,          // Array to use as the task's stack.
            &TimerTaskBuffer );  // Variable to hold the task's data structure.
}

/** ============================================================================
*       void CreateQueueForMailbox()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function creates queue
*
** ============================================================================= */
void CreateQueueForMailbox()
{
    //Create Queue for Mailbox
    xSysMailbox = xQueueCreate( SYS_MAILBOX_MSG_COUNT, sizeof( SysMsg_t ) );
    if ( xSysMailbox == 0 )
    {
    	ESP_LOGE("[Main]", "Mailbox Queue memory allocation failure\r\n");
    }
}

/** ============================================================================
*       void CreateQueueForMqttMessages()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function creates queue
*
** ============================================================================= */
void CreateQueueForMqttMessages()
{
    //Create Queue for Mailbox
    xMqttMessageQueue = xQueueCreate( MQTT_MSG_QUEUE_COUNT, sizeof( MqttQueueMsg_t ) );

    if ( xMqttMessageQueue == 0 )
    {
    	ESP_LOGE("[Main]", "Queue memory allocation failure\r\n");
    }
}

/** ============================================================================
*       void CreateQueueForMqttMessages()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function creates queue
*
** ============================================================================= */
void CreateQueueForMqttExtremeMessages()
{
    //Create Queue for Mailbox
    xMqttExtremeMessageQueue = xQueueCreate( MQTT_EXT_MSG_QUEUE_COUNT, sizeof( MqttQueueExtMsg_t ) );

    if ( xMqttExtremeMessageQueue == 0 )
    {
    	ESP_LOGE("[Main]", "Extreme Queue memory allocation failure\r\n");
    }
}
