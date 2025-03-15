
/** ============================================================================
*   File:   TimerTask.c
*   Dated:  22-03-2021
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
*   Global constant declarations
**  ============================================================================*/
#define MAX_PERIODIC_FUNCTION_ALLOWED        10u

/** ============================================================================
*   Global variable declarations
**  ============================================================================*/
static PtrPeriodicFunc timerArray[MAX_PERIODIC_FUNCTION_ALLOWED];

/** ============================================================================
*   Local function Declarations
**  ============================================================================*/
static void DispatchPeriodicEvents(void);


/** ============================================================================
*   Local function Implementation
**  ============================================================================*/

/** ============================================================================
*       void TimerTask(void *arg)
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function handles the timer task activity
*
** ============================================================================= */
void TimerTask(void *arg)
{
    while(true)
    {
        //LogInfo("TimerTask() in sleep\r\n");
        vTaskDelay(TIMER_PERIODIC_INTERVAL / portTICK_RATE_MS);
        //LogInfo("TimerTask() wokeup\r\n");

        //Dispatch periodic function
        //ESP_LOGI("[TimerTask]", "event dispatch \r\n");
        DispatchPeriodicEvents();
    }
}

/** ============================================================================
*       void RegisterPeriodicFunction(PtrPeriodicFunc ptrFunc)
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function registers the activities to be posted regularly
*
** ============================================================================= */
bool RegisterPeriodicFunction(PtrPeriodicFunc ptrFunc)
{
    bool isRegister = false;
    
    for(uint8_t index =0;index<MAX_PERIODIC_FUNCTION_ALLOWED; ++index)
    {
        if(ptrFunc == timerArray[index])
        {
            //If already registered
            return true;
        }
    }

    for(uint8_t index =0;index<MAX_PERIODIC_FUNCTION_ALLOWED; ++index)
    {
        if(NULL == timerArray[index])
        {
            timerArray[index]=ptrFunc;
            isRegister = true;
            break;
        }
    }
    return isRegister;
}

/** ============================================================================
*       void UnRegisterPeriodicFunction(PtrPeriodicFunc ptrFunc)
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function Unregisters the activity to be posted regularly
*
** ============================================================================= */
bool UnRegisterPeriodicFunction(PtrPeriodicFunc ptrFunc)
{
    bool isUnRegistered = true;
    
    for(uint8_t index =0;index<MAX_PERIODIC_FUNCTION_ALLOWED; ++index)
    {
        if(ptrFunc == timerArray[index])
        {
            //If already registered
            timerArray[index] = NULL;
        }
    }

    return isUnRegistered;
}

/** ============================================================================
*       void DispatchPeriodicEvents()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function posts registered functions regularly
*
** ============================================================================= */
void DispatchPeriodicEvents(void)
{
    
    for(uint8_t index =0;index<MAX_PERIODIC_FUNCTION_ALLOWED; ++index)
    {
        if(NULL != timerArray[index])
        {
            timerArray[index](TIMER_PERIODIC_INTERVAL);
        }
    }
}
