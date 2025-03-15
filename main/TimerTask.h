#ifndef __TIMER_TASK_H_
#define __TIMER_TASK_H_

/** ============================================================================
*   File:   TimerTask.h
*   Dated:  22-03-2021
*
*   Description: This file include Definition of all components and task
**  ============================================================================*/

/** ============================================================================
*   Include files
**  ============================================================================*/
#include "SysDef.h"

/** ============================================================================
*   Global constant declarations
**  ============================================================================*/
#define CLOCK_TICKS_PER_SEC             1000
#define TIMER_PERIODIC_INTERVAL         1*CLOCK_TICKS_PER_SEC
#define INIT_EVENT_DISPATCH_INTERVAL	1*TIMER_PERIODIC_INTERVAL
#define READ_SENSOR_DATA_INTERVAL		10*TIMER_PERIODIC_INTERVAL
#define READ_EXT_SENSOR_DATA_INTERVAL	5*TIMER_PERIODIC_INTERVAL

typedef void(*PtrPeriodicFunc)(uint32_t);


/** ============================================================================
*   Global function Declarations
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
void TimerTask(void *arg);

/** ============================================================================
*       void RegisterPeriodicFunction(PtrPeriodicFunc ptrFunc)
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function registers the activities to be posted regularly
*
** ============================================================================= */
bool RegisterPeriodicFunction(PtrPeriodicFunc ptrFunc);

/** ============================================================================
*       void UnRegisterPeriodicFunction(PtrPeriodicFunc ptrFunc)
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function Unregisters the activity to be posted regularly
*
** ============================================================================= */
bool UnRegisterPeriodicFunction(PtrPeriodicFunc ptrFunc);

#endif /* __TIMER_TASK_H_ */
