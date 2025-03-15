/** ============================================================================
*   File:   System.c
*   Author: Dilawar Ali
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
*   Local constant declarations
**  ============================================================================*/

uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        //.use_ref_tick = true,
    };

/** ============================================================================
*   Local Variable definition
**  ============================================================================*/

spi_bus_config_t buscfg={
    .miso_io_num=PIN_NUM_MISO,
    .mosi_io_num=PIN_NUM_MOSI,
    .sclk_io_num=PIN_NUM_CLK,
    .quadwp_io_num=-1,
    .quadhd_io_num=-1,
    .max_transfer_sz=128
};
spi_device_interface_config_t devcfg={
    .clock_speed_hz=5000000,                //Clock out at 5 MHz
    .mode=0,                                //SPI mode 0
    .spics_io_num=PIN_NUM_CS,               //CS pin
    .queue_size=50                          //We want to be able to queue 50 transactions at a time
};
ADXL357_HANDLER spi = {};
ADXL357Status status = {};
ADXL357Temperature temp = {};
uint8_t spiReturn;
int collectRMS = 0;
int collectExt = 0;
float rmsX1 = 0.0;
float rmsY1 = 0.0;
float rmsZ1 = 0.0;
float vxx = 0.0;
float vyy = 0.0;
float vzz = 0.0;
long int rLoopTime = 0;
long int vLoopTime = 0;
int res = 0;
int checkReturnG = 0;
int checkReturnF = 0;
// If there is a problem with the sensor, we increase below count until sLoop. Then restart hardware
int sLoop = 20;
int sCount = 0;



/** ============================================================================
*   Global Variable definition
**  ============================================================================*/

/** ============================================================================
*   Local function Declaration
**  ============================================================================*/
void ReconfigureSensorSettings(void);
void ShareSensorSettingsOverCloud(void);
void SendDeviceInfoToCloud(void);
void ReadAndSendSensorData(void);
void ProcessExtremeDataRequestForSensor();
void PeriodicReadSensorData(uint32_t timeElapsed);
void ReadExtremeSensorData(uint32_t timeElapsed);

/** ============================================================================
*   Global function Implimentation
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
void SystemTask(void *arg)
{

    while(1)
    {
        //Allow other core to finish initialization
        //vTaskDelay(25000 / portTICK_RATE_MS);

        SysMsg_t xQueueElement = {0};

        //xQueueReceive( xSysMailbox, xQueueElement, portMAX_DELAY );
        //!fk.if( 0 != xQueueGenericReceive( xSysMailbox, ( void * ) &xQueueElement, ( TickType_t ) portMAX_DELAY, false ))
        if( 0 != xQueueReceive( xSysMailbox, ( void * ) &xQueueElement, portMAX_DELAY ))
        {
        	ESP_LOGI("[SystemTask]", "Msg detail: %d, %d \r\n",xQueueElement.msgId, xQueueElement.msgInfo);

            switch (xQueueElement.msgId)
            {
                case INITIALIZATION_SEQUENCE:
                    InitializationRoutine(xQueueElement.msgInfo);
                    break;

                case POST_TEST_DATA_TO_MQTT:
                    //Send Test Data to MQTT server
                    //SignalPublishMessage();
                    break;

                case SEND_DEVICE_INFO_TO_CLOUD:
                    //Send Device and Sensor status to MQTT Brokker
                    SendDeviceInfoToCloud();
                    break;

                case READ_SENSOR_VALUES:
                    //Read Sensor Value here
                    ReadAndSendSensorData();
                    break;

                case RECONFIGURE_SENSOR_PARAMETERS:
                    //Reconfigure sensors after getting the message from MQTT server
                    ReconfigureSensorSettings();
                    break;

                case SHARE_SENSOR_SETTING_OVER_CLOUD:
                    ShareSensorSettingsOverCloud();
                    break;

                case PROCESS_EXTREME_DATA_SENSOR_REQUEST:
                    ProcessExtremeDataRequestForSensor();
                    break;

                case INITIALIZE_WIFI_AP_MODE:
                    //Initialize wifi in Access Point Mode
                    ESP_LOGI("SystemTask", "Initialize Wi-Fi in AccessPoint Mode");
                    wifi_init_softap();
                    break;

                case INITIALIZE_WIFI_ST_MODE:
                    //Initialize wifi in Station Mode
                    ESP_LOGI("SystemTask", "Initialize Wi-Fi in Station Mode");
                    get_saved_wifi();
                    wifi_init_sta();
                    break;

                case EMPTY_EVENT_REQ:
                    ESP_LOGI("[TimerTask]", "SystemTask Empty Event Received\r\n");
                    break;
                    
                default:
                    ESP_LOGI("[TimerTask]", "SystemTask Unhandled Event Received: %d\r\n", xQueueElement.msgId);
                    break;
            }

        }

        //Reset the queue element to its original value
        xQueueElement.msgId = EMPTY_EVENT_REQ;
    }
}

/** ============================================================================
*       int32_t UartTransmit(uint8_t *Data, uint32_t size)
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function sends data to UART
*
** ============================================================================= */
int32_t UartTransmit(uint8_t *data, uint32_t size)
{
    int32_t ret = -1;
    ret = uart_write_bytes(UART_NUMBER, (const char *) data, size);

    return ret;
}

/** ============================================================================
*       int32_t UartTransmit(uint8_t *Data, uint32_t size)
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function receive data from UART
*
** ============================================================================= */
int32_t UartReceive (uint8_t *data, uint32_t size)
{
    int32_t ret = -1;
    ret = uart_read_bytes(UART_NUMBER, data, size, 40 / portTICK_RATE_MS);

    return ret;
}

/** ============================================================================
*       uint32_t GetTime ()
*
*       Author: Dilawar Ali       
*       Dated:  02-02-2021
*
*       Description: This function get time from RTC
*
** ============================================================================= */
uint32_t GetTime()
{
    return (uint32_t)(rtc_time_get() / rtc_clk_slow_freq_get_hz());

}

/** ============================================================================
*       uint32_t GetTimeMS ()
*
*       Author: Dilawar Ali       
*       Dated:  05-02-2021
*
*       Description: This function get time in ms from RTC
*
** ============================================================================= */
uint32_t GetTimeMS()
{
    return (uint32_t)(rtc_time_get() / (rtc_clk_slow_freq_get_hz() / 1000));

}

/** ============================================================================
*       void LogEntry (unint8_t *tag, uint8_t *message)
*
*       Author: Dilawar Ali       
*       Dated:  05-02-2021
*
*       Description: This function get time from RTC
*
** ============================================================================= */
void LogEntry (void *message)
{
    ESP_LOGI("tag", "%s\r\n", (char *)message);
}

/** ============================================================================
*       void UartInit()
*
*       Author: Dilawar Ali       
*       Dated:  06-02-2021
*
*       Description: This function initialize the UART
*
** ============================================================================= */
void UartInit()
{
    //Allow other core to finish initialization
    vTaskDelay(5000 / portTICK_RATE_MS);

    ESP_LOGI("System", "Starting Task");
    ESP_ERROR_CHECK(uart_param_config(UART_NUMBER, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUMBER, UART_PIN_TXD, UART_PIN_RXD, UART_PIN_RTS, UART_PIN_CTS));

    uart_driver_install(UART_NUMBER, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    ///uart_set_mode(UART_NUMBER, UART_MODE_UART);
    ESP_LOGI("System", "UART install Complete");
}

/** ============================================================================
*       void PeriodicInitializationHandler(uint32_t timeElapsed)
*
*       Author: Tayyab Tahir
*       Dated:  21-03-2021
*
*       Description: This function Post's periodic messages to initialize system
*
** ============================================================================= */
void PeriodicInitializationHandler(uint32_t timeElapsed)
{
    SysMsg_t msg;
    static int32_t timeCounter = INIT_EVENT_DISPATCH_INTERVAL;
    static INIT_MSG_ID_t executionStep = INIT_NVS_MODULE;

    timeCounter -= timeElapsed;

    if((timeCounter <= 0) && (executionStep <= INIT_SEQUENCE_COMPLETE))
    {
        //Set the specific state machine to which to post the event 
    	msg.msgSender = MSG_SENDER_TIMER_TASK;
        msg.msgId = INITIALIZATION_SEQUENCE;
        msg.msgInfo = executionStep;
        msg.ptrData = NULL;

        //Send data via Queue    
        //xQueueWrite( xSysMailbox, &msg );
        xQueueGenericSend( xSysMailbox, ( void * ) &msg, ( TickType_t ) 0, queueSEND_TO_BACK );

        //Reset Timer val
        timeCounter = INIT_EVENT_DISPATCH_INTERVAL;
        //Switch to next state
        executionStep++;
    }
}

/** ============================================================================
*       void PeriodicReadSensorData(uint32_t timeElapsed)
*
*       Author: Tayyab Tahir
*       Dated:  21-03-2021
*
*       Description: This function Post's periodic messages to read sensor's data
*
** ============================================================================= */
void PeriodicReadSensorData(uint32_t timeElapsed)
{
    SysMsg_t msg;
    static int32_t timeCounter = READ_SENSOR_DATA_INTERVAL;

    timeCounter -= timeElapsed;

    if((timeCounter <= 0) && (extReq == 0))
    {
        //Set the specific state machine to which to post the event 
        msg.msgSender = MSG_SENDER_TIMER_TASK;
        msg.msgId = READ_SENSOR_VALUES;
        msg.msgInfo = 0;
        msg.ptrData = NULL;
        
        //Send data via Queue    
        //xQueueWrite( xSysMailbox, &msg );
        xQueueGenericSend( xSysMailbox, ( void * ) &msg, ( TickType_t ) 0, queueSEND_TO_BACK );
        //Reset Timer val
        timeCounter = READ_SENSOR_DATA_INTERVAL;
    }
}

/** ============================================================================
*       void ReadExtremeSensorData(uint32_t timeElapsed)
*
*       Author: Tayyab Tahir
*       Dated:  21-03-2021
*
*       Description: This function Post's periodic messages to read sensor's data
*
** ============================================================================= */
void ReadExtremeSensorData(uint32_t timeElapsed)
{
    SysMsg_t msg;
    static int32_t timeCounter = READ_EXT_SENSOR_DATA_INTERVAL;

    timeCounter -= timeElapsed;

    if((timeCounter <= 0) && (extReq == 1))
    {
        //Set the specific state machine to which to post the event
        msg.msgSender = MSG_SENDER_TIMER_TASK;
        msg.msgId = PROCESS_EXTREME_DATA_SENSOR_REQUEST;
        msg.msgInfo = 0;
        msg.ptrData = NULL;

        //Send data via Queue
        //xQueueWrite( xSysMailbox, &msg );
        xQueueGenericSend( xSysMailbox, ( void * ) &msg, ( TickType_t ) 0, queueSEND_TO_BACK );
        //Reset Timer val
        timeCounter = READ_EXT_SENSOR_DATA_INTERVAL;
    }
}
/** ============================================================================
*   Local function Implimentation
**  ============================================================================*/


/** ============================================================================
*       void InitializationRoutine()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function initialize the required modules
*
** ============================================================================= */
void InitializationRoutine(INIT_MSG_ID_t EventType)
{
	esp_err_t ret;

    switch(EventType)
    {       
        case INIT_NVS_MODULE:
            //Initialize NVS
            ret = nvs_flash_init();
            if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
            {
                ESP_ERROR_CHECK(nvs_flash_erase());
                ret = nvs_flash_init();
            }
            ESP_ERROR_CHECK(ret);
            break;
            
        case INIT_GET_MAC_ADDRESS:
            getMacAddress();
            ESP_LOGI(TAG, "System Mac Address: %s", macID);
            break;

        case INIT_GET_FW_VER:
            ESP_LOGI(TAG, "FIRMWARE VERSION: %f\n", softVersion);
            break;

        case INIT_SET_WIFI_TP_AP_THEN_STATION:
            // WIFI RELATED
            /*
             * 1-Initialize Access Point
             * 2-User input some values over Access Point Server
             * 3-Close Access Point 
             * 4-According to user input (wifi name and pass), connect to wifi
            */
        	/* @todo
            ESP_LOGI(TAG, "ESP_WIFI_INIT");
            wifi_init_softap();
            vTaskDelay(2000/portTICK_RATE_MS);      // Wait some time
            ESP_LOGI(TAG, "Get Saved Data");
            get_saved_wifi();
            ESP_LOGI(TAG, "After Get Saved Data ssid and pswd: %s %s\n",ssidData,pswdData);
            wifi_init_sta();
            vTaskDelay(10000/portTICK_RATE_MS);     // Wait some time
            */
            break;

        case INIT_SYNC_TIME_FROM_CELLULAR_DEVICE:
            // TIME SYNCRONIZATION
            //sntp_setoperatingmode(SNTP_OPMODE_POLL);
            //sntp_setservername(0, "pool.ntp.org");
            //sntp_init();        // FATIH: ACABA BUNU SONRADAN DURDURMALI MIYIM?
            //vTaskDelay(5000/portTICK_RATE_MS);
            break;

        case INIT_READ_MQTT_PARAMS_FROM_NVS:
            get_saved_mqtt_host();
            get_saved_mqtt_user();
            ESP_LOGI(TAG, "Get Saved Data mqtt and user and pass: %s %s\n",mqttBrokerUrl,mqttUsername);
            break;

        case INIT_UART_DRIVERS_FOR_CELLULAR:
            //Initilize uart drivers to communicate with the cellular device
            UartInit();
            //Start the cellular task in parallel.
            CreateCellularTask();
            break;

        case INIT_READ_SENSORS_RANGE_FROM_NVS:
            // GET SENSOR RANGE and LOOP FROM NVS
//            get_save_range_data(0);     // 0 means we will read the range, 1 means we will write
//            vTaskDelay(1000/portTICK_RATE_MS);
//            get_save_vloop_data(0);     // 0 means we will read the range
//            vTaskDelay(1000/portTICK_RATE_MS);
//            get_save_rloop_data(0);     // 0 means we will read the range
//            vTaskDelay(1000/portTICK_RATE_MS);
//            get_save_cr_data(0);        // 0 means we will read the range
//            vTaskDelay(1000/portTICK_RATE_MS);
            break;

        case INIT_SPI_DRIVERS_FOR_SENSORS:
            // SPI RELATED
            ESP_LOGI(TAG, "SPI CONFIGURE");

            ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);      //Initialize the SPI bus
            ESP_ERROR_CHECK(ret);

            ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi1);
            ESP_ERROR_CHECK(ret);

            spi.spi = 1;
            spi.spi_channel = 0;
            spi.speed = 5000000;
            spi.spiHand = spi1;

            ESP_LOGI(TAG, "SPI RESET");
            adxl357_reset(&spi);
            adxl357_standby_mode(&spi);

            ESP_LOGI(TAG, "SET RANGE\n");
            adxl357_set_range(&spi, sensorRange);
            adxl357_set_filter(&spi, rFs);
            adxl357_measurement_mode(&spi);
            break;

        case INIT_READ_SENSORS_PROPERTIES:
            // READ SENSOR PROPERTIES
            adxl357_get_devid_ad(&spi, &spiReturn);
            ESP_LOGI(TAG, "SENSOR DEVICE ADDRESS: %02x\n", spiReturn);

            adxl357_get_devid_mst(&spi, &spiReturn);
            ESP_LOGI(TAG, "SENSOR DEVICE MST: %02x\n", spiReturn);

            adxl357_get_partid(&spi, &spiReturn);
            ESP_LOGI(TAG, "SENSOR PART ID: %02x\n", spiReturn);

            adxl357_get_revid(&spi, &spiReturn);
            ESP_LOGI(TAG, "SENSOR REVISION ID:%02x\n", spiReturn);
            break;

        case INIT_READ_SENSOR_STATUS:
            // READ STATUS OF SENSOR
            adxl357_get_status(&spi, &status);
            vTaskDelay(100/portTICK_RATE_MS);
            adxl357_print_status(&status);
            vTaskDelay(100/portTICK_RATE_MS);

            adxl357_get_power_ctl(&spi, &spiReturn);
            vTaskDelay(100/portTICK_RATE_MS);
            ESP_LOGI(TAG, "SENSOR POWER: %02x\n", spiReturn);
            break;


        case INIT_SEND_DEVICE_INFO_TO_CLOUD:
        	collectRMS = sampleRMS + 2 * coldStart;
        	collectExt = eFs + 2 * coldStart;

            //Send Device and Sensor status to MQTT Brokker
            SendDeviceInfoToCloud();
            break;
            
        case INIT_SEQUENCE_COMPLETE:
            ESP_LOGI(TAG, "Initialization Sequence Complete\n");
            //Sequence completed initilization handler not required
            UnRegisterPeriodicFunction(&PeriodicInitializationHandler);
            RegisterPeriodicFunction(&PeriodicReadSensorData);
            RegisterPeriodicFunction(&ReadExtremeSensorData);

            rLoopTime = time(&now);
            vLoopTime = time(&now);
            break;

          default:
            //Do nothing 
            break;
    }

}

/** ============================================================================
*       void NotifySensorSettingToBeSentToCloud()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function posts shares sensors settings to cloud signal
*
** ============================================================================= */
void NotifySensorSettingToBeSentToCloud(MSG_SENDR_t SenderTask)
{
    SysMsg_t msg;
    //Set the specific state machine to which to post the event
    msg.msgSender = SenderTask;
    msg.msgId = SHARE_SENSOR_SETTING_OVER_CLOUD;
    msg.msgInfo = 0;
    msg.ptrData = NULL;

    //Send data via Queue
    //xQueueWrite( xSysMailbox, &msg );
    xQueueGenericSend( xSysMailbox, ( void * ) &msg, ( TickType_t ) 0, queueSEND_TO_BACK );
}

/** ============================================================================
*       void RequestExtremeSensorData(MSG_SENDR_t SenderTask)
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function sends reads sensor data signal
*
** ============================================================================= */
void RequestExtremeSensorData(MSG_SENDR_t SenderTask)
{
    SysMsg_t msg;
    //Set the specific state machine to which to post the event
    msg.msgSender = SenderTask;
    msg.msgId = PROCESS_EXTREME_DATA_SENSOR_REQUEST;
    msg.msgInfo = 0;
    msg.ptrData = NULL;

    //Send data via Queue
    //xQueueWrite( xSysMailbox, &msg );
    xQueueGenericSend( xSysMailbox, ( void * ) &msg, ( TickType_t ) 0, queueSEND_TO_BACK );
}

/** ============================================================================
*       void NotifySensorReconfiguration(MSG_SENDR_t SenderTask)
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function posts sensor reconfiguration signal
*
** ============================================================================= */
void NotifySensorReconfiguration(MSG_SENDR_t SenderTask)
{
    SysMsg_t msg;
    //Set the specific state machine to which to post the event 
    msg.msgSender = SenderTask;
    msg.msgId = RECONFIGURE_SENSOR_PARAMETERS;
    msg.msgInfo = 0;
    msg.ptrData = NULL;
    
    //Send data via Queue    
    //xQueueWrite( xSysMailbox, &msg );
    xQueueGenericSend( xSysMailbox, ( void * ) &msg, ( TickType_t ) 0, queueSEND_TO_BACK );
}

/** ============================================================================
*       void ReconfigureSensorSettings()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function reconfigures sensor.
*
** ============================================================================= */
void ReconfigureSensorSettings(void)
{
    adxl357_standby_mode(&spi);
    vTaskDelay(1000/portTICK_RATE_MS);
    ESP_LOGI(TAG, "SET RANGE");
    adxl357_set_range(&spi, sensorRange);
    vTaskDelay(2000/portTICK_RATE_MS);
    adxl357_measurement_mode(&spi);
    vTaskDelay(2000/portTICK_RATE_MS);
    gotMessage = 0;
}

/** ============================================================================
*       void ShareSensorSettingsOverCloud()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function shares sensors settings to cloud
*
** ============================================================================= */
void ShareSensorSettingsOverCloud(void)
{
    ESP_LOGI(TAG, "Sharing Sensor Setting over cloud");
    share_sensor_settings(&spi);
    shareSettings = 0;
}

/** ============================================================================
*       void SendDeviceInfoToCloud()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function shares sensors ssstatus and device infot to cloud
*
** ============================================================================= */
void SendDeviceInfoToCloud(void)
{
        // If spiReturn is 0, then sensor is working
    if(spiReturn == 0)
    {
        sendJsonLog(101, "L");
    }
    else
    {
        sendJsonLog(901, "L");
    }

    // Send some attributes to Cloud
    sendAttributesFloat("firmVersion", softVersion);
    sendAttributesString("macID", macID);
    sendAttributesString("deviceIP", deviceIP);
}

/** ============================================================================
*       void ReadAndSendSensorData()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function reads sensor data
*
** ============================================================================= */
void ReadAndSendSensorData(void)
{
    // R data related coding
    float *rx, *ry, *rz;
    int j;

    rx = malloc(collectRMS * sizeof(float));
    ry = malloc(collectRMS * sizeof(float));
    rz = malloc(collectRMS * sizeof(float));

    if((extReq == 0) && (updateVersion == 0))
    {
        j = 0;
        // LETS GET R DATA
        do
        {
            timeEp = time(&now);
            res = adxl357_fifo_bucket(&spi, collectRMS, rx, ry, rz, FIFO_STREAM_OVR_BREAK|FIFO_STREAM_FIFO_READ_BREAK);
            if (res == ADXL357_FIFO_READ_ERROR){
                ESP_LOGI(TAG, "FIFO SENSOR READ ERROR!");
                sendJsonLog(921, "L");
                adxl357_standby_mode(&spi);
                vTaskDelay(1000/portTICK_RATE_MS);
                vTaskDelay(2000/portTICK_RATE_MS);
                adxl357_measurement_mode(&spi);
                sCount = sCount + 1;
                vTaskDelay(2000/portTICK_RATE_MS);
                break;
            }
            j++;
        }while(((res == ADXL357_FIFO_STREAM_ERROR) || (res == ADXL357_FIFO_READ_ERROR)) && (j < 10));

        // IF WE CAN NOT GET DATA FROM SENSOR FOR J<10, THEN WE NEED TO SEND LOG TO USER
        if((j == 10) || ((res == ADXL357_FIFO_STREAM_ERROR) || (res == ADXL357_FIFO_READ_ERROR)))
        {
            ESP_LOGI(TAG, "Too Much Overflow !");
            sendJsonLog(911, "L");
            //continue;
        }

        else
        {
        	if(sCount > sLoop)
			{
				ESP_LOGI(TAG, "SENSE LOOP ERROR - RESTART!");
				sendJsonLog(931, "L");
				esp_restart();
			}

			// Make some calculation over Vibration Data and Send Results
			doSomeR_calculation(collectRMS, coldStart, rx, ry, rz, &rmsX1, &rmsY1, &rmsZ1);
			//sendJsonRMS((float)(((int)(rmsX1*10000.0))/10000.0), (float)(((int)(rmsY1*10000.0))/10000.0), (float)(((int)(rmsZ1*10000.0))/10000.0));
			sendRMSIO("RMSX1",(double)(((int)(rmsX1*10000.0))/10000.0));
			sendRMSIO("RMSY1",(double)(((int)(rmsY1*10000.0))/10000.0));
			sendRMSIO("RMSZ1",(double)(((int)(rmsZ1*10000.0))/10000.0));
			
			// Get and Send Temperature Data
			adxl357_read_temperature(&spi, &temp);
			//sendJsonTemp(temp.celsius);
			sendTempIO("Temp",(double)(((int)(temp.celsius*10.0))/10.0));

			// If time loop is OK to calculate V datas, then get into below IF
			// extReq is whether there is E request from user. If there is E request, we pass this IF to get E data.
			if(((time(&now)- vLoopTime)> vLoop) && (extReq != 1))
			{

				vxx = doSomeV_calculation(sampleRMS, rFs, coldStart, rx);
				vTaskDelay(200/portTICK_RATE_MS);
				vyy = doSomeV_calculation(sampleRMS, rFs, coldStart, ry);
				vTaskDelay(200/portTICK_RATE_MS);
				vzz = doSomeV_calculation(sampleRMS, rFs, coldStart, rz);
				vTaskDelay(200/portTICK_RATE_MS);

				sendJsonVRMS((float)(((int)(vxx*10000.0))/10000.0), (float)(((int)(vyy*10000.0))/10000.0), (float)(((int)(vzz*10000.0))/10000.0));

				vLoopTime = time(&now);
			}


			// LET US CHECK SOME PARAMETERS OF SENSOR. SOMETIMES SENSOR GOES TO DEFAULTS-MAYBE BECAUSE OF POWER PROBLEMS
			checkReturnG = adxl357_check_range(&spi, sensorRange);
			checkReturnF = adxl357_check_filter(&spi, rFs);

			// IF THERE ARE ERRORS, LETS MAKE EVERY SETTINGS AGAIN
			if(checkReturnG == 0 || checkReturnF == 0)
			{
				sendJsonLog(904, "L");
				adxl357_standby_mode(&spi);
				vTaskDelay(1000/portTICK_RATE_MS);
				ESP_LOGI(TAG, "SET RANGE AGAIN\n");
				adxl357_set_range(&spi, sensorRange);
				vTaskDelay(500/portTICK_RATE_MS);
				adxl357_set_filter(&spi, rFs);
				vTaskDelay(1000/portTICK_RATE_MS);
				adxl357_measurement_mode(&spi);
				vTaskDelay(1000/portTICK_RATE_MS);
			}

			vTaskDelay(1000/portTICK_RATE_MS);

			/////// IF THERE IS E REQUEST FROM USER, THEN WE CLOSE R LOOP AND FREE R RELATED MALLOCS
			free(rx);
			free(ry);
			free(rz);
        }
    }
}

/** ============================================================================
*       void ProcessExtremeDataRequestForSensor()
*
*       Author: Tayyab Tahir
*       Dated:  22-03-2021
*
*       Description: This function reads sensor data
*
** ============================================================================= */
void ProcessExtremeDataRequestForSensor(void)
{
	/////// IF THERE IS E REQUEST, WE GET IN BELOW LOOP
    /////// There is a limit time space between each E requests. User can not E data les than extReqLoop
    /*
     * For E, user send us sampling frequency (eFS) and time(eTime).
     */

    if((extReq == 1) && ((time(&now) - extReqTime) > extReqLoop))
    {
        vTaskDelay(1000/portTICK_RATE_MS);

        // Change Acquisition Speed according to user sampling frequency
        if(eFs != rFs)
        {
            adxl357_standby_mode(&spi);
            vTaskDelay(1000/portTICK_RATE_MS);
            ESP_LOGI(TAG, "SET FILTER\n");
            adxl357_set_filter(&spi, eFs);
            vTaskDelay(2000/portTICK_RATE_MS);
            adxl357_measurement_mode(&spi);
            vTaskDelay(5000/portTICK_RATE_MS);
        }

        // Calculate and malloc required data bucket 
        collectExt = eFs * eTime + 2 * coldStart;

        //Accelerometer Data initialization:
        float *ex, *ey, *ez;
        int j;

        ex = malloc(collectExt * sizeof(float));
        ey = malloc(collectExt * sizeof(float));
        ez = malloc(collectExt * sizeof(float));
        j = 0;
        do
        {
            res = adxl357_fifo_bucket(&spi, collectExt, ex, ey, ez, FIFO_STREAM_OVR_BREAK|FIFO_STREAM_FIFO_READ_BREAK);
            if (res == ADXL357_FIFO_READ_ERROR){
                ESP_LOGI(TAG, "FIFO SENSOR READ ERROR EXTREME");
                sendJsonLog(922, "L");
                adxl357_standby_mode(&spi);
                vTaskDelay(1000/portTICK_RATE_MS);
                vTaskDelay(2000/portTICK_RATE_MS);
                adxl357_measurement_mode(&spi);
                sCount = sCount + 1;
                vTaskDelay(5000/portTICK_RATE_MS);
                break;
            }
            j++;
        }while(((res == ADXL357_FIFO_STREAM_ERROR) || (res == ADXL357_FIFO_READ_ERROR)) && (j < 10));
        
        // WE STORE EXACT TIME FOR EXTREME. WE WILL SEND THIS TO USER. THEY NEED EXACT TIMING
        timeEp = time(&now);

        // IF WE CAN NOT GET DATA FROM SENSOR FOR J<10, THEN WE NEED TO SEND LOG TO USER
        if((j == 10) || ((res == ADXL357_FIFO_STREAM_ERROR) || (res == ADXL357_FIFO_READ_ERROR)))
        {
            ESP_LOGI(TAG, "Extreme Too Much Overflow!");
            sendJsonLog(912, "L");
            free(ex);
            free(ey);
            free(ez);
            //continue;
        }
        else
        {
        	extReqTime = time(&now);
			sendJsonExtBucket("A1",  ex, ey, ez, (double)timeEp, collectExt, eFs);

			extReq = 0;
			free(ex);
			free(ey);
			free(ez);

			vTaskDelay(500/portTICK_RATE_MS);
			// Change Acquisition Speed For R loop
			if(eFs != rFs)
			{
				adxl357_standby_mode(&spi);
				vTaskDelay(1000/portTICK_RATE_MS);
				ESP_LOGI(TAG, "SET FILTER\n");
				adxl357_set_filter(&spi, rFs);
				vTaskDelay(2000/portTICK_RATE_MS);
				adxl357_measurement_mode(&spi);
				vTaskDelay(5000/portTICK_RATE_MS);
			}

			sendAttributes("lastExtreme", (int)timeEp);
        }
    }
}
