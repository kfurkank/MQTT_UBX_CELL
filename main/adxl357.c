#include "Main.h"
#include "System.h"
#include "CellApplication.h"
#include "TimerTask.h"
#include "adxl357.h"
#include "fft.h"



/*
 * adxl345.c
 *
 *  Created on: May 4, 2020
 *      Author: fatihaltunel
 */

/** ============================================================================
*   Global Variables
**  ============================================================================*/

char cpuID[] = "snESP32001";
char macID[14];

time_t now;         // Time variable

int shareSettings = 0;

char deviceIP[32] = "0.0.0.0";

char ssidData[32];
char pswdData[64];


uint8_t sensorRange = 40;		// Sensor Range Variable

int rFs = 2000;		//Fs for R Loop
int eFs = 4000;		//Fs for E Data
int vTime = 1.0;	//Total time for R data collection
int eTime = 1.0;	//Total time for E data collection
uint8_t compareFilter = 0x01;	// We check sensor filter settings in every R loop because of power related resetting.
uint8_t compareRange = 0x83;	// We chenck sensor range settings in every R loop because of power related resetting


int sampleRMS = 2048;   // R related bucket size
int coldStart = 10;
int rmsLoop = 120;
int vLoop = 240;
float onOffLimit = 0.1;		//Sensörün takıldığı noktanın/makinenin çalıştığı ve çalışmadığı durumu ayrıştıran rms seviyesi

// SENSOR data conversion variables.
double baseConv = 0.00001907348;
double accConv = 0.00001907348;

//Firmware OTA:
int updateVersion = 0;
float softVersion = 7.5;
char updateURL[100];
char fiUpPass[] = "3501963";		// Access Point Related
int fiUpYesNo = 0;
httpd_handle_t http_server = NULL;

long int timeEp;				  //Exact epoch time of data collection
long int extReqTime = 0;
long int extReqLoop = 5;

int extReq = 0;           // This value becomes 1 if user request E data
int gotMessage = 0;	      // This value becomes 1 if there is message from user

char mqttBrokerUrl[40];
char mqttUsername[40];

int mqttConnectFlag = 0;	//mqtt'ye bağlı ise bunu 1 yapıyoruz, bağlı değilse 0 oluyor.
int mqttConC;		// mqtt'ye bağlanamazsa bu count'u teker teker arttıracak.
int mqttConL = 20; 	// mqtt'ye bu kadar adet bağlanmazsa sisteme restart atacak.

int wifiRetryCount = 0;

int haveWifiData = 1;
int somebodyInside = 0;     // If there is somebody inside Access Point, we delay some time for user.

EventGroupHandle_t s_wifi_event_group;

spi_device_handle_t spi1;
esp_err_t ret;

esp_netif_t *ap_netif = NULL;

httpd_handle_t server = NULL;

httpd_uri_t ctrlcfg_get = {
	.uri	  = "/config",
	.method   = HTTP_GET,
	.handler  = ctrlcfg_get_handler,
	.user_ctx = NULL
};

httpd_uri_t index_get = {
	.uri	  = "/update",
	.method   = HTTP_GET,
	.handler  = index_get_handler,
	.user_ctx = NULL
};

httpd_uri_t update_post = {
	.uri	  = "/updatePost",
	.method   = HTTP_POST,
	.handler  = update_post_handler,
	.user_ctx = "TEST2"
};
/** ============================================================================
*   Global Variables-End
**  ============================================================================*/


///// SENSOR & SPI
int adxl357_wpi_spi_write(ADXL357_HANDLER * handler, uint8_t * data, uint8_t * result, size_t len)
{
  if((handler->spi_channel) == 0)
  {
	  uint8_t *tempp=NULL;
	  tempp=(uint8_t *)heap_caps_malloc(len*8, MALLOC_CAP_DMA);

	  //ESP_LOGI(TAG, "DATA LEN: %i",len);
	  spi_transaction_t t;
	  memset(&t, 0, sizeof(t));
	  t.length = len * 8;
	  //t.rxlength	= len * 8;
	  t.tx_buffer = data;
	  t.rx_buffer = tempp;
	  //ESP_LOGI(TAG, "SPI WRITE");

	  spi_device_polling_transmit(handler->spiHand, &t);
	  //esp_err_t ret = spi_device_polling_transmit(handler->spiHand, &t);
	  //assert( ret == ESP_OK );

	  for(int i=1;i<len;i++)
	  {
		  result[i]=tempp[i];
	  }
	  heap_caps_free(tempp);
  }
  else if((handler->spi_channel) == 1)
  {
	  printf("ERROR WRONG CHANNEL!!!");
  }

  return 1;
}

void adxl357_print_command_result(ADXL357Command * sensorCMD)
{
  for (uint8_t i=0; i<sensorCMD->len; i++){
    printf("%d - %02x \n", i, sensorCMD->raw_result[i]);
  }
}

void adxl357_print_status(ADXL357Status * status)
{
  printf("ADXL357 Status register: \n");
  printf("NVM_BUSY: %s\n", status->NVM_BUSY ? "true" : "false");
  printf("ACTIVITY: %s\n", status->ACTIVITY ? "true" : "false");
  printf("FIFO_OVR: %s\n", status->FIFO_OVR ? "true" : "false");
  printf("FIFO_FULL: %s\n", status->FIFO_FULL  ? "true" : "false");
  printf("DATA_RDY: %s\n", status->DATA_RDY ? "true" : "false");
}

void adxl357_prepare_command(ADXL357Command * sensorCMD)
{

  if (!sensorCMD->len){
    switch (sensorCMD->reg){
      case ADXL357_REG_READ(ADXL357_REG_DEVID_AD):
      case ADXL357_REG_READ(ADXL357_REG_DEVID_MST):
      case ADXL357_REG_READ(ADXL357_REG_PARTID):
      case ADXL357_REG_READ(ADXL357_REG_REVID):
      case ADXL357_REG_READ(ADXL357_REG_STATUS):
      case ADXL357_REG_READ(ADXL357_REG_FIFO_ENTRIES):
	    sensorCMD->len = 2;

	break;
      case ADXL357_REG_READ(ADXL357_REG_TEMP2):
	    sensorCMD->len = 3;
	break;
      case ADXL357_REG_READ(ADXL357_REG_XDATA3):
	    sensorCMD->len = 10;
	break;
      case ADXL357_REG_READ(ADXL357_REG_FILTER):
      case ADXL357_REG_WRITE(ADXL357_REG_FILTER):
      case ADXL357_REG_READ(ADXL357_REG_FIFO_SAMPLES):
      case ADXL357_REG_WRITE(ADXL357_REG_FIFO_SAMPLES):
      case ADXL357_REG_READ(ADXL357_REG_INT_MAP):
      case ADXL357_REG_WRITE(ADXL357_REG_INT_MAP):
      case ADXL357_REG_READ(ADXL357_REG_SYNC):
      case ADXL357_REG_WRITE(ADXL357_REG_SYNC):
      case ADXL357_REG_READ(ADXL357_REG_RANGE):
      case ADXL357_REG_WRITE(ADXL357_REG_RANGE):
      case ADXL357_REG_READ(ADXL357_REG_POWER_CTL):
      case ADXL357_REG_WRITE(ADXL357_REG_POWER_CTL):
      case ADXL357_REG_WRITE(ADXL357_REG_RESET):
	    sensorCMD->len = 2;
	break;
      default:
	printf("Command is unknown!\n");
	exit(-1);
	break;
    }
  }

  sensorCMD->prepared[0] = sensorCMD->reg;
  for (uint16_t i=1; i<sensorCMD->len; i++){
    sensorCMD->prepared[i] = sensorCMD->data[i-1];
  }

  sensorCMD->status |= ADXL357_COMMAND_PREPARED;
}

int adxl357_execute_command(ADXL357_HANDLER * handler, ADXL357Command * sensorCMD)
{
  adxl357_prepare_command(sensorCMD);

  if (handler->spi){
    ADXL357_SPI_WRITE(handler, sensorCMD->prepared, sensorCMD->raw_result, sensorCMD->len);
  }
  else {
      printf("ERROR!!!");
      return ADXL357_ERROR;
  }
  return 0;
}

int adxl357_get_devid_ad(ADXL357_HANDLER * handler, uint8_t * b)
{
  ADXL357Command sensorCMD = {};
  sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_DEVID_AD);

  adxl357_execute_command(handler, &sensorCMD);

  *b = sensorCMD.raw_result[1];
  vTaskDelay(100/portTICK_RATE_MS);
  return 0;
}

int adxl357_get_devid_mst(ADXL357_HANDLER * handler, uint8_t * b)
{
  ADXL357Command sensorCMD = {};
  sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_DEVID_MST);

  adxl357_execute_command(handler, &sensorCMD);

  *b = sensorCMD.raw_result[1];
  vTaskDelay(100/portTICK_RATE_MS);
  return 0;
}

int adxl357_get_partid(ADXL357_HANDLER * handler, uint8_t * b)
{
  ADXL357Command sensorCMD = {};
  sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_PARTID);

  adxl357_execute_command(handler, &sensorCMD);

  *b = sensorCMD.raw_result[1];
  vTaskDelay(100/portTICK_RATE_MS);
  return 0;
}

int adxl357_get_revid(ADXL357_HANDLER * handler, uint8_t * b)
{
  ADXL357Command sensorCMD = {};
  sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_REVID);

  adxl357_execute_command(handler, &sensorCMD);

  *b = sensorCMD.raw_result[1];
  vTaskDelay(100/portTICK_RATE_MS);
  return 0;
}

int adxl357_get_status(ADXL357_HANDLER * handler, ADXL357Status * status)
{
  ADXL357Command sensorCMD = {};
  sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_STATUS);

  adxl357_execute_command(handler, &sensorCMD);

  status->NVM_BUSY = sensorCMD.raw_result[1] 	>> 4 & 0x01;
  status->ACTIVITY = sensorCMD.raw_result[1] 	>> 3 & 0x01;
  status->FIFO_OVR = sensorCMD.raw_result[1] 	>> 2 & 0x01;
  status->FIFO_FULL = sensorCMD.raw_result[1]	>> 1 & 0x01;
  status->DATA_RDY = sensorCMD.raw_result[1]	     & 0x01;
  return 0;
}

int adxl357_get_fifo_entries(ADXL357_HANDLER * handler, uint8_t * b)
{
  ADXL357Command sensorCMD = {};
  sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_FIFO_ENTRIES);

  adxl357_execute_command(handler, &sensorCMD);

  *b = sensorCMD.raw_result[1] & 0x7F;
  return 0;
}

int adxl357_get_status_n_fifo(ADXL357_HANDLER * handler, ADXL357StatusAndFifo * status)
{
  ADXL357Command sensorCMD = {};
  sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_STATUS);
  sensorCMD.len = 3;

  adxl357_execute_command(handler, &sensorCMD);

  status->status.NVM_BUSY = sensorCMD.raw_result[1] 	>> 4 & 0x01;
  status->status.ACTIVITY = sensorCMD.raw_result[1] 	>> 3 & 0x01;
  status->status.FIFO_OVR = sensorCMD.raw_result[1] 	>> 2 & 0x01;
  status->status.FIFO_FULL = sensorCMD.raw_result[1]	>> 1 & 0x01;
  status->status.DATA_RDY = sensorCMD.raw_result[1]	     & 0x01;

  status->fifo_entries = sensorCMD.raw_result[2] & 0x7F;
  return 0;
}

int adxl357_read_temperature(ADXL357_HANDLER * handler, ADXL357Temperature * temp)
{
  ADXL357Command sensorCMD = {};
  sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_TEMP2);

  adxl357_execute_command(handler, &sensorCMD);

  temp->raw = ( (sensorCMD.raw_result[1] & 0x0F) << 8) + ( sensorCMD.raw_result[2] );
  temp->celsius = ((ADXL357_TEMP_NOMINAL_LSB - temp->raw)
		      / ADXL357_TEMP_NOMINAL_CELSIUS_SLOPE) + ADXL357_TEMP_NOMINAL_CELSIUS;
  temp->kelvin = temp->celsius + 273.15;
  temp->fahrenheit = temp->celsius * ADXL357_TEMP_C_TO_F_CONST + 32;
  return 0;
}

int adxl357_read_acceleration(ADXL357_HANDLER * handler, ADXL357Acceleration * acc)
{
  ADXL357Command sensorCMD = {};
  sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_XDATA3);

  adxl357_execute_command(handler, &sensorCMD);

  if ((sensorCMD.raw_result[1]>>7) == 1)
  {
      acc->x = ( (sensorCMD.raw_result[1] << 12) + (sensorCMD.raw_result[2] << 4) + (sensorCMD.raw_result[3] >> 4) ) - ADXL357_TWO_COMPONENT_POWER;
  }
  else
  {
      acc->x = ( (sensorCMD.raw_result[1] << 12) + (sensorCMD.raw_result[2] << 4) + (sensorCMD.raw_result[3] >> 4) );
  }

  if ((sensorCMD.raw_result[4]>>7) == 1)
  {
      acc->y = ( (sensorCMD.raw_result[4] << 12) + (sensorCMD.raw_result[5] << 4) + (sensorCMD.raw_result[6] >> 4) ) - ADXL357_TWO_COMPONENT_POWER;
  }
  else
  {
      acc->y = ( (sensorCMD.raw_result[4] << 12) + (sensorCMD.raw_result[5] << 4) + (sensorCMD.raw_result[6] >> 4) );
  }

  if ((sensorCMD.raw_result[7]>>7) == 1)
  {
      acc->z = ( (sensorCMD.raw_result[7] << 12) + (sensorCMD.raw_result[8] << 4) + (sensorCMD.raw_result[9] >> 4)) - ADXL357_TWO_COMPONENT_POWER;
  }
  else
  {
      acc->z = ( (sensorCMD.raw_result[7] << 12) + (sensorCMD.raw_result[8] << 4) + (sensorCMD.raw_result[9] >> 4));
  }
  return 0;

  //printf("1: %d,%d,%d\n", sensorCMD.raw_result[1], sensorCMD.raw_result[2], sensorCMD.raw_result[3]);
  //printf("2: %d,%d,%d\n", (sensorCMD.raw_result[1] << 24), (sensorCMD.raw_result[2] << 16), ((sensorCMD.raw_result[3] << 8) >> 12));
}

int adxl357_read_fifo(ADXL357_HANDLER * handler, ADXL357Fifo * fifo, uint8_t entries_count)
{
  ADXL357Command sensorCMD = {};
  sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_FIFO_DATA);

  uint8_t remainder;
  remainder = entries_count % 3;

  if (remainder > 0)
  {
    entries_count -= remainder;
  }

  if (entries_count == 0){
    fifo->samples = 0;
    return ADXL357_OK;
  } else if (entries_count > ADXL357_FIFO_MAX_COUNT){
    return ADXL357_ERROR;
  }

  // Each entries_count has 3 bytes of data.
  // Plus one byte for register address.
  sensorCMD.len = 1 + entries_count*3;

  adxl357_execute_command(handler, &sensorCMD);

  uint8_t fifo_samples = entries_count / 3;
  uint8_t x_axis_marker = 0;
  uint8_t empty_indicator = 0;

  for (uint8_t i=0; i<fifo_samples; i++){
    x_axis_marker = sensorCMD.raw_result[1+(i*9)+2] & 0x01;
    empty_indicator = sensorCMD.raw_result[1+(i*9)+2] >> 1 & 0x01;

    //printf("X_M, F_X_M, E_M, F_E_M: %i\t%i\t%i\t%i\n",x_axis_marker,fifo->x_marker_error,empty_indicator,fifo->empty_read);
    if (empty_indicator & !fifo->empty_read) {
      fifo->empty_read = 1;
      //printf("empty girildi\n");
      fifo->empty_read_index = i;
    }

    if (!x_axis_marker & !fifo->x_marker_error){
      fifo->x_marker_error = 1;
      fifo->x_marker_error_index = i;
    }

    // Aşağıdan aldım:
    // https://github.com/Seeed-Studio/Seeed_ADXL357B/blob/master/Seeed_adxl357b.cpp
	fifo->data[i].x = ((sensorCMD.raw_result[1+(i*9)] << 12) + (sensorCMD.raw_result[1+(i*9)+1] << 4) + (sensorCMD.raw_result[1+(i*9)+2] >> 4));
	fifo->data[i].y = ((sensorCMD.raw_result[1+(i*9)+3] << 12) + (sensorCMD.raw_result[1+(i*9)+4] << 4) + (sensorCMD.raw_result[1+(i*9)+5] >> 4));
	fifo->data[i].z = ((sensorCMD.raw_result[1+(i*9)+6] << 12) + (sensorCMD.raw_result[1+(i*9)+7] << 4) + (sensorCMD.raw_result[1+(i*9)+8] >> 4));

	if (fifo->data[i].x & 0x80000) {
		fifo->data[i].x = (fifo->data[i].x & 0x7ffff) - 0x80000;
	}
	if (fifo->data[i].y & 0x80000) {
		fifo->data[i].y = (fifo->data[i].y & 0x7ffff) - 0x80000;
	}
	if (fifo->data[i].z & 0x80000) {
		fifo->data[i].z = (fifo->data[i].z & 0x7ffff) - 0x80000;
	}

  }

  fifo->samples = fifo_samples;

  if (fifo->empty_read | fifo->x_marker_error){
    return ADXL357_FIFO_READ_ERROR;
  }

  return ADXL357_OK;
}

void adxl357_empty_fifo(ADXL357_HANDLER * handler, uint8_t readCount)
{
  ADXL357Command sensorCMD = {};
  sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_FIFO_DATA);

  //uint8_t  entries_count = 96;
  uint8_t remainder;
  remainder = readCount % 3;

  if (remainder > 0)
  {
    readCount -= remainder;
  }


  //printf("read Count: %d\n", readCount);

  // Each entries_count has 3 bytes of data.
  // Plus one byte for register address.
  if(readCount > 2)
  {
    sensorCMD.len = 1 + readCount*3;
    adxl357_execute_command(handler, &sensorCMD);
  }
}

int adxl357_fifo_coldstart(ADXL357_HANDLER * handler, int sampleHigh, uint8_t flags )
{
  int res = 0;
  int i;

  ADXL357StatusAndFifo status = {};
  ADXL357Fifo fifo = {};

  // Once FIFO'nun icini bosaltalim.
  adxl357_get_status_n_fifo(handler, &status);
  //printf("Entries Before Empty: %d - %d\n", status.fifo_entries, status.status.FIFO_OVR);
  adxl357_empty_fifo(handler, status.fifo_entries);


  for (i=0; i<sampleHigh;){
    adxl357_get_status_n_fifo(handler, &status);

    if (status.status.FIFO_OVR){
      if (flags & FIFO_STREAM_OVR_BREAK){
        return ADXL357_FIFO_STREAM_ERROR;
      }
    }

    res = adxl357_read_fifo(handler, &fifo, status.fifo_entries);

    if (res < 0){
      if (flags & FIFO_STREAM_FIFO_READ_BREAK) {
          return ADXL357_FIFO_READ_ERROR;
      }
    }
  }

  //printf("Toplam Data2: %d\n",i);
  return 0;
}

int adxl357_fifo_bucket(ADXL357_HANDLER * handler, int sampleHigh, float *ax, float *ay, float *az, uint8_t flags )
{
  int res = 0;
  int i;
  uint8_t samples_count = 0;

  ADXL357StatusAndFifo status = {};
  ADXL357Fifo fifo = {};

  //printf("F_X_M, F_E_M: %i\t%i\n",fifo.x_marker_error,fifo.empty_read);

  //printf("2\n");
  //int startBucket = 0;

  // Once FIFO'nun icini bosaltalim.
  adxl357_get_status_n_fifo(handler, &status);
  //printf("Entries Before Empty: %d - %d\n", status.fifo_entries, status.status.FIFO_OVR);
  adxl357_empty_fifo(handler, status.fifo_entries);
  //adxl357_get_status_n_fifo(handler, &status);
  //printf("Entries After Empty: %d - %d\n", status.fifo_entries, status.status.FIFO_OVR);
  //usleep(100);
  //printf("sampleHigh: %d\n", sampleHigh);

  for (i=0; i<sampleHigh;){
    adxl357_get_status_n_fifo(handler, &status);

    //printf("fifoNumbers: %d\n", status.fifo_entries);

    if (status.status.FIFO_OVR){
      if (flags & FIFO_STREAM_OVR_BREAK){
    	//ESP_LOGI(TAG, "FIFO OVERFLOW!");
        //printf("ERRRRROOORRR FIFO OVERFLOWW! \n");
        //sendLog("OVERFLOW ERROR");
        return ADXL357_FIFO_STREAM_ERROR;
      }
    }

    /*
    if (startBucket == 1)
    {
      printf("11111\n");
    }
    * */
    /*
    if(status.fifo_entries > 10)
    {
      printf("Start: %d FIFO Entries: %d\n", startBucket, status.fifo_entries);
    }
    * */

    //startBucket = startBucket + 1;

    // Each sample consists of 3 fifo entries: X, Y, Z
    samples_count = status.fifo_entries / 3;

    res = adxl357_read_fifo(handler, &fifo, status.fifo_entries);

    // FATIH DIKKAT DIKKAT: Yukarıdan ADXL357_FIFO_READ_ERROR gelirse ona bakmıyor. Bunu da dikkate almak gerekiyor.
    // HATTA BAŞKA ERROR ŞEKİLLERİ DE VAR. BUNLARA İYİCE BAK ARTIK...
    if (res < 0){
      if (flags & FIFO_STREAM_FIFO_READ_BREAK) {
          //printf("Ex_marker_error %d\n", fifo.x_marker_error);
          //sendLog("MARKER ERROR");
          //printf("empty_read %d\n", fifo.empty_read);
          return ADXL357_FIFO_READ_ERROR;
      }
    }

    // FATIH DIKKAT DIKKAT: Bu kısımda *(ax + i) şeklinde mi yazmam gerekir?
    if (status.fifo_entries > 0) {
        for (int j=0; j < samples_count; j++){
          *(ax+i+j) = (float)(fifo.data[j].x * accConv);
          *(ay+i+j) = (float)(fifo.data[j].y * accConv);
          *(az+i+j) = (float)(fifo.data[j].z * accConv);

          if ((i+j) == (sampleHigh-1)){
        	  //printf("Toplam Data1: %d\n",(i+j));
        	  //ESP_LOGI(TAG, "BUCKETSIZE:%i",i);
            return 0;
          }
        }
        i = i + samples_count;
    }
    if((extReq == 1) && (eFs == 125))
		vTaskDelay(100/portTICK_RATE_MS);

	if((extReq == 1) && (eFs == 250))
		vTaskDelay(50/portTICK_RATE_MS);

	if((extReq == 1) && (eFs == 500))
		vTaskDelay(30/portTICK_RATE_MS);

  }

  //printf("Toplam Data2: %d\n",i);
  return 0;
}

int adxl357_fifo_stream(ADXL357_HANDLER * handler, void (* callback)(ADXL357Fifo * fifo), uint8_t flags)
{
  int res = 0;

  ADXL357StatusAndFifo status = {};
  ADXL357Fifo fifo = {};

  while (1){
    adxl357_get_status_n_fifo(handler, &status);

    //printf("1 \n");

    if (status.status.FIFO_OVR){
      if (flags & FIFO_STREAM_OVR_BREAK){
    	  ESP_LOGI(TAG, "FIFO OVERFLOW!");
    	  //printf("FIFO OVERFLOW! \n");
    	  return ADXL357_FIFO_STREAM_ERROR;
      }
    }


    // Each sample consists of 3 fifo entries: X, Y, Z
    //int samples_count = status.fifo_entries / 3;

    res = adxl357_read_fifo(handler, &fifo, status.fifo_entries);
    if (res < 0){
      if (flags & FIFO_STREAM_FIFO_READ_BREAK) {
    	  ESP_LOGI(TAG, "x_marker_error %d\n", fifo.x_marker_error);
    	  ESP_LOGI(TAG, "empty_read %d\n", fifo.empty_read);
    	  //printf("x_marker_error %d\n", fifo.x_marker_error);
    	  //printf("empty_read %d\n", fifo.empty_read);
    	  return ADXL357_FIFO_STREAM_ERROR;
      }
    }

    if (fifo.samples > 0){
      callback(&fifo);
    }
  }
  return 0;
}

int adxl357_measurement_mode(ADXL357_HANDLER * handler)
{
	uint8_t power_ctl = 0;
	adxl357_get_power_ctl(handler, &power_ctl);

	ADXL357Command sensorCMD = {};
	sensorCMD.reg = ADXL357_REG_WRITE(ADXL357_REG_POWER_CTL);
	sensorCMD.data[0] = power_ctl & 0xFE;

	adxl357_execute_command(handler, &sensorCMD);
	ESP_LOGI(TAG, "MEASUREMENT MODE!");
	vTaskDelay(1000/portTICK_RATE_MS);
	return 0;
}

int adxl357_standby_mode(ADXL357_HANDLER * handler)
{
  uint8_t power_ctl = 0;
  adxl357_get_power_ctl(handler, &power_ctl);

  ADXL357Command sensorCMD = {};
  sensorCMD.reg = ADXL357_REG_WRITE(ADXL357_REG_POWER_CTL);
  sensorCMD.data[0] = power_ctl | 0x01;

  adxl357_execute_command(handler, &sensorCMD);
  ESP_LOGI(TAG, "STAND_BY MODE!");
  vTaskDelay(1000/portTICK_RATE_MS);
  return 0;
}

int adxl357_set_range(ADXL357_HANDLER * handler, int setRange)
{
	ADXL357Command sensorCMD = {};
	sensorCMD.reg = ADXL357_REG_WRITE(ADXL357_REG_RANGE);

	if (setRange==10)
	{
		ESP_LOGI(TAG, "Will set range to: %i\n",setRange);
		//printf("Will set range to: %i\n",setRange*10);
		sensorCMD.data[0] = 0x81;
		compareRange = 0x81;
	}
	else if (setRange==20)
		{
		ESP_LOGI(TAG, "Will set range to: %i\n",setRange);
		//printf("Will set range to: %i\n",setRange*10);
		sensorCMD.data[0] = 0x82;
		compareRange = 0x82;
	}
	else if (setRange==40)
	{
		ESP_LOGI(TAG, "Will set range to: %i\n",setRange);
		//printf("Will set range to: %i\n",setRange*10);
		sensorCMD.data[0] = 0x83;
		compareRange = 0x83;
	}
	else
	{
		ESP_LOGI(TAG, "Range is invalid \n");
		//printf("Range is invalid \n");
	}

	int res = adxl357_execute_command(handler, &sensorCMD);
	if (res == ADXL357_ERROR)
	{
	  ESP_LOGI(TAG, "Range Conversion Problem\n");
	  sendJsonLog(902, "L");
	  //printf("Range Conversion Problem\n");
	}

	if ((setRange==10) && (res != ADXL357_ERROR))
	{
		accConv = baseConv;
		ESP_LOGI(TAG, "Range Conversion Changed for 10\n");
	}
	else if ((setRange==20) && (res != ADXL357_ERROR))
		{
		accConv = baseConv*2.0;
		ESP_LOGI(TAG, "Range Conversion Changed for 20\n");
	}
	else if ((setRange==40) && (res != ADXL357_ERROR))
	{
		accConv = baseConv*4.0;
		ESP_LOGI(TAG, "Range Conversion Changed for 40\n");
	}

	sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_RANGE);
	adxl357_execute_command(handler, &sensorCMD);
	vTaskDelay(1000/portTICK_RATE_MS);
	printf("Range After Setting:  %02X \n", sensorCMD.raw_result[1]);
	return 0;
}

int share_sensor_settings(ADXL357_HANDLER * handler)
{
	ADXL357Command sensorCMD = {};
	sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_RANGE);
	adxl357_execute_command(handler, &sensorCMD);
	vTaskDelay(1000/portTICK_RATE_MS);

	cJSON *mainJasonRMS;

	mainJasonRMS = cJSON_CreateObject();
	cJSON_AddNumberToObject(mainJasonRMS, "range", sensorCMD.raw_result[1]);

	char *jsonToPayloadRMS;
	jsonToPayloadRMS = cJSON_Print(mainJasonRMS);
	esp_mqtt_client_publish(client, TOPICPUBLOG, jsonToPayloadRMS, 0, 0, 0);

	cJSON_Delete(mainJasonRMS);
	free(jsonToPayloadRMS);
	return 0;
}

int adxl357_check_range(ADXL357_HANDLER * handler, int setRange)
{

	ADXL357Command sensorCMD = {};
	sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_RANGE);

	adxl357_execute_command(handler, &sensorCMD);
	vTaskDelay(1000/portTICK_RATE_MS);
	//printf("Current Range:  %02X \n", sensorCMD.raw_result[1]);

	//printf("Compare Range:  %X \n", compareRange);

	if((int)sensorCMD.raw_result[1] == compareRange)
	{
		//ESP_LOGI(TAG, "SENSOR RANGE IS OK\n");
		return 1;
	}
	else
	{
		//ESP_LOGI(TAG, "Range Conversion Problem\n");
		return 0;
	}

}

int adxl357_set_filter(ADXL357_HANDLER * handler, int setFilter)
{

  ADXL357Command sensorCMD = {};
  sensorCMD.reg = ADXL357_REG_WRITE(ADXL357_REG_FILTER);

  if (setFilter==4000)
  {
      printf("Will set lpf to: %d\n",setFilter);
      sensorCMD.data[0] = SET_FILTER_4000;
      compareFilter = SET_FILTER_4000;
  }
  else if (setFilter==2000)
  {
      printf("Will set lpf to: %d\n",setFilter);
      sensorCMD.data[0] = SET_FILTER_2000;
      compareFilter = SET_FILTER_2000;
  }
  else if (setFilter==1000)
  {
      printf("Will set lpf to: %d\n",setFilter);
      sensorCMD.data[0] = SET_FILTER_1000;
      compareFilter = SET_FILTER_1000;
  }
  else if (setFilter==500)
  {
      printf("Will set lpf to: %d\n",setFilter);
      sensorCMD.data[0] = SET_FILTER_500;
      compareFilter = SET_FILTER_500;
  }
  else if (setFilter==250)
  {
      printf("Will set lpf to: %d\n",setFilter);
      sensorCMD.data[0] = SET_FILTER_250;
      compareFilter = SET_FILTER_250;
  }
  else if (setFilter==125)
  {
      printf("Will set lpf to: %d\n",setFilter);
      sensorCMD.data[0] = SET_FILTER_125;
      compareFilter = SET_FILTER_125;
  }
  else
  {
      printf("LPF value is invalid \n");
  }

  int res = adxl357_execute_command(handler, &sensorCMD);
  if (res == ADXL357_ERROR)
  {
    printf("LPF Value Conversion Problem\n");
    sendJsonLog(903, "L");
  }

  sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_FILTER);
  adxl357_execute_command(handler, &sensorCMD);
  vTaskDelay(1000/portTICK_RATE_MS);
  printf("LPF Value After Setting:  %X\n", sensorCMD.raw_result[1]);
  return 0;
}

int adxl357_check_filter(ADXL357_HANDLER * handler, int setFilter)
{

  ADXL357Command sensorCMD = {};
  sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_FILTER);
  adxl357_execute_command(handler, &sensorCMD);
  vTaskDelay(1000/portTICK_RATE_MS);

  //printf("LPF Value After Check:  %X\n", sensorCMD.raw_result[1]);

  //printf("Compare Filter:  %X \n", compareFilter);

  	if((int)sensorCMD.raw_result[1] == compareFilter)
  	{
  		//ESP_LOGI(TAG, "SENSOR RANGE IS OK\n");
  		return 1;
  	}
  	else
  	{
  		//ESP_LOGI(TAG, "Range Conversion Problem\n");
  		return 0;
  	}
}

int adxl357_get_power_ctl(ADXL357_HANDLER * handler, uint8_t * b)
{
  ADXL357Command sensorCMD = {};
  sensorCMD.reg = ADXL357_REG_READ(ADXL357_REG_POWER_CTL);

  adxl357_execute_command(handler, &sensorCMD);

  *b = sensorCMD.raw_result[1];
  vTaskDelay(300/portTICK_RATE_MS);
  return 0;
}

int adxl357_reset(ADXL357_HANDLER * handler)
{
  ADXL357Command sensorCMD = {};
  sensorCMD.reg = ADXL357_REG_WRITE(ADXL357_REG_RESET);
  sensorCMD.data[0] = ADXL357_RESET_CODE;

  adxl357_execute_command(handler, &sensorCMD);
  vTaskDelay(1000/portTICK_RATE_MS);
  return 0;
}




/// ESP RELATED
void getMacAddress()
{
	uint8_t baseMac[6];
	// Get MAC address for WiFi station
	esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
	//char baseMacChr[18] = {0};
	sprintf(macID, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
	//sprintf(macID,baseMacChr);
	//printf("mac: %s\n", baseMacChr);
}



// NVS FLASH READ AND WRITE RELATED
esp_err_t save_wifi_data(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    int ssidSize = sizeof(ssidData);
	int pswdSize = sizeof(pswdData);

    uint8_t* ssidDataInt = malloc(ssidSize * sizeof(uint8_t));
    uint8_t* pswdDataInt = malloc(pswdSize * sizeof(uint8_t));

    for (int i = 0; i < ssidSize; i++) {
		ssidDataInt[i] = (uint8_t)(ssidData[i]);
		//printf("%i: %i\n", i, ssidDataInt[i]);
	}

    for (int i = 0; i < pswdSize; i++) {
		pswdDataInt[i] = (uint8_t)(pswdData[i]);
		//printf("%i: %i\n", i, pswdDataInt[i]);
	}

    err = nvs_set_blob(my_handle, "ssid", ssidDataInt, ssidSize);
    err = nvs_set_blob(my_handle, "pswd", pswdDataInt, pswdSize);
    free(ssidDataInt);
    free(pswdDataInt);

    if (err != ESP_OK) return err;

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);

    ESP_LOGI(TAG, "Saved New SSID and PWSD");

    return ESP_OK;
}

esp_err_t get_saved_wifi(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read run time blob
    size_t requiredSizeS = 0;  // value will default to 0, if not set yet in NVS
    size_t requiredSizeP = 0;  // value will default to 0, if not set yet in NVS

    err = nvs_get_blob(my_handle, "ssid", NULL, &requiredSizeS);
    nvs_get_blob(my_handle, "pswd", NULL, &requiredSizeP);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    ESP_LOGI(TAG, "read required size: %i and %i\n", requiredSizeS, requiredSizeP);
    ESP_LOGI(TAG, "ssid and pswd:\n");
    //printf("read required size: %i and %i\n", requiredSizeS, requiredSizeP);
    //printf("ssid and pswd:\n");
    if (requiredSizeS == 0) {
    	ESP_LOGI(TAG, "Nothing saved yet!\n");
        //printf("Nothing saved yet!\n");
    } else {
        uint8_t* ssid_data = malloc(requiredSizeS);
        uint8_t* pswd_data = malloc(requiredSizeP);
        err = nvs_get_blob(my_handle, "ssid", ssid_data, &requiredSizeS);
        nvs_get_blob(my_handle, "pswd", pswd_data, &requiredSizeP);
        if (err != ESP_OK) {
            free(ssid_data);
            free(pswd_data);
            return err;
        }
        ESP_LOGI(TAG, "SSID and PWD from Flash: %s and %s\n", ssid_data, pswd_data);
        /*
        for (int i = 0; i < requiredSizeS / sizeof(uint8_t); i++) {
            //printf("%i: %i\n", i + 1, ssid_data[i]);
            //printf("%c", ssid_data[i]);
            ESP_LOGI(TAG, "%c", ssid_data[i]);
        }
        printf("\n");
        for (int i = 0; i < requiredSizeP / sizeof(uint8_t); i++) {
			//printf("%i: %i", i + 1, pswd_data[i]);
			//printf("%c", pswd_data[i]);
			ESP_LOGI(TAG, "%c", pswd_data[i]);
		}
		*/
        //printf("\n");

        strcpy(ssidData,(char*)ssid_data);
		strcpy(pswdData,(char*)pswd_data);

        free(ssid_data);
        free(pswd_data);
    }

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t save_mqtt_data_host(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    int mqttSize = sizeof(mqttBrokerUrl);

    uint8_t* mqttInt = malloc(mqttSize * sizeof(uint8_t));

    for (int i = 0; i < mqttSize; i++) {
    	mqttInt[i] = (uint8_t)(mqttBrokerUrl[i]);
		//printf("%i: %i\n", i, mqttInt[i]);
	}

    err = nvs_set_blob(my_handle, "mqttH", mqttInt, mqttSize);
    free(mqttInt);

    if (err != ESP_OK) return err;

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t get_saved_mqtt_host(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read run time blob
    size_t requiredSizeH = 0;  // value will default to 0, if not set yet in NVS

    err = nvs_get_blob(my_handle, "mqttH", NULL, &requiredSizeH);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    ESP_LOGI(TAG, "read required size: %i\n", requiredSizeH);
    //ESP_LOGI(TAG, "mqttHost, User:\n");
    if (requiredSizeH == 0) {
    	ESP_LOGI(TAG, "Nothing saved yet!\n");
        //printf("Nothing saved yet!\n");
    } else {
        uint8_t* mqtt_data = malloc(requiredSizeH);
        err = nvs_get_blob(my_handle, "mqttH", mqtt_data, &requiredSizeH);
        if (err != ESP_OK) {
            free(mqtt_data);
            return err;
        }
        strcpy(mqttBrokerUrl,(char*)mqtt_data);
        UpdateMQTTBrookerUrl(mqttBrokerUrl, strlen(mqttBrokerUrl));

        free(mqtt_data);
    }

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t save_mqtt_data_user(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

	int userSize = sizeof(mqttUsername);

    uint8_t* userInt = malloc(userSize * sizeof(uint8_t));

    for (int i = 0; i < userSize; i++) {
    	userInt[i] = (uint8_t)(mqttUsername[i]);
		//printf("%i: %i\n", i, userInt[i]);
	}

    err = nvs_set_blob(my_handle, "mqttU", userInt, userSize);
    free(userInt);

    if (err != ESP_OK) return err;

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t get_saved_mqtt_user(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read run time blob
    size_t requiredSizeU = 0;  // value will default to 0, if not set yet in NVS

    nvs_get_blob(my_handle, "mqttU", NULL, &requiredSizeU);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    ESP_LOGI(TAG, "read required size: %i\n", requiredSizeU);
    //ESP_LOGI(TAG, "mqttHost, User:\n");
    if (requiredSizeU == 0) {
    	ESP_LOGI(TAG, "Nothing saved yet!\n");
        //printf("Nothing saved yet!\n");
    } else {
        uint8_t* mqusr_data = malloc(requiredSizeU);
        nvs_get_blob(my_handle, "mqttU", mqusr_data, &requiredSizeU);
        if (err != ESP_OK) {
            free(mqusr_data);
            return err;
        }
		strcpy(mqttUsername,(char*)mqusr_data);
    UpdateMQTTUserID(mqttUsername, strlen(mqttUsername));

        free(mqusr_data);
    }

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t get_save_range_data(int saveData)
{
    printf("Opening Non-Volatile Storage (NVS) handle for Range... ");

	nvs_handle_t my_handle;
	esp_err_t err;
	int32_t dummyRange = 10; // value will default to 0, if not set yet in NVS

	err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
	if (err != ESP_OK) {
		printf("Error (%s) opening NVS handle for Range!\n", esp_err_to_name(err));
	}
	else
	{
		printf("Opened NVS for Range\n");

		if(saveData == 1)
		{
			// Write
			printf("Updating sensor range in NVS ... ");
			dummyRange = sensorRange;
			err = nvs_set_i32(my_handle, "range", dummyRange);
			printf((err != ESP_OK) ? "Range Set Failed!\n" : "Range Set Done\n");

			// Commit written value.
			// After setting any values, nvs_commit() must be called to ensure changes are written
			// to flash storage. Implementations may write to storage at other times,
			// but this is not guaranteed.
			printf("Committing range updates in NVS ... ");
			err = nvs_commit(my_handle);
			printf((err != ESP_OK) ? "Range Update Failed!\n" : "Range Update Done\n");
		}
		else
		{
			// Read
			printf("Reading range from NVS ... ");
			err = nvs_get_i32(my_handle, "range", &dummyRange);
			switch (err) {
				case ESP_OK:
					printf("Read Range from NVS = %d\n", dummyRange);
					sensorRange = dummyRange;
					break;
				case ESP_ERR_NVS_NOT_FOUND:
					printf("The range value is not initialized yet!\n");
					break;
				default :
					printf("Error (%s) range reading!\n", esp_err_to_name(err));
			}
		}
		// Close
		nvs_close(my_handle);
	}
	return ESP_OK;
}

esp_err_t get_save_vloop_data(int saveData)
{
    printf("Opening Non-Volatile Storage (NVS) handle for VLoop... ");

	nvs_handle_t my_handle;
	esp_err_t err;
	int32_t dummyVLoop = 1; // value will default to 0, if not set yet in NVS

	err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
	if (err != ESP_OK) {
		printf("Error (%s) opening NVS handle for Vloop!\n", esp_err_to_name(err));
	} else
	{
		printf("Opened NVS for Vloop\n");

		if(saveData == 1)
		{
			// Write
			printf("Updating Velocity Loop in NVS ... ");
			dummyVLoop = vLoop;
			err = nvs_set_i32(my_handle, "vloop", dummyVLoop);
			printf((err != ESP_OK) ? "Vloop Set Failed!\n" : "Vloop Set Done\n");

			// Commit written value.
			// After setting any values, nvs_commit() must be called to ensure changes are written
			// to flash storage. Implementations may write to storage at other times,
			// but this is not guaranteed.
			printf("Committing Vloop updates in NVS ... ");
			err = nvs_commit(my_handle);
			printf((err != ESP_OK) ? "Vloop Update Failed!\n" : "Vloop Update Done\n");
		}
		else
		{
			// Read
			printf("Reading Vloop from NVS ... ");
			err = nvs_get_i32(my_handle, "vloop", &dummyVLoop);
			switch (err) {
				case ESP_OK:
					printf("Read Vloop from NVS = %d\n", dummyVLoop);
					vLoop = dummyVLoop;
					break;
				case ESP_ERR_NVS_NOT_FOUND:
					printf("The Vloop value is not initialized yet!\n");
					break;
				default :
					printf("Error (%s) Vloop reading!\n", esp_err_to_name(err));
			}
		}
		// Close
		nvs_close(my_handle);
	}
	return ESP_OK;
}

esp_err_t get_save_rloop_data(int saveData)
{
    printf("Opening Non-Volatile Storage (NVS) handle for RLoop... ");

	nvs_handle_t my_handle;
	esp_err_t err;
	int32_t dummyRLoop = 10; // value will default to 10, if not set yet in NVS

	err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
	if (err != ESP_OK) {
		printf("Error (%s) opening NVS handle for Rloop!\n", esp_err_to_name(err));
	} else
	{
		printf("Opened NVS for Rloop\n");

		if(saveData == 1)
		{
			// Write
			printf("Updating rms loop in NVS ... ");
			dummyRLoop = rmsLoop;
			err = nvs_set_i32(my_handle, "rloop", dummyRLoop);
			printf((err != ESP_OK) ? "Rloop Set Failed!\n" : "Rloop Set Done\n");

			// Commit written value.
			// After setting any values, nvs_commit() must be called to ensure changes are written
			// to flash storage. Implementations may write to storage at other times,
			// but this is not guaranteed.
			printf("Committing Rloop updates in NVS ... ");
			err = nvs_commit(my_handle);
			printf((err != ESP_OK) ? "Rloop Update Failed!\n" : "Rloop Update Done\n");
		}
		else
		{
			// Read
			printf("Reading Rloop from NVS ... ");
			err = nvs_get_i32(my_handle, "rloop", &dummyRLoop);
			switch (err) {
				case ESP_OK:
					printf("Read rloop from NVS = %d\n", dummyRLoop);
					rmsLoop = dummyRLoop;
					break;
				case ESP_ERR_NVS_NOT_FOUND:
					printf("The Rloop value is not initialized yet!\n");
					break;
				default :
					printf("Error (%s) Rloop reading!\n", esp_err_to_name(err));
			}
		}
		// Close
		nvs_close(my_handle);
	}
	return ESP_OK;
}

esp_err_t get_save_cr_data(int saveData)
{

	return ESP_OK;
}




// MQTT RELATED
esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client_F = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            mqttConC = 0;
            mqttConnectFlag = 1;
            //msg_id = esp_mqtt_client_subscribe(client_F, "/topic/qos0", 0);
            //ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client_F, TOPICSUB_ATT, 0);
            //sendJsonLog("SYSTEM STARTED", "L");
			ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
			sendJsonLog(100, "L");
			msg_id = esp_mqtt_client_subscribe(client_F, TOPICSUB_REQ, 0);

            //msg_id = esp_mqtt_client_unsubscribe(client_F, "/topic/qos1");
            //ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            mqttConnectFlag = 0;
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            //msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
            //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            //ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);

            cJSON *rootMesaj = cJSON_Parse(event->data);
            char idFromMessage[15];
      			int idComparison = 1;

      			//char idFromMessage = cJSON_GetObjectItem(rootMesaj,"me")->valuestring;
      			//printf("idFromMessage: %s\n", idFromMessage);
      			if(cJSON_GetObjectItem(rootMesaj,"method") != NULL)
      			{
      				//printf("idFromMessage: %s\n", cJSON_GetObjectItem(rootMesaj,"me")->valuestring);
      				sprintf(idFromMessage,"%s", cJSON_GetObjectItem(rootMesaj,"method")->valuestring);
      				//printf("idFromMessage: %s a\n", idFromMessage);
      				idComparison = strcmp(idFromMessage, "rpcCommand");	//gives 0 if they are equal. gives some number if they are very different.
      				printf("MESSAGE ID COMPARISON: %d\n", idComparison);
      			}

      			//sendJsonLog(110, "L");

      			if(cJSON_GetObjectItem(rootMesaj,"vLoop") != NULL)
      			{
      				int dummyVLoop;
      				dummyVLoop = cJSON_GetObjectItem(rootMesaj,"vLoop")->valueint;
      				printf("VELOCITY LOOP FROM MESSAGE: %i\n", dummyVLoop);
      				if(dummyVLoop > 19 && dummyVLoop < 43200)
      				{

      					if(dummyVLoop == vLoop)
      					{
      						printf("NO CHANGE ON RANGE: %i\n", dummyVLoop);
      					}
      					else
      					{
      						printf("NEW VELOCITY LOOP: %i\n", dummyVLoop);
      						vLoop = dummyVLoop;
      						get_save_vloop_data(1);
      					}
      				}
      				else
      				{
      					printf("WRONG VELOCITY LOOP ON MESSAGE\n");
      				}
      			}

      			if(cJSON_GetObjectItem(rootMesaj,"rLoop") != NULL)
      			{
      				int dummyRLoop;
      				dummyRLoop = cJSON_GetObjectItem(rootMesaj,"rLoop")->valueint;
      				printf("RMS LOOP FROM MESSAGE: %i\n", dummyRLoop);
      				if(dummyRLoop > 19 && dummyRLoop < 43200)
      				{

      					if(dummyRLoop == rmsLoop)
      					{
      						printf("NO CHANGE ON RMS LOOP: %i\n", dummyRLoop);
      					}
      					else
      					{
      						printf("NEW VELOCITY RMS LOOP: %i\n", dummyRLoop);
      						rmsLoop = dummyRLoop;
      						get_save_rloop_data(1);
      					}
      				}
      				else
      				{
      					printf("WRONG RMS LOOP ON MESSAGE\n");
      				}
      			}

      			if(cJSON_GetObjectItem(rootMesaj,"gRange") != NULL)
      			{
      				int dummyRange;
      				dummyRange = cJSON_GetObjectItem(rootMesaj,"gRange")->valueint;
      				printf("RANGE FROM MESSAGE: %i\n", dummyRange);
      				if(dummyRange == 10 || dummyRange == 20 || dummyRange == 40)
      				{

      					if(dummyRange == sensorRange)
      					{
      						printf("NO CHANGE ON RANGE: %i\n", dummyRange);
      					}
      					else
      					{
      						printf("NEW RANGE: %i\n", dummyRange);
      						sensorRange = dummyRange;
      						get_save_range_data(1);
      						gotMessage = 1;
                  NotifySensorReconfiguration(MQTT_RECV_MSG_HANDLER_CELLULAR_TASK);
      					}
      				}
      				else
      				{
      					printf("WRONG RANGE ON MESSAGE\n");
      				}
      			}

      			if(cJSON_GetObjectItem(rootMesaj,"eTime") != NULL)
      			{
      				int dummyTime;
      				dummyTime = cJSON_GetObjectItem(rootMesaj,"eTime")->valueint;
      				printf("TIME FROM MESSAGE: %i\n", dummyTime);
      				if(dummyTime > 0 && dummyTime < 65)
      				{
      					printf("NEW TIME: %i\n", dummyTime);
      					eTime = dummyTime;
      				}
      				else
      				{
      					printf("WRONG TIME ON MESSAGE\n");
      				}

      			}

      			if(cJSON_GetObjectItem(rootMesaj,"eRate") != NULL)
      			{
      				int dummyRate;
      				dummyRate = cJSON_GetObjectItem(rootMesaj,"eRate")->valueint;
      				printf("SAMPLING RATE FROM MESSAGE: %i\n", dummyRate);
      				if(dummyRate == 125 || dummyRate == 250 || dummyRate == 500 || dummyRate == 1000 || dummyRate == 2000 || dummyRate == 4000)
      				{
      					printf("NEW RATE: %i\n", dummyRate);
      					eFs = dummyRate;
      				}
      				else
      				{
      					printf("WRONG RATE ON MESSAGE\n");
      				}
      			}

      			if(cJSON_GetObjectItem(rootMesaj,"eStart") != NULL)
      			{
      				int dummyExt;
      				dummyExt = cJSON_GetObjectItem(rootMesaj,"eStart")->valueint;
      				printf("EXTREME BOOL FROM MESSAGE: %i\n", dummyExt);
      				if(dummyExt == 1)
      				{
      					printf("START EXTREME: %i\n", dummyExt);
      					extReq = 1;
                RequestExtremeSensorData(MQTT_RECV_MSG_HANDLER_CELLULAR_TASK);
      				}
      				else
      				{
      					printf("NO EXTREME WANTED\n");
      				}
      			}

      			if(cJSON_GetObjectItem(rootMesaj,"upHost") != NULL)
      			{
      				char upDummyURL[100];
      				sprintf(upDummyURL,"%s", cJSON_GetObjectItem(rootMesaj,"upHost")->valuestring);
      				printf("UPDATE URL FROM MESSAGE: %s\n", upDummyURL);
      				ESP_LOGI(TAG, "UPDATE URL FROM MESSAGE: %s",upDummyURL);
      				ESP_LOGI(TAG, "BEFORE STR CPY: %s",updateURL);
      				strcpy(updateURL, upDummyURL);
      				ESP_LOGI(TAG, "AFTER STR CPY: %s",updateURL);
      			}

      			if(cJSON_GetObjectItem(rootMesaj,"reStart") != NULL)
      			{
      				int dummyRestart;
      				dummyRestart = cJSON_GetObjectItem(rootMesaj,"reStart")->valueint;
      				printf("RESTART FROM MESSAGE: %i\n", dummyRestart);
      				if(dummyRestart == 1)
      				{
      					esp_restart();

      				}
      				else
      				{
      					printf("RESTART MESSAGE PROBLEM\n");
      				}
      			}

      			if(cJSON_GetObjectItem(rootMesaj,"updateVer") != NULL)
      			{
      				int dummyExt;
      				dummyExt = cJSON_GetObjectItem(rootMesaj,"updateVer")->valueint;
      				printf("EXTREME BOOL FROM MESSAGE: %i\n", dummyExt);
      				if(dummyExt == 1)
      				{
      					printf("UPDATE VERSION: %i\n", dummyExt);
      					//updateVersion = 1;
      				}
      				else
      				{
      					printf("NO UPDATE VERSION\n");
      				}
      			}

      			cJSON_Delete(rootMesaj);

            //strcpy(mesajOrg,event->data);
            //printf("mesajOrg print: %s\n", mesajOrg);
            //messageParseJSON();
            //esp_mqtt_client_publish(client, TOPICPUBLOG, " GOT MESSAGE", 0, 0, 0);

            break;
        case MQTT_EVENT_ERROR:
        	mqttConC++;		// mqtt'ye bağlanamazsa bu count'u teker teker arttıracak.
        	mqttConnectFlag = 0;
        	ESP_LOGI(TAG, "MQTT CONNECT LOOP:%d", mqttConC);
        	if (mqttConC > mqttConL)
        	{
        		ESP_LOGI(TAG, "MQTT CONNECT LOOP OVER MAX");
        		esp_restart();
        	}
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}






// HTTP_OTA
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

void ota_task()
{
    ESP_LOGI(TAG, "Starting OTA example");

    esp_http_client_config_t config = {
        .url = updateURL,
		.event_handler = _http_event_handler,
        //.cert_pem = (char *)server_cert_pem_start,
    };

    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware upgrade failed");
        sendJsonLog(701, "L");
    }
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}








// Wifi RELATED
void wifi_ap_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",MAC2STR(event->mac), event->aid);
        somebodyInside = 1;
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",MAC2STR(event->mac), event->aid);
    }
}

void wifi_sta_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
      ESP_LOGI(TAG, "WIFI CONNECT");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifiRetryCount < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            wifiRetryCount++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        sprintf( deviceIP, "%i.%i.%i.%i", IP2STR(&event->ip_info.ip));
        //ESP_LOGI(TAG,"MY IP IS: %s\n", deviceIP);
        wifiRetryCount = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t thankYou_get_handler(httpd_req_t *req)
{
	httpd_resp_sendstr_chunk(req, "<!DOCTYPE html><html>");

		httpd_resp_sendstr_chunk(req, "<head></head>");

		httpd_resp_sendstr_chunk(req, "<body>");
		httpd_resp_sendstr_chunk(req, "<h1  style=\"text-align:center\">Thank You / Tesekkurler</h1>");
		httpd_resp_sendstr_chunk(req, "</body></html>");

	httpd_resp_send_chunk(req, NULL, 0);
	return ESP_OK;
}

esp_err_t ctrlcfg_get_handler(httpd_req_t *req)
{
	httpd_resp_send(req, (const char *) ctrlcfg_html_start, ctrlcfg_html_end - ctrlcfg_html_start);
	return ESP_OK;
}

esp_err_t config_post_handler(httpd_req_t *req)
{
    char buf[256];
    int ret, remaining = req->content_len;

    while (remaining > 0)
    {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0)
        {
            if (ret == 0)
            {
                ESP_LOGI(TAG, "No content received please try again ...");
            }
            else if (ret == HTTPD_SOCK_ERR_TIMEOUT)
            {

                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "====================================");

        char fiUpPassUser[12];

        cJSON *rootForParse = cJSON_Parse(buf);

        sprintf(ssidData, "%s", cJSON_GetObjectItem(rootForParse, "ssid")->valuestring);
		sprintf(pswdData, "%s", cJSON_GetObjectItem(rootForParse, "pwd")->valuestring);
		sprintf(mqttBrokerUrl,"%s", cJSON_GetObjectItem(rootForParse, "host")->valuestring);
		sprintf(mqttUsername, "%s", cJSON_GetObjectItem(rootForParse, "hostuser")->valuestring);
		sprintf(fiUpPassUser, "%s", cJSON_GetObjectItem(rootForParse, "updatePass")->valuestring);

        cJSON_Delete(rootForParse);

        ESP_LOGI(TAG, "ssid: %s", ssidData);
        ESP_LOGI(TAG, "pwd: %s", pswdData);
        ESP_LOGI(TAG, "mqtt url: %s", mqttBrokerUrl);
		ESP_LOGI(TAG, "mqtt user: %s", mqttUsername);
		ESP_LOGI(TAG, "fimware pass: %s", fiUpPassUser);
    //Update MQTT server URL
    UpdateMQTTBrookerUrl(mqttBrokerUrl, strlen(mqttBrokerUrl));
    UpdateMQTTUserID(mqttUsername, strlen(mqttUsername));

		if(strcmp(fiUpPassUser, fiUpPass) == 0)	//gives 0 if they are equal. gives some number if they are very different.)
		{
			ESP_LOGI(TAG, "Firmware Password is Right");
			fiUpYesNo = 1;
		}

        remaining -= ret;
    }

    httpd_resp_sendstr(req, "##### Thanks for the information. Wait for updates on the system... ######");
    return ESP_OK;
}

httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &thankPage);
        httpd_register_uri_handler(server, &ctrlcfg_get);
        httpd_register_uri_handler(server, &configPostPage);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

httpd_handle_t stop_webserver(void)
{
	// Ensure handle is non NULL
	 if (server != NULL) {
		 // Stop the httpd server
		 httpd_stop(server);
		 ESP_LOGI(TAG, "Server Stopped");
	 }
    return NULL;
}

// We load the new binary
esp_err_t index_get_handler(httpd_req_t *req)
{
	httpd_resp_send(req, (const char *) index_html_start, index_html_end - index_html_start);
	return ESP_OK;
}

esp_err_t update_post_handler(httpd_req_t *req)
{
	/// 1000 byte alalım
	char buf[1000];
	esp_ota_handle_t ota_handle;
	int remaining = req->content_len;

	printf("##### New Partition Will Be Updated %d\n", remaining);
	printf("##### The size of request:%d\n", remaining);

	const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);

	if (NULL != ota_partition)
	{

		printf("##### Ota partion label: %s\n", ota_partition->label);
		printf("##### Ota partion size: %d\n", ota_partition->size);
	}

	ESP_ERROR_CHECK(esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle));

	while (remaining > 0) {
		int recv_len = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));

		// Timeout Error: Just retry
		if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
			continue;

		// Serious Error: Abort OTA
		} else if (recv_len <= 0) {
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Protocol Error");
			return ESP_FAIL;
		}

		// Successful Upload: Flash firmware chunk
		if (esp_ota_write(ota_handle, (const void *)buf, recv_len) != ESP_OK) {
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash Error");
			return ESP_FAIL;
		}

		remaining -= recv_len;
	}

	// Validate and switch to new OTA image and reboot
	if (esp_ota_end(ota_handle) != ESP_OK || esp_ota_set_boot_partition(ota_partition) != ESP_OK) {
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Validation / Activation Error");
			return ESP_FAIL;
	}

	httpd_resp_sendstr(req, "##### Firmware update complete, rebooting now! ######");

	vTaskDelay(5000 / portTICK_PERIOD_MS);
	esp_restart();

	return ESP_OK;
}

esp_err_t http_server_init(void)
{
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();

	if (httpd_start(&http_server, &config) == ESP_OK) {
		httpd_register_uri_handler(http_server, &index_get);
		httpd_register_uri_handler(http_server, &update_post);
	}

	return http_server == NULL ? ESP_FAIL : ESP_OK;
}

esp_err_t http_server_stop(void)
{
	// Ensure handle is non NULL
	if (http_server != NULL) {
	 // Stop the httpd server
	 httpd_stop(http_server);
	 ESP_LOGI(TAG, "Server Stopped");
	}
	return ESP_OK;
}

void wifi_init_softap(void)
{

	// first let us start with AP mode
	//s_wifi_event_group = xEventGroupCreate();

	esp_netif_init();
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	ap_netif = esp_netif_create_default_wifi_ap();
	//assert( ap_netif );
	//esp_netif_ip_info_t ipInfo;
	//IP4_ADDR(&ipInfo.ip, 192,168,2,1);
	//IP4_ADDR(&ipInfo.gw, 192,168,2,1);
	//IP4_ADDR(&ipInfo.netmask, 255,255,255,0);
	//esp_netif_dhcps_stop(ap_netif);
	//esp_netif_set_ip_info(ap_netif, &ipInfo);
	esp_netif_dhcps_start(ap_netif);


	//ESP_ERROR_CHECK(esp_event_loop_init(wifi_sta_event_handler, NULL)); //-> just here to get rid of startup error
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();       //= WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_ap_event_handler, NULL));

	wifi_config_t wifi_config = {
		.ap = {
			.ssid = "TPServer",
			.ssid_len = strlen("ESP32_Server"),
			.password = "TP_1_Takip",
			.max_connection = 2,
			.authmode = WIFI_AUTH_WPA_WPA2_PSK
		},
	};

	//Asagidaki if kısmı gereksizdi, commentledim.
	/*
	if (strlen("ESP32_Server") == 0) //EXAMPLE_ESP_WIFI_PASS
	{
		wifi_config.ap.authmode = WIFI_AUTH_OPEN;
	}
	*/


	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());


    start_webserver();
    vTaskDelay(60000/portTICK_RATE_MS);   //Öncelikle açılışta bu kadar öteleme yapacağız.

    if((somebodyInside == 1) && (fiUpYesNo == 0))
    {
    	ESP_LOGI(TAG, "Somebody Inside Wait More");
    	vTaskDelay(240000/portTICK_RATE_MS);		// Eğer adam giriş yaparsa biraz daha öteleme yaptırırız.
    	somebodyInside = 0;
    }
    stop_webserver();
    ESP_LOGI(TAG, "Main Server Stopped");
    if(fiUpYesNo == 1)
	{
    	ESP_LOGI(TAG, "Firmware Update Server Started");
		http_server_init();
		vTaskDelay(600000/portTICK_RATE_MS);   //Öncelikle açılışta bu kadar öteleme yapacağız.
		http_server_stop();

	}

    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_ap_event_handler));
	esp_wifi_stop();
	esp_wifi_deinit();
	esp_netif_dhcps_stop(ap_netif);
	esp_event_loop_delete_default();
	esp_netif_deinit();
	vTaskDelay(1000/portTICK_RATE_MS);


	ESP_LOGI(TAG, "Before Save Wifi ssid and pswd: %s %s\n",ssidData,pswdData);
	//printf("Before Save Wifi ssid and pswd: %s %s\n",ssidData,pswdData);
    //Eger veri girisi olmuşsa bunu NVS'ye kaydedelim.
    if ((strlen(ssidData) != 0) && (strlen(pswdData) != 0))
	{
    	ESP_LOGI(TAG, "Save wifi data");
    	save_wifi_data();
	}

    if (strlen(mqttBrokerUrl) != 0)
	{
		ESP_LOGI(TAG, "Save mqtt data host");
		save_mqtt_data_host();

	}

    if (strlen(mqttUsername) != 0)
	{
		ESP_LOGI(TAG, "Save mqtt data userAuth");
		save_mqtt_data_user();

	}

}

void wifi_init_sta(void)
{

    //ESP_LOGI(TAG, "Before Get Saved Data ssid and pswd: %s %s\n",ssidData,pswdData);
    //ESP_LOGI(TAG, "Before Get Saved Data mqtt and user and pass: %s %s %s\n",mqttBrokerUrl,mqttUsername,mqttPassword);
    //printf("Before Get Saved Data ssid and pswd: %s %s\n",ssidData,pswdData);
    //Eger veri girisi olmamissa da NVS'den okuyalim.


    if (strlen(ssidData) && strlen(pswdData) != 0)
	{
		s_wifi_event_group = xEventGroupCreate();

		ESP_ERROR_CHECK(esp_netif_init());

		ESP_ERROR_CHECK(esp_event_loop_create_default());
		esp_netif_create_default_wifi_sta();

		wifi_init_config_t cfg2 = WIFI_INIT_CONFIG_DEFAULT();
		ESP_ERROR_CHECK(esp_wifi_init(&cfg2));


		ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&wifi_sta_event_handler,NULL));
		ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,IP_EVENT_STA_GOT_IP,&wifi_sta_event_handler,NULL));


		//wifi_config_t wifi_configSTA = { };
		//strcpy((char*)wifi_configSTA.sta.ssid,"TurkTelekom_T44B8");
		//strcpy((char*)wifi_configSTA.sta.password,"FPfz42xp");

		wifi_config_t wifi_configSTA = { };
		strcpy((char*)wifi_configSTA.sta.ssid,ssidData);
		strcpy((char*)wifi_configSTA.sta.password,pswdData);

		//printf("ssid and pswd size: %i %i\n",sizeof(wifi_configSTA.sta.ssid),sizeof(wifi_configSTA.sta.password));
		//printf("ssid and pswd: %s %s\n",wifi_configSTA.sta.ssid,wifi_configSTA.sta.password);


		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
		//ESP_LOGI(TAG, "wifi111");
		ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configSTA) );
		//ESP_LOGI(TAG, "wifi222");
		ESP_ERROR_CHECK(esp_wifi_start() );

		ESP_LOGI(TAG, "wifi_init_sta finished.");

		/* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
		 * number of re-tries (WIFI_FAIL_BIT). The bits are set by wifi_sta_event_handler() (see above) */
		EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,pdFALSE,pdFALSE,portMAX_DELAY);

		/* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
		 * happened. */
		if (bits & WIFI_CONNECTED_BIT) {
			ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
					 ssidData, pswdData);
		} else if (bits & WIFI_FAIL_BIT) {
			ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
					 ssidData, pswdData);
		} else {
			ESP_LOGE(TAG, "UNEXPECTED EVENT");
		}



		/* The event will not be processed after unregister */
		ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_sta_event_handler));
		ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_sta_event_handler));
		vEventGroupDelete(s_wifi_event_group);
		ESP_LOGI(TAG, "WIFI STA CONNECTION COMPLETED");
	}

}








/// DATA SENDING
void sendAttributes(char attName[], int attValue)
{
	cJSON *mainJasonLog;

	mainJasonLog = cJSON_CreateObject();
	cJSON_AddItemToObject(mainJasonLog, attName, cJSON_CreateNumber(attValue));

	char *jsonToPayloadLog;
	jsonToPayloadLog = cJSON_Print(mainJasonLog);
	//esp_mqtt_client_publish(client, TOPICSUB_ATT, jsonToPayloadLog, 0, 0, 0);
	AddQueueElementForMQTTClient(TOPICSUB_ATT, jsonToPayloadLog, 0, 0, 0);
	cJSON_Delete(mainJasonLog);
	free(jsonToPayloadLog);
}

void sendAttributesFloat(char attName[], float attValue)
{
	cJSON *mainJasonLog;

	mainJasonLog = cJSON_CreateObject();
	cJSON_AddItemToObject(mainJasonLog, attName, cJSON_CreateNumber(attValue));

	char *jsonToPayloadLog;
	jsonToPayloadLog = cJSON_Print(mainJasonLog);
	//esp_mqtt_client_publish(client, TOPICSUB_ATT, jsonToPayloadLog, 0, 0, 0);
	AddQueueElementForMQTTClient(TOPICSUB_ATT, jsonToPayloadLog, 0, 0, 0);
	cJSON_Delete(mainJasonLog);
	free(jsonToPayloadLog);
}

void sendAttributesString(char attName[], char attValue[])
{
	cJSON *mainJasonLog;

	mainJasonLog = cJSON_CreateObject();
	cJSON_AddItemToObject(mainJasonLog, attName, cJSON_CreateString(attValue));

	char *jsonToPayloadLog;
	jsonToPayloadLog = cJSON_Print(mainJasonLog);
	//esp_mqtt_client_publish(client, TOPICSUB_ATT, jsonToPayloadLog, 0, 0, 0);
	AddQueueElementForMQTTClient(TOPICSUB_ATT, jsonToPayloadLog, 0, 0, 0);
	cJSON_Delete(mainJasonLog);
	free(jsonToPayloadLog);
}

char *my_dtoa(double num, char *str)
{
	if(str==NULL)
	{
		return NULL;
	}
	num = sprintf(str,"%f",num);
	return str;

}

void sendRMSIO(char *rms, double value)
{
  char payloadLog[100] = "200,";
  char stringBuffer[15];
  strcat(payloadLog, rms);
  strcat(payloadLog,",meters,");
  strcat(payloadLog,my_dtoa(value,stringBuffer));
  strcat(payloadLog,",g");
  //esp_mqtt_client_publish(client, TOPICPUB, payloadLog, 0, 2, 0);
  AddQueueElementForMQTTClient(TOPICPUB, payloadLog, 0, 0, 0);
}


void sendTempIO(char *tmp, double value)
{
  char payloadLog[100] = "200,";
  char stringBuffer[15];
  strcat(payloadLog, tmp);
  strcat(payloadLog,",degree,");
  strcat(payloadLog,my_dtoa(value,stringBuffer));
  strcat(payloadLog,",C");
  //esp_mqtt_client_publish(client, TOPICPUB, payloadLog, 0, 2, 0);
  AddQueueElementForMQTTClient(TOPICPUB, payloadLog, 0, 0, 0);
}

void sendJsonRMS(float x, float y, float z)
{
	cJSON *mainJasonRMS;

	mainJasonRMS = cJSON_CreateObject();
	cJSON_AddNumberToObject(mainJasonRMS, "xRMS", (float)(((int)(x*10000.0))/10000.0));
	cJSON_AddNumberToObject(mainJasonRMS, "yRMS", (float)(((int)(y*10000.0))/10000.0));
	cJSON_AddNumberToObject(mainJasonRMS, "zRMS", (float)(((int)(z*10000.0))/10000.0));


	char *jsonToPayloadRMS;
	jsonToPayloadRMS = cJSON_Print(mainJasonRMS);
	//esp_mqtt_client_publish(client, TOPICPUB, jsonToPayloadRMS, 0, 0, 0);
	AddQueueElementForMQTTClient(TOPICPUB, jsonToPayloadRMS, 0, 0, 0);

	cJSON_Delete(mainJasonRMS);
	free(jsonToPayloadRMS);
}

void sendJsonVRMS(float x, float y, float z)
{
	cJSON *mainJasonRMS;

	mainJasonRMS = cJSON_CreateObject();
	cJSON_AddNumberToObject(mainJasonRMS, "xVRMS", x);
	cJSON_AddNumberToObject(mainJasonRMS, "yVRMS", y);
	cJSON_AddNumberToObject(mainJasonRMS, "zVRMS", z);

	char *jsonToPayloadRMS;
	jsonToPayloadRMS = cJSON_Print(mainJasonRMS);
	//esp_mqtt_client_publish(client, TOPICPUB, jsonToPayloadRMS, 0, 0, 0);
	AddQueueElementForMQTTClient(TOPICPUB, jsonToPayloadRMS, 0, 0, 0);

	cJSON_Delete(mainJasonRMS);
	free(jsonToPayloadRMS);
}

void sendJsonTemp(float x)
{
	cJSON *mainJasonTemp;

	mainJasonTemp = cJSON_CreateObject();
	cJSON_AddNumberToObject(mainJasonTemp, "temp", x);


	char *jsonToPayloadTemp;
	jsonToPayloadTemp = cJSON_Print(mainJasonTemp);
	//esp_mqtt_client_publish(client, TOPICPUB, jsonToPayloadTemp, 0, 0, 0);
	AddQueueElementForMQTTClient(TOPICPUB, jsonToPayloadTemp, 0, 0, 0);

	cJSON_Delete(mainJasonTemp);
	free(jsonToPayloadTemp);
}

void sendJsonLog(int errorCode, char logType[])
{
	cJSON *mainJasonLog;

	mainJasonLog = cJSON_CreateObject();
	cJSON_AddItemToObject(mainJasonLog, "log", cJSON_CreateNumber(errorCode));

	char *jsonToPayloadLog;
	jsonToPayloadLog = cJSON_Print(mainJasonLog);
	//esp_mqtt_client_publish(client, TOPICPUB, jsonToPayloadLog, 0, 0, 0);
	AddQueueElementForMQTTClient(TOPICPUB, jsonToPayloadLog, 0, 0, 0);
	cJSON_Delete(mainJasonLog);
	free(jsonToPayloadLog);
}

void sendJsonExtBucket(char sensorNumber[], float *x, float *y, float *z, double dataTime, int bucketSize, int Fs)
{

	//double ett = 0.0;
	vTaskDelay(3000/portTICK_RATE_MS);

	int i;
	int j;
	int arraySize = 50;
	int division = (int)((bucketSize-2*coldStart)/50);

	double *exx, *eyy, *ezz;
	exx = malloc(arraySize * sizeof(double));
	eyy = malloc(arraySize * sizeof(double));
	ezz = malloc(arraySize * sizeof(double));
	//ESP_LOGI(TAG, "JSON DIVISION: %d",division);
	//ESP_LOGI(TAG, "JSON BUCKETSIZE: %d",bucketSize);
	for(i=0;i<division;i++)
	{
		/*
		for(int k = 0; k< 200; k++)
		{
			if(mqttConnectFlag == 1)
			{
				break;
			}
			else
			{
				vTaskDelay(3000/portTICK_RATE_MS);
				ESP_LOGI(TAG, "CAN NOT SEND EXTREME WAIT SOME TIME\n");
			}
		}*/

		for(j=0;j<arraySize;j++)
		{
			exx[j] = (double)(((int)(x[i*arraySize+j+coldStart]*10000.0))/10000.0);
			eyy[j] = (double)(((int)(y[i*arraySize+j+coldStart]*10000.0))/10000.0);
			ezz[j] = (double)(((int)(z[i*arraySize+j+coldStart]*10000.0))/10000.0);
		}
		cJSON *mainJasonRMS;

		mainJasonRMS = cJSON_CreateObject();
		cJSON_AddItemToObject(mainJasonRMS, "me", cJSON_CreateString(macID));
		cJSON_AddNumberToObject(mainJasonRMS, "et", (int)dataTime);
		cJSON_AddStringToObject(mainJasonRMS, "se", sensorNumber);
		cJSON_AddStringToObject(mainJasonRMS, "te", "E");
		cJSON_AddNumberToObject(mainJasonRMS, "fs", Fs);
		cJSON_AddNumberToObject(mainJasonRMS, "si", arraySize);
		cJSON_AddNumberToObject(mainJasonRMS, "st", (i+1));
		cJSON_AddNumberToObject(mainJasonRMS, "sa", division);
		cJSON_AddItemToObject(mainJasonRMS, "x", cJSON_CreateDoubleArray(exx, arraySize));
		cJSON_AddItemToObject(mainJasonRMS, "y", cJSON_CreateDoubleArray(eyy, arraySize));
		cJSON_AddItemToObject(mainJasonRMS, "z", cJSON_CreateDoubleArray(ezz, arraySize));

		char *jsonToPayloadRMS;
		jsonToPayloadRMS = cJSON_Print(mainJasonRMS);
		//esp_mqtt_client_publish(client, TOPICPUBEXT, jsonToPayloadRMS, 0, 0, 0);
		AddExtQueueElementForMQTTClient(TOPICPUBEXT, jsonToPayloadRMS, 0, 0, 0);
		cJSON_Delete(mainJasonRMS);
		free(jsonToPayloadRMS);
		vTaskDelay(10000/portTICK_RATE_MS);
	}
	free(exx);
	free(eyy);
	free(ezz);
}





////// CALCULATION
float doSomeR_calculation(int samplesRMS, int coldStart, float *rfx, float *rfy, float *rfz, float *rmsX, float *rmsY, float *rmsZ)
{
  // do some matrix calculation
  // lets calculate mean only
	*rmsX = rfx[0];
	*rmsY = rfy[0];
	*rmsZ = rfz[0];

	return 0;
}

float doSomeV_calculation(int NFFT, int Fs, int coldSt, float *data)
{
	float result = 0.0;
  // do some matrix calculation
  // lets calculate mean only
	result = *data;

	return result;
}

void SubscribeTopics(void)
{
	//int msg_id;

    //msg_id = esp_mqtt_client_subscribe(client_F, TOPICSUB_ATT, 0);
	//AddQueueElementForMQTTClient(TOPICPUB, jsonToPayloadLog, 0, 0, 0);
    //sendJsonLog("SYSTEM STARTED", "L");
	//ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
	//sendJsonLog(100, "L");
	//msg_id = esp_mqtt_client_subscribe(client_F, TOPICSUB_REQ, 0);
}


void ParseMQTTData(char *Data)
{
	ESP_LOGI(TAG, "MQTT_EVENT_DATA");
	//ESP_LOGI("[ADXL357]","TOPIC=%.*s\r\n", strlen(Data), Data);
	ESP_LOGI("[ADXL357]","DATA=%.*s\r\n", strlen(Data), Data);

	cJSON *rootMesaj = cJSON_Parse(Data);
	char idFromMessage[15];
	int idComparison = 1;

	//char idFromMessage = cJSON_GetObjectItem(rootMesaj,"me")->valuestring;
	//ESP_LOGI("[ADXL357]","idFromMessage: %s\n", idFromMessage);
	if(cJSON_GetObjectItem(rootMesaj,"method") != NULL)
	{
		//ESP_LOGI("[ADXL357]","idFromMessage: %s\n", cJSON_GetObjectItem(rootMesaj,"me")->valuestring);
		sprintf(idFromMessage,"%s", cJSON_GetObjectItem(rootMesaj,"method")->valuestring);
		//ESP_LOGI("[ADXL357]","idFromMessage: %s a\n", idFromMessage);
		idComparison = strcmp(idFromMessage, "rpcCommand");	//gives 0 if they are equal. gives some number if they are very different.
		ESP_LOGI("[ADXL357]","MESSAGE ID COMPARISON: %d\n", idComparison);
	}

	//sendJsonLog(110, "L");

	if(cJSON_GetObjectItem(rootMesaj,"vLoop") != NULL)
	{
		int dummyVLoop;
		dummyVLoop = cJSON_GetObjectItem(rootMesaj,"vLoop")->valueint;
		ESP_LOGI("[ADXL357]","VELOCITY LOOP FROM MESSAGE: %i\n", dummyVLoop);
		if(dummyVLoop > 19 && dummyVLoop < 43200)
		{

			if(dummyVLoop == vLoop)
			{
				ESP_LOGI("[ADXL357]","NO CHANGE ON RANGE: %i\n", dummyVLoop);
			}
			else
			{
				ESP_LOGI("[ADXL357]","NEW VELOCITY LOOP: %i\n", dummyVLoop);
				vLoop = dummyVLoop;
				get_save_vloop_data(1);
			}
		}
		else
		{
			ESP_LOGI("[ADXL357]","WRONG VELOCITY LOOP ON MESSAGE\n");
		}
	}

	if(cJSON_GetObjectItem(rootMesaj,"rLoop") != NULL)
	{
		int dummyRLoop;
		dummyRLoop = cJSON_GetObjectItem(rootMesaj,"rLoop")->valueint;
		ESP_LOGI("[ADXL357]","RMS LOOP FROM MESSAGE: %i\n", dummyRLoop);
		if(dummyRLoop > 19 && dummyRLoop < 43200)
		{

			if(dummyRLoop == rmsLoop)
			{
				ESP_LOGI("[ADXL357]","NO CHANGE ON RMS LOOP: %i\n", dummyRLoop);
			}
			else
			{
				ESP_LOGI("[ADXL357]","NEW VELOCITY RMS LOOP: %i\n", dummyRLoop);
				rmsLoop = dummyRLoop;
				get_save_rloop_data(1);
			}
		}
		else
		{
			ESP_LOGI("[ADXL357]","WRONG RMS LOOP ON MESSAGE\n");
		}
	}

	if(cJSON_GetObjectItem(rootMesaj,"gRange") != NULL)
	{
		int dummyRange;
		dummyRange = cJSON_GetObjectItem(rootMesaj,"gRange")->valueint;
		ESP_LOGI("[ADXL357]","RANGE FROM MESSAGE: %i\n", dummyRange);
		if(dummyRange == 10 || dummyRange == 20 || dummyRange == 40)
		{

			if(dummyRange == sensorRange)
			{
				ESP_LOGI("[ADXL357]","NO CHANGE ON RANGE: %i\n", dummyRange);
			}
			else
			{
				ESP_LOGI("[ADXL357]","NEW RANGE: %i\n", dummyRange);
				sensorRange = dummyRange;
				get_save_range_data(1);
				gotMessage = 1;
	  NotifySensorReconfiguration(MQTT_RECV_MSG_HANDLER_CELLULAR_TASK);
			}
		}
		else
		{
			ESP_LOGI("[ADXL357]","WRONG RANGE ON MESSAGE\n");
		}
	}

	if(cJSON_GetObjectItem(rootMesaj,"eTime") != NULL)
	{
		int dummyTime;
		dummyTime = cJSON_GetObjectItem(rootMesaj,"eTime")->valueint;
		ESP_LOGI("[ADXL357]","TIME FROM MESSAGE: %i\n", dummyTime);
		if(dummyTime > 0 && dummyTime < 65)
		{
			ESP_LOGI("[ADXL357]","NEW TIME: %i\n", dummyTime);
			eTime = dummyTime;
		}
		else
		{
			ESP_LOGI("[ADXL357]","WRONG TIME ON MESSAGE\n");
		}

	}

	if(cJSON_GetObjectItem(rootMesaj,"eRate") != NULL)
	{
		int dummyRate;
		dummyRate = cJSON_GetObjectItem(rootMesaj,"eRate")->valueint;
		ESP_LOGI("[ADXL357]","SAMPLING RATE FROM MESSAGE: %i\n", dummyRate);
		if(dummyRate == 125 || dummyRate == 250 || dummyRate == 500 || dummyRate == 1000 || dummyRate == 2000 || dummyRate == 4000)
		{
			ESP_LOGI("[ADXL357]","NEW RATE: %i\n", dummyRate);
			eFs = dummyRate;
		}
		else
		{
			ESP_LOGI("[ADXL357]","WRONG RATE ON MESSAGE\n");
		}
	}

	if(cJSON_GetObjectItem(rootMesaj,"eStart") != NULL)
	{
		int dummyExt;
		dummyExt = cJSON_GetObjectItem(rootMesaj,"eStart")->valueint;
		ESP_LOGI("[ADXL357]","EXTREME BOOL FROM MESSAGE: %i\n", dummyExt);
		if(dummyExt == 1)
		{
			ESP_LOGI("[ADXL357]","START EXTREME: %i\n", dummyExt);
			extReq = 1;
			//RequestExtremeSensorData(MQTT_RECV_MSG_HANDLER_CELLULAR_TASK);
		}
		else
		{
			ESP_LOGI("[ADXL357]","NO EXTREME WANTED\n");
		}
	}

	if(cJSON_GetObjectItem(rootMesaj,"upHost") != NULL)
	{
		char upDummyURL[100];
		sprintf(upDummyURL,"%s", cJSON_GetObjectItem(rootMesaj,"upHost")->valuestring);
		ESP_LOGI("[ADXL357]","UPDATE URL FROM MESSAGE: %s\n", upDummyURL);
		ESP_LOGI(TAG, "UPDATE URL FROM MESSAGE: %s",upDummyURL);
		ESP_LOGI(TAG, "BEFORE STR CPY: %s",updateURL);
		strcpy(updateURL, upDummyURL);
		ESP_LOGI(TAG, "AFTER STR CPY: %s",updateURL);
	}

	if(cJSON_GetObjectItem(rootMesaj,"reStart") != NULL)
	{
		int dummyRestart;
		dummyRestart = cJSON_GetObjectItem(rootMesaj,"reStart")->valueint;
		ESP_LOGI("[ADXL357]","RESTART FROM MESSAGE: %i\n", dummyRestart);
		if(dummyRestart == 1)
		{
			esp_restart();

		}
		else
		{
			ESP_LOGI("[ADXL357]","RESTART MESSAGE PROBLEM\n");
		}
	}

	if(cJSON_GetObjectItem(rootMesaj,"updateVer") != NULL)
	{
		int dummyExt;
		dummyExt = cJSON_GetObjectItem(rootMesaj,"updateVer")->valueint;
		ESP_LOGI("[ADXL357]","EXTREME BOOL FROM MESSAGE: %i\n", dummyExt);
		if(dummyExt == 1)
		{
			ESP_LOGI("[ADXL357]","UPDATE VERSION: %i\n", dummyExt);
			//updateVersion = 1;
		}
		else
		{
			ESP_LOGI("[ADXL357]","NO UPDATE VERSION\n");
		}
	}

	cJSON_Delete(rootMesaj);

}
