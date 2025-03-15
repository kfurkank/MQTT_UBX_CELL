/*
 * adxl357.h
 *
 *  Created on: May 4, 2020
 *      Author: fatihaltunel
 */


/*
 * 100: I am connected to mqtt
 * 101: Sensor is ok
 * 110: Got message
 *
 * 901: Sensor connection problem
 * 902: Sensor range conversion problem
 * 903: LPF Value Conversion Problem
 * 904: Sensor range check problem
 *
 * 911: FIFO OVERFLOW
 * 921: FIFO SENSOR READ ERROR
 * 931: SENSE LOOP ERROR - RESTART
 *
 * 912: FIFO OVERFLOW - EXTREME
 * 922: FIFO SENSOR READ ERROR - EXTREME
 *
 * 701: OTA UPDATE FAILED
 */

#ifndef __ADXL357_H__
#define __ADXL357_H__

#include "SysDef.h"


extern char cpuID[];
extern char macID[14];

#define SPI_MAX_DMA_LEN (4096-4)

#define TAG 					"FATIH"

extern time_t now;         // Time variable

extern int shareSettings;

extern char deviceIP[32];

extern char ssidData[32];
extern char pswdData[64];
#define STORAGE_NAMESPACE "storage"


/* FreeRTOS event group to dataA when we are connected*/
extern EventGroupHandle_t s_wifi_event_group;

extern esp_netif_t *ap_netif;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

extern httpd_handle_t server;

extern uint8_t sensorRange;
extern int rFs;		//Fs for R Loop
extern int eFs;		//Fs for E Data
extern int vTime;	//Total time for R data collection
extern int eTime;	//Total time for E data collection
extern uint8_t compareFilter;	// We check sensor filter settings in every R loop because of power related resetting.
extern uint8_t compareRange;	// We chenck sensor range settings in every R loop because of power related resetting


extern int sampleRMS;   // R related bucket size
extern int coldStart;
extern int rmsLoop;
extern int vLoop;
extern float onOffLimit;		//Sensörün takıldığı noktanın/makinenin çalıştığı ve çalışmadığı durumu ayrıştıran rms seviyesi

// SENSOR data conversion variables.
extern double baseConv;
extern double accConv;

//Firmware OTA:
extern int updateVersion;
extern float softVersion;
extern char updateURL[100];
extern char fiUpPass[];		// Access Point Related
extern int fiUpYesNo;
extern httpd_handle_t http_server;

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
extern const uint8_t ctrlcfg_html_start[] asm("_binary_ctrlcfg_html_start");
extern const uint8_t ctrlcfg_html_end[] asm("_binary_ctrlcfg_html_end");



extern long int timeEp;				  //Exact epoch time of data collection
extern long int extReqTime;
extern long int extReqLoop;

extern int extReq;           // This value becomes 1 if user request E data
extern int gotMessage;	      // This value becomes 1 if there is message from user

extern char mqttBrokerUrl[40];
extern char mqttUsername[40];


esp_mqtt_client_handle_t client;
//#define TOPICPUB       "v1/devices/me/telemetry"     // All pub
#define TOPICPUB       "s/us"     // All pub
#define TOPICPUBEXT       "v1/devices/me/trendopeak"     // Extreme Pub
#define TOPICPUBLOG    "v1/devices/me/attributes"     // All log
#define TOPICSUB_ATT       "v1/devices/me/attributes"     // All sub for attributes
#define TOPICSUB_REQ       "v1/devices/me/rpc/request/+"     // All sub for requests
extern int mqttConnectFlag;	//mqtt'ye bağlı ise bunu 1 yapıyoruz, bağlı değilse 0 oluyor.
extern int mqttConC;		// mqtt'ye bağlanamazsa bu count'u teker teker arttıracak.
extern int mqttConL; 	// mqtt'ye bu kadar adet bağlanmazsa sisteme restart atacak.


extern int wifiRetryCount;
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

extern int haveWifiData;
extern int somebodyInside;     // If there is somebody inside Access Point, we delay some time for user.


extern spi_device_handle_t spi1;
extern esp_err_t ret;
//spi_transaction_t t;

#define PIN_NUM_MISO 			12
#define PIN_NUM_MOSI 			13
#define PIN_NUM_CLK  			14
#define PIN_NUM_CS   			15

#define ADXL357_REG_DEVID_AD		0x00
#define ADXL357_REG_DEVID_MST		0x01
#define ADXL357_REG_PARTID		0x02
#define ADXL357_REG_REVID		0x03
#define ADXL357_REG_STATUS		0x04
#define ADXL357_REG_FIFO_ENTRIES	0x05
#define ADXL357_REG_TEMP2 		0x06
#define ADXL357_REG_TEMP1		0x07
#define ADXL357_REG_XDATA3		0x08
#define ADXL357_REG_XDATA2		0x09
#define ADXL357_REG_XDATA1		0x0A
#define ADXL357_REG_YDATA3		0x0B
#define ADXL357_REG_YDATA2		0x0C
#define ADXL357_REG_YDATA1		0x0D
#define ADXL357_REG_ZDATA3		0x0E
#define ADXL357_REG_ZDATA2		0x0F
#define ADXL357_REG_ZDATA1		0x10
#define ADXL357_REG_FIFO_DATA		0x11
#define ADXL357_REG_OFFSET_X_H 		0x1E
#define ADXL357_REG_OFFSET_X_L 		0x1F
#define ADXL357_REG_OFFSET_Y_H 		0x20
#define ADXL357_REG_OFFSET_Y_L 		0x21
#define ADXL357_REG_OFFSET_Z_H 		0x22
#define ADXL357_REG_OFFSET_Z_L 		0x23
#define ADXL357_REG_ACT_EN		0x24
#define ADXL357_REG_ACT_THRESH_H	0x25
#define ADXL357_REG_ACT_THRESH_L 	0x26
#define ADXL357_REG_ACT_COUNT 		0x27
#define ADXL357_REG_FILTER		0x28
#define ADXL357_REG_FIFO_SAMPLES 	0x29
#define ADXL357_REG_INT_MAP		0x2A
#define ADXL357_REG_SYNC		0x2B
#define ADXL357_REG_RANGE		0x2C
#define ADXL357_REG_POWER_CTL		0x2D
#define ADXL357_REG_RESET		0x2F

#define ADXL357_RESET_CODE		0x52

#define ADXL357_TEMP_NOMINAL_LSB		1852
#define ADXL357_TEMP_NOMINAL_CELSIUS		25
#define ADXL357_TEMP_NOMINAL_CELSIUS_SLOPE	9.05f
#define ADXL357_TEMP_C_TO_F_CONST		9.0f / 5.0f

#define ADXL357_STATUS_DATA_RDY 	1
#define ADXL357_STATUS_FIFO_FULL 	2
#define ADXL357_STATUS_FIFO_OVR 	4
#define ADXL357_STATUS_ACTIVITY 	8
#define ADXL357_STATUS_NVM_BUSY 	16

#define ADXL357_FIFO_MAX_COUNT		96
#define ADXL357_TWO_COMPONENT_POWER		1048576
#define FIFO_STREAM_OVR_BREAK		0x01
#define FIFO_STREAM_FIFO_READ_BREAK	0x02

#define ADXL357_COMMAND_PREPARED	0x01

#define ADXL357_OK			0
#define ADXL357_ERROR			-1
#define ADXL357_FIFO_READ_ERROR		-10
#define ADXL357_FIFO_STREAM_ERROR	-20

#define ADXL357_REG_WRITE(REG) (REG << 1 | 0x00)
#define ADXL357_REG_READ(REG) (REG << 1 | 0x01)

#define ADXL357_SPI_WRITE(handler, data, result, len) adxl357_wpi_spi_write(handler, data, result, len)

#define SET_FILTER_4000   0x00
#define SET_FILTER_2000   0x01
#define SET_FILTER_1000   0x02
#define SET_FILTER_500    0x03
#define SET_FILTER_250    0x04
#define SET_FILTER_125    0x05



typedef struct {
  unsigned spi : 1;
  unsigned i2c : 1;
  int fd;
  int spi_channel;
  int speed;
  spi_device_handle_t spiHand;
} ADXL357_HANDLER;

typedef struct {
  unsigned NVM_BUSY : 1;
  unsigned ACTIVITY : 1;
  unsigned FIFO_OVR : 1;
  unsigned FIFO_FULL : 1;
  unsigned DATA_RDY : 1;
} ADXL357Status;

typedef struct {
  ADXL357Status status;
  uint8_t fifo_entries;
} ADXL357StatusAndFifo;

typedef struct {
  uint16_t raw;
  float celsius;
  float kelvin;
  float fahrenheit;
} ADXL357Temperature;

typedef struct {
  int32_t x;
  int32_t y;
  int32_t z;
} ADXL357Acceleration;

typedef struct {
  ADXL357Acceleration data[ADXL357_FIFO_MAX_COUNT / 3];
  uint8_t samples;
  uint8_t empty_read_index;
  uint8_t x_marker_error_index;
  unsigned empty_read : 1;
  unsigned x_marker_error : 1;
} ADXL357Fifo;

typedef struct {
  uint8_t reg;
  uint8_t status;
  uint8_t data[512];
  uint8_t prepared[512];
  uint8_t raw_result[512];
  size_t len;
} ADXL357Command;


void adxl357_print_command_result(ADXL357Command * sensorCMD);
void adxl357_print_status(ADXL357Status * status);

void adxl357_prepare_command(ADXL357Command * sensorCMD);
int adxl357_execute_command(ADXL357_HANDLER * handler, ADXL357Command * sensorCMD);
int adxl357_wpi_spi_write(ADXL357_HANDLER * handler, uint8_t * data, uint8_t * result, size_t len);

int adxl357_get_devid_ad(ADXL357_HANDLER * handler, uint8_t * b);
int adxl357_get_devid_mst(ADXL357_HANDLER * handler, uint8_t * b);
int adxl357_get_partid(ADXL357_HANDLER * handler, uint8_t * b);
int adxl357_get_revid(ADXL357_HANDLER * handler, uint8_t * b);
int adxl357_get_status(ADXL357_HANDLER * handler, ADXL357Status * status);
int adxl357_get_fifo_entries(ADXL357_HANDLER * handler, uint8_t * b);
int adxl357_get_status_n_fifo(ADXL357_HANDLER * handler, ADXL357StatusAndFifo * status);
int adxl357_read_temperature(ADXL357_HANDLER * handler, ADXL357Temperature * temp);
int adxl357_read_acceleration(ADXL357_HANDLER * handler, ADXL357Acceleration * acc);
int adxl357_read_fifo(ADXL357_HANDLER * handler, ADXL357Fifo * fifo, uint8_t entries_count);
void adxl357_empty_fifo(ADXL357_HANDLER * handler, uint8_t readCount);
int adxl357_fifo_stream(ADXL357_HANDLER * handler, void (* callback)(ADXL357Fifo * fifo), uint8_t flags);
int adxl357_fifo_coldstart(ADXL357_HANDLER * handler, int sampleHigh, uint8_t flags );
int adxl357_fifo_bucket(ADXL357_HANDLER * handler, int sampleHigh, float *ax, float *ay, float *az, uint8_t flags );
int adxl357_measurement_mode(ADXL357_HANDLER * handler);
int adxl357_standby_mode(ADXL357_HANDLER * handler);
int adxl357_set_range(ADXL357_HANDLER * handler, int setRange);
int share_sensor_settings(ADXL357_HANDLER * handler);
int adxl357_check_range(ADXL357_HANDLER * handler, int setRange);
int adxl357_set_filter(ADXL357_HANDLER * handler, int setFilter);
int adxl357_check_filter(ADXL357_HANDLER * handler, int setFilter);
int adxl357_get_power_ctl(ADXL357_HANDLER * handler, uint8_t * b);
int adxl357_reset(ADXL357_HANDLER * handler);


void getMacAddress();

//// NVS
esp_err_t save_wifi_data(void);
esp_err_t get_saved_wifi(void);
esp_err_t save_mqtt_data_host(void);
esp_err_t get_saved_mqtt_host(void);
esp_err_t save_mqtt_data_user(void);
esp_err_t get_saved_mqtt_user(void);
esp_err_t get_save_range_data(int saveData);
esp_err_t get_save_vloop_data(int saveData);
esp_err_t get_save_rloop_data(int saveData);
esp_err_t get_save_cr_data(int saveData);



// MQTT
esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event);
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);



// OTA
esp_err_t _http_event_handler(esp_http_client_event_t *evt);
void ota_task();




// WIFI
void wifi_ap_event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data);
void wifi_sta_event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data);

esp_err_t thankYou_get_handler(httpd_req_t *req);
static const httpd_uri_t thankPage = {
    .uri = "/thanks",
    .method = HTTP_GET,
    .handler = thankYou_get_handler,
    .user_ctx = NULL};

httpd_handle_t start_webserver(void);
httpd_handle_t stop_webserver(void);
//Control Config update ile ilgili sayfalar
esp_err_t ctrlcfg_get_handler(httpd_req_t *req);
//httpd_uri_t ctrlcfg_get = {
//	.uri	  = "/config",
//	.method   = HTTP_GET,
//	.handler  = ctrlcfg_get_handler,
//	.user_ctx = NULL
//};
extern httpd_uri_t ctrlcfg_get;
esp_err_t config_post_handler(httpd_req_t *req);
static const httpd_uri_t configPostPage = {
    .uri = "/configPost",
    .method = HTTP_POST,
    .handler = config_post_handler,
    .user_ctx = "TEST"};


//Firmware update ile ilgili sayfalar
esp_err_t index_get_handler(httpd_req_t *req);
//httpd_uri_t index_get = {
//	.uri	  = "/update",
//	.method   = HTTP_GET,
//	.handler  = index_get_handler,
//	.user_ctx = NULL
//};
extern httpd_uri_t index_get;
esp_err_t update_post_handler(httpd_req_t *req);
//httpd_uri_t update_post = {
//	.uri	  = "/updatePost",
//	.method   = HTTP_POST,
//	.handler  = update_post_handler,
//	.user_ctx = "TEST2"
//};
extern httpd_uri_t update_post;
esp_err_t http_server_init(void);
esp_err_t http_server_stop(void);
void wifi_init_softap(void);
void wifi_init_sta(void);







void sendAttributes(char attName[], int attValue);
void sendAttributesFloat(char attName[], float attValue);
void sendAttributesString(char attName[], char attValue[]);
char *my_dtoa(double num, char *str);
void sendRMSIO(char *rms, double value);
void sendTempIO(char *tmp, double value);
void sendJsonRMS(float x, float y, float z);
void sendJsonVRMS(float x, float y, float z);
void sendJsonTemp(float x);
void sendJsonLog(int errorCode, char logType[]);
void sendJsonExtBucket(char sensorNumber[], float *x, float *y, float *z, double dataTime, int bucketSize, int Fs);
float doSomeR_calculation(int samplesRMS, int coldStart, float *rfx, float *rfy, float *rfz, float *rmsX, float *rmsY, float *rmsZ);
float doSomeV_calculation(int NFFT, int Fs, int coldSt, float *data);
void SubscribeTopics(void);
void ParseMQTTData(char *Data);

#endif /* ADXL357_H_ */



