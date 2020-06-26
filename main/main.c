#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "vl53l1_api.h"
#include "sdkconfig.h"
#include"driver/i2c.h"
#include "esp_log.h"
#include "vl53l1_platform.h"

//sockets bits
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

//heap debugging
#include "esp_heap_caps.h"

#include "ThisProject.h"

#include "cJSON.h"  //https://github.com/espressif/esp-idf/tree/master/components/json

#define do_calibration false
#define CalDistanceMilliMeter 50
#define timingBudget_uS 220000
#define intermeasurement_mS 250
#define distanceMode 2 //VL53L1_DISTANCEMODE_SHORT is 1,VL53L1_DISTANCEMODE_MEDIUM is 2,
// VL53L1_DISTANCEMODE_LONG is 3
#define SOCKET_LOOP_TIME 250 //ms for my socket to generate a new send and collect measurement

VL53L1_RangingMeasurementData_t pRangingMeasurementData;
int8_t error_us;
static const char* TAG = "MainModule";
char* payload = NULL;
//PROTOTYPES==============================================
float  runTOF(void );
char* jsonify(float);


/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define EXAMPLE_ESP_MAXIMUM_RETRY  10
static int s_retry_num = 0;



void i2c_init(void) {
	// initialise the i2c...................
	i2c_config_t conf;
	conf.master.clk_speed = 400000;//400000; //clock speed
	conf.mode = I2C_MODE_MASTER;
	conf.scl_io_num = SCL_IOpin;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.sda_io_num = SDA_IOpin;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;

	/*conf.mode = I2C_MODE_MASTER;
	 conf.sda_io_num = SDA_IOpin;
	 conf.scl_io_num = SCL_IOpin;
	 conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	 conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	 conf.master.clk_speed = 400000;//clock speed
	 conf.*/
	i2c_param_config(I2C_NUM_0, &conf);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

//..................................................................

} //end void I2C init_

static void tcp_client_task(void *pvParameters)

{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {


    	 struct sockaddr_in dest_addr;
    	        dest_addr.sin_addr.s_addr = inet_addr(HOSTNAME_SERVER);
    	        dest_addr.sin_family = AF_INET;
    	        dest_addr.sin_port = htons(PORT_NUMBER_SERVER);
    	        addr_family = AF_INET;
    	        ip_protocol = IPPROTO_IP;
    	        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);


        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket, try again in 2 secs: errno %d", errno);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
           // break;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", HOSTNAME_SERVER, PORT_NUMBER_SERVER);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect try agian in 2 secs: errno %d", errno);
            vTaskDelay(2000 / portTICK_PERIOD_MS);

           // break;
        }
        ESP_LOGI(TAG, "Successfully connected");
int forWhile=1;
int var=0;
float mins=0.0;
        while (forWhile) {

if (var % 120 ==0){
	heap_caps_print_heap_info(MALLOC_CAP_EXEC);
	mins++;
	ESP_LOGE(TAG,"mins sice start %f==========================/n", mins/2);
}
var++;
        	float intMeasurement=   runTOF( );
        	  payload =jsonify(intMeasurement);
        	//payload= "{\"distanceMM\":1583,\"device\":4}";
        	// ESP_LOGI(TAG, "sending %s", payload);
            int err = send(sock, payload, strlen(payload), 0);
          free(payload);
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d/n", errno);
                forWhile=0;
                break;
            }

            //free(payload);



            vTaskDelay(SOCKET_LOOP_TIME / portTICK_PERIOD_MS);
        }  //close the while L113

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting.../n");
            shutdown(sock, 0);
            close(sock);
        }
        }// closw while L87
    vTaskDelete(NULL);
} // close method tcp_client_task()

char* jsonify(float measurement){
	// website for the Json;
	//https://github.com/espressif/esp-idf/tree/master/components/jsonS
	cJSON *root;
	root=cJSON_CreateObject();
	// char *string = NULL;


		cJSON_AddNumberToObject(root,"distanceMM",		measurement);
		cJSON_AddNumberToObject(root,"device",		DEVICE_NUM);




	/*//	payload = (char *)malloc(150);//, sizeof(char));
	//	payload = (char *)pvPortMalloc(150);
		payload = pvPortMalloc(150);

			     Check to see if we were successful
			    if (payload == NULL)
			    {
			         We were not so display a message
			        printf("Could not allocate required memory\n");

			         And exit
			        exit(1);
			    }*/
		payload= cJSON_PrintUnformatted(root);
		 if (payload == NULL)
		    {
		        fprintf(stderr, "Failed to print monitor.\n");
		    }
		cJSON_Delete(root);

return payload;

}//close sendadistance

float  runTOF(void ) {
	error_us = VL53L1_GetRangingMeasurementData(dev2,	&pRangingMeasurementData);
	if (error_us != 0) {
		ESP_LOGI(TAG, "mGetRangingMeasurementData error %d", error_us);
	}
	//vTaskDelay(1050 / portTICK_PERIOD_MS);
	error_us = VL53L1_ClearInterruptAndStartMeasurement(dev2);
	if (error_us != 0) {
		ESP_LOGI(TAG, "ClearInterruptAndStartMeasurement error %d", error_us);
	}
	if (pRangingMeasurementData.RangeStatus == 0) {
		/*ESP_LOGI(TAG,
				"ranging status; %d; distanceMM; %d ambient; %f, reflectance rate; %f, sigma mm %f",
				pRangingMeasurementData.RangeStatus,
				pRangingMeasurementData.RangeMilliMeter,
				(float) pRangingMeasurementData.AmbientRateRtnMegaCps / 65536,
				(float) pRangingMeasurementData.SignalRateRtnMegaCps / 65536,
				(float) pRangingMeasurementData.SigmaMilliMeter / 65536);*/
	}
	return pRangingMeasurementData.RangeMilliMeter;
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}

void app_main(void) {
	nvs_flash_init();
	tcpip_adapter_init();
	wifi_init_sta();


//initialise I2C
	i2c_init();
	  /* Allocate and zero memory for the socket sending json string  */



	VL53L1_Version_t pVersion;
	VL53L1_DeviceInfo_t pVL53L1_DeviceInfo;

	VL53L1_CalibrationData_t CalibrationData;

	dev2 = &dev;

	dev.I2cDevAddr = 0x52;
	dev.comms_speed_khz = 400;//400;

	dev.new_data_ready_poll_duration_ms = 10;
///////basic low level i2c qureiy
	uint8_t tempdeleteme = 0;
	if (error_us != 0) {

	}
	error_us = VL53L1_RdByte(dev2, 0x52, &tempdeleteme);
//ESP_LOGI(TAG, "sbinary of 0x010f';%b", 0x010f);
	ESP_LOGI(TAG, "should be \'0xEA\';%x", tempdeleteme);
	if (error_us != 0) {
		ESP_LOGI(TAG, "error from readbyte;%d", error_us);
	}

//vTaskDelay(3000000000 / portTICK_PERIOD_MS);

/////////////////////////////start the driver etc
	if (error_us != 0) {
		error_us = VL53L1_GetVersion(&pVersion);
	}

	ESP_LOGI(TAG, "GetVersion %d", error_us);
	ESP_LOGI(TAG, "version %d major %d, minor %d, build %d \n", pVersion.revision, pVersion.major, pVersion.minor,
			pVersion.build);
//get device info

	/*error_us= VL53L1_GetDeviceInfo( dev2,	&pVL53L1_DeviceInfo);
	 if (error_us!=0) {
	 ESP_LOGI(TAG, "GetDeviceInfor;%d======================================================", error_us);
	 }

	 //show the device info to monitor
	 ESP_LOGI(TAG, "device name is; %s", pVL53L1_DeviceInfo.Name);
	 ESP_LOGI(TAG, "device ProductId is; %s", pVL53L1_DeviceInfo.ProductId);
	 ESP_LOGI(TAG, "device Type is; %s", pVL53L1_DeviceInfo.Type);
	 ESP_LOGI(TAG, "device ProductRevisionMajor is; %d", pVL53L1_DeviceInfo.ProductRevisionMajor);
	 ESP_LOGI(TAG, "device ProductRevisionMinor is; %d", pVL53L1_DeviceInfo.ProductRevisionMinor);
	 ESP_LOGI(TAG, "device ProductType is; %d", pVL53L1_DeviceInfo.ProductType);*/

	/*error_us=VL53L1_WaitDeviceBooted(dev2);

	 if (error_us!=0) {
	 ESP_LOGI(TAG, "WaitDeviceBooted error;%d", error_us);
	 }*/
	error_us = VL53L1_DataInit(dev2);

	if (error_us != 0) {
		ESP_LOGI(TAG, "DataInit error;%d", error_us);
	}
	error_us = VL53L1_StaticInit(dev2);
	if (error_us != 0) {
		ESP_LOGI(TAG, "StaticInit error;%d", error_us);
	}

//CALIBRATIONCALIBRATIONCALIBRATION///////////////////////////////////////////////////////////////////////
	if (do_calibration) {
		error_us = VL53L1_PerformRefSpadManagement(dev2);
		if (error_us != 0) {
			ESP_LOGE(TAG, "PerformRefSpadManagement error;%d", error_us);
		}

		//offset calibration- object at measured distance and in low light
		error_us = VL53L1_PerformOffsetSimpleCalibration(dev2, CalDistanceMilliMeter);
		if (error_us != 0) {
			ESP_LOGE(TAG, "erformOffsetSimpleCalibration error;%d", error_us);
		}

		//crosstalk calibration object at x mm and with no ir
		error_us = VL53L1_PerformSingleTargetXTalkCalibration(dev2, CalDistanceMilliMeter);
		if (error_us != 0) {
			ESP_LOGE(TAG, "VL53L1_PerformSingleTargetXTalkCalibration error;%d", error_us);
		}
		ESP_LOGI(TAG, "Calibrations Done");
	}
	//END CALIBRATIONS////////////////////////////////////////////////////////

	///////get didstance mode and preset modes----------------------
		VL53L1_DistanceModes DistanceMode = distanceMode;
		VL53L1_PresetModes PresetMode;
		printf("hiya\n");

///set preset mode--------------------
	error_us = VL53L1_SetPresetMode(dev2, VL53L1_PRESETMODE_AUTONOMOUS);
	if (error_us != 0) {
		ESP_LOGE(TAG, "SetPreseteMode(dev2) error;%d", error_us);
	}

	///set didstance m ode----------------------------
	error_us = VL53L1_SetDistanceMode(dev2, DistanceMode);
	if (error_us != 0) {
		ESP_LOGI(TAG, "SetDistanceMode(dev2) error;%d", error_us);
	}
	PresetMode=0;
//get preset mode --------------------------------------
	error_us = VL53L1_GetPresetMode(dev2, &PresetMode);
	if (error_us != 0) {
		ESP_LOGI(TAG, "GetPreseteMode(dev2) error;%d", error_us);
	}

	ESP_LOGI(TAG, "GetPresetMode return ;%d", PresetMode);

//get timing budget us =======================================

	error_us = VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev2, timingBudget_uS);
	if (error_us != 0) {
		ESP_LOGI(TAG, "VL53L1_GetMeasurementTimingBudgetMicroSeconds(dev2) error;%d", error_us);
	}
	ESP_LOGI(TAG, "VL53L1_GetMeasurementTimingBudgetMicroSeconds return ;%d", timingBudget_uS);

//set timing budget us =======================================

	error_us = VL53L1_SetInterMeasurementPeriodMilliSeconds(dev2, intermeasurement_mS);
	if (error_us != 0) {
		ESP_LOGI(TAG, "VL53L1_SetInterMeasurementPeriodMilliSeconds error;%d", error_us);
	}
	ESP_LOGI(TAG, "VL53L1_GetInterMeasurementPeriodMilliSeconds return ;%d", intermeasurement_mS);

	//GET Calibration Data=========================================================
	error_us = VL53L1_GetCalibrationData(dev2, &CalibrationData);
	if (error_us != 0) {
		ESP_LOGI(TAG, "StartMeasurement(dev2) error;%d", error_us);
	}
	ESP_LOGI(TAG, "CalibrationData is in");

	error_us = VL53L1_StartMeasurement(dev2);
	if (error_us != 0) {
		ESP_LOGI(TAG, "StartMeasurement(dev2) error;%d", error_us);

	}
	error_us = VL53L1_WaitMeasurementDataReady(dev2);

	if (error_us != 0) {
		ESP_LOGI(TAG, "WaitMeasurementDataReady error;%d", error_us);

	}
	error_us = VL53L1_GetRangingMeasurementData(dev2, &pRangingMeasurementData);
	if (error_us != 0) {
		ESP_LOGI(TAG, "mGetRangingMeasurementData error %d", error_us);

	}
	error_us = VL53L1_ClearInterruptAndStartMeasurement(dev2);
	if (error_us != 0) {
		ESP_LOGI(TAG, "ClearInterruptAndStartMeasurement error %d", error_us);

	}
//printmm

	ESP_LOGI(TAG, "AmbientRateRtnMegaCps/65536 is; %f",	(float )(pRangingMeasurementData.AmbientRateRtnMegaCps / 65536));
	ESP_LOGI(TAG, "RangeStatus; %d", pRangingMeasurementData.RangeStatus);
	ESP_LOGI(TAG, "SigmaMilliMeter/65536; %f", (float )(pRangingMeasurementData.SigmaMilliMeter / 65536));
	ESP_LOGI(TAG, "MM really================= is; %d", pRangingMeasurementData.RangeMilliMeter);



}//close method app_main


