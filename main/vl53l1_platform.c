
/*
* Copyright (c) 2017, STMicroelectronics - All Rights Reserved
*
* This file is part of VL53L1 Core and is dual licensed,
* either 'STMicroelectronics
* Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, VL53L1 Core may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
********************************************************************************
*
*/


#include "vl53l1_platform.h"
// #include "vl53l1_platform_log.h"
#include "vl53l1_api.h"

// #include "stm32xxx_hal.h"
#include <string.h>
#include "esp32/rom/ets_sys.h"
// #include <time.h>
// #include <math.h>
//jonases preprocesser---------------------------------------------------
#include "sdkconfig.h"
#include"driver/i2c.h"
#include "esp_log.h"
#include"esp_err.h"

//jonases globals-------------------------------------------------------
static const char* TAG = "platformI2cModule";
uint8_t I2CAddress = 0x52;

// #define I2C_TIME_OUT_BASE   10
// #define I2C_TIME_OUT_BYTE   1

// #ifdef VL53L1_LOG_ENABLE
// #define trace_print(level, ...) VL53L1_trace_print_module_function(VL53L1_TRACE_MODULE_PLATFORM, level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)
// #define trace_i2c(...) VL53L1_trace_print_module_function(VL53L1_TRACE_MODULE_NONE, VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)
// #endif

// #ifndef HAL_I2C_MODULE_ENABLED
// #warning "HAL I2C module must be enable "
// #endif

//extern I2C_HandleTypeDef hi2c1;
//#define VL53L0X_pI2cHandle    (&hi2c1)

/* when not customized by application define dummy one */
// #ifndef VL53L1_GetI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
// #   define VL53L1_GetI2cBus(...) (void)0
// #endif

// #ifndef VL53L1_PutI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
// #   define VL53L1_PutI2cBus(...) (void)0
// #endif

// uint8_t _I2CBuffer[256];

// int _I2CWrite(VL53L1_DEV Dev, uint8_t *pdata, uint32_t count) {
//     int status = 0;
//     return status;
// }

// int _I2CRead(VL53L1_DEV Dev, uint8_t *pdata, uint32_t count) {
//    int status = 0;
//    return Status;
// }

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	ESP_LOGD(TAG, "debug log tester in platform WriteMulti VL53L1_WriteMulti %s\n",	"stringtester");
		esp_err_t error_error;

	//	ESP_LOGI(TAG, "index from parameter input %x", index);
		uint16_t indextemp;
		indextemp = index;
	//	ESP_LOGI(TAG, "indextemp should be same as index %x", indextemp);

		uint8_t indexmsb = indextemp >> 8;
	//	ESP_LOGI(TAG, "indextemp sjust after rightshif %x", indextemp);
		uint8_t indexlsb = indextemp & 0xff;
	//	ESP_LOGI(TAG, "indexmsb, indexlsb; %x %x", indexmsb,indexlsb);
//


		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		//strat i2c and send read index address of 2 bytes
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, ( I2CAddress ), I2C_MASTER_ACK);
		i2c_master_write_byte(cmd, indexmsb, I2C_MASTER_ACK);
		i2c_master_write_byte(cmd, indexlsb, I2C_MASTER_ACK);

		i2c_master_write(cmd, pdata, count, I2C_MASTER_LAST_NACK);
		i2c_master_stop(cmd);

	//start i2c commands
		error_error = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS );
		i2c_cmd_link_delete(cmd);

		if (error_error != ESP_OK ) {
			ESP_LOGI(TAG, "Error in I2C WriteMulti %s    ", esp_err_to_name(error_error));


		}  //close if
		else {
			for (int var = 0; var < count; ++var) {
				ESP_LOGD(TAG, "wrote to WriteMulti %d\n", *(pdata+var));

			}


		}
	return error_error;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	ESP_LOGD(TAG, "debug log tester in platform multibyteread VL53L1_ReadMulti %s",	"==============================================");
	esp_err_t error_error;

	//ESP_LOGI(TAG, "index from parameter input %x", index);
	uint16_t indextemp;
	indextemp = index;
//	ESP_LOGI(TAG, "indextemp should be same as index %x", indextemp);

	uint8_t indexmsb = indextemp >> 8;
//	ESP_LOGI(TAG, "indextemp sjust after rightshif %x", indextemp);
	uint8_t indexlsb = indextemp & 0xff;
//	ESP_LOGI(TAG, "indexmsb, indexlsb; %x %x", indexmsb,indexlsb);



	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	//strat i2c and send read index address of 2 bytes
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, ( I2CAddress ), I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, indexmsb, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, indexlsb, I2C_MASTER_ACK);
	//i2c_master_write_byte(cmd, 0x01, 1);
	//i2c_master_write_byte(cmd, 0x10, 1);
	//i2c_master_stop(cmd);

	//read the device data- send adress plus 1, then read the byte
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, ( I2CAddress ) | 0x01, I2C_MASTER_ACK); //or dev addtress wit 1 to ready for read
		i2c_master_read(cmd, pdata, count, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);

//start i2c commands
	error_error = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS );
	i2c_cmd_link_delete(cmd);

	if (error_error != ESP_OK) {
		ESP_LOGI(TAG, "Error in I2C read multi %s  \n  ", esp_err_to_name(error_error));


	}  //close if
	else {
		for (int var = 0; var < count; ++var) {
		//	ESP_LOGI(TAG, "value from i2c device VL53L1_RdByte %d\n", *(pdata+var));

		}


	}
	return error_error; // to be implemented
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data) {
	ESP_LOGI(TAG, "debug log tester in platform write8bit VL53L1_WrByte %s",	"================");
	esp_err_t error_error;

		//	ESP_LOGI(TAG, "index from parameter input %x", index);
			uint16_t indextemp;
			indextemp = index;
		//	ESP_LOGI(TAG, "indextemp should be same as index %x", indextemp);

			uint8_t indexmsb = indextemp >> 8;
		//	ESP_LOGI(TAG, "indextemp sjust after rightshif %x", indextemp);
			uint8_t indexlsb = indextemp & 0xff;
		//	ESP_LOGI(TAG, "indexmsb, indexlsb; %x %x", indexmsb,indexlsb);



			i2c_cmd_handle_t cmd = i2c_cmd_link_create();
			//strat i2c and send read index address of 2 bytes
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, I2CAddress, I2C_MASTER_ACK);
			i2c_master_write_byte(cmd, indexmsb, I2C_MASTER_ACK);
			i2c_master_write_byte(cmd, indexlsb, I2C_MASTER_ACK);

			//read the device data- send adress plus 1, then read the byte
			//i2c_master_start(cmd);
			//i2c_master_write_byte(cmd, I2CAddress | 0x01, I2C_MASTER_ACK); //or dev addtress wit 1 to ready for read
				i2c_master_write_byte(cmd, data,  I2C_MASTER_ACK);
			i2c_master_stop(cmd);

		//start i2c commands
			error_error = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS );
			i2c_cmd_link_delete(cmd);

			if (error_error != ESP_OK ) {
				ESP_LOGI(TAG, "Error in I2C WriteByte %s    ", esp_err_to_name(error_error));


			}  //close if
			else {

					ESP_LOGI(TAG, "wrote to WriteByte %d\n", data);

							}//close else
		return error_error;
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data) {
	ESP_LOGI(TAG, "debug log tester in platform write16bit VL53L1_WrWord %s",	"================");
	return 0; // to be implemented
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data) {
	ESP_LOGI(TAG, "debug log tester in platform write32bit VL53L1_WrDWord %s",	"================");
	return 0; // to be implemented
}

VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData) {
	ESP_LOGI(TAG, "debug log tester in platform UpdateByte %s",	"======================");
	return 0; // to be implemented
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data) {
	ESP_LOGI(TAG, "debug log tester in platform read8bit VL53L1_RdByte %s",	"======================");
	esp_err_t error_error;

	//ESP_LOGI(TAG, "index from parameter input %x", index);
	uint16_t indextemp;
	indextemp = index;
	//ESP_LOGI(TAG, "indextemp should be same as index %x", indextemp);

	uint8_t indexmsb = indextemp >> 8;
//	ESP_LOGI(TAG, "indextemp sjust after rightshif %x", indextemp);
	uint8_t indexlsb = indextemp & 0xff;
	//ESP_LOGI(TAG, "indexmsb, indexlsb; %x %x", indexmsb,indexlsb);



	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	//strat i2c and send read index address of 2 bytes
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, I2CAddress, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, indexmsb, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, indexlsb, I2C_MASTER_ACK);
	//i2c_master_write_byte(cmd, 0x01, 1);
	//i2c_master_write_byte(cmd, 0x10, 1);
	//i2c_master_stop(cmd);

	//read the device data- send adress plus 1, then read the byte
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, I2CAddress | 0x01, I2C_MASTER_ACK); //or dev addtress wit 1 to ready for read
	i2c_master_read_byte(cmd, data,  I2C_MASTER_NACK);
	i2c_master_stop(cmd);

//start i2c commands
	error_error = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS );
	i2c_cmd_link_delete(cmd);

	if (error_error != ESP_OK) {
		ESP_LOGI(TAG, "Error in I2C 8bit comm %s   \n ", esp_err_to_name(error_error));


	}  //close if
	else {
		ESP_LOGI(TAG, "value from i2c device VL53L1_RdByte hex %x, decimal %d \n", *data, *data);

	}
	return error_error;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data) {
	ESP_LOGD(TAG, "debug log tester in platform read16bit VL53L1_RdWord %s",	"======================");
	esp_err_t error_error;
	uint8_t tempdata[2] ={0,0};

	//ESP_LOGI(TAG, "index from parameter input %x", index);
	uint16_t indextemp;
	indextemp = index;
	//ESP_LOGI(TAG, "indextemp should be same as index %x", indextemp);

	uint8_t indexmsb = indextemp >> 8;
//	ESP_LOGI(TAG, "indextemp sjust after rightshif %x", indextemp);
	uint8_t indexlsb = indextemp & 0xff;
//	ESP_LOGI(TAG, "indexmsb, indexlsb; %x %x", indexmsb,indexlsb);



	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	//strat i2c and send read index address of 2 bytes
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, I2CAddress, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, indexmsb, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, indexlsb, I2C_MASTER_ACK);
	//i2c_master_write_byte(cmd, 0x01, 1);
	//i2c_master_write_byte(cmd, 0x10, 1);
	//i2c_master_stop(cmd);

	//read the device data- send adress plus 1, then read the byte
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, I2CAddress | 0x01, I2C_MASTER_ACK); //or dev addtress wit 1 to ready for read
	i2c_master_read_byte(cmd, tempdata+1, I2C_MASTER_ACK);
	i2c_master_read_byte(cmd, tempdata, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

//start i2c commands
	error_error = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS );
	i2c_cmd_link_delete(cmd);

	if (error_error != ESP_OK) {
		ESP_LOGI(TAG, "Error in I2C 16bit comm %s   \n ", esp_err_to_name(error_error));


	}  //close if
	else {
		ESP_LOGD(TAG, "tempdata i2c device has read first byte and MSB %0x and second byte and LSB %0x", *(tempdata+1), *(tempdata));
		//assemble tempdata 8bit to data 16bit
		*data   = (uint16_t)tempdata[0] ;

		*data  = ((uint16_t)tempdata[1] << 8) | *data;


		for (int var = 0; var < 2; ++var) {
		ESP_LOGD(TAG, "tempdata i2c device VL53L1_RdByte16bit hex %x, decimal %d itteration %d of 2", *(tempdata+var), *(tempdata+var), var);
			}//close for

	}
	return error_error;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data) {
	ESP_LOGI(TAG, "debug log tester in platform read32bit VL53L1_RdDWord %s",	"======================");
	return 0; // to be implemented
}

VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	return status;
}

//#define trace_print(level, ...)
//	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_PLATFORM,
//	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

//#define trace_i2c(...)
//	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_NONE,
//	VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	ESP_LOGI(TAG, "debug log tester in platform GetTimerFrequency %s",	"======================");
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	return status;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms){
	ESP_LOGI(TAG, "debug log tester in platform WaitMs %s",	"======================");
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	return status;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us){
	ESP_LOGI(TAG, "debug log tester in platform Waitus %s",	"======================");
	ets_delay_us(wait_us);
	return 0; // to be implemented
}

VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	return status;
}




