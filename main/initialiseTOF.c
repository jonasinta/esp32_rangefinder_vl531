/*
 * initialiseTOF.c
 *
 *  Created on: 24/04/2020
 *      Author: jonas
 */

	gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
//initialise I2C
	i2c_init();
	int8_t error_us;
	uint16_t temp16bit = 0;
	uint32_t temp32bit = 0;
	VL53L1_Version_t pVersion;
	VL53L1_DeviceInfo_t pVL53L1_DeviceInfo;
	VL53L1_RangingMeasurementData_t pRangingMeasurementData;
	VL53L1_CalibrationData_t CalibrationData;

	dev2 = &dev;

	dev.I2cDevAddr = 0x52;
	dev.comms_speed_khz = 50;//400;

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
	temp32bit = timingBudget_uS;
	error_us = VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev2, temp32bit);
	if (error_us != 0) {
		ESP_LOGI(TAG, "VL53L1_GetMeasurementTimingBudgetMicroSeconds(dev2) error;%d", error_us);
	}
	ESP_LOGI(TAG, "VL53L1_GetMeasurementTimingBudgetMicroSeconds return ;%d", temp32bit);

//set timing budget us =======================================
	temp32bit = intermeasurement_mS;
	error_us = VL53L1_SetInterMeasurementPeriodMilliSeconds(dev2, temp32bit);
	if (error_us != 0) {
		ESP_LOGI(TAG, "VL53L1_SetInterMeasurementPeriodMilliSeconds error;%d", error_us);
	}
	ESP_LOGI(TAG, "VL53L1_GetInterMeasurementPeriodMilliSeconds return ;%d", temp32bit);

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

	int level = 0;
