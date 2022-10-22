typedef struct location_config_servier {
	char strServerHost[2][32];
	unsigned int U32ServerPort[2];
	char strServerPass[2][32];
	char strServerUserName[2][32];
	char strFirstServerHost[32];
	unsigned int U32ReconnectTime;
}STRU_location_config_servier;

typedef struct location_config_power {
	int slPowerState;//0 not sleep 1 sleep.the device sleep state

}STRU_location_config_power;

typedef struct location_config_hardware {
	unsigned int ulTemp;//251 means 25.1
	unsigned int ulcollision;//1 true 0 false
	unsigned int ulBatteryCap;//99%
	unsigned int ulOutoftheFense;//1-out 0-in
	unsigned int ulUpdateSucc;//1-success 2-software download fail 3-update fail 4-other
	unsigned int ulHarewareInfo;//4-date 20-ICCID 1-softwareversion 2-hardwareversion 6-wifimac
	unsigned int ulChargerState;//0-Not Charger 1-Charging
	unsigned int ulStepToday;//Today Step
	unsigned int *pulStep;//other step except today.everyday
	unsigned int ulLedStatus;//led status
	unsigned int ulLastDeviceOps;//1-reboot 2-poweroff 3-cleandata
	unsigned int ulDeviceRunTime;//seconds
}STRU_location_config_hardware;

typedef struct location_config_appconfig {
	unsigned int ulAppuser;//app user online
	int ulServiceAskDeviceSleep;//0-Not sleep 1-sleep
	unsigned int ulLocationSleepContinue;//1 true 0 false
	unsigned int ulGpsFenseReport;//distance.A > B.then report
	unsigned int ulLocationData;//GPS+WIFI+LBS
	unsigned int ulUserInfo;//message 9.I dont understand
	unsigned int ulWifiData;//message 10 I dont understand
	unsigned int ulUpdata;//OTA Update
	unsigned int *pulVoiceData;//message 14
	unsigned int ulSADops;//SAD-service ask device 1-reboot 2-poweroff 3-cleandata
	unsigned int ulConfigPara;//1-collision 2-tempUlimit 3-tempDlimit 4-led 5-temp I don;t understand
	unsigned int ulConfigparastyle;//message 19 I don;t understand
	unsigned int ulTemp;//message 20.I dont understand
	unsigned int ulBaterrySet;//Unreasonable
}STRU_location_config_appconfig;

typedef struct location_config {
	STRU_location_config_servier stlocation_config_service;
	STRU_location_config_power  stlocation_config_power;
	STRU_location_config_hardware stlocation_config_hardware;
	STRU_location_config_appconfig stlocation_config_appconfig;
}STRU_location_config;
