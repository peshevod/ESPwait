#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include <string.h>
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_int_wdt.h"
#include "esp_task_wdt.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "spi_intf.h"
//#include "shell.h"
#include "s2lp_console.h"
#include "cmd_nvs.h"
#include "main.h"
#include "S2LP_Config.h"
#include "radio.h"
#include "MCU_interface.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"
#include "aws_iot_shadow_interface.h"
#include "spp_server.h"
//#include "bt/host/bluedroid/api/include/api/esp_bt_main.h"
//#include "soc/rtc.h"

#define SLEEP
#define AWS_CLIENT_ID "721730703209"
#define MAX_LENGTH_OF_UPDATE_JSON_BUFFER 512

//static QueueHandle_t uart2_queue;
static uint8_t* data0;
static int16_t i0;
//static uint8_t* data2;
static const char* TAG = "ESPwait";
static const char* TAG1 = "SendToCloud";
static const char *TAGW = "wifi station";
char buf[MAX_MESSAGE_SIZE];
char mes1[MAX_MESSAGE_SIZE];
char mes2[MAX_MESSAGE_SIZE];
uint8_t con,mqtt_con,aws_con;
tcpip_adapter_if_t ifindex;
static input_data_t mes,data;
static DRAM_ATTR xQueueHandle s2lp_evt_queue = NULL;
static S2LPIrqs xIrqStatus;
static wifi_config_t sta_config;
static uint8_t ready_to_send=0;
uint32_t uid;

extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_private_pem_key_end");
/**
 * @brief Default MQTT HOST URL is pulled from the aws_iot_config.h
 */
char HostAddress[255] = "af0rqdl7ywamp-ats.iot.us-east-2.amazonaws.com";

char JsonDocumentBuffer[MAX_LENGTH_OF_UPDATE_JSON_BUFFER];
size_t sizeOfJsonDocumentBuffer = sizeof(JsonDocumentBuffer) / sizeof(JsonDocumentBuffer[0]);

RTC_SLOW_ATTR sn_table_t table;
RTC_SLOW_ATTR uint32_t seq;

static int s_retry_num = 0;
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about one event
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;


static void initialize_nvs()
{
    esp_err_t err = nvs_flash_init_partition("s2lp");
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "s2lp partition error: %s", esp_err_to_name(err));
    }
    ESP_ERROR_CHECK(err);

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Default partition error: %s", esp_err_to_name(err));
    }
    ESP_ERROR_CHECK(err);
}

void init_uart0()
{
    uart_config_t uart_config0 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config0);
    uart_set_pin(UART_NUM_0, UART0_TXD, UART0_RXD, UART0_RTS, UART0_CTS);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    data0 = (uint8_t *) malloc(BUF_SIZE);
}


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START && con) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    	if(con)
    	{
    		if (s_retry_num < ESP_MAXIMUM_RETRY) {
    			esp_wifi_connect();
    			xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    			s_retry_num++;
    			ESP_LOGI(TAGW, "retry to connect to the AP ssid=%s pass=%s retry=%d",sta_config.sta.ssid,sta_config.sta.password,s_retry_num);
    		}
    		ESP_LOGI(TAGW,"connect to the AP fail");
    	}
    	else
   		{
   			tcpip_adapter_stop(ifindex);
    		ESP_LOGI(TAGW,"wifi disconnected");
   		}
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAGW, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        ifindex=event->if_index;
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ready_to_send=1;
    }
}

void send_to_cloud()
{
    con=1;
    ESP_ERROR_CHECK( esp_wifi_start() );
//    ESP_ERROR_CHECK( esp_wifi_connect() );
   	ESP_LOGI(TAG1,"--- Begin of transfer\n");
    while(1)
    {
    	ESP_LOGI(TAG1,"REC: Power: %d dbm 0x%08X 0x%08X 0x%08X\n",mes.input_signal_power,mes.seq_number,mes.serial_number,mes.data[0]);
    	vTaskDelay(10000 / portTICK_PERIOD_MS);
    	if(!xQueueReceive(s2lp_evt_queue,&mes,100/portTICK_PERIOD_MS)) break;
    }
   	ESP_LOGI(TAG1,"--- End of transfer\n");
	con=0;
    ESP_ERROR_CHECK( esp_wifi_disconnect() );
    ESP_ERROR_CHECK( esp_wifi_stop() );
}


static void IRAM_ATTR s2lp_intr_handler(void* arg)
{
	input_data_t data_in;
	S2LPTimerLdcIrqWa(S_ENABLE);
	S2LPGpioIrqGetStatus(&xIrqStatus);
    if(xIrqStatus.RX_DATA_READY)
    {
        //Get the RX FIFO size
        uint8_t cRxData = S2LPFifoReadNumberBytesRxFifo();
        //Read the RX FIFO
        S2LPSpiReadFifo(cRxData, (uint8_t*)(&(data_in.seq_number)));
        //Flush the RX FIFO
        S2LPCmdStrobeFlushRxFifo();
        data_in.input_signal_power=S2LPRadioGetRssidBm();
        S2LPCmdStrobeSleep();
        xQueueSendFromISR(s2lp_evt_queue,&data_in,0);
    }
    S2LPTimerLdcIrqWa(S_DISABLE);
}

void test_gpio(void)
{

	gpio_num_t gpio_nums[4]={4,2,26,25};


	for(S2LPGpioPin pin=0;pin<4;pin++)
	{
		gpio_iomux_out(gpio_nums[pin], 2, false);
		gpio_set_direction(gpio_nums[pin], GPIO_MODE_INPUT);
		printf("\nTest of s2lppin %d and gpio_pin %d:\n",pin,gpio_nums[pin]);

		OutputLevel ol=OUTPUT_LOW;

		for(int j=0;j<4;j++)
		{
			S2LPGpioSetLevel(pin,ol);
			printf("Status 0x%02X 0x%02X\n",g_xStatus.MC_STATE,g_xStatus.XO_ON);
			vTaskDelay(500 / portTICK_PERIOD_MS);
			int il=gpio_get_level(gpio_nums[pin]);
			if((il && ol) || (!il && !ol)) printf("outputlevel=%d inputlevel=%d S2LPPin %d GPIO %d - OK\n",ol,il,pin,gpio_nums[pin]);
			else printf("outputlevel=%d inputlevel=%d S2LPPin %d GPIO %d - FAIL\n",ol,il,pin,gpio_nums[pin]);
			ol=1-ol;
			vTaskDelay(500 / portTICK_PERIOD_MS);
		}
	}

}

static void s2lp_wait(void *arg)
{
    int32_t t=600;
    uint32_t xc=0;
    s_wifi_event_group = xEventGroupCreate();
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    get_value_from_nvs("SSID",0,NULL,sta_config.sta.ssid);
    get_value_from_nvs("PASSWD",0,NULL,sta_config.sta.password);
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    while (1)
    {
    	if(s2lp_evt_queue==NULL)
		{
    		vTaskDelay(100/portTICK_PERIOD_MS);
    		continue;
		}
    	if(xQueueReceive(s2lp_evt_queue,&mes,100/portTICK_PERIOD_MS))
        {
        	ESP_LOGI(TAG,"REC: Power: %d dbm 0x%08X 0x%08X 0x%08X\n",mes.input_signal_power,mes.seq_number,mes.serial_number,mes.data[0]);
//        	send_to_cloud();
        };
        t--;
        if(t<=0)
        {
        	ESP_LOGI(TAG,"i am alive 0x%08X\n",xc);
        	t=600;
        	xc++;
        }
    }

}


static void wifi_handlers()
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
}


static void wifi_prepare()
{
	wifi_handlers();
    tcpip_adapter_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    get_value_from_nvs("SSID",0,NULL,sta_config.sta.ssid);
    get_value_from_nvs("PASSWD",0,NULL,sta_config.sta.password);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    con=1;
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );
}

static int16_t test_update_table(uint32_t ser, uint32_t seq)
{
	uint16_t n=table.n_of_rows;
	int16_t i;
	for(i=0;i<n;i++)
	{
		if(ser==table.row[i].serial_number) break;
	}
	if( i==n || table.row[i].seq!=(uint16_t)(seq&0xffff)) return i;
	else return -1;
}

static void update_table(uint32_t ser, uint32_t seq,int16_t i)
{
	table.row[i].serial_number=ser;
	table.row[i].seq=(uint16_t)(seq&0xffff);
	if(i>=table.n_of_rows) table.n_of_rows++;
}

static bool shadowUpdateInProgress;

void ShadowUpdateStatusCallback(const char *pThingName, ShadowActions_t action, Shadow_Ack_Status_t status,
                                const char *pReceivedJsonDocument, void *pContextData) {
    IOT_UNUSED(pThingName);
    IOT_UNUSED(action);
    IOT_UNUSED(pReceivedJsonDocument);
    IOT_UNUSED(pContextData);

    shadowUpdateInProgress = false;

    if(SHADOW_ACK_TIMEOUT == status) {
        ESP_LOGE(TAG, "Update timed out");
    } else if(SHADOW_ACK_REJECTED == status) {
        ESP_LOGE(TAG, "Update rejected");
    } else if(SHADOW_ACK_ACCEPTED == status) {
        ESP_LOGI(TAG, "Update accepted");
    }
}

void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData) {
    ESP_LOGI(TAG, "Subscribe callback");
    ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);
}

void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data) {
    ESP_LOGW(TAG, "MQTT Disconnect");

    if(!mqtt_con) return;

    IoT_Error_t rc = FAILURE;

    if(NULL == pClient) {
        return;
    }

    if(aws_iot_is_autoreconnect_enabled(pClient)) {
        ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
    } else {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if(NETWORK_RECONNECTED == rc) {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        } else {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}

AWS_IoT_Client client;
IoT_Client_Init_Params mqttInitParams;
IoT_Client_Connect_Params connectParams;
IoT_Publish_Message_Params paramsQOS0;
IoT_Publish_Message_Params paramsQOS1;
const char *TOPIC = "espwait/sensor";

ShadowInitParameters_t sp;
ShadowConnectParameters_t scp;

jsonStruct_t seq_h, power_h,modseq_h,modsn_h,mod_jp4_h,mod_jp5_h,state_jp4_h,state_jp5_h,alarm_jp4_h,alarm_jp5_h;
uint8_t mod_jp4,mod_jp5;
bool state_jp4,state_jp5,alarm_jp4,alarm_jp5;
char ThingName[20];


void send_to_cloud1(bool shadow)
{
    IoT_Error_t rc = FAILURE;
    char cPayload[256];

    if(shadow)
    {
        seq_h.cb = NULL;
        seq_h.pKey = "GW_SEQ";
        seq_h.pData = &seq;
        seq_h.type = SHADOW_JSON_UINT32;
        seq_h.dataLength = sizeof(uint32_t);

        power_h.cb = NULL;
        power_h.pKey = "POWER";
        power_h.pData = &(data.input_signal_power);
        power_h.type = SHADOW_JSON_INT32;
        power_h.dataLength = sizeof(int32_t);

        modseq_h.cb = NULL;
        modseq_h.pKey = "MOD_SEQ";
        modseq_h.pData = &(data.seq_number);
        modseq_h.type = SHADOW_JSON_UINT16;
        modseq_h.dataLength = sizeof(uint16_t);

        modsn_h.cb = NULL;
        modsn_h.pKey = "MOD_SN";
        modsn_h.pData = &(data.serial_number);
        modsn_h.type = SHADOW_JSON_UINT32;
        modsn_h.dataLength = sizeof(uint32_t);

        mod_jp4_h.cb = NULL;
        mod_jp4_h.pKey = "MOD_JP4";
        mod_jp4_h.pData = &mod_jp4;
        mod_jp4_h.type = SHADOW_JSON_UINT8;
        mod_jp4_h.dataLength = sizeof(uint8_t);

        mod_jp5_h.cb = NULL;
        mod_jp5_h.pKey = "MOD_JP5";
        mod_jp5_h.pData = &mod_jp5;
        mod_jp5_h.type = SHADOW_JSON_UINT8;
        mod_jp5_h.dataLength = sizeof(uint8_t);

        state_jp4_h.cb = NULL;
        state_jp4_h.pKey = "STATE_JP4";
        state_jp4_h.pData = &state_jp4;
        state_jp4_h.type = SHADOW_JSON_BOOL;
        state_jp4_h.dataLength = sizeof(bool);

        state_jp5_h.cb = NULL;
        state_jp5_h.pKey = "STATE_JP5";
        state_jp5_h.pData = &state_jp5;
        state_jp5_h.type = SHADOW_JSON_BOOL;
        state_jp5_h.dataLength = sizeof(bool);

        alarm_jp4_h.cb = NULL;
        alarm_jp4_h.pKey = "ALARM_JP4";
        alarm_jp4_h.pData = &alarm_jp4;
        alarm_jp4_h.type = SHADOW_JSON_BOOL;
        alarm_jp4_h.dataLength = sizeof(bool);

        alarm_jp5_h.cb = NULL;
        alarm_jp5_h.pKey = "ALARM_JP5";
        alarm_jp5_h.pData = &alarm_jp5;
        alarm_jp5_h.type = SHADOW_JSON_BOOL;
        alarm_jp5_h.dataLength = sizeof(bool);

    }

    if(!aws_con)
    {


    	ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    	if(!shadow)
    	{
     		mqttInitParams = iotClientInitParamsDefault;
    		mqttInitParams.enableAutoReconnect = false; // We enable this later below
    		mqttInitParams.pHostURL = HostAddress;
    		mqttInitParams.port = AWS_IOT_MQTT_PORT;

    		mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
    		mqttInitParams.pDeviceCertLocation = (const char *)certificate_pem_crt_start;
    		mqttInitParams.pDevicePrivateKeyLocation = (const char *)private_pem_key_start;

    		mqttInitParams.mqttCommandTimeout_ms = 20000;
    		mqttInitParams.tlsHandshakeTimeout_ms = 5000;
    		mqttInitParams.isSSLHostnameVerify = true;
    		mqttInitParams.disconnectHandler = disconnectCallbackHandler;
    		mqttInitParams.disconnectHandlerData = NULL;

    		rc = aws_iot_mqtt_init(&client, &mqttInitParams);
    		if(SUCCESS != rc) {
    			ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
    			abort();
    		}
    	}
    	else
    	{

    		sp = ShadowInitParametersDefault;
    		sp.pHost = HostAddress;
            sp.port = AWS_IOT_MQTT_PORT;

            sp.pClientCRT = (const char *)certificate_pem_crt_start;
            sp.pClientKey = (const char *)private_pem_key_start;
            sp.pRootCA = (const char *)aws_root_ca_pem_start;

            sp.enableAutoReconnect = false;
            sp.disconnectHandler = disconnectCallbackHandler;

            ESP_LOGI(TAG, "Shadow Init");
            rc = aws_iot_shadow_init(&client, &sp);
            if(SUCCESS != rc) {
            	ESP_LOGE(TAG, "aws_iot_shadow_init returned error %d, aborting...", rc);
            	abort();
            }
    	}

    	if(!shadow)
    	{
         	connectParams = iotClientConnectParamsDefault;
    		connectParams.keepAliveIntervalInSec = 10;
    		connectParams.isCleanSession = true;
    		connectParams.MQTTVersion = MQTT_3_1_1;
    		/* Client ID is set in the menuconfig of the example */
    		connectParams.pClientID = "721730703209";
    		connectParams.clientIDLen = (uint16_t) strlen("721730703209");
    		connectParams.isWillMsgPresent = false;

    		ESP_LOGI(TAG, "Connecting to AWS...");
    		mqtt_con=1;
    		do {
    			rc = aws_iot_mqtt_connect(&client, &connectParams);
    			if(SUCCESS != rc) {
    				ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
    				vTaskDelay(1000 / portTICK_RATE_MS);
    			}
    		} while(SUCCESS != rc);
    		aws_con=1;
    	}
    	else
    	{
    		scp = ShadowConnectParametersDefault;
    		sprintf(ThingName,"espwait-%08x",uid);
    		scp.pMyThingName=ThingName;
    	    scp.pMqttClientId = "721730703209";
    	    scp.mqttClientIdLen = (uint16_t) strlen("721730703209");

    	    ESP_LOGI(TAG, "Shadow Connecting...");
    		mqtt_con=1;
    		do {
    			rc = aws_iot_shadow_connect(&client, &scp);
    			if(SUCCESS != rc) {
    				ESP_LOGE(TAG, "aws_iot_shadow_connect returned error %d, aborting...", rc);
    			}
    		} while(SUCCESS != rc);
    	    ESP_LOGI(TAG, "Shadow Connected");
    		aws_con=1;
    	}



    		/*
     * Enable Auto Reconnect functionality. Minimum and Maximum time of Exponential backoff are set in aws_iot_config.h
     *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
     *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
     */
    	if(!shadow)
    	{
    		rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
    		if(SUCCESS != rc) {
    			ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
    			abort();
    		}
    	}
    	else
    	{
    	    /*
    	     * Enable Auto Reconnect functionality. Minimum and Maximum time of Exponential backoff are set in aws_iot_config.h
    	     *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
    	     *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
    	     */
    	    rc = aws_iot_shadow_set_autoreconnect_status(&client, true);
    	    if(SUCCESS != rc) {
    	        ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d, aborting...", rc);
    	        abort();
    	    }
    	}
    }
    	/*ESP_LOGI(TAG, "Subscribing...");
    	rc = aws_iot_mqtt_subscribe(&client, TOPIC, TOPIC_LEN, QOS0, iot_subscribe_callback_handler, NULL);
    	if(SUCCESS != rc) {
    		ESP_LOGE(TAG, "Error subscribing : %d ", rc);
    		abort();
    	}*/

//    	sprintf(cPayload, "%s : %d ", "hello from SDK", i);

    	/*paramsQOS0.qos = QOS0;
    	paramsQOS0.payload = (void *) cPayload;
    	paramsQOS0.isRetained = 0;*/

    paramsQOS1.qos = QOS1;
    paramsQOS1.payload = (void *) cPayload;
    paramsQOS1.isRetained = 0;

    do {

/*    		//Max time the yield function will wait for read messages
    		rc = aws_iot_mqtt_yield(&client, 100);
    		if(NETWORK_ATTEMPTING_RECONNECT == rc) {
    			// If the client is attempting to reconnect we will skip the rest of the loop.
    			continue;
    		}

        ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(1000 / portTICK_RATE_MS);
        sprintf(cPayload, "%s : %d ", "hello from ESP32 (QOS0)", i++);
        paramsQOS0.payloadLen = strlen(cPayload);
        rc = aws_iot_mqtt_publish(&client, TOPIC, TOPIC_LEN, &paramsQOS0);

        sprintf(cPayload, "%s : %d ", "hello from ESP32 (QOS1)", i++);
        paramsQOS1.payloadLen = strlen(cPayload);
        rc = aws_iot_mqtt_publish(&client, TOPIC, TOPIC_LEN, &paramsQOS1);
        if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
            ESP_LOGW(TAG, "QOS1 publish ack not received.");
            rc = SUCCESS;
        }*/
//    	ESP_LOGI("send_t0_cloud","UID=%08X",uid);
    	if(!shadow)
    	{
    		sprintf(cPayload, "{\n\t\"GW_SEQ\" : %d\n\t\"GW_UID\" : \"0x%08X\"\n\t\"POWER\" : %d\n\t\"MOD_SEQ\" : \"0x%08X\"\n\t\"MOD_SN\" : \"0x%08X\"\n\t\"SENSOR\" : \"0x%08X\"\n\t\"JP4_MODE\" : %d\n\t\"JP5_MODE\" : %d\n\t\"JP4_STATE\" : \"%s\"\n\t\"JP5_STATE\" : \"%s\"\n\t\"JP4_ALARM\" : \"%s\"\n\t\"JP5_ALARM\" : \"%s\"\n}\n"
    			,seq++,uid,data.input_signal_power,data.seq_number,data.serial_number,data.data[0],(data.data[0]&0x00FF0000)>>16,(data.data[0]&0xFF000000)>>24,
				data.data[0]&0x00000100 ? "ON": "OFF",data.data[0]&0x00000200 ? "ON": "OFF",data.data[0]&0x00000001 ? "ON": "OFF",data.data[0]&0x00000002 ? "ON": "OFF");
    		paramsQOS1.payloadLen = strlen(cPayload);
    		rc = aws_iot_mqtt_publish(&client, TOPIC, strlen(TOPIC), &paramsQOS1);
    		if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
    			ESP_LOGW(TAG, "QOS1 publish ack not received.");
//            	rc = SUCCESS;
    		}
    		else ESP_LOGI(TAG,"Published to %s:\n%s",TOPIC,cPayload);
    	}
    	else
    	{
            mod_jp4=(data.data[0]&0x00FF0000)>>16;
            mod_jp5=(data.data[0]&0xFF000000)>>24;
            state_jp4=data.data[0]&0x00000100 ? true : false;
            state_jp5=data.data[0]&0x00000200 ? true : false;
            alarm_jp4=data.data[0]&0x00000001 ? true : false;
            alarm_jp5=data.data[0]&0x00000002 ? true : false;

            rc = aws_iot_shadow_init_json_document(JsonDocumentBuffer, sizeOfJsonDocumentBuffer);
            if(SUCCESS == rc) {
        	    ESP_LOGI(TAG, "JsonDocumentBuffer inited\n%s", JsonDocumentBuffer);
                rc = aws_iot_shadow_add_reported(JsonDocumentBuffer, sizeOfJsonDocumentBuffer,10,&seq_h,&power_h,&modseq_h,&modsn_h,&mod_jp4_h,&mod_jp5_h,&state_jp4_h,&state_jp5_h,&alarm_jp4_h,&alarm_jp5_h);
                if(SUCCESS == rc) {
                	ESP_LOGI(TAG, "JsonDocumentBuffer add reported\n%s", JsonDocumentBuffer);
                    rc = aws_iot_finalize_json_document(JsonDocumentBuffer, sizeOfJsonDocumentBuffer);
                    if(SUCCESS == rc) {
                        ESP_LOGI(TAG, "Update Shadow: %s", JsonDocumentBuffer);
                        rc = aws_iot_shadow_update(&client, scp.pMyThingName, JsonDocumentBuffer,
                                                   ShadowUpdateStatusCallback, NULL, 4, true);
                        if(rc==SUCCESS) ESP_LOGI(TAG, "Updated Shadow to device %s: %s",scp.pMyThingName, JsonDocumentBuffer);
                        shadowUpdateInProgress = true;
                    }
                }
            }

    	}
    	seq++;
    } while((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS != rc));

//     ESP_LOGE(TAG, "An error occurred in the main loop.");
//    abort();
}


static void s2lp_getdata()
{
	input_data_t data_in;
	S2LPTimerLdcIrqWa(S_ENABLE);
	S2LPGpioIrqGetStatus(&xIrqStatus);
    if(xIrqStatus.RX_DATA_READY)
    {
        //Get the RX FIFO size
        uint8_t cRxData = S2LPFifoReadNumberBytesRxFifo();
        //Read the RX FIFO
        S2LPSpiReadFifo(cRxData, (uint8_t*)(&(data_in.seq_number)));
        //Flush the RX FIFO
        S2LPCmdStrobeFlushRxFifo();
        data_in.input_signal_power=S2LPRadioGetRssidBm();
        xQueueSend(s2lp_evt_queue,&data_in,0);
        S2LPCmdStrobeSleep();
    }
    S2LPTimerLdcIrqWa(S_DISABLE);
}

static void s2lp_wait1()
{
    uint32_t dt=100;
    uint32_t x0=16000/dt;
    uint32_t x;
    ready_to_send=0;
    aws_con=0;
    s2lp_evt_queue = xQueueCreate(10, sizeof(input_data_t));
    s2lp_getdata();
    xTaskCreatePinnedToCore(s2lp_rec_start2, "s2lp_rec_start2", 8192, NULL, 10, NULL,1);
	while(xQueueReceive(s2lp_evt_queue,&data,500/portTICK_PERIOD_MS))
	{
        ESP_LOGI("s2lp_getdata","REC: Power: %d dbm 0x%08X 0x%08X 0x%08X\n",data.input_signal_power,data.seq_number,data.serial_number,data.data[0]);
		i0=test_update_table(data.serial_number,data.seq_number);
		if(i0!=-1)
		{

			if(!ready_to_send)
			{
				wifi_prepare();

				x=x0;
				while(x--)
				{
					if(ready_to_send) break;
					else
					{
						vTaskDelay(dt/portTICK_PERIOD_MS);
					}
				}
			}

			if(!ready_to_send)
			{
				ESP_LOGE(TAG,"Cannot connect to %s with %s",sta_config.sta.ssid,sta_config.sta.password);
				continue;
			}

			send_to_cloud1(true);
			update_table(data.serial_number,data.seq_number,i0);
		}
		else ESP_LOGI(TAG,"DO NOT SEND: Power: %d dbm 0x%08X 0x%08X 0x%08X\n",data.input_signal_power,data.seq_number,data.serial_number,data.data[0]);
	}
	if(mqtt_con)
	{
	    mqtt_con=0;
	    IoT_Error_t rc=aws_iot_mqtt_disconnect(&client);
	    if(rc==SUCCESS) ESP_LOGI(TAG,"Disconnected from AWS");
	    aws_con=0;
	}
	if(ready_to_send)
	{
		con=0;
		esp_wifi_disconnect();
		esp_wifi_stop();
	}
}

static void config_isr0(void)
{
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_LOW_LEVEL;
    ESP_LOGI(TAG,"intr level set low");
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
     ESP_LOGI(TAG,"pin set 0x%016llX",io_conf.pin_bit_mask);
   //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    ESP_LOGI(TAG,"gpio mode set input");
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    ESP_LOGI(TAG,"pullup disabled");
    gpio_config(&io_conf);
    ESP_LOGI(TAG,"gpio set");
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    ESP_LOGI(TAG,"isr service set");
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, s2lp_intr_handler, (void*) GPIO_INPUT_IO_0);
    ESP_LOGI(TAG,"handler added to isr service");
}

static void s2lp_rec_start(void *arg)
{

	S2LPSpiInit();
	S2LPExitShutdown();

	start_s2lp_console();

//	test_gpio();

    radio_rx_init(PACKETLEN);
    ESP_LOGI(TAG,"radio_rx_init proceed");

    config_isr0();

	S2LPGpioIrqGetStatus(&xIrqStatus);

    ESP_LOGI(TAG,"s2lp irq Status get");
    S2LPCmdStrobeRx();
    s2lp_evt_queue = xQueueCreate(10, sizeof(input_data_t));
    while(1) vTaskDelay(60000);
}

static void s2lp_rec_start2(void *arg)
{
	config_isr0();
    while(1) vTaskDelay(60000);
}

static void s2lp_rec_start1()
{

	table.n_of_rows=0;
	seq=0;
	S2LPSpiInit();
	S2LPEnterShutdown();
	vTaskDelay(5/portTICK_PERIOD_MS);
	S2LPExitShutdown();

	init_spp_server();
	start_s2lp_console();
	ESP_LOGI("start1","UID=%08X",uid);

	radio_rx_init(PACKETLEN);
    ESP_LOGI(TAG,"radio_rx_init proceed");

    S2LPGpioIrqGetStatus(&xIrqStatus);

    ESP_LOGI(TAG,"s2lp irq Status get");
    S2LPCmdStrobeRx();
};


void to_sleep()
{
	esp_sleep_enable_timer_wakeup(60000000);
	esp_sleep_enable_ext1_wakeup(0x00000010,ESP_EXT1_WAKEUP_ALL_LOW);
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
	rtc_gpio_isolate(GPIO_INPUT_IO_0);
	rtc_gpio_hold_en(PIN_NUM_SDN);
	esp_deep_sleep_start();
}

void app_main(void)
{
//    CLEAR_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG,RTC_CNTL_DBROWN_OUT_THRES_M);
//    CLEAR_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG,RTC_CNTL_BROWN_OUT_RST_ENA_M);
//    CLEAR_PERI_REG_MASK(RTC_CNTL_INT_ENA_REG,RTC_CNTL_BROWN_OUT_INT_ENA_M);
	init_uart0();
	initialize_nvs();
#ifdef SLEEP
	switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_EXT1: {
        	ESP_LOGI("app_main","Wakeup!!! num_of_rows=%d",table.n_of_rows);
			rtc_gpio_hold_dis(PIN_NUM_SDN);
			S2LPSpiInit();
			s2lp_wait1();
        	break;
        }
        case ESP_SLEEP_WAKEUP_TIMER: {
        	ESP_LOGI("app_main","I am alive");
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
        	ESP_LOGI("app_main","Reset!!!");
        	s2lp_rec_start1();
    }
	ESP_LOGI("app_main","Going to sleep...");
	to_sleep();
#endif
//    ESP_ERROR_CHECK(esp_event_loop_create_default());
//    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
//	uart2_queue=xQueueCreate(MAX_MESSAGES_IN_QUEUE, MAX_MESSAGE_SIZE);
//	con=0;
//	xTaskCreate(uart_rec_task, "uart_rec_task", 2048, NULL, 10, NULL);
//    xTaskCreate(queue_watch_task, "queue_watch_task", 4096, NULL, 10, NULL);
	//    xTaskCreate(send_to_cloud_task, "send_to_cloud_task", 4096, NULL, 10, NULL);
//	start_x_shell();
#ifndef SLEEP
    xTaskCreatePinnedToCore(s2lp_rec_start, "s2lp_rec_start", 8192, NULL, 10, NULL,1);
    xTaskCreatePinnedToCore(s2lp_wait, "s2lp_wait", 8192, NULL, 10, NULL,0);
#endif
    while(1)
    {

    }
}
