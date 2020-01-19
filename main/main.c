#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
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


//static QueueHandle_t uart2_queue;
static uint8_t* data0;
//static uint8_t* data2;
static const char* TAG = "ESPwait";
static const char* TAG1 = "SendToCloud";
static const char *TAGW = "wifi station";
char buf[MAX_MESSAGE_SIZE];
char mes1[MAX_MESSAGE_SIZE];
char mes2[MAX_MESSAGE_SIZE];
uint8_t con;
tcpip_adapter_if_t ifindex;
static input_data_t mes;
static DRAM_ATTR xQueueHandle s2lp_evt_queue = NULL;
static S2LPIrqs xIrqStatus;
static wifi_config_t sta_config;


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
    	ESP_LOGI(TAG1,"REC: Power: %d dbm 0x%08X 0x%08X 0x%08X\n",mes.input_signal_power,mes.data[0],mes.data[1],mes.data[2]);
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
	input_data_t data;
    S2LPTimerLdcIrqWa(S_ENABLE);
	S2LPGpioIrqGetStatus(&xIrqStatus);
    if(xIrqStatus.RX_DATA_READY)
    {
        //Get the RX FIFO size
        uint8_t cRxData = S2LPFifoReadNumberBytesRxFifo();
        //Read the RX FIFO
        S2LPSpiReadFifo(cRxData, (uint8_t*)data.data);
        //Flush the RX FIFO
        S2LPCmdStrobeFlushRxFifo();
        data.input_signal_power=S2LPRadioGetRssidBm();
        S2LPCmdStrobeSleep();
        xQueueSendFromISR(s2lp_evt_queue,&data,0);
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
    get_value_from_nvs("SSID",0,sta_config.sta.ssid);
    get_value_from_nvs("PASSWD",0,sta_config.sta.password);
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
        	ESP_LOGI(TAG,"REC: Power: %d dbm 0x%08X 0x%08X 0x%08X\n",mes.input_signal_power,mes.data[0],mes.data[1],mes.data[2]);
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

static void s2lp_rec_start(void *arg)
{

	S2LPSpiInit();
	S2LPExitShutdown();

	start_s2lp_console();

//	test_gpio();

    radio_rx_init(PACKETLEN);
    ESP_LOGI(TAG,"radio_rx_init proceed");

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

	S2LPGpioIrqGetStatus(&xIrqStatus);

    ESP_LOGI(TAG,"s2lp irq Status get");
    S2LPCmdStrobeRx();
    s2lp_evt_queue = xQueueCreate(10, sizeof(input_data_t));
    while(1) vTaskDelay(60000);
}

void app_main(void)
{
	initialize_nvs();
    init_uart0();
//    ESP_ERROR_CHECK(esp_event_loop_create_default());
//    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
//	uart2_queue=xQueueCreate(MAX_MESSAGES_IN_QUEUE, MAX_MESSAGE_SIZE);
//	con=0;
//	xTaskCreate(uart_rec_task, "uart_rec_task", 2048, NULL, 10, NULL);
//    xTaskCreate(queue_watch_task, "queue_watch_task", 4096, NULL, 10, NULL);
	//    xTaskCreate(send_to_cloud_task, "send_to_cloud_task", 4096, NULL, 10, NULL);
//	start_x_shell();
    xTaskCreatePinnedToCore(s2lp_rec_start, "s2lp_rec_start", 8192, NULL, 10, NULL,1);
    xTaskCreatePinnedToCore(s2lp_wait, "s2lp_wait", 8192, NULL, 10, NULL,0);
    while(1)
    {

    }
}
