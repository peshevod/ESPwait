#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "main.h"
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "spi_intf.h"
#include "shell.h"


static QueueHandle_t uart2_queue;
static uint8_t* data0;
static uint8_t* data2;
static const char* TAG = "ESPwait";
static const char* TAG1 = "SendToCloud";
static const char *TAGW = "wifi station";
char buf[MAX_MESSAGE_SIZE];
char mes1[MAX_MESSAGE_SIZE];
char mes2[MAX_MESSAGE_SIZE];
uint8_t con;
tcpip_adapter_if_t ifindex;

static int s_retry_num = 0;
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about one event
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;


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

void init_uart2()
{
    uart_config_t uart_config2 = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_2, &uart_config2);
    uart_set_pin(UART_NUM_2, UART2_TXD, UART2_RXD, UART2_RTS, UART2_CTS);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, BUF_SIZE * 2, 20, uart2_queue, 0);

    data2 = (uint8_t *) malloc(BUF_SIZE);
}


static void uart_rec_task(void *arg)
{
	int n=0;
	init_uart2();
	uart_flush(UART_NUM_2);
	xQueueReset(uart2_queue);
	buf[0]=0;
	while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_2, data2, BUF_SIZE, 20 / portTICK_RATE_MS);
        uart_write_bytes(UART_NUM_2, (const char*)data2, len);
        if(len!=0) ESP_LOGI(TAG,"Chars received len=%d, buf=%s",len,buf);
        for(int i=0;i<len;i++)
        {
        	if(data2[i]=='\r' || data2[i]=='\n')
        	{
        		if(n!=0)
       			{
        			if(memcmp(buf,"MES:",4)==0)
        			{
        				ESP_LOGI(TAG,"Service Message received %s",buf);
        			}
        			else if(memcmp(buf,"REC:",4)==0)
        			{
        				ESP_LOGI(TAG,"Message from device received %s",buf);
        				if( xQueueSend( uart2_queue, ( void * )buf, ( TickType_t ) 10 ) != pdPASS )
        				{
        					ESP_LOGE(TAG,"QueueSend problem");
        				}
        			}
        			else
        			{
        				ESP_LOGI(TAG,"Unknown Message received %s",buf);
        			}
        			n=0;
        			buf[0]=0;
        			uart_flush(UART_NUM_2);
       			}
        		continue;
        	}
        	buf[n++]=data2[i];
        	buf[n]=0;
        }
    }
}

static void queue_watch_task(void *arg)
{
	while(1)
	{
		if(xQueueReceive(uart2_queue, mes1, 20 / portTICK_RATE_MS)==pdTRUE)
		{
			send_to_cloud(mes1);
		}
		vTaskDelay(300 / portTICK_PERIOD_MS);
	}
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
    			ESP_LOGI(TAGW, "retry to connect to the AP ssid=%s pass=%s retry=%d",CONFIG_ESP_WIFI_SSID,CONFIG_ESP_WIFI_PASSWORD,s_retry_num);
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

static void send_to_cloud_task(void *arg)
{

	while(1)
	{
		send_to_cloud("test");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

}

static void get_s2lp_status(void *arg)
{

//	int i=0;
	gpio_set_direction(PIN_NUM_SDN, GPIO_MODE_OUTPUT);
	gpio_iomux_out(PIN_NUM_CS, FUNC_MTDO_HSPICS0, false);
//	gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
//	gpio_iomux_out(PIN_NUM_MISO, FUNC_MTDI_HSPIQ, false);
	gpio_iomux_in(PIN_NUM_MISO, HSPIQ_IN_IDX);
	//	gpio_set_direction(PIN_NUM_MISO, GPIO_MODE_OUTPUT);
	gpio_iomux_out(PIN_NUM_MOSI, FUNC_MTCK_HSPID, false);
//	gpio_set_direction(PIN_NUM_MOSI, GPIO_MODE_OUTPUT);
	gpio_iomux_out(PIN_NUM_CLK, FUNC_MTMS_HSPICLK, false);
//	gpio_set_direction(PIN_NUM_CLK, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_NUM_SDN, 0);
	init_spi_intf();
	while(1)
	{
/*		if(i==0)
		{
			gpio_set_level(PIN_NUM_SDN, 0);
			gpio_set_level(PIN_NUM_CS, 1);
			gpio_set_level(PIN_NUM_MISO, 1);
			gpio_set_level(PIN_NUM_MOSI, 1);
			gpio_set_level(PIN_NUM_CLK, 1);
			i=1;
		}
		else
		{
			gpio_set_level(PIN_NUM_SDN, 1);
			gpio_set_level(PIN_NUM_CS, 0);
			gpio_set_level(PIN_NUM_MISO, 0);
			gpio_set_level(PIN_NUM_MOSI, 0);
			gpio_set_level(PIN_NUM_CLK, 0);
			i=0;
		}*/
		get_status();
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}

}

void send_to_cloud(char* mes)
{
	s_retry_num=0;
	ESP_LOGI(TAG1,"Message from device received %d %s",strlen(mes),mes);
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    s_wifi_event_group = xEventGroupCreate();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
//            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    con=1;
    ESP_ERROR_CHECK( esp_wifi_start() );
//    ESP_ERROR_CHECK( esp_wifi_connect() );
	vTaskDelay(10000 / portTICK_PERIOD_MS);
	con=0;
    ESP_ERROR_CHECK( esp_wifi_disconnect() );
    ESP_ERROR_CHECK( esp_wifi_stop() );
    ESP_ERROR_CHECK( esp_wifi_deinit() );
    vEventGroupDelete(s_wifi_event_group);
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler));
}


void app_main(void)
{
	esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    tcpip_adapter_init();
    init_uart0();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
//    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
	uart2_queue=xQueueCreate(MAX_MESSAGES_IN_QUEUE, MAX_MESSAGE_SIZE);
	con=0;
//	xTaskCreate(uart_rec_task, "uart_rec_task", 2048, NULL, 10, NULL);
//    xTaskCreate(queue_watch_task, "queue_watch_task", 4096, NULL, 10, NULL);
	//    xTaskCreate(send_to_cloud_task, "send_to_cloud_task", 4096, NULL, 10, NULL);
	start_x_shell();
    xTaskCreate(get_s2lp_status, "get_s2lp_status", 4096, NULL, 10, NULL);
}
