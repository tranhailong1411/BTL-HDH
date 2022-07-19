/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"


#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

/* Constants that aren't configurable in menuconfig */


/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;
static const char *TAG = "example";
#define WEB_SERVER "api.thingspeak.com"
#define WEB_PORT "80"
char REQUEST[512];
char SUBREQUEST[100];
char recv_buf[512];

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.

        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            switch(event.type) {
                case UART_DATA:
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                    printf("<--Send Data:\n");
                    printf("%s",(const char*)dtmp);
                    printf("<--Receiver Data:\n");
                    const struct addrinfo hints = {
                        .ai_family = AF_INET,
                        .ai_socktype = SOCK_STREAM,
                    };
                struct addrinfo *res;
                struct in_addr *addr;
                int s, r;
                        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

                        if(err != 0 || res == NULL) {
                            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            continue;
                        }

                        /* Code to print the resolved IP.

                        Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
                        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
                        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

                        s = socket(res->ai_family, res->ai_socktype, 0);
                        if(s < 0) {
                            ESP_LOGE(TAG, "... Failed to allocate socket.");
                            freeaddrinfo(res);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            continue;
                        }
                        ESP_LOGI(TAG, "... allocated socket");

                        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
                            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
                            close(s);
                            freeaddrinfo(res);
                            vTaskDelay(4000 / portTICK_PERIOD_MS);
                            continue;
                        }

                        ESP_LOGI(TAG, "... connected");
                        freeaddrinfo(res);

                        sprintf(SUBREQUEST,"api_key=L2RNOISN3VW5K130&field1=%s&field2=%s",(const char*)dtmp,(const char*)dtmp);
                        sprintf(REQUEST,"POST /update.json HTTP/1.1\nHost: api.thingspeak.com\nConnection: close\nContent-Type: application/x-www-form-urlencoded\nContent-Length:%d\n\n%s\n",strlen(SUBREQUEST),SUBREQUEST);
                        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
                            ESP_LOGE(TAG, "... socket send failed");
                            close(s);
                            vTaskDelay(4000 / portTICK_PERIOD_MS);
                            continue;
                        }
                        ESP_LOGI(TAG, "... socket send success");                        
                        struct timeval receiving_timeout;
                        receiving_timeout.tv_sec = 5;
                        receiving_timeout.tv_usec = 0;
                        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                                sizeof(receiving_timeout)) < 0) {
 
                            close(s);
                            vTaskDelay(4000 / portTICK_PERIOD_MS);
                            continue;
                        }

                        /* Read HTTP response */
                        do {
                            bzero(recv_buf, sizeof(recv_buf));
                            r = read(s, recv_buf, sizeof(recv_buf)-1);
                            for(int i = 0; i < r; i++) {
                                putchar(recv_buf[i]);
                            }
                        } while(r > 0);
                        close(s);
                        break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos(EX_UART_NUM);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(EX_UART_NUM);
                    } else {
                        uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : %s", pat);
                    }
                    break;
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}



void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, 23, 22, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}
