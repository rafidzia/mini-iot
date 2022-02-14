
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "driver/gpio.h"
#include "driver/adc.h"

#include "esp_adc_cal.h"

#include "esp_sntp.h"

#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "bt.c"

#define DEFAULT_VREF    1100
#define NO_OF_SAMPLES   64

#define LED_0    19
#define LED_1    18
#define LED_2    5

#define GPIO_PWM0A_OUT 15   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 16   //Set GPIO 16 as PWM0B

static const char *TAG = "MQTT_WANNALOG";

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t adc_channel = ADC_CHANNEL_0; 

static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

esp_mqtt_client_handle_t global_client;
int msg_id;

char mqtt_topic[30], mqtt_data[30], temp_string[30];

#if CONFIG_IDF_TARGET_ESP32
static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}
#endif

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "ntp.bmkg.go.id");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();
}

static void obtain_time(void)
{
    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 30;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);
}

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}

static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}

static void get_task(void *pvParameters){
    int x = 0;
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];
    char strftime_set[64];

    uint32_t adc_reading;

    float voltage, lm35v, celcius;

    char color[10];
    float speed;
    char result[500];
    int save = 1;

    while(1){
        if(x == 300){
            esp_restart();
        }
        x++;
        
        time(&now);
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        strftime(strftime_set, sizeof(strftime_set), "%H.%M", &timeinfo);
        check_alarm(strftime_set);

        adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)adc_channel);
        }
        adc_reading /= NO_OF_SAMPLES;
        
        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        lm35v = voltage / 2;
        celcius = lm35v / 10;

        strcpy(color, "");

        if (celcius < 25.00) {
            sprintf(color, "BLUE");
            gpio_set_level(LED_0, 0);
            gpio_set_level(LED_1, 0);
            gpio_set_level(LED_2, 1);
        } else if (celcius <= 30.00) {
            sprintf(color, "GREEN");
            gpio_set_level(LED_0, 0);
            gpio_set_level(LED_1, 1);
            gpio_set_level(LED_2, 0);
        } else {
            sprintf(color, "RED");
            gpio_set_level(LED_0, 1);
            gpio_set_level(LED_1, 0);
            gpio_set_level(LED_2, 0);
        }
        
        if(celcius <= 25.0){
            speed = 0.0;
            brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        }else if(celcius > 25.0 && celcius <= 35.0){
            speed = (celcius - 25.0) * 10.0;
            brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, speed);
        }else{
            speed = 100.0;
            brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 100.0);
        }

        ESP_LOGI(TAG, "fan speed is %.2f", speed);
        ESP_LOGI(TAG, "temperature is %.2f", celcius);            

        strcpy(result, "");
        sprintf(result, "{\"save\" : %d, \"adc\" : %d, \"opamp\" : %.2f, \"lm35\" : %.2f, \"temp\" : %.2f, \"led\" : \"%s\", \"time\" : \"%s\", \"speed\" : %.2f, \"ledbt\" : %d}", save, adc_reading, voltage, lm35v, celcius, color, strftime_buf, speed, ledbt_status);
        save = 0;

        msg_id = esp_mqtt_client_publish(global_client, "/mofuban/wannalog", result, 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void mqtt_data_handle(int topic_len, int data_len, uint8_t *topic, uint8_t *data){
    memset(mqtt_topic,0,strlen(mqtt_topic));
    memset(mqtt_data,0,strlen(mqtt_data));
    for(int i = 0; i < topic_len; i++){
        mqtt_topic[i] = (char)topic[i];
    }
    for(int i = 0; i < data_len; i++){
        mqtt_data[i] = (char)data[i];
    }

    if(!strcmp(mqtt_topic, "/mofuban/ledtoggle")){
        if(ledbt_status){
            gpio_set_level(LED_BT, 0);
            ledbt_status = 0;
        }else{
            gpio_set_level(LED_BT, 1);
            ledbt_status = 1;
        }
        sprintf(temp_string, "%d", ledbt_status);
        msg_id = esp_mqtt_client_publish(global_client, "/mofuban/ledstat", temp_string, 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    }
    if(!strcmp(mqtt_topic, "/mofuban/alarmset")){
        reti = regexec(&regexbt, mqtt_data, 0, NULL, 0);
        ESP_LOGI(SPP_TAG, "alarm set to %s", mqtt_data);
        ESP_LOGI(SPP_TAG, "reti : %d", reti);
        if(!reti) {
            strcpy(alarmtime, mqtt_data);
            alarmsetted = 1;
        }
    }
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    global_client = client;
    // your_context_t *context = event->context;
    if(event->event_id == MQTT_EVENT_CONNECTED){
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        sprintf(temp_string, "%d", ledbt_status);
        msg_id = esp_mqtt_client_publish(client, "/mofuban/ledstat", temp_string, 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/mofuban/ledtoggle", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/mofuban/alarmset", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        xTaskCreate(&get_task, "mqtt_task", 8192, NULL , 5, NULL);
        
    }
    if(event->event_id == MQTT_EVENT_DISCONNECTED){
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    }
    if(event->event_id == MQTT_EVENT_PUBLISHED){
        // ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    }
    if(event->event_id == MQTT_EVENT_DATA){
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        mqtt_data_handle(event->topic_len, event->data_len, (uint8_t *)event->topic, (uint8_t *)event->data);
    }
    if(event->event_id == MQTT_EVENT_ERROR){
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://broker.emqx.io",
        .client_id = "mqttjsfarid2",
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    #if CONFIG_IDF_TARGET_ESP32
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
    #endif

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }

    char strftime_buf[64];
    // Set timezone to Asia/Jakarta (WIB) and print local time
    setenv("TZ", "WIB-7", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Jakarta is: %s", strftime_buf);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(adc_channel, atten);

    gpio_set_direction(LED_0, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_2, GPIO_MODE_OUTPUT);

    mcpwm_example_gpio_initialize();
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   

    bt_start();
    mqtt_app_start();
}
