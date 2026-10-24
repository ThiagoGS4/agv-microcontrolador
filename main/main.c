#include <stdbool.h>
#include <stdint.h>
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "vl53l0x.h"
#include "driver/i2c_master.h"
#include "math.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "neopixel.h"
#include "cJSON.h"
#include "esp_timer.h"
#include "iot_button.h"
#include "button_gpio.h"

// temporário
#include "esp_rom_sys.h"

static const char *TAG = "MOTOR_TEST";

static esp_err_t startWifi();
static void mqtt_app_start();
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void log_error_if_nonzero(const char *message, int error_code);

// wifi
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
// numero de tentativas para reconectar
static int s_retry_num = 0;
// FreeRTOS event group to signal when we are connected
static EventGroupHandle_t s_wifi_event_group;

// MQTT
esp_mqtt_client_handle_t mqttClient;

// pinos do l289n
#define ENB_PIN       GPIO_NUM_13
#define ENA_PIN       GPIO_NUM_25
#define IN4_PIN       GPIO_NUM_12
#define IN3_PIN       GPIO_NUM_14
#define IN2_PIN       GPIO_NUM_27
#define IN1_PIN       GPIO_NUM_26

// canais motores
#define LEFT_CH   LEDC_CHANNEL_0  // ENA
#define RIGHT_CH  LEDC_CHANNEL_1  // ENB

// velocidades motores
#define BASE_DUTY 5000
#define MAX_DUTY  8191

// pinos para os sensores inferiores (detecção de linha e cor)
//olhei no datasheet página 134 - 784
#define LEFT_LINE_PIN       ADC_CHANNEL_6 // GPIO_NUM_34
#define MIDDLE_LINE_PIN       ADC_CHANNEL_3 // GPIO_NUM_39
#define RIGH_LINE_PIN       ADC_CHANNEL_0 // GPIO_NUM_36

#define BUZZER_PIN       GPIO_NUM_21

// sensores de distância (VL53L0X)
#define DISTANCE_SDA GPIO_NUM_19
#define DISTANCE_SCL GPIO_NUM_18
#define BACK_DISTANCE_XSHUT GPIO_NUM_5
#define FRONT_DISTANCE_XSHUT GPIO_NUM_15

#define COLOR_SDA GPIO_NUM_33
#define COLOR_SCL GPIO_NUM_32

#define LINE_READER_DELAY 10
#define DISTANCE_SENSORS_DELAY 20
#define COLOR_SENSOR_DELAY 25

// LED RGB
#define LED_DATA_PIN GPIO_NUM_22

typedef enum {
    BREAK_MOTORS = 0,
    EMERGENCY_BREAK,
    STAY,
    FORWARD,
    ADJUST_RIGHT,
    ADJUST_LEFT,
    BACKWARDS,
    LEFT,
    RIGHT,
    PREV_CMD
} MotorCommand;

typedef enum {
    OK,
    ASKEW_RIGHT,
    ASKEW_LEFT,
    OUT_OF_TRACK
} AGVPos;

typedef enum {
    INIT_BLINK,
    WARNING_BLINK,
    WALKING,
    STOPED_ACTION_NEEDED,
    STOPED
} LEDState;

// distance sensor handles
vl53l0x_handle_t back_sensor = NULL;
vl53l0x_handle_t front_sensor = NULL;

// color sensor handle
i2c_master_dev_handle_t color_dev_handle;

// variáveis de comandos das salas
// MODIFICADO: roomSquareCounter agora = quantos quadrados azuis o AGV deve IGNORAR (só contar)
// antes de parar. Ex.: 0 = para no 1º azul, 2 = passa por 2 azuis e para no 3º.
// (nome mantido para não quebrar o JSON "roomSquareCounter" que vem do MQTT)
int roomSquareCounter = 0;
int lastTripSquareCounter = 0;
bool turningRoutine = false;
bool ignoreBluesquare = false;

//  variaveis globais sensor de cor
bool onBlueSquare = false;
bool onRedSquare = false;

typedef enum{
    RIGHT_ROOM,
    LEFT_ROOM
} RoomDirectionEnum;

RoomDirectionEnum roomDirection = 0;

typedef enum{
    AGV_FORWARD,
    AGV_BACKWARDS
} AgvDirectionDirectionEnum;

AgvDirectionDirectionEnum agvDirection = 0;

// variavel de controle de virada
int64_t turnStartTime = 0;

void activateMotors(void *pvParameters);
void stopMotors();
void goForward();
void goBackwards();
void adjustLeft();
void adjustRight();
void turnLeft();
void turnRight();
static void printDistanceData(const vl53l0x_data_t *data);
// esp_err_t vl53l0x_set_address(i2c_master_dev_handle_t dev_handle, uint8_t new_address);
void configDistanceSensors();
void readDistanceSensors(void *pvParameters);
void readLineFollowerSensors(void *pvParameters);
int normalize(int raw, int w, int b);
float PIDController(int measurement, float Kp, float Ki, float Kd);
void PIDReset();
void configColorSensor();
void readColorSensor(void *pvParameters);
void activateLed(void *pvParameters);
void extractAndAssignWebCommands(char* responseJSON, int dataLen);
static void button_single_click_cb(void *arg, void *usr_data);
static void button_double_click_cb(void *arg, void *usr_data);
static void button_hold_cb(void *btn_handle, void *usr_data);
void configButton();

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);

// handle ADC para sensores seguidores de linha
adc_oneshot_unit_handle_t adc1_handle;

// handles das tarefas (referencio tarefas por eles...)
TaskHandle_t MotorTaskHandle;
TaskHandle_t LineFollowerSensorsTaskHandle;
TaskHandle_t DistanceSensorsTaskHandle;
TaskHandle_t ColorSensorTaskHandle;
TaskHandle_t LedTaskHandle;

// controle PID
static volatile float kp = 1.5f, ki = 0.0f, kd = 0.5f;

void app_main(void)
{
    xTaskCreatePinnedToCore(activateLed, "LedTask", 2048, NULL, 1, &LedTaskHandle, APP_CPU_NUM);
    const esp_err_t wifi_err_check = startWifi();

    if(wifi_err_check == ESP_OK){
        mqtt_app_start();

    } else {
        ESP_LOGE(TAG, "Nao foi possivel inicializar wifi...");
    }

    // config dos pinos equivalente à cada motor
    gpio_config_t gpioConfMotors = {
        .pin_bit_mask = (1ULL << IN1_PIN) | (1ULL << IN2_PIN) | (1ULL << IN3_PIN) | (1ULL << IN4_PIN) | (1ULL << ENA_PIN) | (1ULL << ENB_PIN),
        .mode = GPIO_MODE_OUTPUT
    };

    gpio_config(&gpioConfMotors);
    
    // config ADC (analógio para digital)
    
    adc_oneshot_unit_init_cfg_t LF_adc_init_config = {
        .unit_id = ADC_UNIT_1, // unidade 2 conflita com wifi e usa pinos diferentes
        .ulp_mode = ADC_ULP_MODE_DISABLE, // ULP = Ultra Low Power...
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&LF_adc_init_config, &adc1_handle));
    
    adc_oneshot_chan_cfg_t LF_adc_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_11,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, LEFT_LINE_PIN, &LF_adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MIDDLE_LINE_PIN, &LF_adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, RIGH_LINE_PIN, &LF_adc_config));

    // config do pino do buzzer
    gpio_config_t gpioConfBottomBuzzer = {
        .pin_bit_mask = (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT
    };
    
    gpio_config(&gpioConfBottomBuzzer);

    // config do timer do pwm
    ledc_timer_config_t pwmTimerConfig = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT, //8192 duty cycles
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK // config automárica de clock
    };

    ESP_ERROR_CHECK(ledc_timer_config(&pwmTimerConfig));

    // config do canal do pwm ENA
    ledc_channel_config_t pwmChannelConfig_ENA = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEFT_CH,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = ENA_PIN,
        .duty = 0,
        .hpoint = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&pwmChannelConfig_ENA));


    // config do canal do pwm ENB
    ledc_channel_config_t pwmChannelConfig_ENB = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = RIGHT_CH,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = ENB_PIN,
        .duty = 0,
        .hpoint = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&pwmChannelConfig_ENB));

ledc_timer_config_t pwmTimerConfigBuzzer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwmTimerConfigBuzzer));

    // config do canal do pwm do BUZZER
    //TODO: organizar sapoha
    ledc_channel_config_t pwmChannelConfig_BUZ = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BUZZER_PIN,
        .duty = 0,
        .hpoint = 0
    };
    
    ESP_ERROR_CHECK(ledc_channel_config(&pwmChannelConfig_BUZ));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEFT_CH, 8192 - ceil(8192/3)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEFT_CH));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, RIGHT_CH, 8192 - ceil(8192/3)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, RIGHT_CH));

    // config do botão de comando
    configButton();

    configDistanceSensors();

    configColorSensor();

    xTaskCreatePinnedToCore(activateMotors, "MotorTask", 4096, NULL, 2, &MotorTaskHandle, APP_CPU_NUM);
    xTaskCreatePinnedToCore(readLineFollowerSensors, "LineFollowerSensorsTask", 4096, NULL, 5, &LineFollowerSensorsTaskHandle, APP_CPU_NUM);
    xTaskCreatePinnedToCore(readDistanceSensors, "DistanceSensorsTask", 4096, NULL, 4, &DistanceSensorsTaskHandle, APP_CPU_NUM);
    xTaskCreatePinnedToCore(readColorSensor, "ColorSensorTask", 4096, NULL, 3, &ColorSensorTaskHandle, APP_CPU_NUM);
}

esp_err_t startWifi(){

wifi_config_t wifi_config = {
    .sta = {
        .pmf_cfg = {
            //.capable = true,
            .required = false
        },
    },
};

    strncpy((char*)wifi_config.sta.ssid, CONFIG_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wifi_config.sta.password));

     esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (CONFIG_LOG_MAXIMUM_LEVEL > CONFIG_LOG_DEFAULT_LEVEL) {
        /* If you only want to open more logs in the wifi module, you need to make the max level greater than the default level,
         * and call esp_log_level_set() before esp_wifi_init() to improve the log level of the wifi module. */
        esp_log_level_set("wifi", CONFIG_LOG_MAXIMUM_LEVEL);
    }

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    
    s_wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());
    
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");


  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 CONFIG_WIFI_SSID, "senha_bixo");
    return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 CONFIG_WIFI_SSID, "senha_bixo");
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    return ESP_ERR_WIFI_BASE;
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 30) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    mqttClient = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(mqttClient, "/agv/1/status", "data_3", 0, 0, 0);
        ESP_LOGI(TAG, "sent PUBLISH successful, msg_id=%d", msg_id);

        esp_mqtt_client_subscribe(mqttClient, "/agv/kp", 0);
        esp_mqtt_client_subscribe(mqttClient, "/agv/ki", 0);
        esp_mqtt_client_subscribe(mqttClient, "/agv/kd", 0);
        esp_mqtt_client_subscribe(mqttClient, "/agv/command", 0);

        msg_id = esp_mqtt_client_subscribe(mqttClient, "/agv/2/status", 0);
        ESP_LOGI(TAG, "sent SUBSCRIBE successful, msg_id=%d", msg_id);
        break;
        
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d, return code=0x%02x ", event->msg_id, (uint8_t)*event->data);
        // msg_id = esp_mqtt_client_publish(mqttClient, "/agv/1/status", "data", 0, 0, 0);
        // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    // importante!!
    case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
    printf("DATA=%.*s\r\n", event->data_len, event->data);
    if (event->topic_len == strlen("/agv/kp") && strncmp(event->topic, "/agv/kp", event->topic_len) == 0) {
        sscanf(event->data, "%f", &kp);

        ESP_LOGE(TAG, "Kp: %f", kp);
    } else if (event->topic_len == strlen("/agv/ki") && strncmp(event->topic, "/agv/ki", event->topic_len) == 0) {
        sscanf(event->data, "%f", &ki);

        ESP_LOGE(TAG, "Ki: %f", ki);
    } else if (event->topic_len == strlen("/agv/kd") && strncmp(event->topic, "/agv/kd", event->topic_len) == 0) {
        sscanf(event->data, "%f", &kd);

        ESP_LOGE(TAG, "Kd: %f", kd);
    } else if(event->topic_len == strlen("/agv/command") && strncmp(event->topic, "/agv/command", event->topic_len) == 0){
        printf("chegou aki\n");
        extractAndAssignWebCommands(event->data, event->data_len);
    }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGE(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = "mqtt://192.168.0.11",
            .address.port = 1883
            //.verification.certificate = (const char) = mqtt_p 
        }
    };

    ESP_LOGI(TAG, "[APP] fres memory: " PRIu32 "bytes", esp_get_free_heap_size());

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

bool obstacleDetected = false;
uint32_t motorCommand = BREAK_MOTORS;

void activateMotors(void *pvParameters){
    // força aplicar na primeira vez
    int applied = -1;
    MotorCommand prevComd = BREAK_MOTORS;
    
    while (1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(motorCommand == PREV_CMD){
            motorCommand = prevComd;
        }

        MotorCommand commanOnTask = obstacleDetected ? EMERGENCY_BREAK : motorCommand;
        if ((int)commanOnTask == applied) continue;
        applied = commanOnTask;


        // buzzer liga só quando freio de emergência está em ação
        //TODO: reativar essa buzzer depois (funcionando OK!)
        // ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, commanOnTask == EMERGENCY_BREAK ? 8191/2 : 0);
        // ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

        switch(commanOnTask)
        {
        case EMERGENCY_BREAK:
            stopMotors();
            xTaskNotify(LedTaskHandle, WARNING_BLINK,eSetValueWithOverwrite);
            break;
        case BREAK_MOTORS:
            stopMotors();
            xTaskNotify(LedTaskHandle, STOPED_ACTION_NEEDED, eSetValueWithOverwrite);
            break;
        case STAY:
            stopMotors();
            xTaskNotify(LedTaskHandle, STOPED, eSetValueWithOverwrite);
            break;
        case FORWARD:
            goForward();
            prevComd = FORWARD;
            xTaskNotify(LedTaskHandle, WALKING,eSetValueWithOverwrite);
            break;
        case BACKWARDS:
            goBackwards();
            prevComd = BACKWARDS;
            xTaskNotify(LedTaskHandle, WALKING,eSetValueWithOverwrite);
            break;
        case ADJUST_RIGHT:
            adjustRight();
            xTaskNotify(LedTaskHandle, WALKING,eSetValueWithOverwrite);
            break;
        case ADJUST_LEFT:
            adjustLeft();
            xTaskNotify(LedTaskHandle, WALKING,eSetValueWithOverwrite);
            break;
        case RIGHT:
            turnRight();
            xTaskNotify(LedTaskHandle, WALKING,eSetValueWithOverwrite);
            break;
        case LEFT:
            turnLeft();
            xTaskNotify(LedTaskHandle, WALKING,eSetValueWithOverwrite);
            break;
        case PREV_CMD:
            break;
        }
        // vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void setDuty(ledc_channel_t ch, int duty){
    if (duty < 0) duty = 0;
    if (duty > MAX_DUTY) duty = MAX_DUTY;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

void readLineFollowerSensors(void *pvParameters){
    int sensLeft = 0;
    int sensMid = 0;
    int sensRight = 0;
    
    AGVPos currPos = OUT_OF_TRACK;
    uint16_t position = 1000;
    int lastPosition = 1000;
    
    static const int WHITE[3] = {20, 20, 35};
    static const int BLACK[3] = {1200, 300, 1000};

    #define LINE_MIN 100

    TickType_t lastWake = xTaskGetTickCount();

    while (1){
        
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, LEFT_LINE_PIN, &sensLeft));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, MIDDLE_LINE_PIN, &sensMid));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, RIGH_LINE_PIN, &sensRight));

        int leftNormalized = normalize(sensLeft,  WHITE[0], BLACK[0]);
        int middleNormalized = normalize(sensMid,   WHITE[1], BLACK[1]);
        int rightNormalized = normalize(sensRight, WHITE[2], BLACK[2]);

        // controle para virar carro
        if (motorCommand == RIGHT || motorCommand == LEFT) {
            int64_t currentTime = esp_timer_get_time() / 1000;
            turningRoutine = true;
            
            if (currentTime - turnStartTime > 3000) { 
                turningRoutine = false;
                PIDReset();
                motorCommand = PREV_CMD; // Volta a ir para a frente (ou trás)
                
                if(onBlueSquare){
                    ignoreBluesquare = true; // Levanta o escudo para não frear ao retomar a viagem
                }
                xTaskNotifyGive(MotorTaskHandle);
            }
            
            // Pula o resto da task. O PID não roda enquanto estiver virando.
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(LINE_READER_DELAY));
            continue; 
        }

        int sum = leftNormalized + middleNormalized + rightNormalized;
        //int maxv = leftNormalized > middleNormalized ? (leftNormalized > rightNormalized ? leftNormalized : rightNormalized) : (middleNormalized > rightNormalized ? middleNormalized : rightNormalized);

        if (sum > 0) {
            position = (middleNormalized * 1000 + rightNormalized * 2000) / sum;
            lastPosition = position;
        } else {
            // Se sum for 0 (perdeu a linha), usa a memória da última posição lida
            position = lastPosition;
        }

        bool lineVisible = (sum >= LINE_MIN);
        char payload_buffer[16];

        //todo: remover ou usar esse trecho
        // if (lineVisible) {
        //     position = (middleNormalized * 1000 + rightNormalized * 2000) / sum;
        //     lastPosition = position;
        // }
        // if(!lineVisible){
        //     PIDReset();
        //     currPos = OUT_OF_TRACK;
        //     esp_mqtt_client_publish(mqttClient, "/agv/bs/currPos", "FORA DA LINHA", 0, 0, 0);

        //     snprintf(payload_buffer, sizeof(payload_buffer), "%d", position);
        //     esp_mqtt_client_publish(mqttClient, "/agv/bs/posicao", payload_buffer, 0, 0, 0);

        //     //motorCommand = EMERGENCY_BREAK;

        //     setDuty(LEFT_CH,  BASE_DUTY);
        //     setDuty(RIGHT_CH, BASE_DUTY);

        //     motorCommand = (lastPosition > 1000) ? ADJUST_LEFT : ADJUST_RIGHT;
        //     xTaskNotifyGive(MotorTaskHandle);
        // } else{
/*                     snprintf(payload_buffer, sizeof(payload_buffer), "%d", position);
            
                    esp_mqtt_client_publish(mqttClient, "/agv/bs/posicao", payload_buffer, 0, 0, 0);
                    ESP_LOGI(TAG, "posicao: %" PRIu16, position); */
                    
                    int deadZone = 3;
            
                    float correction = PIDController(position, kp, ki, kd);

                    int pid = (int)correction;
                    int mag  = pid < 0 ? -pid : pid;
                    int base = BASE_DUTY - mag / 4; // todo: deixar isso supimpa
                    setDuty(LEFT_CH,  base + pid);
                    setDuty(RIGHT_CH, base - pid);

                    if(currPos != OK){
                        motorCommand = FORWARD;
                        xTaskNotifyGive(MotorTaskHandle);
                    }
                        currPos = OK;
                // }
                vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(LINE_READER_DELAY));
    }
}

int normalize(int raw, int w, int b){
    int v = (raw - w) * 1000 / (b - w);
    if (v < 150) v = 0;
    if (v > 1000) v = 1000;
    return v;
}


int setpoint = 1000;
#define IMAX 2000
int I = 0;
int prevError = 0;
static float dFilt = 0; 
#define PID_MAX_CORR 2000 

float PIDController(int measurement, float Kp, float Ki, float Kd){ // uma entrada e uma saida...
    int P = 0, D = 0;
    // calculando erro:
    float error = (float)(measurement - setpoint);

    P = error;
    I = I + (int)error;
    dFilt += 0.3f * ((error - prevError) - dFilt);

    if (I < -IMAX) {
        I = -IMAX;
    } else if (I > IMAX) {
        I = IMAX;
    }

    float Pvalue = 0;
    float Ivalue = 0;
    float Dvalue = 0;

    Pvalue = Kp * P;
    Ivalue = Ki * I;
    Dvalue = Kd * dFilt;

    float PIDvalue = Pvalue + Ivalue + Dvalue;
    if (PIDvalue >  PID_MAX_CORR) PIDvalue =  PID_MAX_CORR;
    if (PIDvalue < -PID_MAX_CORR) PIDvalue = -PID_MAX_CORR;

    prevError = error;
    //todo: remover debug LOG
    // ESP_LOGI(TAG, "P=%.0f I=%.0f D=%.0f", Pvalue, Ivalue, Dvalue);
    return PIDvalue;
}

void PIDReset(){
    dFilt = 0.0f;
    I  = 0.0f;
    prevError = 0.0f;
}

void stopMotors(){
    gpio_set_level(IN1_PIN, 0);
    gpio_set_level(IN2_PIN, 0);
    gpio_set_level(IN3_PIN, 0);
    gpio_set_level(IN4_PIN, 0);
}

void goForward(){
    gpio_set_level(IN1_PIN, 0);
    gpio_set_level(IN2_PIN, 1);
    gpio_set_level(IN3_PIN, 1);
    gpio_set_level(IN4_PIN, 0);
}

void goBackwards(){
    gpio_set_level(IN1_PIN, 1);
    gpio_set_level(IN2_PIN, 0);
    gpio_set_level(IN3_PIN, 0);
    gpio_set_level(IN4_PIN, 1);
}

void adjustLeft(){

    gpio_set_level(IN1_PIN, 1);
    gpio_set_level(IN2_PIN, 1);
    gpio_set_level(IN3_PIN, 1);
    gpio_set_level(IN4_PIN, 0);
}

void adjustRight(){
    gpio_set_level(IN1_PIN, 0);
    gpio_set_level(IN2_PIN, 1);
    gpio_set_level(IN3_PIN, 1);
    gpio_set_level(IN4_PIN, 1);
}

void turnLeft(){
    setDuty(LEFT_CH,  BASE_DUTY);
    setDuty(RIGHT_CH, BASE_DUTY);
    gpio_set_level(IN1_PIN, 1);
    gpio_set_level(IN2_PIN, 0);
    gpio_set_level(IN3_PIN, 1);
    gpio_set_level(IN4_PIN, 0);
}

void turnRight(){ // fix: timar a virada e parar o PID enquanto vira o carro
    setDuty(LEFT_CH,  BASE_DUTY);
    setDuty(RIGHT_CH, BASE_DUTY);
    gpio_set_level(IN1_PIN, 0);
    gpio_set_level(IN2_PIN, 1);
    gpio_set_level(IN3_PIN, 0);
    gpio_set_level(IN4_PIN, 1);
}

void configDistanceSensors(){
    // config dos pinos equivalente à cada motor
    gpio_config_t gpioConfXshut = {
        .pin_bit_mask = (1ULL << BACK_DISTANCE_XSHUT) | (1ULL << FRONT_DISTANCE_XSHUT),
        .mode = GPIO_MODE_OUTPUT
    };

    gpio_config(&gpioConfXshut);

    // identificador da porta (bus) I2C
    i2c_master_bus_handle_t distance_bus = NULL;

    const i2c_master_bus_config_t distance_bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = DISTANCE_SDA,
        .scl_io_num = DISTANCE_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&distance_bus_cfg, &distance_bus));

    // desligando sensores p/ não conflitarem
    gpio_set_level(BACK_DISTANCE_XSHUT, 0);
    gpio_set_level(FRONT_DISTANCE_XSHUT, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // ----- SENSOR DE TRÁS -----

    // ligando sensor de trás
    gpio_set_level(BACK_DISTANCE_XSHUT, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // cria descritor do sensor e o inicializa
    ESP_ERROR_CHECK(vl53l0x_create(&back_sensor, distance_bus));

    // mudando endereço do VL53L0X para não conflitar com o outro VL53L0X ou o TCS34725
    ESP_ERROR_CHECK(vl53l0x_set_address(back_sensor, distance_bus, 0x30));

    ESP_ERROR_CHECK(vl53l0x_init(back_sensor));

    // calibração
    ESP_ERROR_CHECK(vl53l0x_perform_ref_calibration(back_sensor, &(vl53l0x_ref_calibration_t){}));

    // perfil de medição e modo modo medição único (consome mais energia mas exige 1 pino a menos)
    ESP_ERROR_CHECK(vl53l0x_set_profile(back_sensor, VL53L0X_PROFILE_DEFAULT));
    ESP_ERROR_CHECK(vl53l0x_set_mode(back_sensor, VL53L0X_MODE_SINGLE));

    // estrutura para dados

     // ----- SENSOR DA FORWARD -----
     // mesmo processo...
    gpio_set_level(FRONT_DISTANCE_XSHUT, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(vl53l0x_create(&front_sensor, distance_bus));

    ESP_ERROR_CHECK(vl53l0x_set_address(front_sensor, distance_bus, 0x31));

    ESP_ERROR_CHECK(vl53l0x_init(front_sensor));

    ESP_ERROR_CHECK(vl53l0x_perform_ref_calibration(front_sensor, &(vl53l0x_ref_calibration_t){}));

    ESP_ERROR_CHECK(vl53l0x_set_profile(front_sensor, VL53L0X_PROFILE_DEFAULT));
    ESP_ERROR_CHECK(vl53l0x_set_mode(front_sensor, VL53L0X_MODE_SINGLE));
}

void readDistanceSensors(void *pvParameters){
    #define OBSTACLE_STOP_MM  400
    #define OBSTACLE_CLEAR_MM 500

while (1) {
        ESP_ERROR_CHECK(vl53l0x_start_measurement(back_sensor));
        ESP_ERROR_CHECK(vl53l0x_start_measurement(front_sensor));

        bool back_ready = false;
        bool front_ready = false;



        while(true) {
        ESP_ERROR_CHECK(vl53l0x_get_ready(back_sensor, &back_ready));
        ESP_ERROR_CHECK(vl53l0x_get_ready(front_sensor, &front_ready));
            if (back_ready && front_ready) {
                break;
            } else {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        vl53l0x_data_t back_data = {0};
        vl53l0x_data_t front_data = {0};

        // Get the measurement result
        ESP_ERROR_CHECK(vl53l0x_get_data(back_sensor, &back_data));
        ESP_ERROR_CHECK(vl53l0x_get_data(front_sensor, &front_data));

        int dmin = back_data.distance_mm < front_data.distance_mm ? back_data.distance_mm : front_data.distance_mm;

        bool blocked = obstacleDetected;

        if (!blocked && dmin <= OBSTACLE_STOP_MM){
            blocked = true;
        } else if (blocked && dmin > OBSTACLE_CLEAR_MM){
            blocked = false;
        }

        if (blocked != obstacleDetected){
            obstacleDetected = blocked;
            xTaskNotifyGive(MotorTaskHandle);
        }
        // printDistanceData(&back_data);
        // printDistanceData(&front_data);

        vTaskDelay(pdMS_TO_TICKS(DISTANCE_SENSORS_DELAY));
    }
}

static void printDistanceData(const vl53l0x_data_t *data)
{
    printf("\n");
    ESP_LOGI(TAG,
        "distance=%u mm, valid=%s, status=%u (%s), signal=%.3f Mcps, ambient=%.3f Mcps",
        (unsigned)data->distance_mm,
        data->valid ? "true" : "false",
        (unsigned)data->range_status,
        vl53l0x_range_status_str(data->range_status),
        (double)data->signal_rate_mcps,
        (double)data->ambient_rate_mcps);
}

void configColorSensor(){
    const i2c_master_bus_config_t color_bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_1,
        .sda_io_num = COLOR_SDA,
        .scl_io_num = COLOR_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t color_bus_handle = NULL;

    ESP_ERROR_CHECK(i2c_new_master_bus(&color_bus_cfg, &color_bus_handle));

    i2c_device_config_t color_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x29,
        .scl_speed_hz = 100000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(color_bus_handle, &color_dev_cfg, &color_dev_handle));

    uint8_t RGB_SENS_ID = 0x80 | 0x12;
    uint8_t returned_id = 0;

    // tentando pegar ID para ver se o sensor é funcional:
    ESP_ERROR_CHECK(i2c_master_transmit_receive(color_dev_handle, &RGB_SENS_ID, sizeof(RGB_SENS_ID), &returned_id, 1, 5000));

    if(returned_id == 0x44 || returned_id == 0x4D){
        ESP_LOGI(TAG, "Sensor funcionando... %" PRIu8, returned_id);
    } else {
        ESP_LOGE(TAG, "nao foi possivel reconhecer resposta do sensor...");
        return;
    }
    
    // Ativando sensor
    uint8_t WAKE_SIGN = 0x80 | 0x00;

    uint8_t WAKE_ARR[2] = {WAKE_SIGN, 0x03};

    ESP_ERROR_CHECK(i2c_master_transmit(color_dev_handle, WAKE_ARR, sizeof(WAKE_ARR), 5000));
    uint8_t ATIME_ARR[2] = {0x80 | 0x01, 0xF6};   // 10 ciclos = ~24 ms
    ESP_ERROR_CHECK(i2c_master_transmit(color_dev_handle, ATIME_ARR, sizeof(ATIME_ARR), 5000));

    uint8_t AGAIN_ARR[2] = {0x80 | 0x0F, 0x02};   // ganho 16x
    ESP_ERROR_CHECK(i2c_master_transmit(color_dev_handle, AGAIN_ARR, sizeof(AGAIN_ARR), 5000));
}

#define colorLimit 200
#define redColorLimit 130
#define greenColorLimit 200
#define blueColorLimit 200

// MODIFICADO: função nova. Bipa o buzzer (canal LEDC 2 / timer 1, já configurados no app_main).
// Bloqueia só a task do sensor de cor, e o AGV já está parado quando ela é chamada.
// times = quantidade de bipes | onMs = duração de cada bipe | offMs = pausa entre bipes
void beepBuzzer(int times, uint32_t onMs, uint32_t offMs){
    for(int i = 0; i < times; i++){
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 8191/2);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
        vTaskDelay(pdMS_TO_TICKS(onMs));
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
        vTaskDelay(pdMS_TO_TICKS(offMs));
    }
}

void readColorSensor(void *pvParameters){
    // sequencia de endereços CDATA(0x14) => CDATAH => RDATA => RDATAH => GDATA => GDATAH => BDATA => BDATAH
    // CLEAR => RED => GREEN => BLUE

    // primeiro endereco para ler registradores em cadeia
    // 0x80: command
    // 0x20: auto increment
    // 0x14: first data byte (CDATA)
    uint8_t FIRST_ADDR = 0x80 | 0x20 | 0x14;

    uint8_t sens_resp_buff[8];

    TickType_t lastWake = xTaskGetTickCount();

    char buffer[16];

    // conta leituras seguidas SEM azul (usado para rearmar o onBlueSquare)
    uint8_t notBlueReads = 0;
    uint8_t notRedReads = 0;
    
    while(1){
    // função para me ajudar a ler um burst, 8 endereços em sequência:
    //ESP_ERROR_CHECK(readBurst(dev_handle, FIRST_ADDR, sens_resp_buff, sizeof(sens_resp_buff))); // retorna valores CRGB de 0 a 255

    ESP_ERROR_CHECK(i2c_master_transmit_receive(color_dev_handle, &FIRST_ADDR, sizeof(FIRST_ADDR), sens_resp_buff, sizeof(sens_resp_buff), 5000));

    uint16_t CLEAR = 0;
    uint16_t RED = 0;
    uint16_t GREEN = 0;
    uint16_t BLUE = 0;

    CLEAR = (sens_resp_buff[1] << 8) | sens_resp_buff[0];
    RED = (sens_resp_buff[3] << 8) | sens_resp_buff[2];
    GREEN = (sens_resp_buff[5] << 8) | sens_resp_buff[4];
    BLUE = (sens_resp_buff[7] << 8) | sens_resp_buff[6];

    if(CLEAR > 0){
        float R_MAX = 5000.0f;
        float G_MAX = 2000.0f;
        float B_MAX = 2000.0f;

        uint16_t R_norm = ((float)RED / R_MAX) * 255.0f;
        uint16_t G_norm = ((float)GREEN / G_MAX) * 255.0f;
        uint16_t B_norm = ((float)BLUE / B_MAX) * 255.0f;

        /*         
        ESP_LOGI(TAG, "RED: %" PRIu16, R_norm);
        ESP_LOGI(TAG, "GREEN: %" PRIu16, G_norm);
        ESP_LOGI(TAG, "BLUE: %" PRIu16, B_norm);
        */

        // ESP_LOGI(TAG, "\033[0;31mRED: %" PRIu16, R_norm);
        // ESP_LOGI(TAG, "\033[0;32mGREEN: %" PRIu16, G_norm);
        // ESP_LOGI(TAG, "\033[0;34mBLUE: %" PRIu16, B_norm);

        snprintf(buffer, sizeof(buffer), "%" PRIu16, R_norm);
        esp_mqtt_client_publish(mqttClient, "/agv/color/R", buffer, 0, 0, 0);
        snprintf(buffer, sizeof(buffer), "%" PRIu16, G_norm);
        esp_mqtt_client_publish(mqttClient, "/agv/color/G", buffer, 0, 0, 0);
        snprintf(buffer, sizeof(buffer), "%" PRIu16, B_norm);
        esp_mqtt_client_publish(mqttClient, "/agv/color/B", buffer, 0, 0, 0);

        // MODIFICADO: durante a virada a cor não decide nada
        if (motorCommand == RIGHT || motorCommand == LEFT) {
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(COLOR_SENSOR_DELAY));
            continue;
        }

        // rearma a detecção do azul depois de 3 leituras seguidas (~75 ms) sem azul.
        // Assim um tremido na borda do quadrado não conta o mesmo quadrado duas vezes.
        if (B_norm >= blueColorLimit && B_norm > R_norm && B_norm > G_norm) {
            notBlueReads = 0;
        } else if (notBlueReads < 3) {
            notBlueReads++;
            if (notBlueReads == 3){
                onBlueSquare = false;
                ignoreBluesquare = false;
            } 
        }

        if (motorCommand == STAY && R_norm >= redColorLimit && R_norm > G_norm && R_norm > B_norm) {
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(COLOR_SENSOR_DELAY));
            continue;
        }

        if (R_norm >= redColorLimit && R_norm > G_norm && R_norm > B_norm) {
            notRedReads = 0;
        } else if (notRedReads < 3) {
            notRedReads++;
            if (notRedReads == 3) onRedSquare = false;
        }

        // análise das variáveis e atribuição das funções de cada cor
        if(R_norm >= colorLimit && G_norm >= colorLimit && B_norm >= colorLimit){
            ESP_LOGI(TAG, "\037[0;37mWHITE: %" PRIu16, R_norm + G_norm + B_norm);
            motorCommand = FORWARD;
            xTaskNotifyGive(MotorTaskHandle);
            } else if(R_norm >= redColorLimit && R_norm > G_norm && R_norm > B_norm){
                // MODIFICADO: freia só na ENTRADA do vermelho (antes: a cada 25 ms)
                if(!onRedSquare){
                    onRedSquare = true;
                    motorCommand = BREAK_MOTORS;
                    xTaskNotifyGive(MotorTaskHandle);
                    esp_mqtt_client_publish(mqttClient, "/agv/atBase", "1", 0, 0, 0);
                }
            }
        // else if(G_norm >= greenColorLimit && G_norm > R_norm && G_norm > B_norm){
        //     //TODO: colocar função do verde
        //     motorCommand = BREAK_MOTORS;
        //     xTaskNotifyGive(MotorTaskHandle);
        // } 
        // COR AZUL:
        // MODIFICADO: o AGV NÃO vira mais para entrar nas salas. Agora ele só conta os quadrados azuis
        // (roomSquareCounter) e, quando chega no desejado (counter == 0), para e bipa, igual ao vermelho
        // (BREAK_MOTORS -> LED vermelho piscando; duplo clique no botão -> STAY).
            else if(B_norm >= blueColorLimit &&  B_norm > R_norm && B_norm > G_norm){
                if(!onBlueSquare){
                    onBlueSquare = true;

                    // Se o escudo estiver ativo (acabou de virar 180°), passa direto e consome o escudo
                    if (ignoreBluesquare) {
                        ignoreBluesquare = false;
                        motorCommand = FORWARD;
                        xTaskNotifyGive(MotorTaskHandle);
                    } 
                    // Navegação normal
                    else {
                        if(roomSquareCounter > 0){
                            roomSquareCounter--;
                            snprintf(buffer, sizeof(buffer), "%d", roomSquareCounter);
                            esp_mqtt_client_publish(mqttClient, "/agv/squarePassed", buffer, 0, 0, 0);
                        }
                        if(roomSquareCounter > 0){
                            motorCommand = FORWARD;
                            xTaskNotifyGive(MotorTaskHandle);
                        } else {
                            motorCommand = BREAK_MOTORS;
                            xTaskNotifyGive(MotorTaskHandle);
                            beepBuzzer(3, 150, 100);
                            lastWake = xTaskGetTickCount();
                        }
                    }
                }
        }
        else {
            if(!turningRoutine){
                motorCommand = FORWARD;
                xTaskNotifyGive(MotorTaskHandle);
            }
        }
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(COLOR_SENSOR_DELAY));
    } 
}

void activateLed(void *pvParameters){
    // config LED RGB
    tNeopixelContext neopixel = neopixel_Initialize(1, LED_DATA_PIN, NEOPIXEL_MODE_WS2812B);
    uint32_t LED_STATE = INIT_BLINK;
    uint32_t NEW_STATE = INIT_BLINK;

    tNeopixel pixel[] =
    {
        { 0, NP_RGB(50, 0,  0) }, /* red */
        { 0, NP_RGB(0,  50, 0) }, /* green */
        { 0, NP_RGB(0,  0, 50) }, /* blue */
        { 0, NP_RGB(50,  50, 0) }, /* yellow */
        { 0, NP_RGB(0,  0,  0) }, /* off */
    };

    while(1){
        if (xTaskNotifyWait(0x00, UINT32_MAX, &NEW_STATE, 0) == pdTRUE) {
            LED_STATE = NEW_STATE;
        }

        switch(LED_STATE){
            case(INIT_BLINK):{
                neopixel_SetPixel(neopixel, &pixel[2], 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                neopixel_SetPixel(neopixel, &pixel[4], 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                neopixel_SetPixel(neopixel, &pixel[2], 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                neopixel_SetPixel(neopixel, &pixel[4], 1);
                vTaskDelay(pdMS_TO_TICKS(300));
                break;
            }
            case(WALKING):{
                neopixel_SetPixel(neopixel, &pixel[1], 1);
                vTaskDelay(pdMS_TO_TICKS(50));
                break;
            }
            case(WARNING_BLINK):{
                neopixel_SetPixel(neopixel, &pixel[3], 1);
                vTaskDelay(pdMS_TO_TICKS(300));
                neopixel_SetPixel(neopixel, &pixel[4], 1);
                vTaskDelay(pdMS_TO_TICKS(300));
                break;
            }
            case(STOPED_ACTION_NEEDED):{
                neopixel_SetPixel(neopixel, &pixel[0], 1);
                vTaskDelay(pdMS_TO_TICKS(500));
                neopixel_SetPixel(neopixel, &pixel[4], 1);
                vTaskDelay(pdMS_TO_TICKS(500));
                break;
            }
            case(STOPED):{
                neopixel_SetPixel(neopixel, &pixel[0], 1);
                vTaskDelay(pdMS_TO_TICKS(50));
                break;
            }
        }
    }
    // neopixel_Deinit(neopixel);
}

void extractAndAssignWebCommands(char* responseJSON, int dataLen){
    cJSON *parsed = cJSON_ParseWithLength(responseJSON, (size_t)dataLen);
    if (parsed == NULL) {
        // dup de código aqui salva RAM
        ESP_LOGE(TAG, "Erro ao parsear o JSON. Formato inválido.");
        esp_mqtt_client_publish(mqttClient, "/agv/error", "Erro ao parsear o JSON. Formato inválido.", 0, 0, 0);
        return;
    } else{
        ESP_LOGI(TAG, "sucesso ao parsear");
    }

    cJSON *roomSquareCounterExtracted = cJSON_GetObjectItem(parsed, "roomSquareCounter");
    cJSON *agvDirectionExtracted = cJSON_GetObjectItem(parsed, "agvDirection");
    cJSON *roomDirectionExtracted = cJSON_GetObjectItem(parsed, "roomDirection");

    roomSquareCounter = roomSquareCounterExtracted->valueint;
    lastTripSquareCounter = roomSquareCounter;
    agvDirection = agvDirectionExtracted->valueint;
    roomDirection = roomDirectionExtracted->valueint;

    if (roomSquareCounter > 0 && (motorCommand == BREAK_MOTORS || motorCommand == STAY)) {
        motorCommand = FORWARD;
        xTaskNotifyGive(MotorTaskHandle);
    }
}

#define DEBOUNCE_TIME 100

#define DOUBLE_PRESS_MS 400

int64_t prevTime = 0;

bool waitDouble = false;

void configButton() {
    button_gpio_config_t gpio_btn_cfg = {
        .gpio_num = 4,
        .active_level = 0,
    };
    button_config_t btn_config = {
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
    };

    button_handle_t gpio_btn = NULL;
    esp_err_t err = iot_button_new_gpio_device(&btn_config, &gpio_btn_cfg, &gpio_btn);
    
    if (err != ESP_OK || gpio_btn == NULL) {
        ESP_LOGE(TAG, "Button create failed");
        return;
    }

    iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, NULL, button_single_click_cb, NULL);
    iot_button_register_cb(gpio_btn, BUTTON_DOUBLE_CLICK, NULL, button_double_click_cb, NULL);
    iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_START, NULL, button_hold_cb, NULL);
}
static void button_single_click_cb(void *btn_handle, void *usr_data) {
    beepBuzzer(1, 150, 100);
    // MODIFICADO: só vale parado e com uma viagem para desfazer
    if (motorCommand == BREAK_MOTORS && lastTripSquareCounter > 0) {
        roomSquareCounter = lastTripSquareCounter;   // mesma contagem, sentido contrário
        lastTripSquareCounter = 0;                   // consumida: um 2º clique não faz nada
        turnStartTime = esp_timer_get_time() / 1000; // arma o tempo mínimo da virada
        motorCommand = RIGHT;
        xTaskNotifyGive(MotorTaskHandle);
    }
    ESP_LOGI(TAG, "BUTTON_SINGLE_CLICK");
}

static void button_double_click_cb(void *btn_handle, void *usr_data) {
    beepBuzzer(2, 150, 100);
    if(motorCommand == BREAK_MOTORS){
        ESP_LOGI(TAG, "BUTTON_DOUBLE_CLICK");
        motorCommand = STAY;
        xTaskNotifyGive(MotorTaskHandle);
    }
}

static void button_hold_cb(void *btn_handle, void *usr_data) {
        beepBuzzer(1, 650, 100);
        roomSquareCounter = 0;
        lastTripSquareCounter = 0;
        esp_mqtt_client_publish(mqttClient, "/agv/atBase", "1", 0, 0, 0);
        ESP_LOGI(TAG, "BUTTON_HOLD");
        motorCommand = STAY;
        xTaskNotifyGive(MotorTaskHandle);

}