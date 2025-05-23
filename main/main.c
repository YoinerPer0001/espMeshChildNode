#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_mesh.h"
#include "cJSON.h"
#include "driver/gpio.h"
#include <math.h>  // Asegúrate de incluir esto

#define TAG "MESH_NODE"
#define BUF_SIZE 256

extern const char root_cert_pem_start[] asm("_binary_root_cert_pem_start");
extern const char root_cert_pem_end[] asm("_binary_root_cert_pem_end");

#define WIFI_SSID "FLIA PERTUZ"
#define WIFI_PASS "1079913099"

#define LED_GPIO GPIO_NUM_2

void mesh_receive_task(void *arg)
{
    mesh_addr_t from;
    mesh_data_t data;
    int flag;
    esp_err_t err;

    uint8_t rx_buf[150];
    data.data = rx_buf;
    data.size = sizeof(rx_buf);
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;

    while (1)
    {
        err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
        if (err == ESP_OK)
        {
            ESP_LOGI("MESH_RECV", "Mensaje desde %02x:%02x:%02x:%02x:%02x:%02x -> %s",
                     from.addr[0], from.addr[1], from.addr[2],
                     from.addr[3], from.addr[4], from.addr[5],
                     (char *)data.data);

            cJSON *root = cJSON_Parse((char *)data.data);

            if (root != NULL)
            {
                cJSON *flag_item = cJSON_GetObjectItem(root, "flag");
                if (flag_item != NULL && cJSON_IsNumber(flag_item))
                {
                    if (flag_item->valueint == 1)
                    {
                        gpio_set_level(LED_GPIO, 1);
                    }
                    else
                    {
                        gpio_set_level(LED_GPIO, 0);
                    }
                }
                cJSON_Delete(root); // Liberar memoria del JSON
            }
            else
            {
                ESP_LOGE("JSON", "Error al parsear JSON");
            }
        }
        else
        {
            ESP_LOGE("MESH_RECV", "Error al recibir datos: %s", esp_err_to_name(err));
        }
    }
}

// data mesh
static uint8_t MESH_ID[6] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB};
#define CONFIG_MESH_CHANNEL 11 // Canal 1
#define CONFIG_MESH_AP_CONNECTIONS 10
#define CONFIG_MESH_AP_PASSWD "meshTest"
static const char *MESH_TAG = "mesh_main";

#define BUF_SIZE 256
uint8_t recv_buf[BUF_SIZE];

void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(MESH_TAG, "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&event->ip_info.ip));
}

static bool is_root = false;

void mesh_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case MESH_EVENT_STARTED:
        ESP_LOGI(TAG, "Malla inicializada");
        break;
    case MESH_EVENT_STOPPED:
        ESP_LOGI(TAG, "Malla detenida");
        break;
    case MESH_EVENT_CHILD_CONNECTED:
        ESP_LOGI(TAG, "Nuevo hijo conectado");
        break;
    case MESH_EVENT_CHILD_DISCONNECTED:
        ESP_LOGI(TAG, "Hijo desconectado");
        break;
    case MESH_EVENT_PARENT_CONNECTED:
        ESP_LOGI(TAG, "Conectado al padre");
        // encender led

        if (esp_mesh_is_root())
        {
            ESP_LOGI(TAG, "Soy el nodo root");
            is_root = true;
        }
        else
        {
            ESP_LOGI(TAG, "No soy el nodo root");
        }
        break;
    default:
        break;
    }
}

void wifi_init()
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA)); // Cambiado aquí
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect()); //
}

void mesh_init()
{
    /*  mesh initialization */
    esp_mesh_init();
    /*  register mesh events handler */
    esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL);

    /* Enable the Mesh IE encryption by default */
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    /* mesh ID */
    memcpy((uint8_t *)&cfg.mesh_id, MESH_ID, sizeof(MESH_ID));
    /* channel (must match the router's channel) */
    cfg.channel = CONFIG_MESH_CHANNEL;
    /* router */
    cfg.router.ssid_len = strlen(WIFI_SSID);
    memcpy((uint8_t *)&cfg.router.ssid, WIFI_SSID, cfg.router.ssid_len);
    memcpy((uint8_t *)&cfg.router.password, WIFI_PASS,
           strlen(WIFI_PASS));
    /* mesh softAP */
    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    memcpy((uint8_t *)&cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD,
           strlen(CONFIG_MESH_AP_PASSWD));
    esp_mesh_set_config(&cfg);

    esp_mesh_set_self_organized(true, false);

    /* mesh start */
    ESP_ERROR_CHECK(esp_mesh_start());

    esp_mesh_connect();
}

void mesh_send_data(const char *msg)
{
    mesh_data_t mesh_data;
    mesh_addr_t dest_addr;
    esp_err_t err;

    // Obtener la dirección del nodo padre (normalmente el root)
    err = esp_mesh_get_parent_bssid(&dest_addr);
    if (err != ESP_OK)
    {
        ESP_LOGE("MESH_SEND", "No se pudo obtener dirección del padre: %s", esp_err_to_name(err));
        return;
    }

    // Imprimir la dirección del nodo padre (root o el nodo al que está conectado)
    ESP_LOGI("MEEEEESH MAC DAD", "Dirección del nodo padre: %02x:%02x:%02x:%02x:%02x:%02x",
             dest_addr.addr[0], dest_addr.addr[1], dest_addr.addr[2],
             dest_addr.addr[3], dest_addr.addr[4], dest_addr.addr[5]);

    // Configurar datos
    mesh_data.data = (uint8_t *)msg;
    mesh_data.size = strlen(msg) + 1; // incluir el '\0'
    mesh_data.proto = MESH_PROTO_BIN;
    mesh_data.tos = MESH_TOS_P2P;

    if (mesh_data.size > MESH_MTU)
    {
        ESP_LOGE("MESH_SEND", "El mensaje es demasiado grande para enviarlo");
        return;
    }

    // Enviar
    err = esp_mesh_send(NULL, &mesh_data, MESH_DATA_TODS, NULL, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE("MESH_SEND", "Error al enviar datos: %s", esp_err_to_name(err));
        switch (err)
        {
        case ESP_ERR_MESH_ARGUMENT:
            ESP_LOGE("MESH_SEND", "Argumento inválido");
            break;
        case ESP_ERR_MESH_NO_MEMORY:
            ESP_LOGE("MESH_SEND", "Memoria insuficiente");
            break;
        case ESP_ERR_MESH_EXCEED_MTU:
            ESP_LOGE("MESH_SEND", "El tamaño del paquete excede el MTU");
            break;
        default:
            ESP_LOGE("MESH_SEND", "Error desconocido");
            break;
        }
    }
    else
    {
        ESP_LOGI("MESH_SEND", "El mensaje se ha enviado: %s", mesh_data.data);
    }
}

void mesh_send_task(void *pvParameters)
{
    // Lista de sensores
    const char *sensor_names[] = {"Temperatura", "Humedad"};
    const int num_sensores = sizeof(sensor_names) / sizeof(sensor_names[0]);

    while (true)
    {
        // Crear un objeto cJSON
        cJSON *root = cJSON_CreateObject();
        cJSON *sensores = cJSON_CreateObject();

        // Añadir datos al JSON
        cJSON_AddStringToObject(root, "id", "cc:7b:5c:1e:f8:b8");

        float temperatura = floorf((20.0 + (esp_random() % 1500) / 100.0f) * 100) / 100.0f;
        float humedad = floorf((30.0 + (esp_random() % 700) / 10.0f) * 100) / 100.0f;

        cJSON_AddNumberToObject(sensores, "Temperatura", temperatura);
        cJSON_AddNumberToObject(sensores, "Humedad", humedad);

        // Añadir el objeto "sensores" al objeto raíz
        cJSON_AddItemToObject(root, "sensores", sensores);

        // Convertir el objeto cJSON a string
        char *json_str = cJSON_PrintUnformatted(root);

        // Enviar el JSON
        mesh_send_data(json_str);

        // Liberar la memoria del JSON
        free(json_str);
        cJSON_Delete(root);

        vTaskDelay(pdMS_TO_TICKS(5000)); // cada 10 segundos
    }
}

void app_main(void)
{
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1);
    nvs_flash_init();
    wifi_init();
    mesh_init();
    // Crear una tarea para la envio de datos
    xTaskCreate(&mesh_send_task, "Mesh_send_Task", 4096, NULL, 5, NULL);
    xTaskCreate(mesh_receive_task, "mesh_recv", 4096, NULL, 5, NULL);
}
