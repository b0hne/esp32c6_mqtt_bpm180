#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "protocol_examples_common.h"

#include "esp_crt_bundle.h"
#include "mqtt_client.h"
#include "bmp180.h"

static const char *TAG = "mqtts_thermostat";

#if CONFIG_EXAMPLE_BROKER_CERTIFICATE_OVERRIDDEN
static const char cert_override_pem[] =
    "-----BEGIN CERTIFICATE-----\n" CONFIG_EXAMPLE_BROKER_CERTIFICATE_OVERRIDE "\n"
    "-----END CERTIFICATE-----";
#endif

#if CONFIG_EXAMPLE_CERT_VALIDATE_MOSQUITTO_CA
/* Embedded Mosquitto CA certificate for test.mosquitto.org:8883 */
extern const uint8_t mosquitto_org_crt_start[] asm("_binary_mosquitto_org_crt_start");
extern const uint8_t mosquitto_org_crt_end[] asm("_binary_mosquitto_org_crt_end");
#endif

char payload[32];

static esp_mqtt_client_handle_t s_client = NULL;

/**
 * @brief FreeRTOS task for periodic temperature publishing.
 *
 * Discards initial startup readings to avoid outliers,
 * then reads temperature & pressure every 10 seconds and publishes
 * to the configured MQTT topics.
 *
 * @param arg  Unused task parameter
 *
 * @return void (runs indefinitely)
 */
static void temp_publish_task(void *arg)
{
    // throw away first readings
    for (int i = 0; i < 5; i++) {
        float t; int32_t p;
        bmp180_read(&t, &p);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    while (1) {
        float t; int32_t p;
        if (bmp180_read(&t, &p) && s_client) {
            // Temperature (°C)
            char t_payload[32];
            snprintf(t_payload, sizeof(t_payload), "%.2f", (double)t);

            // Pressure (hPa)
            char p_payload[32];
            snprintf(p_payload, sizeof(p_payload), "%.2f", (double)p / 100.0);
            esp_mqtt_client_publish(s_client, "esp32a/temperature", t_payload, 0, 0, 1);
            esp_mqtt_client_publish(s_client, "esp32a/pressure_hpa", p_payload, 0, 0, 1);
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

/**
 * @brief MQTT event callback.
 *
 * Handles connection lifecycle events.
 * On successful connection:
 *   - Publishes Home Assistant discovery payload (retained)
 *   - Starts periodic temperature publishing task
 *
 * @param handler_args  User data (unused)
 * @param base          Event base
 * @param event_id      MQTT event ID
 * @param event_data    Event data structure
 *
 * @return void
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED: {
        static bool started = false;

        const char *discovery_c =
            "{"
            "\"name\":\"BMP180 Temperature\","
            "\"state_topic\":\"esp32a/temperature\","
            "\"unit_of_measurement\":\"°C\","
            "\"device_class\":\"temperature\","
            "\"state_class\":\"measurement\","
            "\"unique_id\":\"esp32a_temperature\","
            "\"value_template\":\"{{ value | float }}\","
            "\"device\":{"
            "\"identifiers\":[\"esp32a\"],"
            "\"name\":\"ESP32-C6 a\","
            "\"model\":\"ESP32-C6\","
            "\"manufacturer\":\"Espressif\""
            "}"
            "}";

        esp_mqtt_client_publish(
            client,
            "homeassistant/sensor/esp32a/temperature/config",
            discovery_c,
            0,
            1,  // retain
            0
        );
        const char *discovery_p =
            "{"
            "\"name\":\"BMP180 Pressure\","
            "\"state_topic\":\"esp32a/pressure_hpa\","
            "\"unit_of_measurement\":\"hPa\","
            "\"device_class\":\"pressure\","
            "\"state_class\":\"measurement\","
            "\"unique_id\":\"esp32_a_pressure\","
            "\"value_template\":\"{{ value | float }}\","
            "\"device\":{"
            "\"identifiers\":[\"esp32a\"],"
            "\"name\":\"ESP32-C6 a\","
            "\"model\":\"ESP32-C6\","
            "\"manufacturer\":\"Espressif\""
            "}"
            "}";

        esp_mqtt_client_publish(
            client,
            "homeassistant/sensor/esp32a/pressure/config",
            discovery_p,
            0,
            1, // retain
            0
        );

        if (!started) {
            started = true;
            xTaskCreate(temp_publish_task, "temp_pub", 4096, NULL, 5, NULL);
        }

        ESP_LOGI(TAG, "MQTT connected");
        break;
    }

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        break;

    default:
        break;
    }
}

/**
 * @brief Initialize and start MQTT client.
 *
 * Creates MQTT client instance using URI defined in menuconfig,
 * registers event handler, and starts connection.
 *
 * @return void
 */
static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = CONFIG_EXAMPLE_MQTT_BROKER_URI,
#if CONFIG_EXAMPLE_BROKER_CERTIFICATE_OVERRIDDEN
            .verification.certificate = cert_override_pem,
#elif CONFIG_EXAMPLE_CERT_VALIDATE_MOSQUITTO_CA
            .verification.certificate = (const char *)mosquitto_org_crt_start,
#else
            .verification.crt_bundle_attach = esp_crt_bundle_attach, /* Use built-in certificate bundle */
#endif
        },
    };

    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    s_client = client;

    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

/**
 * @brief Application entry point.
 *
 * Initializes NVS, network stack, Wi-Fi connection,
 * BMP180 sensor, and MQTT client.
 *
 * @return void
 */
void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    bmp180_init();
    mqtt_app_start();
}
