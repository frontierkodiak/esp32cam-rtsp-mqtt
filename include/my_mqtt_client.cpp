#include "my_mqtt_client.h"
#include "esp_log.h"

static constexpr const char* TAG = "MyMqttClient";

esp_err_t MyMqttClient::mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
    return ESP_OK;
}

void MyMqttClient::mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(static_cast<esp_mqtt_event_handle_t>(event_data));
}

esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
    MyMqttClient::mqtt_event_handler_cb(event);
    return ESP_OK;
}


MyMqttClient::MyMqttClient(const char* mqtt_server, const char* mqtt_port, const char* mqtt_user, const char* mqtt_password) {
    char mqtt_uri[128];
    snprintf(mqtt_uri, sizeof(mqtt_uri), "mqtt://%s:%s", mqtt_server, mqtt_port);
    
    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = mqtt_uri,
        .username = mqtt_user,
        .password = mqtt_password
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, client);
}

void MyMqttClient::start() {
    esp_mqtt_client_start(client);
}

void MyMqttClient::publish(const char* topic, const char* data) {
    esp_mqtt_client_publish(client, topic, data, 0, 1, 0);
}
