// Home Assistant MQTT Autodiscovery for mmWave Presence Sensor
// This module handles automatic discovery of the presence sensor in Home Assistant

#include <ArduinoJson.h>

/**
 * Publishes a Home Assistant MQTT autodiscovery message for the presence sensor
 * This allows Home Assistant to automatically discover and configure the sensor
 * without requiring manual YAML configuration.
 * 
 * Topic: homeassistant/sensor/{node_id}/presence/config
 * Where:
 *   - node_id = the value of the "Where" variable
 *   - object_id = "presence"
 *   - state_topic = the value of MQTT_state_topic variable
 * 
 * The payload includes all necessary autodiscovery fields for Home Assistant
 */
void publish_ha_autodiscovery() {
    // Verify MQTT connection and required variables
    if (!connectedMQTT) {
        log_w("Cannot publish HA autodiscovery, not connected to MQTT");
        return;
    }
    
    if (!strlen(Where)) {
        log_e("Cannot publish HA autodiscovery, 'Where' variable is empty");
        return;
    }
    
    if (!strlen(MQTT_state_topic)) {
        log_e("Cannot publish HA autodiscovery, 'MQTT_state_topic' is empty");
        return;
    }
    
    // Create the discovery topic
    // Format: homeassistant/sensor/{node_id}/presence/config
    char discovery_topic[256];
    snprintf(discovery_topic, sizeof(discovery_topic) - 1, 
             "homeassistant/sensor/%s/presence/config", Where);
    
    // Create JSON payload using ArduinoJson
    JsonDocument doc;
    
    // Required fields
    doc["name"] = Where;  // Human-readable name (can be "Where" value)
    doc["unique_id"] = Where;  // Unique identifier for this sensor
    doc["state_topic"] = MQTT_state_topic;  // Topic where state is published
    
    // Object ID (used to generate entity_id in Home Assistant)
    doc["object_id"] = "presence";
    
    // Device class - indicates this is a presence/occupancy sensor
    doc["device_class"] = "occupancy";
    
    // Payload values for on/off states
    doc["payload_on"] = "1";
    doc["payload_off"] = "0";
    
    // Retain the message so Home Assistant picks it up
    // Unit of measurement (presence is unitless)
    doc["unit_of_measurement"] = "";
    
    // MQTT discovery recommended setting: retain message
    doc["availability_topic"] = ""; // Optional: can be used to indicate device availability
    
    // Device information for grouping sensors
    JsonObject device = doc.createNestedObject("device");
    device["identifiers"].add(Where);  // Device identifier
    device["name"] = Where;  // Device name
    device["model"] = "mmWave Presence Sensor";  // Model description
    device["manufacturer"] = "Seeed Studio";  // Manufacturer
    device["sw_version"] = VERSION;  // Software version
    
    // Optional: expire_after (in seconds) - time before sensor is considered unavailable
    // Uncomment if you want Home Assistant to mark sensor unavailable after this time
    // doc["expire_after"] = 300;  // 5 minutes
    
    // Serialize JSON to buffer
    char json_buffer[512];
    size_t json_length = serializeJson(doc, json_buffer, sizeof(json_buffer));
    
    if (json_length == 0) {
        log_e("Failed to serialize HA autodiscovery JSON");
        return;
    }
    
    log_d("HA Autodiscovery JSON: %s", json_buffer);
    
    // Publish with retain flag (QoS 1, retain = true)
    uint16_t packetId = mqttClient.publish(discovery_topic, 1, true, json_buffer);
    
    if (packetId != 0) {
        log_i("Published HA autodiscovery to topic: %s", discovery_topic);
        log_i("Payload: %s", json_buffer);
    } else {
        log_e("Failed to publish HA autodiscovery message");
    }
}

/**
 * Alternative lightweight version without ArduinoJson (if memory is constrained)
 * Uses sprintf to build JSON string manually
 */
void publish_ha_autodiscovery_lightweight() {
    // Verify MQTT connection and required variables
    if (!connectedMQTT) {
        log_w("Cannot publish HA autodiscovery, not connected to MQTT");
        return;
    }
    
    if (!strlen(Where)) {
        log_e("Cannot publish HA autodiscovery, 'Where' variable is empty");
        return;
    }
    
    if (!strlen(MQTT_state_topic)) {
        log_e("Cannot publish HA autodiscovery, 'MQTT_state_topic' is empty");
        return;
    }
    
    // Create the discovery topic
    char discovery_topic[256];
    snprintf(discovery_topic, sizeof(discovery_topic) - 1, 
             "homeassistant/sensor/%s/presence/config", Where);
    
    // Build JSON manually
    char json_buffer[512];
    snprintf(json_buffer, sizeof(json_buffer) - 1,
             "{"
             "\"name\":\"%s\","
             "\"unique_id\":\"%s\","
             "\"state_topic\":\"%s\","
             "\"device_class\":\"occupancy\","
             "\"payload_on\":\"1\","
             "\"payload_off\":\"0\","
             "\"device\":{"
             "\"identifiers\":[\"%s\"],"
             "\"name\":\"%s\","
             "\"model\":\"mmWave Presence Sensor\","
             "\"manufacturer\":\"Seeed Studio\","
             "\"sw_version\":\"%s\""
             "}"
             "}",
             Where,
             Where,
             MQTT_state_topic,
             Where,
             Where,
             VERSION);
    
    log_d("HA Autodiscovery JSON: %s", json_buffer);
    
    // Publish with retain flag (QoS 1, retain = true)
    uint16_t packetId = mqttClient.publish(discovery_topic, 1, true, json_buffer);
    
    if (packetId != 0) {
        log_i("Published HA autodiscovery to topic: %s", discovery_topic);
    } else {
        log_e("Failed to publish HA autodiscovery message");
    }
}
