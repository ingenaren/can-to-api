#include <Arduino.h>
#include "driver/twai.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ==============================
// CONFIGURACIÓN CONSTANTES
// ==============================
static const gpio_num_t TWAI_TX_PIN = GPIO_NUM_5;
static const gpio_num_t TWAI_RX_PIN = GPIO_NUM_4;

// WiFi
const char* ssid =  "Monitoreo";
const char* password = "Monitoreo123";

// APIs
const char* api_local_endpoint = "http://192.168.0.156:4000/api/monitoring/save";
const char* api_remote_endpoint = "https://apiingenare-production.up.railway.app/api/grafana/save";

const uint32_t API_TIMEOUT_MS = 5000;
const uint32_t API_SEND_INTERVAL = 3000;
const uint32_t WIFI_RECONNECT_INTERVAL = 300000; // 5 minutos
const uint32_t WIFI_RECONNECT_TIMEOUT = 10000;   // 10 segundos para intentar reconexión

// ==============================
// VARIABLES GLOBALES
// ==============================
String device_id = "GEN-SM002";;

//"GEN-SM001";
//"GEN-SM002";
//"GEN-SM003";
//"MBR-SM001";
//"MEBR-SM001";

String deviceDiagnostic = "DC127";
bool enable_remote_api = true;
uint32_t last_api_send = 0;
uint32_t last_wifi_check = 0;
bool wifi_reconnecting = false;
uint32_t wifi_reconnect_start = 0;

// ==============================
// ESTRUCTURAS
// ==============================
struct CanMessage {
    uint32_t id;
    uint8_t length;
    uint8_t data[8];
    uint64_t timestamp;
    bool is_j1939;
    uint32_t pgn;
    uint8_t sa;
};

struct SpnMap {
    uint16_t spn;
    uint32_t pgn;
    uint8_t offset;
    uint8_t size;
    float resolution;
    float offset_val;
    bool is_signed;
    const char* diagnostic_id;
};



// ==============================
// MAPA SPN
// ==============================
SpnMap kSpnMap[] = {
    { 91,  0x00F003, 1, 1, 0.4f,   0,     false, "a8E-MlXuWg0-3oH2dO424Qw" },    //Accelerator pedal position
    { 92,  0x00F003, 2, 1, 1.0f,   0,     false, "a1Q7QlJDugkq92LxZ12KY1A" },    //Carga de motor
    { 94,  0x00FEEF, 0, 1, 4.0f,   0,     false, "arL7yn032ZE-yLC-BaASo1g" },    //Engine fuel delivery pressure (Spn 94)
    { 100, 0x00FEEF, 3, 1, 4.0f,   0,     false, "DiagnosticOilPressureId" },    //Presion del aceite
    { 101, 0x00FEEF, 4, 2, 1/128.0f, -250, false, "aCoq_uYk4Q0aKLR_jmWYZtg" },   //Engine crankcase pressure
    { 102, 0x00FEF6, 1, 1, 2.0f,   0,     false, "aszf6mhMM40mxfis2hzLLtA" },    //Engine intake manifold 1 pressure
    { 105, 0x00FEF6, 2, 1, 1.0f, -40,     false, "aSKae-1FNFkKaGvIVqOiE3w" },    //Engine intake manifold 1 temperature
    { 110, 0x00FEEE, 0, 1, 1.0f, -40,     false, "DiagnosticEngineCoolantTemperatureId" }, //Temperatura del refrigerante del motor
    { 168, 0x00FEF7, 4, 2, 0.05f, 0,      false, "DiagnosticBatteryVoltageId" }, //Battery potential / Power input 1
    { 174, 0x00FEEE, 1, 1, 1.0f, -40,     false, "aX01bmhZNoESjujTKxJBniQ" },    //Engine Fuel Temperature 1
    { 175, 0x00FEEE, 2, 2, 1/32.0f, -273, false, "DiagnosticEngineOilTemperatureID" }, //Temperatura de aceite del motor
    { 190, 0x00F004, 3, 2, 0.125f, 0,     false, "DiagnosticEngineSpeedId" },    //Velocidad del Motor
    { 177, 0x00FEF6, 0, 1, 1.0f, -40,     false, "DiagnosticTransmissionOilTemperatureId" }, //Transmission Oil TemperatureId
    { 127, 0x00FEEF, 5, 1, 4.0f,   0,     false, "DiagnosticTransmissionOilPressureId" },    //Transmission Oil PressureId
    { 244, 0x00FEEE, 4, 2, 0.05f, 0, false, "DiagnosticIdleFuelUsedId" }                // Cantidad de combustible utilizado en relentí 
};

// ==============================
// CACHE PARA API
// ==============================
struct ApiSentData {
    float last_value;
    uint32_t last_send_time;
    bool has_been_sent = false;
};

const size_t kSpnMapSize = sizeof(kSpnMap) / sizeof(kSpnMap[0]);
ApiSentData api_sent_cache[300];

// ==============================
// PROTOTIPOS
// ==============================
void setupWiFi();
void setupCAN();
void checkWiFiConnection();
void handleWiFiReconnection();
void receiveAndProcessCAN();
void processCANMessage(const CanMessage &msg);
void j1939Decode(const CanMessage &msg);
String getISOTimestamp();
void sendDataToAPI(const char* diagnostic_id, float value);
bool shouldSendToAPI(uint16_t spn, float value);
void updateApiCache(uint16_t spn, float value);
bool sendToEndpoint(const char* endpoint, const char* diagnostic_id, float value, const String& timestamp);

// ==============================
// SETUP
// ==============================
void setup() {
    Serial.begin(115200);
    delay(500);

    setupWiFi();
    setupCAN();

    configTime(0, 0, "pool.ntp.org");

    Serial.println("\nSistema iniciado.\n");
}

// ==============================
// LOOP
// ==============================
void loop() {
    // Procesar mensajes CAN (siempre activo)
    receiveAndProcessCAN();
    
    // Verificar WiFi periódicamente de forma no bloqueante
    checkWiFiConnection();
    
    // Manejar reconexión si está en proceso
    if (wifi_reconnecting) {
        handleWiFiReconnection();
    }
    
    delay(5); // Pequeño delay para evitar sobrecarga
}

// ==============================
// CONFIGURACIÓN WiFi INICIAL
// ==============================
void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid, password);

    Serial.print("Conectando al WiFi...");

    // Espera no bloqueante con timeout
    uint8_t retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
        retries++;
        Serial.print(".");
        delay(250);
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✓ WiFi conectado");
        deviceDiagnostic = "DC" + WiFi.macAddress().substring(12, 17);
        Serial.printf("Device ID: %s\n", deviceDiagnostic.c_str());
        enable_remote_api = true;
    } else {
        Serial.println("\n✗ WiFi NO conectado inicialmente");
        Serial.println("API remota deshabilitada hasta que se establezca conexión");
        enable_remote_api = false;
    }
    
    last_wifi_check = millis();
}

// ==============================
// VERIFICACIÓN PERIÓDICA DE WiFi
// ==============================
void checkWiFiConnection() {
    uint32_t now = millis();
    
    // Verificar cada 5 minutos (o después del timeout de reconexión)
    if ((now - last_wifi_check > WIFI_RECONNECT_INTERVAL) || 
        (wifi_reconnecting && (now - wifi_reconnect_start > WIFI_RECONNECT_TIMEOUT))) {
        
        last_wifi_check = now;
        
        // Si ya está conectado, no hacer nada
        if (WiFi.status() == WL_CONNECTED) {
            if (!enable_remote_api) {
                // Se recuperó la conexión
                enable_remote_api = true;
                Serial.println("✓ Conexión WiFi recuperada - API remota habilitada");
            }
            wifi_reconnecting = false;
            return;
        }
        
        // Si no está conectado, iniciar proceso de reconexión
        if (!wifi_reconnecting) {
            Serial.println("⚠ WiFi desconectado - Iniciando reconexión...");
            wifi_reconnecting = true;
            wifi_reconnect_start = now;
            enable_remote_api = false;
            
            // Intentar reconectar de inmediato
            WiFi.disconnect();
            delay(100);
            WiFi.reconnect();
        }
    }
}

// ==============================
// MANEJO DE RECONEXIÓN (NO BLOQUEANTE)
// ==============================
void handleWiFiReconnection() {
    uint32_t now = millis();
    
    // Verificar estado actual
    wl_status_t status = WiFi.status();
    
    switch (status) {
        case WL_CONNECTED:
            // ¡Conectado exitosamente!
            wifi_reconnecting = false;
            enable_remote_api = true;
            deviceDiagnostic = "DC" + WiFi.macAddress().substring(12, 17);
            Serial.printf("✓ Reconexión WiFi exitosa - Device ID: %s\n", deviceDiagnostic.c_str());
            break;
            
        case WL_CONNECT_FAILED:
        case WL_CONNECTION_LOST:
            // Falló la conexión, reintentar
            if (now - wifi_reconnect_start > 2000) { // Esperar 2 segundos entre intentos
                Serial.println("↻ Reintentando conexión WiFi...");
                WiFi.reconnect();
                wifi_reconnect_start = now;
            }
            break;
            
        case WL_IDLE_STATUS:
            // En proceso de conexión
            if (now - wifi_reconnect_start > 5000) { // Mostrar progreso cada 5 segundos
                Serial.print(".");
                wifi_reconnect_start = now;
            }
            break;
            
        case WL_NO_SSID_AVAIL:
            Serial.println("✗ SSID no disponible");
            wifi_reconnecting = false;
            break;
            
        case WL_DISCONNECTED:
            // Estado normal cuando se intenta conectar
            break;
            
        default:
            break;
    }
    
    // Timeout de reconexión (10 segundos)
    if (now - wifi_reconnect_start > WIFI_RECONNECT_TIMEOUT) {
        Serial.println("\n✗ Timeout de reconexión WiFi");
        wifi_reconnecting = false;
        enable_remote_api = false;
        Serial.println("API remota permanecerá deshabilitada hasta el próximo chequeo");
    }
}

// ==============================
// CONFIGURACIÓN CAN (SIN CAMBIOS)
// ==============================
void setupCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 100;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK &&
        twai_start() == ESP_OK) {
        Serial.println("✓ CAN OK (250 kbps)");
    } else {
        Serial.println("✗ Error inicializando CAN");
    }
}

// ==============================
// RECEPCIÓN CAN (SIN CAMBIOS)
// ==============================
void receiveAndProcessCAN() {
    twai_message_t m;

    if (twai_receive(&m, 0) == ESP_OK) {
        CanMessage msg;
        msg.id = m.identifier;
        msg.length = m.data_length_code;
        msg.is_j1939 = (m.flags & TWAI_MSG_FLAG_EXTD);
        memcpy(msg.data, m.data, 8);

        if (msg.is_j1939) {
            msg.pgn = (msg.id >> 8) & 0x3FFFF;
            msg.sa = msg.id & 0xFF;
        }

        processCANMessage(msg);
    }
}

// ==============================
// PROCESAR MENSAJE CAN (SIN CAMBIOS)
// ==============================
void processCANMessage(const CanMessage &msg) {
    if (msg.is_j1939) j1939Decode(msg);
}

// ==============================
// DECODIFICACIÓN J1939 (SIN CAMBIOS)
// ==============================
void j1939Decode(const CanMessage &msg) {
    for (size_t i = 0; i < kSpnMapSize; i++) {
        const SpnMap& m = kSpnMap[i];

        if (m.pgn != msg.pgn) continue;
        if (m.offset + m.size > msg.length) continue;

        uint32_t raw = 0;

        if (m.size == 1) raw = msg.data[m.offset];
        else if (m.size == 2) raw = msg.data[m.offset] | (msg.data[m.offset + 1] << 8);

        int32_t val = raw;
        if (m.is_signed && m.size == 2) val = (int16_t)raw;

        float phys = val * m.resolution + m.offset_val;

        if (shouldSendToAPI(m.spn, phys)) {
            sendDataToAPI(m.diagnostic_id, phys);
            updateApiCache(m.spn, phys);
        }
    }
}

// ==============================
// FUNCIONES API (CON MEJORAS PARA WiFi)
// ==============================
String getISOTimestamp() {
    time_t now = time(nullptr);
    if (now < 1609459200) { // 1 de enero de 2021
        // Si no hay tiempo NTP válido, usar tiempo desde inicio
        uint32_t uptime = millis() / 1000;
        char buf[30];
        snprintf(buf, 30, "UPTIME-%08lu", uptime);
        return String(buf);
    }
    
    struct tm* t = gmtime(&now);
    char buf[30];
    snprintf(buf, 30, "%04d-%02d-%02dT%02d:%02d:%02dZ",
        t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
        t->tm_hour, t->tm_min, t->tm_sec);

    return String(buf);
}

bool shouldSendToAPI(uint16_t spn, float value) {
    uint32_t now = millis();
    ApiSentData &c = api_sent_cache[spn];

    const uint32_t NO_REPEAT_WINDOW = 300000;  // 5 min

    if (!c.has_been_sent) return true;

    if (fabs(c.last_value - value) > 1.5f) return true;

    if ((now - c.last_send_time) > NO_REPEAT_WINDOW) return true;

    return false;
}

void updateApiCache(uint16_t spn, float value) {
    ApiSentData &c = api_sent_cache[spn];
    c.last_value = value;
    c.last_send_time = millis();
    c.has_been_sent = true;
}

bool sendToEndpoint(const char* endpoint, const char* diagnostic_id, float value, const String& timestamp) {
    HTTPClient http;
    http.setTimeout(API_TIMEOUT_MS);
    http.begin(endpoint);
    http.addHeader("Content-Type", "application/json");

    StaticJsonDocument<300> doc;

    doc["dateTime"] = timestamp;
    doc["diagnosticId"] = diagnostic_id;
    doc["createdAt"] = timestamp;
    doc["updatedAt"] = timestamp;
    doc["deviceId"] = device_id;
    doc["data"] = value;

    String jsonStr;
    serializeJson(doc, jsonStr);

    int code = http.POST(jsonStr);

    if (code <= 0) {
        Serial.printf("[API] Error enviando a %s: %s\n", 
                     endpoint, http.errorToString(code).c_str());
        http.end();
        return false;
    } else {
        Serial.printf("[API] Respuesta HTTP %d de %s\n", code, endpoint);
        http.end();
        return true;
    }
}

void sendDataToAPI(const char* diagnostic_id, float value) {
    if (millis() - last_api_send < API_SEND_INTERVAL) return;

    last_api_send = millis();

    String timestamp = getISOTimestamp();

    Serial.printf("[API] Enviando %s = %.2f\n", diagnostic_id, value);
    
    // SIEMPRE enviar a la API local (sin verificar WiFi)
    Serial.println("[API Local] Enviando datos...");
    sendToEndpoint(api_local_endpoint, diagnostic_id, value, timestamp);
    
    // Enviar a API remota SOLO si hay conexión WiFi estable
    if (WiFi.status() == WL_CONNECTED && enable_remote_api && !wifi_reconnecting) {
        Serial.println("[API Remota] Enviando datos...");
        sendToEndpoint(api_remote_endpoint, diagnostic_id, value, timestamp);
    } else if (WiFi.status() != WL_CONNECTED) {
        // Solo mostrar mensaje si realmente no hay conexión
        static uint32_t last_warning = 0;
        if (millis() - last_warning > 60000) { // Mostrar cada minuto máximo
            Serial.println("[API Remota] WiFi no disponible, omitiendo envío remoto");
            last_warning = millis();
        }
    }
}