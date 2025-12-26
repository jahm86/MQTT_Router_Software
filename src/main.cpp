/****************************************************************************************************************************
  main.cpp

  Usando bibliotecas:

  AsyncMqtt_Generic para la comunicacion con el servicio mqtt del servidor

  ESPAsyncWebServer para la configuracion inicial (y posterior, de ser necesaria) del dispositivo, sus buses y sus puertos de interfaz

  ModbusClientRTU para comunicacion dispositivo en el puerto Modbus

  Se usa programacion manejada por eventos para optimizar el uso de recursos y poder responder a varios sistemas simultaneamente

  Este proyecto puede ser compilado con SSL habilitado o desabilitado

  Referir a platformio.ini configuracion de compilacion e instalacion de la plataforma
 *****************************************************************************************************************************/

// Arduino libraries
#include <Arduino.h>
#include <esp_err.h>
#include <WiFi.h>
#include <Ticker.h>
#include <SPIFFS.h>
#include <HardwareSerial.h>

// Self libraries
#include "definitions.h"
#include "regs.h"
#include "CustomServer.h"

// External libraries
#include <PsychicMqttClient.h>
#include <ArduinoJson.h>
#include <DataIntegrator.h>
#include <diutils.h>

//AsyncWebServer server(WEBSERVER_PORT);
PsychicMqttClient mqttClient;
Ticker mqttConnectTicker;
Ticker wifiConnectTicker;
// If stay true, enters web server mode. If false, enters MQTT client mode
bool cfg_mode_http_server = true;
DataBus dataBus;
Ticker MbRequestTicker;
// Create custom server instance
CustomServer* server;

// Parameters (All these parameters MUST be in global scope)
// Local network
String netw_ssid;
String netw_password;
String netw_ipaddr;
String netw_gateway;
String netw_subnet;
// MQTT server
String mqtt_server_uri;
String mqtt_user;
String mqtt_password;
String mqtt_topic;
// Certificate pointer and size
size_t rootCAlen = 0;
char *root_ca = nullptr;

// *** mqtt I/O topics *** //
String mqtt_topic_in;
String mqtt_topic_out;
void setTopics() {
  mqtt_topic_in = mqtt_topic + String(in_subtopic);
  mqtt_topic_out = mqtt_topic + String(out_subtopic);
  log_d("Topic in: %s, Topic out: %s", mqtt_topic_in.c_str(), mqtt_topic_out.c_str());
}

// Assembles mqtt URI, based on socket type, URL and port
String assemble_uri(const char *socket_type, const char *url, int port) {
  // TODO: test if url already comes with socket type and/or port
  if (strlen(url) == 0)
    return "";
  String uri("");
  if (strlen(socket_type) > 0) {
    uri += socket_type;
    uri += "://";
  }
  uri += url;
  if (port > 0)
    uri += ":" + String(port);
  return uri;
}

// Funciton to connect WiFi client
void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  IPAddress ipaddr, gateway, subnet;
  if (!netw_ipaddr.isEmpty() && ipaddr.fromString(netw_ipaddr)) {
    Serial.println("Configuring auto ip");
    gateway.fromString(netw_gateway);
    subnet.fromString(netw_subnet);
    if (!WiFi.config(ipaddr, gateway, subnet))
      Serial.println("STA Failed to configure auto ip");
  }
  WiFi.begin(netw_ssid, netw_password);
}

// Function to connect MQTT client
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

// WiFi stack generic callback
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
#if USING_CORE_ESP32_CORE_V200_PLUS

    case ARDUINO_EVENT_WIFI_READY:
      Serial.println("WiFi ready!");
      break;

    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("WiFi STA starting...");
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("WiFi STA connected!");
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("WiFi connected!");
      Serial.print("[*] Network information for ");
      Serial.println(netw_ssid);
      Serial.println("[+] BSSID : " + WiFi.BSSIDstr());
      Serial.print("[+] Gateway IP : ");
      Serial.println(WiFi.gatewayIP());
      Serial.print("[+] Subnet Mask : ");
      Serial.println(WiFi.subnetMask());
      Serial.println((String)"[+] RSSI : " + WiFi.RSSI() + " dB");
      Serial.print("[+] ESP32 IP : ");
      Serial.println(WiFi.localIP());
      mqttConnectTicker.once(1, connectToMqtt);
      break;

    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      Serial.println("WiFi lost IP");
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      mqttConnectTicker.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      Serial.println("WiFi lost connection");
      wifiConnectTicker.once(3, connectToWifi);
      break;
#else

    case SYSTEM_EVENT_WIFI_READY:
      Serial.println("WiFi ready!");
      break;

    case SYSTEM_EVENT_STA_START:
      Serial.println("WiFi STA starting...");
      break;

    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("WiFi STA connected!");
      break;

    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected!");
      Serial.print("[*] Network information for ");
      Serial.println(netw_ssid);
      Serial.println("[+] BSSID : " + WiFi.BSSIDstr());
      Serial.print("[+] Gateway IP : ");
      Serial.println(WiFi.gatewayIP());
      Serial.print("[+] Subnet Mask : ");
      Serial.println(WiFi.subnetMask());
      Serial.println((String)"[+] RSSI : " + WiFi.RSSI() + " dB");
      Serial.print("[+] ESP32 IP : ");
      Serial.println(WiFi.localIP());
      mqttConnectTicker.once(1, connectToMqtt);
      break;

    case SYSTEM_EVENT_STA_LOST_IP:
      Serial.println("WiFi lost IP");
      break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
      mqttConnectTicker.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      Serial.println("WiFi lost connection");
      wifiConnectTicker.once(3, connectToWifi);
      break;
#endif

    default:
      log_e("WiFi event loop shouldn't get here");
      break;
  }
}

// MQTT connection error callback
void onMqttError(esp_mqtt_error_codes_t error) {
  const __FlashStringHelper *error_type, *connect_return_code;
  // Search error type
  switch (error.error_type ) {
  case esp_mqtt_error_type_t::MQTT_ERROR_TYPE_NONE:
    error_type = F("No MQTT error");
    break;
  case esp_mqtt_error_type_t::MQTT_ERROR_TYPE_TCP_TRANSPORT:
    error_type = F("MQTT TCP transport error");
    break;
  case esp_mqtt_error_type_t::MQTT_ERROR_TYPE_CONNECTION_REFUSED:
    error_type = F("MQTT connection refused");
    break;
  default:
    error_type = F("MQTT unknown error");
  }
  // Search connection return code
  switch (error.connect_return_code)
  {
  case esp_mqtt_connect_return_code_t::MQTT_CONNECTION_ACCEPTED:
    connect_return_code = F("Connection accepted");
    break;
  case esp_mqtt_connect_return_code_t::MQTT_CONNECTION_REFUSE_PROTOCOL:
    connect_return_code = F("Wrong protocol");
    break;
  case esp_mqtt_connect_return_code_t::MQTT_CONNECTION_REFUSE_ID_REJECTED:
    connect_return_code = F("ID rejected");
    break;
  case esp_mqtt_connect_return_code_t::MQTT_CONNECTION_REFUSE_SERVER_UNAVAILABLE:
    connect_return_code = F("Server unavailable");
    break;
  case esp_mqtt_connect_return_code_t::MQTT_CONNECTION_REFUSE_BAD_USERNAME:
    connect_return_code = F("Wrong user");
    break;
  case esp_mqtt_connect_return_code_t::MQTT_CONNECTION_REFUSE_NOT_AUTHORIZED:
    connect_return_code = F("Wrong username or password");
    break;
  default:
    connect_return_code = F("Unknown return error");
  }
  // Print the error
  Serial.print(F("MQTT error reported.\nError type: "));
  Serial.println(error_type);
  Serial.print(F("MQTT connection return code: "));
  Serial.println(connect_return_code);
  Serial.print(F("Error number from underlying socket: "));
  Serial.println(error.esp_transport_sock_errno);
}

// MQTT connection notification callback
void onMqttConnect(bool sessionPresent) {
  Serial.printf("Connected to MQTT broker: %s, Tx Topic: %s Rx Topic: %s\n"
    , mqtt_server_uri.c_str(), mqtt_topic_out.c_str(), mqtt_topic_in.c_str());

  Serial.printf("Session present: %s\n", sessionPresent ? "true" : "false");

  uint16_t packetIdSub = mqttClient.subscribe(mqtt_topic_in.c_str(), 2);
  Serial.printf("Subscribing at QoS 2, packetId: %d\n", packetIdSub);

  // Start bus communications
  if (dataBus.isStarted()) {
    log_d("Resuming data bus");
    dataBus.resume();
  }
  else {
    log_d("Starting data bus integrator...");
    dataBus.startCom();
  }
}

// MQTT disconnection notification callback
void onMqttDisconnect(bool sessionPresent) {

  Serial.print(F("Disconnected from MQTT. Session present: "));
  Serial.println(sessionPresent);

  if (dataBus.isStarted()) {
    log_d("Suspending data bus");
    dataBus.suspend();
  }
}

// MQTT subcribtion response callback
void onMqttSubscribe(int msgId) {
  Serial.printf("Subscribe acknowledged, msgid: %d", msgId);
}

// MQTT unsubscription response callback
void onMqttUnsubscribe(int msgId) {
  Serial.printf("Unsubscribe acknowledged, msgId: %d\n", msgId);
}

// MQTT message notification callback
void onMqttMessage(char *topic, char *payload, int retain, int qos, bool dup) {
  size_t len = strlen(payload);

  log_d("Publish received:  topic: %s  qos: %d  dup: %d  retain: %d  len: %d",
    topic, qos, dup, retain, len);

  DIError::ErrCode accept = dataBus.receive(payload, len);
  log_d("Response: %s", DIError::ErrStr(accept));
}

// MQTT message publication response callback
void onMqttPublish(int msgId) {
  log_d("Publish acknowledged, msgId: %d", msgId);
  dataBus.ack();
}

// Publish DataIntegrator message callback
void onSendJson(const char* output, size_t size) {
  if (!mqttClient.connected())
    return;
  const uint8_t QoS = 1;
  log_d("Publishing %u bytes", size);
  uint16_t packetIdPub = mqttClient.publish(mqtt_topic_out.c_str(), QoS, true, output, size);
  log_d("Packet QoS: %u, packetId: %u", QoS, packetIdPub);
}

// ******************************** Program configuration ******************************** //
void setup() {
  cfg_mode_http_server = true;
  // File handlers
  File nodesFile, paramsFile, certFile;

  // primary configs
  pinMode(CFG_MODE_SW, INPUT_PULLUP);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
  Serial.begin(DEBUG_SERIAL_SPEED);
 
  // Wait for serial por to be available to start
  while (!Serial) {}
  Serial.println(F("ESP32 Modbus to MQTT server"));
  Serial.print(F("Board: "));
  Serial.println(F(ARDUINO_BOARD));
  Serial.print(F("MQTT cliente version: "));
  Serial.println(F(PSYCHIC_MQTT_CLIENT_VERSION_STR));
  Serial.print("You can enter HTTP server configuration mode by pressing button associated to digital port #");
  Serial.println(CFG_MODE_SW);

  // Test ESP log levels
  Serial.println("Testing ESP log levels...");
  log_v("Verbose");
  log_d("Debug");
  log_i("Info");
  log_w("Warning");
  log_e("Error");
  Serial.println("ESP log levels test ended!");

  // https://randomnerdtutorials.com/esp32-vs-code-platformio-spiffs/
  // pio run -t uploadfs
  if(!SPIFFS.begin(true)){
    log_e("An Error has occurred while mounting SPIFFS");
    return;
  }


  // Make some checks to verify if can enter to MQTT mode
  do {
    JsonDocument doc;
    // Check if parameters file exist and open
    log_d("Opening json parameters file");
    if (!SPIFFS.exists(Params_path)) {
      Serial.println("Parameters file doesn't exist");
      break;
    }
    paramsFile = SPIFFS.open(Params_path);
    if (!paramsFile) {
      Serial.println("Can't open parameters file");
      break;
    }
    deserializeJson(doc, paramsFile);
    paramsFile.close();

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
    String output;
    serializeJsonPretty(doc, output);
    log_d("***************************** Show parameters *****************************");
    log_d("%s", output.c_str());
    log_d("***************************** End  parameters *****************************");
#endif //ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG

    // Check if data nodes file exist and open
    log_d("Opening JSON Data Nodes file");
    if (!SPIFFS.exists(Nodes_path)) {
      Serial.println("Data Nodes file doesn't exist");
      break;
    }
    nodesFile = SPIFFS.open(Nodes_path);
    if (!nodesFile) {
      Serial.println("Can't open Data Nodes file");
      break;
    }

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
    JsonDocument doc2;
    deserializeJson(doc2, nodesFile);
    serializeJsonPretty(doc2, output);
    // Close and open back to avoid end of file when its read by DataBuilder instance
    nodesFile.close();
    nodesFile = SPIFFS.open(Nodes_path);
    log_d("***************************** Show nodes *****************************");
    log_d("%s", output.c_str());
    log_d("***************************** End  nodes *****************************");
#endif //ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG

    // Get Configuration parameters
    // ********** Getting network parameters ********** //
    log_d("Getting network parameters");
    // Temporal variant value
    JsonVariant dummy_var = doc[F(WF_OBJ_NAME)];
    netw_ssid = dummy_var[F(WF_SSID_KEY)] | EMPTY_STR;
    netw_password = dummy_var[F(WF_PASS_KEY)] | EMPTY_STR;
    netw_ipaddr = dummy_var[F(WF_IP_KEY)] | def_ipaddr;
    netw_gateway = dummy_var[F(WF_GW_KEY)] | def_gateway;
    netw_subnet = dummy_var[F(WF_SUBNET_KEY)] | def_subnet;
    log_d("SSID: \"%s\"  Pwd: \"%s\"  IP: \"%s\"  Gw: \"%s\"  Sn: \"%s\"",
    netw_ssid.c_str(), netw_password.c_str(), netw_ipaddr.c_str(), netw_gateway.c_str(), netw_subnet.c_str());
    if(netw_ssid.isEmpty() || netw_password.isEmpty()) { // Yes, you must have network protected!
      Serial.println("Network parameters not valid");
      break;
    }
    // ********** Getting MQTT parameters ********** //
    log_d("Getting MQTT parameters");
    dummy_var = doc[F(MQ_OBJ_NAME)];
    mqtt_server_uri = assemble_uri(
      dummy_var[F(MQ_SOCK_TYPE)] | def_sock,
      dummy_var[F(MQ_URL_KEY)] | EMPTY_STR,
      dummy_var[F(MQ_PORT_KEY)].as<int>()
    );
    mqtt_user = dummy_var[F(MQ_USER_KEY)] | EMPTY_STR;
    mqtt_password = dummy_var[F(MQ_PASS_KEY)] | EMPTY_STR;
    mqtt_topic =  dummy_var[F(MQ_DEVID_KEY)] | EMPTY_STR;
    String origin_ca_name = dummy_var[F(CERT_OBJ_NAME)] | EMPTY_STR;
    log_d("MQTT server URI: \"%s\"  User: \"%s\"  Pwd: \"%s\"  Topic: \"%s\"",
    mqtt_server_uri.c_str(), mqtt_user.c_str(), mqtt_password.c_str(), mqtt_topic.c_str());
    setTopics();
    // Detects TLS connection and open certificate
    if (mqtt_server_uri.startsWith(sock_mqtts) || mqtt_server_uri.startsWith(sock_wss)) {
      log_d("Obtain certificate");
      if(SPIFFS.exists(Cert_path)) {
        certFile = SPIFFS.open(Cert_path, "rb");
        rootCAlen = certFile.size();
        root_ca = (char*) malloc((rootCAlen + 1) * sizeof(char));
        certFile.readBytes(root_ca, rootCAlen);
        root_ca[rootCAlen] = '\0';
        certFile.close();
        log_d("Certificate of %d bytes read", rootCAlen);
        log_d("Certificate content:\n%s", root_ca);
        // If the file is not .der, setCACert gets in trouble with rootCAlen != 0
        if (!(origin_ca_name.endsWith(ext_der)) || origin_ca_name.endsWith(ext_cer)) {
          log_d("File is not \"DER\", setting root CA length to 0");
          rootCAlen = 0;
        }
      } else {
        Serial.println(F("Warning: Certificate not found for secure connection!"));
      }
    }
    if(mqtt_server_uri.isEmpty() || mqtt_user.isEmpty() ||mqtt_topic.isEmpty()) {
      Serial.println("MQTT parameters not valid");
      break;
    }
    // ********** Getting Serial RS485 (Modbus) parameters ********** //
    log_d("Getting Serial parameters");
    dummy_var = doc[F(SP_OBJ_NAME)];
    uint32_t serial_speed = dummy_var[F(SP_SPEED_KEY)].as<uint32_t>();
    int serial_data_bits = dummy_var[F(SP_BITS_KEY)].as<int>();
    int serial_stop_bits = dummy_var[F(SP_STOPB_KEY)].as<int>();
    std::string serial_parity = dummy_var[F(SP_PARITY_KEY)] | def_parity;
    if(serial_speed == 0 || serial_data_bits == 0 || serial_stop_bits == 0) {
      Serial.println("Serial parameters not valid");
      break;
    }
    const std::string modbus_key = "ModbusRTU@0";
    ConfigRegistry::registerProtocol(modbus_key);
    ConfigRegistry::setConfig(modbus_key, SP_SPEED_KEY, serial_speed);
    ConfigRegistry::setConfig(modbus_key, SP_INST_KEY, RS485_UART_NUM);
    ConfigRegistry::setConfig(modbus_key, SP_ROPIN_KEY, RS485_ROPIN);
    ConfigRegistry::setConfig(modbus_key, SP_DIPIN_KEY, RS485_DIPIN);
    ConfigRegistry::setConfig(modbus_key, SP_DRPIN_KEY,RS485_DEREPIN );
    ConfigRegistry::setConfig(modbus_key, SP_TOUT_KEY, (uint32_t) RS485_TIMEOUT_INTERVAL);
    ConfigRegistry::setConfig(modbus_key, SP_BITS_KEY, serial_data_bits);
    ConfigRegistry::setConfig(modbus_key, SP_STOPB_KEY, serial_stop_bits);
    ConfigRegistry::setConfig(modbus_key, SP_PARITY_KEY, serial_parity);
    // ********** Getting CANBUS parameters ********** //
    log_d("Getting CANBUS parameters");
    dummy_var = doc[F(CB_OBJ_NAME)];
    uint32_t canbus_speed = dummy_var[F(CB_BAUD_KEY)].as<uint32_t>();
    if(canbus_speed == 0) {
      Serial.println("CANBUS parameters not valid");
      break;
    }
    const std::string canbus_key = "CANBUS@0";
    ConfigRegistry::registerProtocol(canbus_key);
    ConfigRegistry::setConfig(canbus_key, CB_BAUD_KEY, canbus_speed);
    ConfigRegistry::setConfig(canbus_key, CB_RX_KEY, TWAI_RXDPIN);
    ConfigRegistry::setConfig(canbus_key, CB_TX_KEY, TWAI_TXDPIN);
    ConfigRegistry::setConfig(canbus_key, CB_RS_KEY, TWAI_RSPIN);
    ConfigRegistry::setConfig(canbus_key, CB_MODE_KEY, dummy_var[F(CB_MODE_KEY)].as<int>());
    ConfigRegistry::setConfig(canbus_key, CB_ACCEPT_KEY, dummy_var[F(CB_ACCEPT_KEY)].as<std::string>());
    ConfigRegistry::setConfig(canbus_key, CB_MASK_KEY, dummy_var[F(CB_MASK_KEY)].as<std::string>());
    ConfigRegistry::setConfig(canbus_key, CB_SINGLE_KEY, dummy_var[F(CB_SINGLE_KEY)].as<bool>());
    // ********** Checks if user requested HTTP server mode ********** //
    if (digitalRead(CFG_MODE_SW) == LOW) {
      Serial.println("Config button pressed. HTTP server mode requested by user");
      break;
    }
    // If code can get here, enters MQTT mode
    cfg_mode_http_server = false;
  } while (false);


  if (cfg_mode_http_server) {
    nodesFile.close();
    // Enter HTTP server mode
    Serial.println(F("Entering HTTP server mode..."));
    Serial.print(F("Setting Access Point, SSID: "));
    Serial.println(F(SAP_SSID));
    WiFi.softAP(SAP_SSID, NULL);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    server = new CustomServer(DEF_HTTP_PORT, Params_path, Nodes_path, Cert_path);
    server->start();
  } else {
    // Enter MQTT mode
    Serial.println("Entering MQTT client mode");
    // Open json configuration file and build data instances
    log_d("Heap: %d", xPortGetFreeHeapSize());
    log_d("Building data instances");
    DataBuilder builder = DataBuilder(nodesFile, dataBus);
    nodesFile.close();

    // On error, publish failure and exit
    DIError::ErrCode error = builder.error();
    if (error != DIError::ErrCode::SUCCESS) {
      cfg_mode_http_server = true;
      log_e("Build failed: %s", DIError::ErrStr(error));
      if (error == DIError::ErrCode::FILE_DES_ERROR) {
        DeserializationError desError = builder.desError();
        log_e("Deserialization error: %s", desError.c_str());
      }
      return;
    }

    // Debug registry details
    ConfigRegistry::debugPrint();

    // Variable to store the MAC address
    uint8_t baseMac[6];
    // Get MAC address of the WiFi station interface
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    Serial.print("Station MAC: ");
    for (int i = 0; i < 5; i++) {
      Serial.printf("%02X:", baseMac[i]);
    }
    Serial.printf("%02X\n", baseMac[5]);

    // Set Wifi mode to station and event callback
    log_d("Setting Wifi mode to station and event callback");
    WiFi.mode(WIFI_STA);
    WiFi.onEvent(WiFiEvent);

    // Set MQTT Event callback
    mqttClient.onError(onMqttError);
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onSubscribe(onMqttSubscribe);
    mqttClient.onUnsubscribe(onMqttUnsubscribe);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.onPublish(onMqttPublish);
    dataBus.onSend(onSendJson);

    // Stablish MQTT connection parameters
    log_d("Setting MQTT parameters");
    mqttClient.setServer(mqtt_server_uri.c_str());
    mqttClient.setBufferSize(1024);
    mqttClient.setCredentials(mqtt_user.c_str(), mqtt_password.c_str());
    if (root_ca != nullptr)
      mqttClient.setCACert(root_ca, rootCAlen);

    // Try initial connection to WiFi
    log_d("Here we go, WiFi!");
    connectToWifi();
  }
  log_d("All ready!");
}

// Main program loop (almost unused in asynchronous programs)
void loop()
{
  if(cfg_mode_http_server) {} else {}
  vTaskDelay(pdMS_TO_TICKS(20000));
  /*if (!dataBus.isStarted()){
    log_d("not started");
    return;}
  log_d("started");
  if (dataBus.isRunning()) {
    log_d("Sunpend");
    dataBus.suspend();
  } else {
    log_d("Resume");
    dataBus.resume();
  }*/
}
