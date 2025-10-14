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
#include <AsyncMqtt_Generic.h>
#include <ModbusClientRTU.h>
#include <ArduinoJson.h>
#include <DataIntegrator.h>
#include <diutils.h>
#include <TWAI_Object.h>

//AsyncWebServer server(WEBSERVER_PORT);
AsyncMqttClient mqttClient;
Ticker mqttConnectTicker;
Ticker wifiConnectTicker;
bool cfg_mode_http_server = true;
DataBus dataBus;
// Create a ModbusRTU client instance
// The RS485 module has no halfduplex, so the parameter with the DE/RE pin is required!
ModbusClientRTU MBCRTU(REDEPIN);
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
String mqtt_server;
int mqtt_port;
String mqtt_user;
String mqtt_password;
String mqtt_topic;

// *** mqtt I/O topics *** //
String mqtt_topic_in;
String mqtt_topic_out;
void setTopics() {
  mqtt_topic_in = mqtt_topic + String(in_subtopic);
  mqtt_topic_out = mqtt_topic + String(out_subtopic);
  log_d("Topic in: %s, Topic out: %s", mqtt_topic_in.c_str(), mqtt_topic_out.c_str());
}

// Expression function to encode data_b, stop_b and parity_b into an unique value for switch statement
constexpr uint encode_par_mode(uint data_b, uint stop_b, uint parity_b) {
  return ((data_b << 16) & 0x00ff0000) | ((stop_b << 8) & 0x0000ff00) | (parity_b & 0x000000ff);
}

// Returns a SerialConfig value in function of data_b, stop_b and parity_b
uint32_t set_data_stop_parity(int data_b, int stop_b, String parity_b) {
  // TODO: put all modes
  uint parity = parity_b == parity_odd ? par_int_odd : parity_b == parity_even ? par_int_even : par_int_none;
  uint data_bits = data_b > 8 ? 8 : data_b < 5 ? 5 : data_b;
  uint stop_bits = stop_b > 2 ? 2 : stop_b < 1 ? 1 : stop_b;
  uint encoded = encode_par_mode(data_bits, stop_bits, parity);
  log_d("Encoded value %d", encoded);
  switch (encoded) {
  // None
  case encode_par_mode(5, 1, par_int_none):
    return SERIAL_5N1;
  case encode_par_mode(6, 1, par_int_none):
    return SERIAL_6N1;
  case encode_par_mode(7, 1, par_int_none):
    return SERIAL_7N1;
  case encode_par_mode(8, 1, par_int_none):
    return SERIAL_8N1;
  case encode_par_mode(5, 2, par_int_none):
    return SERIAL_5N2;
  case encode_par_mode(6, 2, par_int_none):
    return SERIAL_6N2;
  case encode_par_mode(7, 2, par_int_none):
    return SERIAL_7N2;
  case encode_par_mode(8, 2, par_int_none):
    return SERIAL_8N2;
  // Even
  case encode_par_mode(5, 1, par_int_even):
    return SERIAL_5E1;
  case encode_par_mode(6, 1, par_int_even):
    return SERIAL_6E1;
  case encode_par_mode(7, 1, par_int_even):
    return SERIAL_7E1;
  case encode_par_mode(8, 1, par_int_even):
    return SERIAL_8E1;
  case encode_par_mode(5, 2, par_int_even):
    return SERIAL_5E2;
  case encode_par_mode(6, 2, par_int_even):
    return SERIAL_6E2;
  case encode_par_mode(7, 2, par_int_even):
    return SERIAL_7E2;
  case encode_par_mode(8, 2, par_int_even):
    return SERIAL_8E2;
  // Odd
  case encode_par_mode(5, 1, par_int_odd):
    return SERIAL_5O1;
  case encode_par_mode(6, 1, par_int_odd):
    return SERIAL_6O1;
  case encode_par_mode(7, 1, par_int_odd):
    return SERIAL_7O1;
  case encode_par_mode(8, 1, par_int_odd):
    return SERIAL_8O1;
  case encode_par_mode(5, 2, par_int_odd):
    return SERIAL_5O2;
  case encode_par_mode(6, 2, par_int_odd):
    return SERIAL_6O2;
  case encode_par_mode(7, 2, par_int_odd):
    return SERIAL_7O2;
  case encode_par_mode(8, 2, par_int_odd):
    return SERIAL_8O2;
  }
  return SERIAL_8N1;
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

// Function to disconnect MQTT client
void disconnectMqtt() {
  if (mqttClient.connected()) {
    Serial.println("Disconnecting MQTT...");
    mqttClient.disconnect(true);
  }
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
      disconnectMqtt();
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
      disconnectMqtt();
      wifiConnectTicker.once(3, connectToWifi);
      break;
#endif

    default:
      log_e("WiFi event loop shouldn't get here");
      break;
  }
}

// MQTT connection notification callback
void onMqttConnect(bool sessionPresent) {
  Serial.printf("Connected to MQTT broker: %s, port: %d, Tx Topic: %s Rx Topic: %s\n"
    , mqtt_server.c_str(), mqtt_port, mqtt_topic_out.c_str(), mqtt_topic_in.c_str());

  Serial.printf("Session present: %b\n", sessionPresent);

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
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  (void) reason;

  Serial.println(F("Disconnected from MQTT."));

  if (dataBus.isStarted()) {
    log_d("Suspending data bus");
    dataBus.suspend();
  }

  if (WiFi.isConnected())
  {
    mqttConnectTicker.once(2, connectToMqtt);
  }
}

// MQTT subcribtion response callback
void onMqttSubscribe(const uint16_t& packetId, const uint8_t& qos) {
  Serial.printf("Subscribe acknowledged.  packetId: %d  qos: %d\n", packetId, qos);
}

// MQTT unsubscription response callback
void onMqttUnsubscribe(const uint16_t& packetId) {
  Serial.printf("Unsubscribe acknowledged.  packetId: %d\n", packetId);
}

// MQTT message notification callback
void onMqttMessage(char* topic, char* payload, const AsyncMqttClientMessageProperties& properties,
                   const size_t& len, const size_t& index, const size_t& total) {

  log_d("Publish received:  topic: %s  qos: %d  dup: %d  retain: %d  len: %d  index: %d  total: %d"
    , topic, properties.qos, properties.dup, properties.retain, len, index, total);

  DIError::ErrCode accept = dataBus.receive(payload, len);
  log_d("Response: %s", DIError::ErrStr(accept));
}

// MQTT message publication response callback
void onMqttPublish(const uint16_t& packetId) {
  log_d("Publish acknowledged:  packetId: %d", packetId);
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
  File nodesFile, paramsFile;

#if ASYNC_TCP_SSL_ENABLED
  // Server SHA1 fingerprint. Can be obtained with command:
  // sudo openssl x509 -noout -fingerprint -sha1 -inform pem -in /path/to/cert.[pem/crt]
  uint8_t MQTT_SERVER_FINGERPRINT[SHA1_SIZE];
  String mqtt_fprint;
#endif

  // Parameters (Only serial parameters in local scope)
  // Serial devides port
  unsigned long serial_speed;
  int serial_data_bits;
  int serial_stop_bits;
  String serial_parity;

  // primary configs
  pinMode(CFG_MODE_SW, INPUT_PULLUP);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
  Serial.begin(DEBUG_SERIAL_SPEED);
 
  // Wait for serial por to be available to start
  while (!Serial) {}
  Serial.println(F("ESP32 Modbus to MQTT server"));
  Serial.println(F(ARDUINO_BOARD));
  Serial.println(F(ASYNC_MQTT_GENERIC_VERSION));
  Serial.print("You can enter HTTP server configuration mode by pressing button associated to digital port #");
  Serial.println(CFG_MODE_SW);

  // Test ESP log levels
  Serial.println("Testing ESP log levels");
  log_v("Verbose");
  log_d("Debug");
  log_i("Info");
  log_w("Warning");
  log_e("Error");

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
    netw_ssid = doc[F(WF_OBJ_NAME)][F(WF_SSID_KEY)] | EMPTY_STR;
    netw_password = doc[F(WF_OBJ_NAME)][F(WF_PASS_KEY)] | EMPTY_STR;
    netw_ipaddr = doc[F(WF_OBJ_NAME)][F(WF_IP_KEY)] | def_ipaddr;
    netw_gateway = doc[F(WF_OBJ_NAME)][F(WF_GW_KEY)] | def_gateway;
    netw_subnet = doc[F(WF_OBJ_NAME)][F(WF_SUBNET_KEY)] | def_subnet;
    log_d("SSID: \"%s\"  Pwd: \"%s\"  IP: \"%s\"  Gw: \"%s\"  Sn: \"%s\"",
    netw_ssid.c_str(), netw_password.c_str(), netw_ipaddr.c_str(), netw_gateway.c_str(), netw_subnet.c_str());
    if(netw_ssid.isEmpty() || netw_password.isEmpty()) { // Yes, you must have network protected!
      Serial.println("Network parameters not valid");
      break;
    }
    // ********** Getting MQTT parameters ********** //
    log_d("Getting MQTT parameters");
    mqtt_server = doc[F(MQ_OBJ_NAME)][F(MQ_URL_KEY)] | EMPTY_STR; // test.mosquitto.org
    mqtt_port = doc[F(MQ_OBJ_NAME)][F(MQ_PORT_KEY)].as<int>();
    mqtt_user = doc[F(MQ_OBJ_NAME)][F(MQ_USER_KEY)] | EMPTY_STR;
    mqtt_password = doc[F(MQ_OBJ_NAME)][F(MQ_PASS_KEY)] | EMPTY_STR;
    mqtt_topic =  doc[F(MQ_OBJ_NAME)][F(MQ_DEVID_KEY)] | EMPTY_STR;
#if ASYNC_TCP_SSL_ENABLED
    mqtt_fprint = doc[F(MQ_OBJ_NAME)][F(MQ_FI_PRINT)] | EMPTY_STR;
    if (!stringHex2ByteArray(mqtt_fprint, (uint8_t*) MQTT_SERVER_FINGERPRINT, SHA1_SIZE)) {
      Serial.println("SHA1 MQTT server fingerprint format not valid");
      break;
    }
#endif
    log_d("MQTT server: \"%s\"  Port: %d  User: \"%s\"  Pwd: \"%s\"  Topic: \"%s\"",
    mqtt_server.c_str(), mqtt_port, mqtt_user.c_str(), mqtt_password.c_str(), mqtt_topic.c_str());
    setTopics();
    if(mqtt_server.isEmpty() || mqtt_user.isEmpty() ||mqtt_topic.isEmpty()) {
      Serial.println("MQTT parameters not valid");
      break;
    }
    // ********** Getting Serial parameters ********** //
    log_d("Getting Serial parameters");
    serial_speed = doc[F(SP_OBJ_NAME)][F(SP_SPEED_KEY)].as<int>();
    serial_data_bits = doc[F(SP_OBJ_NAME)][F(SP_BITS_KEY)].as<int>();
    serial_stop_bits = doc[F(SP_OBJ_NAME)][F(SP_STOPB_KEY)].as<int>();
    serial_parity = doc[F(SP_OBJ_NAME)][F(SP_PARITY_KEY)] | def_parity;
    log_d("Baudrate: %u  Data: %d  Stop: %d  Parity: \"%s\"", serial_speed, serial_data_bits, serial_stop_bits, serial_parity.c_str());
    if(serial_speed == 0 || serial_data_bits == 0 || serial_stop_bits == 0) {
      Serial.println("Serial parameters not valid");
      break;
    }
    // Checks if user requested HTTP server mode
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
    server = new CustomServer(DEF_HTTP_PORT, Params_path, Nodes_path);
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

    // Variable to store the MAC address
    uint8_t baseMac[6];
    // Get MAC address of the WiFi station interface
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    Serial.print("Station MAC: ");
    for (int i = 0; i < 5; i++) {
      Serial.printf("%02X:", baseMac[i]);
    }
    Serial.printf("%02X\n", baseMac[5]);

    // Set up SerialRef connected to Modbus RTU
    HardwareSerial& SerialRef = Serial2;
    uint32_t mode_config = set_data_stop_parity(serial_data_bits, serial_stop_bits, serial_parity);
    log_d("Returned value %d", mode_config);
    log_d("Setting up Serial connection for Modbus RTU");
    RTUutils::prepareHardwareSerial(SerialRef);
    SerialRef.begin(serial_speed, mode_config, RXPIN, TXPIN);

    // Set up ModbusRTU client
    // Set message timeout
    MBCRTU.setTimeout(TIMEOUT_INTERVAL);
    // Start ModbusRTU background task
    StreamLink::Instance().SetNew(&MBCRTU);
    MBCRTU.begin(SerialRef);

    // Set Wifi mode to station and event callback
    log_d("Setting Wifi mode to station and event callback");
    WiFi.mode(WIFI_STA);
    WiFi.onEvent(WiFiEvent);

    // Set MQTT Event callback
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onSubscribe(onMqttSubscribe);
    mqttClient.onUnsubscribe(onMqttUnsubscribe);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.onPublish(onMqttPublish);
    dataBus.onSend(onSendJson);

    // Stablish MQTT connection parameters
    log_d("Setting MQTT parameters");
    mqttClient.setServer(mqtt_server.c_str(), mqtt_port);
    mqttClient.setCredentials(mqtt_user.c_str(), mqtt_password.c_str());
#if ASYNC_TCP_SSL_ENABLED
    mqttClient.setSecure(true);
    mqttClient.addServerFingerprint((const uint8_t *)MQTT_SERVER_FINGERPRINT);
#endif

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
