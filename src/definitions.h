/****************************************************************************************************************************
  definitions.h

  Project basic definitions common to all source files
 ***************************************************************************************************************************************/

#ifndef DEFINITIONS_H
#define DEFINITIONS_H

// Default http server port
#define DEF_HTTP_PORT (80U)

// Empty string parameter
#define EMPTY_STR ""

// I/O parameters
#define BUILTIN_LED GPIO_NUM_2
#define CFG_MODE_SW GPIO_NUM_3
#define DEBUG_SERIAL_SPEED 115200
#define WEBSERVER_PORT 80
#define SAP_SSID "ESP32-WIFI-MANAGER"

// Parity modes
#define parity_none   "none"
#define parity_even   "even"
#define parity_odd    "odd"
#define par_int_none  0
#define par_int_even  1
#define par_int_odd   2

// Default parameters
#define def_mqtt_port 8885
#define def_speed 9600
#define def_slave_id 1
#define def_data_bits 8
#define def_stop_bits 1
#define def_ipaddr EMPTY_STR
#define def_gateway "8.8.4.4"
#define def_subnet "255.255.255.0"
#define def_parity parity_none

// I/O sub-topics
#define in_subtopic "/in"
#define out_subtopic "/out"

// ArduinoJson definitions
#define ARDUINOJSON_STRING_LENGTH_SIZE 1
#define ARDUINOJSON_USE_DOUBLE 0
#define ARDUINOJSON_USE_LONG_LONG 0

// Communications serial port parameters
#define RXPIN GPIO_NUM_16
#define TXPIN GPIO_NUM_17
#define REDEPIN GPIO_NUM_4 //TODO: redefine pin to allow TWAI communications
#define BAUDRATE 9600
#define READ_INTERVAL 5000
#define TIMEOUT_INTERVAL 2000

// TWAI (CAN BUS) serial port parameters
#define TW_RXPIN GPIO_NUM_4
#define TW_TXPIN GPIO_NUM_5

// *** parameter keys ***
// WiFi
#define WF_OBJ_NAME "wifi"
#define WF_SSID_KEY "ssid"
#define WF_PASS_KEY "wpass"
#define WF_IP_KEY "ip"
#define WF_GW_KEY "gateway"
#define WF_SUBNET_KEY "subnet"
// MQTT
#define MQ_OBJ_NAME "mqtt"
#define MQ_URL_KEY "url"
#define MQ_PORT_KEY "port"
#define MQ_USER_KEY "user"
#define MQ_PASS_KEY "pass"
#define MQ_DEVID_KEY "devid"
#define MQ_FI_PRINT "fprint"
// Serial Port
#define SP_OBJ_NAME "serial"
#define SP_SPEED_KEY "speed"
#define SP_BITS_KEY "bits"
#define SP_STOPB_KEY "stop"
#define SP_PARITY_KEY "parity"
// Node
#define ND_OBJ_NAME "node"
#define ND_SEPARATOR ':'

// *** file paths ***
#define Params_path "/parameters.json"
#define Nodes_path  "/nodes.json"
#define Serve_file  "/index.html"
#define Css_file    "/style.css"
#define Js_file     "/script.js"
#define Icon_file   "/favicon.png"

#endif //DEFINITIONS_H
