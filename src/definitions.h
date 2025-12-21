/****************************************************************************************************************************
  definitions.h

  Project basic definitions common to all source files
 ***************************************************************************************************************************************/

#ifndef DEFINITIONS_H
#define DEFINITIONS_H

// Default http server port
#define DEF_HTTP_PORT   (80U)
#define DEF_HTTPS_PORT  (443U)

// Empty string parameter
#define EMPTY_STR ""

// I/O parameters
#define BUILTIN_LED         GPIO_NUM_2  // Built-in LED
#define CFG_MODE_SW         GPIO_NUM_25 // When this pin is low, enters configuration web server mode
#define OPTO_1              GPIO_NUM_34 // Optocouple 1 input
#define OPTO_2              GPIO_NUM_35 // Optocouple 2 input
#define RELAY_K1            GPIO_NUM_27 // Relay 1 output
#define RELAY_K2            GPIO_NUM_26 // Relay 2 output
#define SDA_PIN             GPIO_NUM_18 // Optional I2C SDA pin
#define SCL_PIN             GPIO_NUM_19 // Optional I2C SCL pin
#define S_VP_PIN            GPIO_NUM_36 // Sensor VP / ADC1_CH0 pin
#define S_VN_PIN            GPIO_NUM_39 // Sensor VN / ADC1_CH3 pin
#define SS_PIN              GPIO_NUM_5  // Optional, if you want SPI (bypassing I2C and changing TWAI RS bridges), this is Chip Select.
#define DEBUG_SERIAL_SPEED  115200
#define WEBSERVER_PORT      DEF_HTTP_PORT
#define SAP_SSID            "ESP32-WIFI-MANAGER"

// Parity modes
#define parity_none   "none"
#define parity_even   "even"
#define parity_odd    "odd"
#define par_int_none  0
#define par_int_even  1
#define par_int_odd   2

// MQTT socket types
#define sock_mqtt   "mqtt"
#define sock_mqtts  "mqtts"
#define sock_ws     "ws"
#define sock_wss    "wss"

// Certificate extensions
#define ext_der ".der"
#define ext_cer ".cer"
#define ext_crt ".crt"
#define ext_pem ".pem"

// Default parameters
#define def_mqtt_port 8885
#define def_speed     9600
#define def_slave_id  1
#define def_data_bits 8
#define def_stop_bits 1
#define def_ipaddr    EMPTY_STR
#define def_gateway   "8.8.4.4"
#define def_subnet    "255.255.255.0"
#define def_parity    parity_none
#define def_sock      sock_mqtt

// I/O sub-topics
#define in_subtopic   "/in"
#define out_subtopic  "/out"

// ArduinoJson definitions
#define ARDUINOJSON_STRING_LENGTH_SIZE  1
#define ARDUINOJSON_USE_DOUBLE          0
#define ARDUINOJSON_USE_LONG_LONG       0

// RS485 serial port parameters
#define RS485_UART_NUM          2           // RS485 UART device number
#define RS485_ROPIN             GPIO_NUM_16 // RS485 RX PIN (Receiver Output)
#define RS485_DIPIN             GPIO_NUM_17 // RS485 TX PIN (Driver Input)
#define RS485_DEREPIN           GPIO_NUM_4  // RS485 DEREPIN (Driver Output Enable/Receiver Output Enable)
#define RS485_BAUDRATE          9600
#define RS485_READ_INTERVAL     5000
#define RS485_TIMEOUT_INTERVAL  2000

// TWAI (ESP CAN BUS) serial port parameters
#define TWAI_RXDPIN GPIO_NUM_22 // TWAI RX PIN (Receive Data Output)
#define TWAI_TXDPIN GPIO_NUM_21 // TWAI TX PIN (Transmit Data Input)
#define TWAI_RSPIN  GPIO_NUM_23 // TWAI RS PIN (Slope-Control Input).

// *** parameter keys ***
// WiFi
#define WF_OBJ_NAME   "wifi"
#define WF_SSID_KEY   "ssid"
#define WF_PASS_KEY   "wpass"
#define WF_IP_KEY     "ip"
#define WF_GW_KEY     "gateway"
#define WF_SUBNET_KEY "subnet"
// MQTT
#define MQ_OBJ_NAME   "mqtt"
#define MQ_SOCK_TYPE  "sock"
#define MQ_URL_KEY    "url"
#define MQ_PORT_KEY   "port"
#define MQ_USER_KEY   "user"
#define MQ_PASS_KEY   "pass"
#define MQ_DEVID_KEY  "devid"
#define MQ_FI_PRINT   "fprint"
// Serial (RS485) Port
#define SP_OBJ_NAME   "serial"
#define SP_INST_KEY   "inst"
#define SP_SPEED_KEY  "speed"
#define SP_ROPIN_KEY  "rxpin"
#define SP_DIPIN_KEY  "txpin"
#define SP_DRPIN_KEY  "rtspin"
#define SP_TOUT_KEY   "timeout"
#define SP_BITS_KEY   "bits"
#define SP_STOPB_KEY  "stop"
#define SP_PARITY_KEY "parity"
// CANBUS Port
#define CB_OBJ_NAME   "canbus"
#define CB_BAUD_KEY   "baud"
#define CB_RX_KEY     "rx_pin"
#define CB_TX_KEY     "tx_pin"
#define CB_RS_KEY     "rs_pin"
#define CB_MODE_KEY   "mode"
#define CB_ACCEPT_KEY "f_accept"
#define CB_MASK_KEY   "f_mask"
#define CB_SINGLE_KEY "f_single"
// Node
#define ND_OBJ_NAME   "node"
#define ND_SEPARATOR  ':'
// Certificate
#define CERT_OBJ_NAME "cacert"

// *** file paths ***
#define Params_path "/parameters.json"
#define Nodes_path  "/nodes.json"
#define Cert_path   "/CaCert"
#define Serve_file  "/index.html"
#define Css_file    "/style.css"
#define Js_file     "/script.js"
#define Icon_file   "/favicon.png"

#endif //DEFINITIONS_H
