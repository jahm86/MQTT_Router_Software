
#include "DsModbus.h"
#include "diutils.h"
#include <unordered_map>
#include <ModbusClientRTU.h>


// Parity modes
#define parity_none   "none"
#define parity_even   "even"
#define parity_odd    "odd"
#define par_int_none  0
#define par_int_even  1
#define par_int_odd   2


//************************************** Auxiliary functions **************************************//

// Expression function to encode data_b, stop_b and parity_b into an unique value for switch statement
constexpr uint encodeSerialMode(uint data_b, uint stop_b, uint parity_b) {
  return ((data_b << 16) & 0x00ff0000) | ((stop_b << 8) & 0x0000ff00) | (parity_b & 0x000000ff);
}

// Returns a SerialConfig value in function of data_b, stop_b and parity_b
uint32_t setDataStopParity(int data_b, int stop_b, std::string parity_b) {
  // TODO: put all modes
  uint parity = parity_b == parity_odd ? par_int_odd : parity_b == parity_even ? par_int_even : par_int_none;
  uint data_bits = data_b > 8 ? 8 : data_b < 5 ? 5 : data_b;
  uint stop_bits = stop_b > 2 ? 2 : stop_b < 1 ? 1 : stop_b;
  uint encoded = encodeSerialMode(data_bits, stop_bits, parity);
  log_d("Encoded value %d", encoded);
  switch (encoded) {
  // None
  case encodeSerialMode(5, 1, par_int_none):
    return SERIAL_5N1;
  case encodeSerialMode(6, 1, par_int_none):
    return SERIAL_6N1;
  case encodeSerialMode(7, 1, par_int_none):
    return SERIAL_7N1;
  case encodeSerialMode(8, 1, par_int_none):
    return SERIAL_8N1;
  case encodeSerialMode(5, 2, par_int_none):
    return SERIAL_5N2;
  case encodeSerialMode(6, 2, par_int_none):
    return SERIAL_6N2;
  case encodeSerialMode(7, 2, par_int_none):
    return SERIAL_7N2;
  case encodeSerialMode(8, 2, par_int_none):
    return SERIAL_8N2;
  // Even
  case encodeSerialMode(5, 1, par_int_even):
    return SERIAL_5E1;
  case encodeSerialMode(6, 1, par_int_even):
    return SERIAL_6E1;
  case encodeSerialMode(7, 1, par_int_even):
    return SERIAL_7E1;
  case encodeSerialMode(8, 1, par_int_even):
    return SERIAL_8E1;
  case encodeSerialMode(5, 2, par_int_even):
    return SERIAL_5E2;
  case encodeSerialMode(6, 2, par_int_even):
    return SERIAL_6E2;
  case encodeSerialMode(7, 2, par_int_even):
    return SERIAL_7E2;
  case encodeSerialMode(8, 2, par_int_even):
    return SERIAL_8E2;
  // Odd
  case encodeSerialMode(5, 1, par_int_odd):
    return SERIAL_5O1;
  case encodeSerialMode(6, 1, par_int_odd):
    return SERIAL_6O1;
  case encodeSerialMode(7, 1, par_int_odd):
    return SERIAL_7O1;
  case encodeSerialMode(8, 1, par_int_odd):
    return SERIAL_8O1;
  case encodeSerialMode(5, 2, par_int_odd):
    return SERIAL_5O2;
  case encodeSerialMode(6, 2, par_int_odd):
    return SERIAL_6O2;
  case encodeSerialMode(7, 2, par_int_odd):
    return SERIAL_7O2;
  case encodeSerialMode(8, 2, par_int_odd):
    return SERIAL_8O2;
  }
  return SERIAL_8N1;
}

/**
* @brief Checks if a Modbus function code is a read operation.
* @param funcCode Modbus function code to check.
* @return true if code corresponds to a read operation.
* @details Supported read operations:
* - READ_COIL (0x01)
* - READ_HOLD_REGISTER (0x03)
* - READ_INPUT_REGISTER (0x04)
* - Others...
*/
bool opIsRead(Modbus::FunctionCode funcCode) {
  switch (funcCode) {
  case READ_COIL:
  case READ_COMM_CNT_SERIAL:
  case READ_COMM_LOG_SERIAL:
  case READ_DISCR_INPUT:
  case READ_EXCEPTION_SERIAL:
  case READ_FIFO_QUEUE:
  case READ_FILE_RECORD:
  case READ_HOLD_REGISTER:
  case READ_INPUT_REGISTER:
    return true;
  default:
    return false;
  }
}

/**
* @brief Checks if a Modbus function code is a write operation.
* @param funcCode Modbus function code to check.
* @return true if code corresponds to a write operation.
* @details Supported write operations:
* - WRITE_COIL (0x05)
* - WRITE_HOLD_REGISTER (0x06)
* - WRITE_MULT_COILS (0x0F)
* - Others...
*/
bool opIsWrite(Modbus::FunctionCode funcCode) {
  switch (funcCode) {
  case WRITE_COIL:
  case WRITE_FILE_RECORD:
  case WRITE_HOLD_REGISTER:
  case WRITE_MULT_COILS:
  case WRITE_MULT_REGISTERS:
    return true;
  default:
    return false;
  }
}


//************************************** ModbusProtocol **************************************//

const uint8_t MAX_NODE_REQUESTS = 3; ///< Maximum number of requests

/**
 * @typedef MBProtoOnResponse
 * @brief Response signature function
 */
typedef std::function<void(ModbusMessage& msg)> MBProtoOnResponse;

/**
 * @struct Busy_t
 * @brief Tracks active requests count per node.
 */
struct Busy_t {
  uint8_t read;   ///< Active read requests (max = MAX_NODE_REQUESTS)
  uint8_t write;  ///< Active write requests (max = MAX_NODE_REQUESTS)
};

/**
 * @struct Tracker_t
 * @brief Stores MBProtoOnResponse function and Busy_t struct.
 */
struct Tracker_t {
  MBProtoOnResponse fn; ///< MBProtoOnResponse function
  Busy_t busy;          ///< Busy_t struct
};

/**
 * @class ModbusProtocol
 * @brief Handles Modbus protocol for Modbus nodes
 * @details Handles Modbus protocol, request and interrups, queueing and response processing. Features:
 * - Limits concurrent requests per node (MAX_NODE_REQUESTS)
 * - Maps responses to nodes using tokens
 * - Thread-safe operations with semaphores
 */
class ModbusProtocol : public BaseProtocol {
private:

  // ========== MODBUS SPECIFIC MEMBERS ==========

  unordered_map<uint32_t, Tracker_t> m_token_map;
  HardwareSerial *m_serial_p;
  ModbusClientRTU *m_rtu_client;
  uint32_t m_serial_speed = 0;
  int8_t m_rx_pin = -1;
  int8_t m_tx_pin = -1;
  uint32_t m_mode_config = 0;


  // Private constructor to force factory usage
  ModbusProtocol(const std::string& key) : BaseProtocol(key), m_serial_p(nullptr), m_rtu_client(nullptr), m_token_map() {};

  // Friend declaration
  friend ModbusProtocol* BaseProtocol::newInstance<ModbusProtocol>(const std::string& key);

public:
  ~ModbusProtocol() {
    end();
  }
  
  // ========== FACTORY METHODS ==========
  
  /**
   * @brief Creates or gets a shared instance
   */
  static ModbusProtocol* createShared(const std::string& key, const char* dataSourceName) {
    return BaseProtocol::createShared<ModbusProtocol>(key, dataSourceName);
  }
  
  /**
   * @brief Creates unique instance (not shared)
   */
  static std::unique_ptr<ModbusProtocol> createUnique(const std::string& key) {
    return BaseProtocol::createUnique<ModbusProtocol>(key);
  }

  // ========== PUBLIC INTERFACE IMPLEMENTATION ==========

  bool configure() override {
    LockGuard lock(m_rec_mutex);
    // Set up m_serial_p connected to Modbus RTU
    int port_num = getInt("inst", -1);
    switch (port_num) {
      case 0:
        m_serial_p = &Serial;
        break;
      case 1:
        m_serial_p = &Serial1;
        break;
      case 2:
        m_serial_p = &Serial2;
        break;
      default:
        log_e("[%s] Wrong port value(%d)", m_key.c_str(), port_num);
        return false;
    }
    log_w("[%s] Using serial port #(%d) at %p", m_key.c_str(), port_num, m_serial_p);
    // Get bits parameters
    m_mode_config = setDataStopParity(getInt("bits"), getInt("stop"), getString("parity"));
    log_d("[%s] Serial configuration code: %d", m_key.c_str(), m_mode_config);
    RTUutils::prepareHardwareSerial(*m_serial_p);
    m_serial_speed = getU32("speed", 9600);
    m_rx_pin = getConfig<int8_t>("rxpin");
    m_tx_pin = getConfig<int8_t>("txpin");
    // Create a ModbusRTU client instance
    // The RS485 module has no halfduplex, so the parameter with the RTS pin is required!
    int8_t rts_pin = getConfig<int8_t>("rtspin");
    if (m_rtu_client) {
      log_d("[%s] Deleting previous client at %p", m_key.c_str(), m_rtu_client);
      delete m_rtu_client;
    }
    m_rtu_client = new ModbusClientRTU(rts_pin);
    log_d("[%s] New Modbus client instance at %p", m_key.c_str(), m_rtu_client);
    // Set up ModbusRTU client
    // Set message timeout
    uint32_t timeout = getU32("timeout");
    m_rtu_client->setTimeout(timeout);
    log_d("[%s] Settings: speed(%u), tx(%u) rx(%u) rts(%u) timeout(%u)", m_key.c_str(), m_serial_speed, m_tx_pin, m_rx_pin, rts_pin, timeout);
    // Set ModbusRTU response handler
    m_rtu_client->onResponseHandler([this](ModbusMessage msg, uint32_t token) {
      this->handleResponse(msg, token);
    });

    m_configured = true;
    m_started = false;
    return true;
  }

  bool begin() override {
    LockGuard lock(m_rec_mutex);

    if (!m_configured) {
      log_d("[%s] Not configured", m_key.c_str());
      return false;
    }

    if (m_started) {
      log_d("[%s] Already started", m_key.c_str());
      return false;
    }

    if (!m_serial_p) {
      log_e("[%s] Port not instanced", m_key.c_str());
      return false;
    }
    // Start serial port
    m_serial_p->begin(m_serial_speed, m_mode_config, m_rx_pin, m_tx_pin);
    // Start ModbusRTU background task
    m_rtu_client->begin(*m_serial_p);

    m_started = true;
    return true;
  }

  void end() override {
    LockGuard lock(m_rec_mutex);
    if(m_serial_p && m_rtu_client) {
      m_rtu_client->onResponseHandler(nullptr);
      m_rtu_client->end();
      m_serial_p->end();
    }
    m_token_map.clear();
    m_serial_p = nullptr;
    delete m_rtu_client;
    m_rtu_client = nullptr;
    m_started = false;
  }

  // ========== MODBUS SPECIFIC METHODS ==========

  void subscribe(uint32_t token, MBProtoOnResponse fn) {
    LockGuard lock(m_rec_mutex);
    m_token_map[token] = {fn: fn, busy: {read: 0, write: 0}};
    log_d("[%s] Function %p subscribed with token %u", m_key.c_str(), fn, token);
  }

  bool unsubscribe(uint32_t token) {
    LockGuard lock(m_rec_mutex);
    auto id_it = m_token_map.find(token);
    if (id_it != m_token_map.end()) {
      MBProtoOnResponse fn = id_it->second.fn;
      m_token_map.erase(token);
      log_d("[%s] Function %p and token %u unsubscribed", m_key.c_str(), fn, token);
      return true;
    }
    log_e("[%s] Token %u not found!", m_key.c_str(), token);
    return false;
  }

  /**
   * @brief Adds a Modbus request to the queue.
   * @param token Identifier for next response
   * @param id Modbus slave id
   * @param funcCode Modbus function code (READ_HOLD_REGISTER, etc.)
   * @param args Function-specific arguments (address, data, etc.)
   * @return Modbus::Error code (SUCCESS if queued successfully)
   * 
   * @example 
   * addRequest(READ_HOLD_REGISTER, 40001, 1); // Read 1 register at 40001
   */
  template <typename... Args>
  Modbus::Error addRequest(uint32_t token, uint8_t id, Modbus::FunctionCode func_code, Args&&... args) {
  if (m_rtu_client == nullptr) {
    log_e("[%s] RTU client not instanced", m_key.c_str());
    return Error::GATEWAY_PATH_UNAVAIL;
  }

  bool isRead = opIsRead(func_code);
  bool isWrite = opIsWrite(func_code);
  LockGuard lock(m_rec_mutex);
  auto token_it = m_token_map.find(token);
  if (token_it == m_token_map.end()) {
    log_e("[%s] Map token not found", m_key.c_str());
    return Error::UNDEFINED_ERROR;
  }
  Busy_t busy_state = token_it->second.busy;
  if ((busy_state.read >= MAX_NODE_REQUESTS && isRead) || (busy_state.write >= MAX_NODE_REQUESTS && isWrite))
    return Error::SERVER_DEVICE_BUSY;
  if (isRead)
    busy_state.read = busy_state.read >= MAX_NODE_REQUESTS ? MAX_NODE_REQUESTS : busy_state.read++;
  if (isWrite)
    busy_state.write = busy_state.write >= MAX_NODE_REQUESTS ? MAX_NODE_REQUESTS : busy_state.write++;
  m_token_map[token].busy = busy_state;
  return m_rtu_client->addRequest(token, id, func_code, args ...);
}

private:
  /**
  * @brief Method for processing Modbus responses. Handles incoming Modbus responses and routes them to nodes.
  * @param msg Modbus response message.
  * @param token Node identifier (m_index of target node).
  * @details Workflow:
  * 1. Finds ModbusManager instance via token in sm_map.
  * 2. Updates busy_map request counters.
  * 3. Delegates message processing to node's onModbusResponse().
  */
  void handleResponse(ModbusMessage& msg, uint32_t token) {
    log_d("[%s] Message received with token(%u)", m_key.c_str(), token);
    Modbus::FunctionCode funcCode = static_cast<Modbus::FunctionCode>(msg.getFunctionCode());
    bool isRead = opIsRead(funcCode);
    bool isWrite = opIsWrite(funcCode);
    // Apply mutex from here
    LockGuard lock(m_rec_mutex);
    // Find element in iterator
    auto token_it = m_token_map.find(token);
    if (token_it != m_token_map.end()) {
      // Update busy read and write elements
      Busy_t busy_item = token_it->second.busy;
      if (isRead)
        busy_item.read = busy_item.read == 0 ? 0 : busy_item.read--;
      if (isWrite)
        busy_item.write = busy_item.write == 0 ? 0 : busy_item.write--;
      m_token_map[token].busy = busy_item;
      // Send message to function
      MBProtoOnResponse fn = token_it->second.fn;
      if (!fn)
        return;
      fn(msg);
      log_d("[%s] Message sent to function %p", m_key.c_str(), fn);
      return;
    }
    log_e("[%s] Token not found in map", m_key.c_str());
    return;
  }
};


//************************************** Modbus RTU Source **************************************//

int DsModbus::build(JsonObject source, int index) {
  int id = source["id"] | 1;
  if (id < 0 || id > UINT8_MAX) {
    log_e("Id outside reange: %d", id);
    return DIError::ErrCode::PARAM_OUT_BOUNDS;
  }
  m_id = id;
  // 1. Instance modbus protocol, to register name
  std::string name = std::string(type()) + "@0";
  m_modbus = ModbusProtocol::createShared(name, type());
  log_d("%s protocol driver started at address %p", type(), m_modbus);
  // 2. Set protocol parameters
  m_cptreq = source["compactRequest"] | false;
  std::string order = source["order"];
  int ord = setOrdering(order);
  if (ord < 0) {
    log_e("Invalid ordering");
    return DIError::ErrCode::WRONG_ENUM;
  }
  // 3. Start configuration
  if (!m_modbus->configure()) {
    log_e("Modbus protocol driver not instanced", name.c_str());
    return DIError::BUS_REQ_ERROR; // TODO: create bus init error
  }
  // Then, wait to one data node to start the protocol

  return DataSource::build(source, index);
}

ModbusProtocol *DsModbus::protocol() {
  if (!m_modbus)
    log_w("Modbus protocol driver instance lost");
  return m_modbus;
}

int DsModbus::setOrdering(std::string& ordering) { // TODO: implement ordering
  return 0;
}


//************************************** Free checks **************************************//

DIError::ErrCode checkAddress(JsonObject node, uint16_t& m_address) {
  int address = node["address"] | -1;
  if(address < 0 || address > UINT16_MAX) {
    log_e("Address not supplied or outside reange: %d", address);
    return DIError::ErrCode::PARAM_OUT_BOUNDS;
  }
  m_address = address;
  return DIError::ErrCode::SUCCESS;
}


//************************************** ModbusHAndled Node **************************************//

void ModbusHAndled::start() { // This is the same member function for all subclasses, so added here...
  log_d("static_cast DataSource* to DsModbus*");
  if (m_source == nullptr) {
    // purpouselly unprotected to generate crash if m_source == nullptr), but inform the problem before
    log_e("Modbus data source is null");
  }
  DsModbus* ds = static_cast<DsModbus*>(m_source);
  ModbusProtocol *protocol = ds->protocol();
  log_d("Starting Modbus Manager");
  protocol->subscribe(token(), [this](ModbusMessage &msg) {
    this->onModbusResponse(msg);
  });
  protocol->begin();
  request();
}


//************************************** Holding Register Node **************************************//

MbHReg::~MbHReg() {
  delete m_calc;
  delete m_sendman;
}

int MbHReg::build(JsonObject node, int index) {
  DIError::ErrCode code = checkAddress(node, m_address);
  if (code != DIError::ErrCode::SUCCESS)
    return code;
  // Add linear calculator
  float offset = node["off"];
  float gain = node["gain"] | 1.0;
  m_calc = new LinearCalc<>(offset, gain, UINT16_MAX);
  // add send manager
  uint32_t mintime = node["mintime"] | 1;
  uint32_t maxtime = node["maxtime"] | 60;
  float delta = node["delta"] | 0.1;
  m_sendman = new AnSendManager(mintime, maxtime, delta);

  m_unit = node["unit"].as<std::string>();
  return DataNode::build(node, index);
}

void MbHReg::request() {
  log_d("Adding Modbus request %d", m_index);
  if (m_source == nullptr) {
    log_e("Modbus data source is null");
    return;
  }
  DsModbus* ds = static_cast<DsModbus*>(m_source);
  ModbusProtocol *protocol = ds->protocol();
  if (protocol == nullptr) {
    log_e("ModbusProtocol not started");
    return;
  }
  Modbus::Error error;
  // 1. Validate and convert input value
  if (m_cmd == DataNode::WRITE) {
    if (!m_variant.can_convert<float>()) {
      sendError(DIError::PARAM_WORNG_TYPE);
      return;
    }
    float fValue = m_variant.convert<float>();
    uint32_t value = m_calc->receive(fValue);
    // 2. Send Modbus write request
    error = protocol->addRequest(token(), ds->serverId(), WRITE_HOLD_REGISTER, m_address, value);
  } else {
    // 3. Send Modbus read request
    error = protocol->addRequest(token(), ds->serverId(), READ_HOLD_REGISTER, m_address, 1);
  }
  // 4. Handle Modbus layer errors
  if (error != Error::SUCCESS) {
    log_d("Request failed");
    ModbusError moderr = ModbusError(error);
    const char* errs(moderr);
    sendError(DIError::BUS_REQ_ERROR, errs);
  }
}

void MbHReg::onModbusResponse(ModbusMessage &msg) {
  ModbusError error = ModbusError(msg.getError());
  if (error != Error::SUCCESS) {
    log_d("Async request failed");
    const char * errs(error);
    sendError(DIError::BUS_REQ_ERROR, errs);
  } else {
    uint16_t value;
    msg.get(3, value);
    float fValue = m_calc->send(value);
    log_d("Async %d -> Modbus HREG: %u  Calc: %f", m_index, value, fValue);
    if (!m_sendman->updateRequired(fValue, m_cmd == Cmd::READ_FORCED || m_cmd == Cmd::WRITE))
      return;
    std::string& unit = m_unit;
    bool found = m_source->onResponse(m_name, [fValue, unit](JsonObject object) {
      object["value"] = fValue;
      if(!unit.empty())
        object["unit"] = unit;
    });
    log_d("Add %s: %s", m_name.c_str(), found ? "exist" : "new");
  }
}
