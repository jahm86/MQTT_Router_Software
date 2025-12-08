
#include "DsModbus.h"
#include <limits>
#include "diutils.h"
#include <unordered_map>

using namespace std;


//************************************** Modbus RTU Source **************************************//

int DsModbus::build(JsonObject source, int index) {
  int id = source["id"] | 1;
  if (id < 0 || id > UINT8_MAX) {
    log_e("Id outside reange: %d", id);
    return DIError::ErrCode::PARAM_OUT_BOUNDS;
  }
  m_id = id;
  m_cptreq = source["compactRequest"] | false;
  string order = source["order"];
  int ord = setOrdering(order);
  if (ord < 0) {
    log_e("Invalid ordering");
    return DIError::ErrCode::WRONG_ENUM;
  }
  return DataSource::build(source, index);
}

int DsModbus::setOrdering(string& ordering) {
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


//************************************** Modbus manager **************************************//

template<class DataSourceT, class DataNodeT>
Semaphore ModbusManager<DataSourceT, DataNodeT>::m_mutex(Semaphore::create_mutex());

template<class DataSourceT, class DataNodeT>
bool ModbusManager<DataSourceT, DataNodeT>::respHandlerIsSet(false);

template<class DataSourceT, class DataNodeT>
unordered_map<uint16_t, typename ModbusManager<DataSourceT, DataNodeT>::Busy_t> ModbusManager<DataSourceT, DataNodeT>::busy_map{};

template<class DataSourceT, class DataNodeT>
unordered_map<uint16_t, ModbusManager<DataSourceT, DataNodeT>*> ModbusManager<DataSourceT, DataNodeT>::sm_map{};

template<class DataSourceT, class DataNodeT>
ModbusManager<DataSourceT, DataNodeT>::ModbusManager(DataSourceT* dsInstance, DataNodeT* nodeInstance) :
    m_dsInstance(dsInstance), m_nodeInstance(nodeInstance) {
  uint32_t token = nodeInstance->token();
  do { // To apply mutex just here
    LockGuard lg(m_mutex);
    sm_map[token] = this;
    busy_map[token] = {false, false};
  } while(false);
  if (!respHandlerIsSet)
    setResponseHandler();
}

template<class DataSourceT, class DataNodeT>
bool ModbusManager<DataSourceT, DataNodeT>::opIsRead(Modbus::FunctionCode funcCode) {
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

template<class DataSourceT, class DataNodeT>
bool ModbusManager<DataSourceT, DataNodeT>::opIsWrite(Modbus::FunctionCode funcCode) {
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

template<class DataSourceT, class DataNodeT>
void ModbusManager<DataSourceT, DataNodeT>::setResponseHandler() {
  ModbusClient* mbInstance = StreamLink::Instance().Get();
  if (mbInstance == nullptr)
    return;
  respHandlerIsSet = true;
  mbInstance->onResponseHandler(ModbusManager::onModbusResponse);
}

template<class DataSourceT, class DataNodeT>
void ModbusManager<DataSourceT, DataNodeT>::onModbusResponse(ModbusMessage msg, uint32_t token) {
  // Find element in iterator
  typename unordered_map<uint16_t, ModbusManager*>::const_iterator got = sm_map.find(token);
  if (got == sm_map.end()) {
    log_e("Response points to unregistered token");
    return;
  }
  ModbusManager* instance = got->second;
  DataNodeT* nodeInstance = instance->m_nodeInstance;
  Modbus::FunctionCode funcCode = static_cast<Modbus::FunctionCode>(msg.getFunctionCode());
  bool isRead = opIsRead(funcCode);
  bool isWrite = opIsWrite(funcCode);
  do { // To apply mutex just here
    LockGuard lg(m_mutex);
    typename unordered_map<uint16_t, Busy_t>::const_iterator busy_it = busy_map.find(token);
    assert(busy_it != busy_map.end());
    Busy_t busy_item = busy_it->second;
    if (isRead)
      busy_item.read = busy_item.read == 0 ? 0 : busy_item.read--;
    if (isWrite)
      busy_item.write = busy_item.write == 0 ? 0 : busy_item.write--;
    busy_map[token] = busy_item;
  } while(false);
  if (nodeInstance == nullptr) {
    log_e("Modbus handler instance is null");
    return;
  }
  nodeInstance->onModbusResponse(msg);
}

template<class DataSourceT, class DataNodeT>
template <typename... Args>
Modbus::Error ModbusManager<DataSourceT, DataNodeT>::addRequest(Modbus::FunctionCode funcCode, Args&&... args) {
  ModbusClient* mbInstance = StreamLink::Instance().Get();
  if (mbInstance == nullptr) {
    log_e("Modbus stream not instanced");
    return Error::GATEWAY_PATH_UNAVAIL;
  }
  if (m_dsInstance == nullptr || m_nodeInstance == nullptr) {
    log_e("Some of constructor parameters are null");
    return Error::UNDEFINED_ERROR;
  }
  uint32_t token = m_nodeInstance->token();
  uint8_t id = m_dsInstance->serverId();
  bool isRead = opIsRead(funcCode);
  bool isWrite = opIsWrite(funcCode);
  LockGuard lg(m_mutex);
  Busy_t& busyState = busy_map[token];
  if ((busyState.read >= MAX_NODE_REQUESTS && isRead) || (busyState.write >= MAX_NODE_REQUESTS && isWrite))
    return Error::SERVER_DEVICE_BUSY;
  if (isRead)
    busyState.read = busyState.read >= MAX_NODE_REQUESTS ? MAX_NODE_REQUESTS : busyState.read;
  if (isWrite)
    busyState.write = busyState.write >= MAX_NODE_REQUESTS ? MAX_NODE_REQUESTS : busyState.write;
  return mbInstance->addRequest(token, id, funcCode, args ...);
}


//************************************** ModbusHAndled Node **************************************//

void ModbusHAndled::start() { // This is the same member function for all subclasses, so added here...
  log_d("static_cast DataSource* to DsModbus*");
  if (m_source == nullptr) {
    // purpouselly unprotected to generate crash if m_source == nullptr), but inform the problem before
    log_e("Modbus data source is null");
  }
  DsModbus* ds = static_cast<DsModbus*>(m_source);
  log_d("Starting Modbus Manager");
  m_manager = new ModbusManager<DsModbus, ModbusHAndled>(ds, this);
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
  m_calc = new LinearCalc(offset, gain, UINT16_MAX);
  // add send manager
  uint32_t mintime = node["mintime"] | 1;
  uint32_t maxtime = node["maxtime"] | 60;
  float delta = node["delta"] | 0.1;
  m_sendman = new AnSendManager(mintime, maxtime, delta);

  m_unit = node["unit"].as<string>();
  return DataNode::build(node, index);
}

void MbHReg::request() {
  log_d("Adding Modbus request %d", m_index);
  if (m_manager == nullptr) {
    log_e("ModbusManager not started");
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
    error = m_manager->addRequest(WRITE_HOLD_REGISTER, m_address, value);
  } else {
    // 3. Send Modbus read request
    error = m_manager->addRequest(READ_HOLD_REGISTER, m_address, 1);
  }
  // 4. Handle Modbus layer errors
  if (error != Error::SUCCESS) {
    log_d("Request failed");
    ModbusError moderr = ModbusError(error);
    const char* errs(moderr);
    sendError(DIError::BUS_REQ_ERROR, errs);
  }
}

void MbHReg::onModbusResponse(ModbusMessage msg) {
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
    string& unit = m_unit;
    bool found = m_source->onResponse(m_name, [fValue, unit](JsonObject object) {
      object["value"] = fValue;
      if(!unit.empty())
        object["unit"] = unit;
    });
    log_d("Add %s: %s", m_name.c_str(), found ? "exist" : "new");
  }
}
