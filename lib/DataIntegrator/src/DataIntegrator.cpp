/*
  Data integration library, using json objects

  Copyleft (c) 2024 Johautt Hernández.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

//#include <ArduinoJson.h>
#include "DataIntegrator.h"
#include <esp_err.h>
#include <memory>
#include "DsIO.h"
#include "DsModbus.h"
#include "DsCANBUS.h"
#include "freertos/timers.h"
#include "diutils.h"

using namespace std;

// Hash function with result calculated at complile time for constant values
constexpr unsigned int hashfcn(const char *s, int off = 0) {                        
    return !s[off] ? 5381 : (hashfcn(s, off+1)*33) ^ s[off];                           
}


//************************************** Data Factory **************************************//

template<>
DataFactory<DataSource>::DataFactory() {
  log_d("Creating DataSource subtypes");
  // I/O source
  Register("io", DsIO::Create);
  // Modbus source
  Register("rtu", DsModbus::Create);
  // Canbus source
  Register("can", DsCANBUS::Create);
}

template<>
DataFactory<DataNode>::DataFactory() {
  log_d("Creating DataNode subtypes");
  // I/O objects
  Register("ai", AnIn::Create);
  Register("di", DgIn::Create);
  Register("do", DgOut::Create);
  Register("pwm", PWM::Create);
  // Modbus objects
  Register("HREG", MbHReg::Create);
  // Canbus objects
  Register("uint8", CAN_uint8::Create);
  Register("uint16", CAN_uint16::Create);
  Register("uint32", CAN_uint32::Create);
  Register("int8", CAN_int8::Create);
  Register("int16", CAN_int16::Create);
  Register("int32", CAN_int32::Create);
  Register("float", CAN_float::Create);
}


//************************************** Data Builder **************************************//

DataBuilder::DataBuilder(Stream& jsonFile, DataBus& databus) :
  m_error(DIError::ErrCode::SUCCESS), m_desError(DeserializationError::Code::Ok) {
  // Deserialize JSON
#ifdef USE_TOAD_ALLOCATOR
  JsonDocument doc(Toadllocator::instance( __FILE__, __LINE__ ));
#else
  JsonDocument doc;
#endif
  // Index for every element
  int index = 1;
  // Requires: https://github.com/bblanchon/ArduinoStreamUtils
  //ReadBufferingStream bufferingStream(jsonFile, 64);
  //m_error = deserializeJson(doc, bufferingStream);
  m_desError = deserializeJson(doc, jsonFile);
  if (m_desError) {
    log_e("deserializeJson error: %s", m_desError.c_str());
    m_error = DIError::ErrCode::FILE_DES_ERROR;
    return;
  }
  // Checks for JSON array
  JsonArray root = doc.as<JsonArray>();
  if (root == nullptr) {
    log_e("Root must be an array");
    m_error = DIError::ErrCode::UNEXPECTED_JSON_TYPE;
    return;
  }
  // Walk the JsonArray
  for (JsonObject source : root) {
    // Checks for source type
    const char* type = source["type"];
    if (type == nullptr) {
      log_e("Source type not found in file");
      m_error = DIError::ErrCode::TYPE_NOT_INCLUDED;
      return;
    }
    DataSource* ds = DataFactory<DataSource>::Instance().Create(type);
    if (ds == nullptr) {
      log_e("Source %s doesn't exits", type);
      m_error = DIError::ErrCode::TYPE_NOT_EXIST;
      return;
    }
    log_d("Source type: %s", ds->type());
    index = ds->build(source, index);
    if (index < DIError::ErrCode::SUCCESS) {
      log_e("Data source %s exit with error", ds->name().c_str());
      m_error = static_cast<DIError::ErrCode>(index);
      return;
    }
    databus.add(ds);
  }
}


//************************************** Data Bus **************************************//

/**
 * @brief DataBus constructor.
 * @details Initializes JSON document and internal structures.
 */
DataBus::DataBus() : m_dsv(), m_dscb(nullptr), m_started(false), m_running(false), m_taskHandler(nullptr) {
#ifdef USE_TOAD_ALLOCATOR
  m_txDoc = new JsonDocument(Toadllocator::instance( __FILE__, __LINE__ ));
#else
  m_txDoc = new JsonDocument();
#endif
}

void DataBus::add(DataSource* ds) {
  m_dsv.push_back(ds);
  log_d("Elements: %u", m_dsv.size());
}

/**
 * @brief Initializes and starts the DataBus communication task.
 * @details This FreeRTOS task periodically:
 * - Collects data from all registered DataSources
 * - Sends consolidated data via the registered callback
 * - Manages timing constraints (min/max send intervals)
 * @warning Call only once per DataBus instance.
 */
void DataBus::startCom() {
  if (m_started)
    return;
  log_d("Starting DataBus");
  log_d("Elements: %d", m_dsv.size());
  resetJSON();
  for (DataSource* source : m_dsv) {
    source->start();
  }
  // Starts data bus task
  DataBus* inst = this;
  m_taskHandler = new extTask("DataBus", 4096, tskIDLE_PRIORITY + 1, [inst]() {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
    // Checks for resource usage in debug mode
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    log_d("Stack unused %d", uxHighWaterMark);
    UBaseType_t uxHighWaterMark2;
#endif
    log_d("Starting Data Bus Task...");
    inst->m_started = true;
    inst->m_running = true;
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(5000));
      log_i("Send: start");
      inst->request();
      inst->send(inst->m_txDoc);
      log_i("Send: end");
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
      // Checks for resource usage in debug mode
      uxHighWaterMark2 = uxTaskGetStackHighWaterMark( NULL );
      if (uxHighWaterMark2 < uxHighWaterMark) {
        uxHighWaterMark = uxHighWaterMark2;
        log_d("Stack unused lower %d", uxHighWaterMark);
        Toadllocator::check( __FILE__, __LINE__ );
      }
#endif
    }
  });
}

void DataBus::suspend() {
  if (!m_started)
    return;
  m_running = false;
  m_taskHandler->suspend();
}

void DataBus::resume() {
  if (!m_started)
    return;
  m_taskHandler->resume();
  m_running = true;
}

void DataBus::resetJSON() {
  for (DataSource* source : m_dsv) {
    source->resetJSON();
  }
  log_d("Reset DataBus array");
  m_txDoc->to<JsonArray>();
}

void DataBus::send(JsonDocument* docP) {
  JsonDocument doc = *docP;
  if (!m_running || m_dscb == nullptr || doc.size() == 0)
    return;
  size_t size = measureJson(doc);
  unique_ptr<char[]> alloc(new char[size]());
  char* output = alloc.get();
  // Please, avoid to use anonimous declaration for ScopedAllocator
  //ScopedAllocator<char> alloc(size);
  //char* output = alloc.pointer();
  log_d("Sending %d bytes, addr: %p", size, output);
  serializeJson(doc, output, size);
  m_dscb(output, size);
}

/**
 * @brief DataBus::ack implementation.
 * @details Security considerations:
 * - Calls resetJSON() to clear sensitive data
 * - Ensures no residual data remains in buffers
 */
void DataBus::ack() {
  if (!m_running || !m_started)
    return;
  log_d("Request acklowledged. Cleaning...");
  resetJSON();
}

//bool DataBus::receive(JsonDocument doc) {  // Gives linker error: undefined reference to `DataBus::receive(ArduinoJson::V704JB2::JsonDocument)'
DIError::ErrCode DataBus::receive(const char* payload, size_t len) {
  if (!m_running)
    return DIError::PAUSED;
#ifdef USE_TOAD_ALLOCATOR
  JsonDocument doc(Toadllocator::instance( __FILE__, __LINE__ ));
#else
  JsonDocument doc;
#endif
  DIError::ErrCode errcode = DIError::SUCCESS;
  // Deserialize and check for errors
  DeserializationError err = deserializeJson(doc, payload);
  log_d("Document length: %d, error: %s", len, err.c_str());
  if (err) {
    JsonObject obj = doc.to<JsonObject>();
    errcode = DIError::FILE_DES_ERROR;
    obj["error"] = DIError::ErrStr(errcode);
    obj["deserror"] = err.c_str();
    send(&doc);
    return errcode;
  }
  // Check for json object
  JsonObject obj = doc.as<JsonObject>();
  if (obj.isNull())
    errcode = DIError::UNEXPECTED_JSON_TYPE;
  else 
    errcode = cmd(obj);
  if (errcode != DIError::SUCCESS) {
    JsonObject obj = doc.to<JsonObject>();
    obj["error"] = DIError::ErrStr(errcode);
    send(&doc);
  }
  return errcode;
}

/**
 * @brief DataBus::request implementation.
 * @details Workflow:
 * 1. Locks mutex for thread-safe JSON access
 * 2. Iterates through all DataSources
 * 3. Checks if source needs update (needsUpdate())
 * 4. Updates existing JSON object or creates new entry
 * 5. Unlocks mutex on completion
 */
void DataBus::request() {
  // TODO: Put mutex here
  JsonArray array = m_txDoc->as<JsonArray>();
  bool found;
  for (DataSource* source : m_dsv) {
    found = false;
    if (source->needsUpdate()) {
      const string& name = source->name();
      for (JsonObject object : array) {
        if (object["name"] == name) {
          found = true;
          source->retrieve(object);
        }
      }
      if (!found) {
        JsonObject object = array.add<JsonObject>();
        source->retrieve(object);
      }
    }
  }
}

/**
 * @brief DataBus::findInDV implementation.
 * @details Search workflow:
 * 1. Logs search attempt: "DataSource to find: [name]"
 * 2. Linear search through m_dsv (DataSource vector)
 * 3. On match: executes DataSource::cmd() with provided JSON
 * 4. Returns SOURCE_NOT_FOUND if no match
 */
DIError::ErrCode DataBus::findInDV(const string& name, JsonObject obj) {
  log_d("DataSource to find: \"%s\"", name.c_str());
  for (DataSource* item : m_dsv) {
    if (item->name() == name) {
      log_d("String found");
      return item->cmd(obj);
    }
  }
  log_d("Source not found");
  return DIError::SOURCE_NOT_FOUND;
}

DIError::ErrCode DataBus::cmd(JsonObject obj) {
  log_d("Enter bus command");
  // Checks if is single command, or multiple command (if "sources" exits and is array, is multiple)
  JsonArray arr = obj["sources"].as<JsonArray>();
  if (arr.isNull()) { // If single command, enters here
    log_d("Command is single");
    string sourcename = obj["source"] | "";
    // checks if "source" string has something
    if (!sourcename.empty()) {
      return findInDV(sourcename, obj);
    }
  } else { // If array of commands, enters here
    DIError::ErrCode errcode;
    log_d("Command is array of sources");
    for (JsonObject sourceObj : arr) {
      string sourcename = sourceObj["name"];
      errcode = findInDV(sourcename, sourceObj);
      if (errcode != DIError::SUCCESS)
        return errcode;
    }
    return DIError::SUCCESS;
  }
  // Find command for bus
  string cmd = obj["cmd"] | "";
  // If read, aborts delay task and begin send of data
  if (cmd == "read") {
    log_d("Aborting main task delay...");
    vTaskDelay(pdMS_TO_TICKS(250));
    m_taskHandler->abortTaskDelay();
    return DIError::SUCCESS;
  }
  return DIError::WRONG_CMD;
}



//************************************** Data Source **************************************//

DataSource::DataSource() : m_index(-1), m_dnv(), m_started(false),
m_mutex(Semaphore::create_mutex()) {
#ifdef USE_TOAD_ALLOCATOR
  m_doc = new JsonDocument(Toadllocator::instance( __FILE__, __LINE__ ));
#else
  m_doc = new JsonDocument();
#endif
}

int DataSource::build(JsonObject source, int index) {
  m_index = index;
  m_name = source["name"] | "";
  if (m_name.empty()) {
    log_e("Source name not found in file");
    return DIError::ErrCode::NAME_NOT_INCLUDED;
  }
  // Checks for JSON array
  JsonArray root = source["nodes"].as<JsonArray>();
  if (root == nullptr) {
    log_e("Source root must be an array");
    return DIError::ErrCode::UNEXPECTED_JSON_TYPE;
  }
  // Walk the JsonArray
  for (JsonObject node : root) {
    // Checks for node type
    const char* type = node["type"];
    if (type == nullptr) {
      log_e("Node type not found in file");
      return DIError::ErrCode::TYPE_NOT_INCLUDED;
    }
    DataNode* dn = DataFactory<DataNode>::Instance().Create(type);
    if (dn == nullptr) {
      log_e("Node %s doesn't exists", type);
      return DIError::ErrCode::TYPE_NOT_EXIST;
    }
    log_d("Node type: %s", dn->type());
    m_index = dn->build(node, m_index);
    if (m_index < DIError::ErrCode::SUCCESS) {
      return static_cast<DIError::ErrCode>(m_index);
    }
    m_dnv.push_back(dn);
  }
  log_d("Elements: %d", m_dnv.size());
  return m_index + 1;
}

void DataSource::start() {
  if (m_started)
    return;
  log_d("Starting DataSource: %s  index: %d", m_name.c_str(), m_index);
  // Not needed, as it's been reset by DataBus
  //resetJSON();
  for (DataNode* node : m_dnv) {
    node->start(this);
  }
  // Start data source task
  DataSource* inst = this;
  m_taskHandler = new extTask(m_name.c_str(), 4096, tskIDLE_PRIORITY + 2, [inst]() {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
    // Checks for resource usage in debug mode
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    log_d("Stack unused %d  Datasource %s", uxHighWaterMark, inst->m_name.c_str());
    UBaseType_t uxHighWaterMark2;
#endif
    log_d("Starting Data Source %s Task...", inst->m_name.c_str());
    inst->m_started = true;
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(2000));
      inst->request();
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
      // Checks for resource usage in debug mode
      uxHighWaterMark2 = uxTaskGetStackHighWaterMark( NULL );
      if (uxHighWaterMark2 < uxHighWaterMark) {
        uxHighWaterMark = uxHighWaterMark2;
        log_d("Stack unused lower %d  Datasource %s", uxHighWaterMark, inst->m_name.c_str());
        Toadllocator::check( __FILE__, __LINE__ );
      }
#endif
    }
  });
}

void DataSource::resetJSON() {
  log_d("Reset DataSource %s object", m_name.c_str());
  LockGuard lg(m_mutex);
  JsonObject object = m_doc->to<JsonObject>();
  object["name"] = m_name;
  object["nodes"].to<JsonArray>();
}

bool DataSource::needsUpdate() {
  JsonDocument& doc = *m_doc;
  return doc["nodes"].as<JsonArray>().size() > 0;
}

bool DataSource::retrieve(JsonObject object) {
  JsonObjectConst objectToCopy = m_doc->as<JsonObjectConst>();
  bool success = object.set(objectToCopy);
  log_d("Added %s: %s", m_name.c_str(), success ? "success" : "fail");
  return success;
}

void DataSource::request(JsonObject* objP) {
  for (DataNode* node : m_dnv) {
    node->request(objP);
  }
}

bool DataSource::onResponse(string& name, DataIntegrator::OnObjectFillCallback ofcb) {
  LockGuard lg(m_mutex);
  JsonDocument& doc = *m_doc;
  JsonArray array = doc["nodes"].as<JsonArray>();
  bool found = false;
  for (JsonObject object : array) {
    if (object["name"] == name) {
      found = true;
      ofcb(object);
    }
  }
  if (!found) {
    JsonObject object = array.add<JsonObject>();
    object["name"] = name;
    ofcb(object);
  }
  return found;
}

DIError::ErrCode DataSource::findInDV(const string& name, JsonObject obj) {
  log_d("DataNode to find: %s", name.c_str());
  for (DataNode* node : m_dnv) {
    if (node->name() == name) {
      log_d("String found");
      node->request(&obj);
      return DIError::SUCCESS;
    }
  }
  log_d("Node not found");
  return DIError::NODE_NOT_FOUND;
}

DIError::ErrCode DataSource::cmd(JsonObject obj) {
  log_d("Enter data source command");
  // checks if "node" string has something
  string nodename = obj["node"] | "";
  if ( !nodename.empty() ) {
    return findInDV(nodename, obj);
  }
  // Find command for bus
  string cmd = obj["cmd"] | "";
  // If read, aborts delay task and begin send of data
  if ( cmd == "read") {
    request(&obj);
    return DIError::SUCCESS;
  }
  // Checks if is single command, or multiple command (if "nodes" exits and is array, is multiple)
  JsonArray arr = obj["nodes"].as<JsonArray>();
  if (!arr.isNull()) {
    DIError::ErrCode errcode;
    log_d("Is array of nodes");
    for (JsonObject nodeObj : arr) {
      string nodename = nodeObj["name"];
      errcode = findInDV(nodename, nodeObj);
      if (errcode != DIError::SUCCESS)
        return errcode;
    }
    return DIError::SUCCESS;
  }
  return DIError::WRONG_CMD;
}


//************************************** Data Node **************************************//

int DataNode::build(JsonObject node, int index) {
  m_index = index;
  m_name = node["name"] | "";
  if (m_name.empty()) {
    log_e("Source name not found in file");
    return DIError::NAME_NOT_INCLUDED;
  }
  return m_index + 1;
}

void DataNode::start(DataSource* source) {
  log_d("Starting DataNode: %s  index: %d", m_name.c_str(), m_index);
  m_source = source;
  start();
}

DataNode::Cmd DataNode::decodeCmd(const string& cmds) {
  if (cmds == "read")
    return DataNode::READ_FORCED;
  else if (cmds == "write")
    return DataNode::WRITE;
  //else if (cmds == "status") // Not implemented yet
  //  return DataNode::STATUS;
  return DataNode::UNKNOWN;
}

void DataNode::request(JsonObject* objP) {
  if (objP == nullptr)
    m_cmd = DataNode::READ;
  else {
    log_d("Proccessing external rqeuest");
    JsonObject obj = *objP;
    string cmds = obj["cmd"] | "";
    m_cmd = decodeCmd(cmds);
  }
  switch (m_cmd) {
  case DataNode::UNKNOWN:
    sendError(DIError::WRONG_CMD);
    return;
  case DataNode::WRITE: {
    JsonObject obj = *objP;
    m_variant = obj["value"].as<JsonVariant>();
    break;
  }
  default:
    break;
  }
  request();
}

/**
 * @brief DataNode::sendError implementation.
 * @details Workflow:
 * 1. Creates error entry in the parent DataSource's JSON:
 * @code{.json}
 * {
 *   "name": "node_name",
 *   "error": "Error code",
 *   "detail": "Additional info" // Optional
 * }
 * @endcode
 * 2. Uses DataSource::onResponse() to inject the error
 * 3. Logs error via ESP32 logging system
 */
void DataNode::sendError(DIError::ErrCode errcode, const char* detailed) {
  const char* error = DIError::ErrStr(errcode);
  if (detailed == nullptr)
    log_d("Error code: %s", error);
  else
    log_d("Error code: %s  detail: %s", error, detailed);
  bool found = m_source->onResponse(m_name, [error, detailed](JsonObject object) {
    object["error"] = error;
    if (detailed != nullptr)
      object["detail"] = detailed;
  });
  log_d("Add %s: %s", m_name.c_str(), found ? "exist" : "new");
}
