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

/**
 * @file DataIntegrator.h
 * @brief JSON-based data integration system for IoT/ESP32 devices.
 * @author Johautt Hernández
 * @date 2024-2025
 */

#ifndef DATA_INTEGRATOR_H
#define DATA_INTEGRATOR_H

#include <ArduinoJson.h>
#include <unordered_map>
#include <vector>
#include <DIErrors.h>
#include "diutils.h"

using namespace std;

/*****************************************************************************************
 * @defgroup DataIntegrationCore Data Integration Core
 * @brief Base classes for data sources and nodes
 */
namespace DataIntegrator {

  /**
   * @brief User callback for sending processed data.
   * @param payload Pointer to serialized data.
   * @param size Size of payload in bytes.
   */
  typedef function<void(const char*, size_t)> OnDataSendCallback;

  // Internal callbacks
  typedef function<void(JsonObject)> OnObjectFillCallback;
  typedef function<void(JsonArray)> OnArrayFillCallback;
}


/*****************************************************************************************
 * @class DataFactory
 * @tparam DataT Data type to create (DataSource or DataNode)
 * @brief Implements the Factory pattern for DataSources/Nodes.
 * @ingroup DataIntegrationCore
 * 
 * @code
 * DataFactory<DataSource>::Instance().Create("rtu"); // Creates DsModbus
 * @endcode
 */
template<class DataT>
class DataFactory {
public:

  /**
   * @brief Gets the singleton factory instance.
   */
  static DataFactory& Instance() {
    static DataFactory factory;
    return factory;
  }


  /**
   * @brief Creates a new instance of the registered type.
   * @param name Type identifier (e.g., "rtu", "ai")
   * @return Pointer to created object or nullptr if type not found.
   */
  DataT* Create(string& name) {
    typename unordered_map<string, function<DataT*()>>::const_iterator it = m_table.find(name);
    if( it != m_table.end() )
      return (it->second)();
    return nullptr;
  }

  DataT* Create(const char* name) {
    string str(name);
    return Create(str);
  }

private:
  unordered_map<string, function<DataT*()>> m_table;

  /**
   * @brief Registers a new type in the factory.
   * @param name Type name (e.g., "rtu", "ai")
   * @param crFunc Creation function
   */
  void Register(string name, function<DataT*()> crFunc) {
    m_table[name] = crFunc;
  }

  DataFactory();
};

class Commm;
class DataSource;
class DataBus;
class extTask;
//class CustomVariant;


/*****************************************************************************************
 * @class DataNode
 * @brief Abstract base node class for data read/write operations in data nodes (sensors/actuators).
 * @ingroup DataIntegrationCore
 */
class DataNode {
  friend class Commm;

public:

  /**
   * @enum Cmd
   * @brief Supported node commands
   */
  enum Cmd {
    READ,
    READ_FORCED,
    WRITE,
    STATUS, // Not implemented yet
    UNKNOWN
  };

  DataNode() : m_index(-1), m_source(nullptr), m_cmd(READ) {}

  /**
   * @brief Converts a string command to the Cmd enum.
   * @param cmds Command string ("read", "write", etc.).
   * @return Corresponding Cmd value or UNKNOWN if invalid.
   */
  Cmd decodeCmd(const string& cmd);

  virtual ~DataNode() = default;
  virtual const char* type() = 0;
  const string& name() { return m_name; }
  int index() { return m_index; }
  virtual int build(JsonObject, int);

  /**
   * @brief Initializes the node with its parent DataSource.
   * @param source Pointer to the parent DataSource.
   * @note Calls the pure virtual start() method for node-specific setup.
   */
  void start(DataSource*);

  /**
   * @brief Processes a JSON-formatted read/write request for this node.
   * @param objP Pointer to JSON object with command parameters (optional).
   * @details Handles two scenarios:
   * - If objP == nullptr: Default read operation
   * - If objP != nullptr: Executes command from JSON (read/write)
   */
  void request(JsonObject* objP);
  /**
   * @brief Node-specific request handling (implemented by subclasses).
   */
  virtual void request() = 0;

protected:

  /**
   * @brief Sends an error notification.
   * @param errcode Error code from DIError::ErrCode.
   * @param detailed Additional error details (optional).
   */
  void sendError(DIError::ErrCode errcode, const char* detailed = nullptr);

  /**
   * @brief Node-specific initialization.
   * @note Must be implemented by derived classes.
   * @details Examples:
   * - Modbus: Configures register addresses
   * - GPIO: Sets pin modes
   */
  virtual void start() = 0;

  DataSource* m_source;
  string m_name;
  /**
   * @brief Unique index within the parent DataSource.
   * @details Used for:
   * - Ordering nodes during JSON serialization
   * - Debugging/logging identification
   * - Mapping responses to specific nodes
   */
  int m_index;
  /**
   * @brief Current command to execute (read/write/status).
   * @details Populated by decodeCmd() from JSON input.
   * @see decodeCmd()
   */
  Cmd m_cmd;
  /**
   * @brief Container for data values during read/write operations.
   * @details Stores:
   * - Values received from JSON for write commands
   * - Intermediate values before applying scaling/transformations
   * @note Uses type-safe conversions via convert<T>().
   */
  CustomVariant m_variant;
};


/*****************************************************************************************
 * @class DataSource
 * @brief Represents a data source (Modbus, CAN, I/O).
 * @ingroup DataIntegrationCore
 */
class DataSource {
  friend class Commm;
public:
  DataSource();
  virtual ~DataSource() = default;
  virtual const char* type() = 0;
  const string& name() { return m_name; }
  int index() { return m_index; }


  /**
   * @brief Constructs the data source from JSON configuration.
   * @param source JSON object with source parameters.
   * @param index Unique identifier for the source.
   * @return if value is negative, DIError::ErrCode failure code.
   * if value is positive, returns the ID which will be assigned to
   * the next object, be it a DataSource or a DataNode.
   * @details Parses:
   * - "name": Unique source identifier (required)
   * - "nodes": Array of data nodes (required)
   * @example
   * DataSource::build() returns 5 → Next node starts at index 5.
   * DataNode::build() returns -3 → Error: DIError::ErrCode::PARAM_OUT_BOUNDS.
   */
  virtual int build(JsonObject, int);

  /**
   * @brief Initializes the data source and starts its operation.
   * @details Creates a FreeRTOS task for periodic data polling.
   */
  virtual void start();

  /**
   * @brief Checks if the source has new data to send.
   * @return true if JSON document contains updated data.
   */
  bool needsUpdate();

  /**
   * @brief Retrieves current data state as JSON.
   * @param object JSON object to fill.
   */
  bool retrieve(JsonObject);

  /**
   * @brief Registers a callback for data responses.
   * @param name Node identifier.
   * @param ofcb Callback to populate JSON with node data.
   * @return true if node exists, false otherwise.
   */
  bool onResponse(string&, DataIntegrator::OnObjectFillCallback);

  /**
   * @brief Clears the internal JSON document.
   * @details Removes all node entries while keeping the source name.
   */
  void resetJSON();

  /**
   * @brief Triggers data collection from all nodes in this source.
   * @param objP Optional JSON object with specific read/write commands.
   */
  void request(JsonObject* objP = nullptr);

  /**
   * @brief Executes a command on a specific node.
   * @param obj JSON command structure.
   * @return DIError::ErrCode indicating result.
   * @details Command format:
   * @code{.json}
   * {
   *   "node": "node_name",
   *   "cmd": "read/write",
   *   "value": ... // Optional
   * }
   * @endcode
   */
  DIError::ErrCode cmd(JsonObject obj);

protected:
  JsonDocument* m_doc;
  bool m_started;
  vector<DataNode*> m_dnv;
  string m_name;

  /**
   * @brief Unique index for the source (internal tracking).
   * @details Used for:
   * - Ordering in JSON output
   * - Debugging/logging identification
   */
  int m_index;

private:

  /**
   * @brief Finds a data node by name within this source.
   * @param name Node identifier to search.
   * @param obj JSON command to apply.
   * @return DIError::ErrCode Result of the operation.
   */
  DIError::ErrCode findInDV(const string& name, JsonObject obj);

  Semaphore m_mutex;
  extTask* m_taskHandler;
};

/*class Commm {
  friend class DataNode;
  friend class DataSource;
  static inline void update(DataNode* obj, int id) { obj->update(id); }
  static inline void update(DataSource* obj, int id) { obj->update(id); }
};*/


/*****************************************************************************************
 * @class DataBus
 * @brief Central coordinator for all data sources.
 * @ingroup DataIntegrationCore
 * 
 * Usage example:
 * @code
 * DataBus bus;
 * bus.onSend([](const char* data, size_t len) { ... });
 * bus.startCom();
 * @endcode
 */
class DataBus {
public:
  DataBus();

  // State methods

  /**
   * @brief Starts all registered data sources and begins communication.
   * @note Creates FreeRTOS task for periodic data collection.
   */
  void startCom();
  /**
   * @brief Checks if the bus has been initialized.
   * @return true if startCom() was called successfully.
   */
  bool isStarted() { return m_started; }
  /**
   * @brief Temporarily suspends data collection/transmission.
   * @warning Leaves hardware interfaces active.
   */
  void suspend();
  /**
   * @brief Resumes operation after suspend().
   */
  void resume();
  /**
   * @brief Checks if the bus is actively processing data.
   * @return true between resume() and suspend() calls.
   */
  bool isRunning() { return m_running; }

  // Communication methods

  /**
     * @brief Sets callback for sending processed data.
     * @param dscb Callback with signature: void(const char* payload, size_t size)
     */
  void onSend(DataIntegrator::OnDataSendCallback dscb) { m_dscb = dscb; }

  /**
   * @brief Sends collected data through registered callback.
   * @param docP Pointer to JSON document with data.
   */
  void send(JsonDocument* docP);
  //bool receive(JsonDocument doc); // Gives linker error: undefined reference to `DataBus::receive(ArduinoJson::V704JB2::JsonDocument)'

  /**
   * @brief Processes incoming JSON commands.
   * @param payload Raw JSON data.
   * @param len Data length.
   * @return DIError::ErrCode indicating result.
   */
  DIError::ErrCode receive(const char*, size_t);

  /**
   * @brief Acknowledges data reception completion.
   * @details Resets internal buffers after transmission, to prepare for next transmission.
   * @note Clears sensitive data after sending.
   */
  void ack();

  // Json proccesing methods

  /**
   * @brief Triggers data collection from all sources.
   * @details Iterates through registered DataSources, calls their request() methods,
   *          and consolidates results into the internal JSON document.
   * @note Populates internal JSON document with fresh data.
   */
  void request();

  /**
   * @brief Clears all accumulated data in the JSON document.
   */
  void resetJSON();

  // Additional methods
  /**
     * @brief Adds a data source to the bus.
     * @param ds Configured DataSource pointer
     */
  void add(DataSource* ds);

  /**
     * @brief Executes a command from JSON object.
     * @param obj JSON command structure.
     * @return DIError::ErrCode execution result.
     */
  DIError::ErrCode cmd(JsonObject obj);

private:

  /**
   * @brief Finds a DataSource by name and executes a command on it.
   * @param name Name of the DataSource to search for.
   * @param obj JSON command to apply if found.
   * @return DIError::ErrCode Operation result.
   * @details 
   * - Iterates through m_dsv (DataSource vector)
   * - Compares names using item->name() == name
   * - Executes item->cmd(obj) on match
   */
  DIError::ErrCode findInDV(const string& name, JsonObject obj);

  JsonDocument* m_txDoc;
  bool m_started;
  bool m_running;
  vector<DataSource*> m_dsv;
  DataIntegrator::OnDataSendCallback m_dscb;
  extTask* m_taskHandler;
};


/*****************************************************************************************
 * @class DataBuilder
 * @brief Constructs the data integration system from JSON configuration.
 * @ingroup DataIntegrationCore
 */
class DataBuilder {
public:
  /**
   * @brief Initializes the builder with a JSON stream and target DataBus.
   * @param jsonFile Input stream containing JSON configuration.
   * @param databus Target DataBus to configure.
   * @details Parses JSON structure:
   * @code
   * [ // Root array
   *   { // DataSource 1
   *     "type": "rtu",
   *     "name": "ModbusDevice1",
   *     "nodes": [ ... ]
   *   }
   * ]
   * @endcode
   */
  DataBuilder(Stream&, DataBus&);
  ~DataBuilder() {}

  /**
   * @brief Gets the last integration error code.
   * @return DIError::ErrCode Error code from parsing/configuration.
   */
  DIError::ErrCode error() { return m_error; }

  /**
   * @brief Gets the JSON deserialization error details.
   * @return DeserializationError Specific parsing error.
   */
  DeserializationError desError() { return m_desError; }

private:
  DIError::ErrCode m_error;
  DeserializationError m_desError;
};

#endif // DATA_INTEGRATOR_H
