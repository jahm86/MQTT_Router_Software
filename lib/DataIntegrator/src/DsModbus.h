/**
 * @file DsModbus.h
 * @brief Modbus RTU integration for industrial devices.
 */

#ifndef DS_MODBUS_H
#define DS_MODBUS_H

#include "DataIntegrator.h"
#include "diutils.h"

using namespace std;

#define MAX_NODE_REQUESTS 3

/**
 * @class DsModbus
 * @brief DataSource for Modbus RTU devices.
 * 
 * @details Configures global Modbus client parameters:
 * - Slave device ID
 * - Byte order (big/little-endian)
 * - Request compaction
 */
class DsModbus : public DataSource {
public:
  virtual ~DsModbus() {}

  /**
   * @brief Builds the DataSource from JSON configuration.
   * @param source JSON configuration object:
   * @code{.json}
   * {
   *   "id": 1,              // Modbus device ID (1-247)
   *   "order": "little",    // Byte order
   *   "compactRequest": true
   * }
   * @endcode
   * @return if value is negative, DIError::ErrCode failure code.
   * if value is positive, returns the ID which will be assigned to
   * the next object, be it a DataSource or a DataNode.
   */
  int build(JsonObject, int) override;

  /// @brief Returns "ModbusRTU" for type identification
  const char* type() override { return "ModbusRTU"; }

  /// @brief Factory method to create a new instance
  static DataSource* Create() { return new DsModbus(); }

  inline uint8_t serverId() { return m_id; }

private:
  int setOrdering(string&);
  uint8_t m_id;   ///< Modbus device ID (1-247)
  bool m_cptreq;
};


/**
 * @class ModbusManager
 * @tparam DataSourceT DataSource type (must be DsModbus)
 * @tparam DataNodeT DataNode type (e.g., MbHReg)
 * @brief Manages Modbus request queuing and response handling.
 * 
 * @details Handles Modbus request queueing and response processing. Features:
 * - Limits concurrent requests per node (MAX_NODE_REQUESTS)
 * - Maps responses to nodes using tokens (m_index)
 * - Thread-safe operations with semaphores
 */
template<class DataSourceT, class DataNodeT>
class ModbusManager {
public:
  ModbusManager(DataSourceT* dsInstance, DataNodeT* modInstance);

  /**
   * @brief Adds a Modbus request to the queue.
   * @param funcCode Modbus function code (READ_HOLD_REGISTER, etc.)
   * @param args Function-specific arguments (address, data, etc.)
   * @return Modbus::Error code (SUCCESS if queued successfully)
   * 
   * @example 
   * addRequest(READ_HOLD_REGISTER, 40001, 1); // Read 1 register at 40001
   */
  template <typename... Args>
  Modbus::Error addRequest(Modbus::FunctionCode funcCode, Args&&... args);

  /**
   * @brief Static function for installing Modbus callback.
   * @todo evaluate if needs to be static
   */
  static void setResponseHandler();

private:
  DataSourceT* m_dsInstance;  ///< Bound DsModbus instance
  DataNodeT* m_nodeInstance;  ///< Target DataNode for response

  /**
   * @struct Busy_t
   * @brief Tracks active requests count per node.
   */
  struct Busy_t {
    uint8_t read;   ///< Active read requests (max = MAX_NODE_REQUESTS)
    uint8_t write;  ///< Active write requests (max = MAX_NODE_REQUESTS)
  };

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
  static bool opIsRead(Modbus::FunctionCode funcCode);

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
  static bool opIsWrite(Modbus::FunctionCode funcCode);

  static Semaphore m_mutex;                        ///< Semaphore for thread safety
  static unordered_map<uint16_t, Busy_t> busy_map;        ///< Tracks node's active requests
  static unordered_map<uint16_t, ModbusManager*> sm_map;  ///< Token-to-Manager mapping
  static bool respHandlerIsSet;

  /**
  * @brief Static callback for processing Modbus responses. Handles incoming Modbus responses and routes them to nodes.
  * @param msg Modbus response message.
  * @param token Node identifier (m_index of target node).
  * @details Workflow:
  * 1. Finds ModbusManager instance via token in sm_map.
  * 2. Updates busy_map request counters.
  * 3. Delegates message processing to node's onModbusResponse().
  */
  static void onModbusResponse(ModbusMessage msg, uint32_t token);
};


/**
 * @class ModbusHAndled
 * @brief Abstract base class for Modbus response handlers.
 * 
 * @details Subclasses must implement:
 * - onModbusResponse(): Processes received Modbus data/errors
 * - request(): Handles read/write operations
 */
class ModbusHAndled : public DataNode {
public:

  /**
   * @brief Processes a Modbus response (pure virtual).
   * @param msg Modbus message containing data or error
   */
  virtual void onModbusResponse(ModbusMessage msg) = 0;

  /**
   * @brief Initializes the ModbusManager linked to this node.
   */
  void start() override;
  inline uint32_t token() { return static_cast<uint32_t>(m_index); }

protected:
  ModbusManager<DsModbus, ModbusHAndled>* m_manager;  ///< Associated Modbus manager
};


/**
 * @class MbHReg
 * @brief Node for Modbus Holding Registers (16-bit read/write).
 * 
 * @example JSON configuration:
 * @code{.json}
 * {
 *   "name": "Temperature",
 *   "type": "HREG",
 *   "address": 40001,
 *   "gain": 0.1,
 *   "offset": -20.0,
 *   "unit": "°C"
 * }
 * @endcode
 */
class MbHReg : public ModbusHAndled {
public:
  virtual ~MbHReg();

  int build(JsonObject, int);

  /**
   * @brief Handles read/write requests for a Modbus Holding Register.
   * @details Behavior depends on the current command (m_cmd):
   * - READ/READ_FORCED: Sends Modbus read request for the register.
   * - WRITE: Validates input value, converts it to raw format, and sends write request.
   * 
   * @note Uses ModbusManager to queue requests and handle concurrency.
   */
  void request() override;

  /**
   * @brief Processes Holding Register responses.
   * @param msg Modbus response message
   */
  void onModbusResponse(ModbusMessage msg) override;

  const char* type() override { return "Register"; }
  static DataNode* Create() { return new MbHReg(); }

private:
  uint16_t m_address;       ///< Modbus register address
  AnalogCalc* m_calc;
  AnSendManager* m_sendman;
  string m_unit;
};

#endif // DS_MODBUS_H
