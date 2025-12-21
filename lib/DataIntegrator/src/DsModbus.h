/**
 * @file DsModbus.h
 * @brief Modbus RTU integration for industrial devices.
 */

#ifndef DS_MODBUS_H
#define DS_MODBUS_H

#include "DataIntegrator.h"

// Forward declarations
class ModbusProtocol;

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

  /// @brief Returns Modbus server id
  inline uint8_t serverId() { return m_id; }

  /// @brief Modbus protocol instance
  /// @return Protocol instance, for ModbusHAndled derived classes to manage
  ModbusProtocol *protocol();

private:
  int setOrdering(std::string&);
  uint8_t m_id;   ///< Modbus device ID (1-247)
  ModbusProtocol *m_modbus; ///< Ptotocol instance
  bool m_cptreq;
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
  virtual void onModbusResponse(ModbusMessage &msg) = 0;

  /**
   * @brief Initializes the ModbusManager linked to this node.
   */
  void start() override;
  inline uint32_t token() { return static_cast<uint32_t>(m_index); }
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
  void onModbusResponse(ModbusMessage &msg) override;

  const char* type() override { return "Register"; }
  static DataNode* Create() { return new MbHReg(); }

private:
  uint16_t m_address;       ///< Modbus register address
  AnalogCalc<>* m_calc;
  AnSendManager* m_sendman;
  std::string m_unit;
};

#endif // DS_MODBUS_H
