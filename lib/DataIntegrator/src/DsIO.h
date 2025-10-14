/**
 * @file DsIO.h
 * @brief DataSource implementation for ESP32 GPIO and PWM.
 */

#ifndef DS_IO_H
#define DS_IO_H

#include "DataIntegrator.h"
#include "diutils.h"

using namespace std;


/**
 * @class DsIO
 * @brief DataSource for digital/analog I/O operations.
 */
class DsIO : public DataSource {
public:
  virtual ~DsIO() {}

  const char* type() override { return "Input/Output"; }
  static DataSource* Create() { return new DsIO(); }
};


/**
 * @class AnIn
 * @brief Analog input node (e.g., ADC readings).
 * @details Configures ADC pins and applies linear scaling.
 */
class AnIn : public DataNode {
public:
  virtual ~AnIn();

  /**
   * @brief Configures the node from JSON parameters.
   * @param node JSON configuration:
   * @code{.json}
   * {
   *   "pin": 34,           // ADC pin (GPIO32-39)
   *   "bits": 12,          // ADC resolution (9-12 bits)
   *   "gain": 0.1,         // Scaling factor (e.g., 0.1 V/bit)
   *   "offset": 0.5,       // Zero adjustment
   *   "mintime": 5,        // Minimum send interval (seconds)
   *   "maxtime": 60,       // Maximum send interval
   *   "delta": 0.5         // Minimum change to trigger send
   * }
   * @endcode
   */
  int build(JsonObject, int) override;

  /**
   * @brief Initializes ADC settings and resolution.
   */
  void start() override;

  /**
   * @brief Reads ADC value and triggers send if criteria met.
   */
  void request() override;

  const char* type() override { return "AnalogInput"; }
  static DataNode* Create() { return new AnIn(); }

private:
  uint8_t m_pin;            ///< GPIO pin number (e.g., GPIO34)
  uint8_t m_bits;           ///< ADC resolution (e.g., 12 bits)
  AnalogCalc* m_calc;       ///< Scaling calculator (raw → engineering units)
  string m_unit;            ///< Engineering unit (e.g., "V", "°C")
  AnSendManager* m_sendman; ///< Send policy manager
};


/**
 * @class DgIn
 * @brief Node for digital input monitoring with debouncing.
 */
class DgIn : public DataNode {
public:
  virtual ~DgIn();

  /**
   * @brief Configures the node from JSON parameters.
   * @param node JSON configuration:
   * @code{.json}
   * {
   *   "pin": 25,           // GPIO pin
   *   "inv": true,         // Logic inversion (true = active low)
   *   "mintime": 1,        // Debounce time (seconds)
   *   "maxtime": 300       // Max send interval
   * }
   * @endcode
   */
  int build(JsonObject, int) override;

  /**
   * @brief Sets pin mode to INPUT.
   */
  void start() override;

  /**
   * @brief Reads GPIO state and checks for changes.
   */
  void request() override;

  const char* type() override { return "DigitalInput"; }
  static DataNode* Create() { return new DgIn(); }

private:
  uint8_t m_pin;            ///< GPIO pin number
  DigitalCalc* m_calc;      ///< Logic inversion handler
  DgSendManager* m_sendman; ///< Debounce and timing manager
};


/**
 * @class DgOut
 * @brief Node for digital output control (e.g., relays, LEDs).
 */
class DgOut : public DataNode {
public:
  virtual ~DgOut();

  /**
   * @brief Configures the node from JSON parameters.
   * @param node JSON configuration:
   * @code{.json}
   * {
   *   "pin": 26,           // GPIO pin
   *   "init": true,        // Initial state (true = HIGH)
   *   "inv": false         // Hardware inversion (true = active low)
   * }
   * @endcode
   */
  int build(JsonObject, int) override;

  /**
   * @brief Sets pin mode to OUTPUT and writes initial state.
   */
  void start() override;

  /**
   * @brief Handles write commands and state confirmation.
   */
  void request() override;

  const char* type() override { return "DigitalOutput"; }
  static DataNode* Create() { return new DgOut(); }

private:
  uint8_t m_pin;            ///< GPIO pin (e.g., GPIO25)
  DigitalCalc* m_calc;      ///< Logic inversion handler
  bool m_init_state;        ///< Initial state at startup
  DgSendManager* m_sendman; ///< Send policy manager
};


/**
 * @class PWM
 * @brief Node for PWM signal generation with LEDC peripheral.
 */
class PWM : public DataNode {
public:
  virtual ~PWM();

  /**
   * @brief Configures the node from JSON parameters.
   * @param node JSON configuration:
   * @code{.json}
   * {
   *   "pin": 27,           // GPIO pin
   *   "freq": 5000,        // PWM frequency (Hz)
   *   "bits": 8,           // Duty resolution (1-16 bits)
   *   "init": 50.0,        // Initial duty cycle (%)
   *   "gain": 2.55,        // Scaling factor (%/bit)
   *   "offset": 0.0        // Duty cycle offset
   * }
   * @endcode
   */
  int build(JsonObject, int) override;

  /**
   * @brief Allocates LEDC channel and configures PWM.
   */
  void start() override;

  /**
   * @brief Updates duty cycle and manages send policy.
   */
  void request() override;

  const char* type() override { return "PWM"; }
  static DataNode* Create() { return new PWM(); }

private:
  uint8_t m_pin;            ///< GPIO pin number
  AnalogCalc* m_calc;       ///< Scaling calculator (% → raw duty)
  uint16_t m_freq;          ///< PWM frequency (Hz)
  uint8_t m_bits;           ///< Duty resolution (e.g., 8 bits = 0-255)
  int8_t m_channel;         ///< Allocated LEDC channel (-1 if invalid)
  float m_init_value;       ///< Initial duty cycle (engineering units)
  string m_unit;            ///< Unit (default "%")
  AnSendManager* m_sendman; ///< Send policy manager
};

#endif // DS_IO_H
