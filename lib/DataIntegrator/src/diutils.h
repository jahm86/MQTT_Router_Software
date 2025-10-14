
#ifndef DIUTILS_H
#define DIUTILS_H

#include <limits>
#include <ModbusClientRTU.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#ifdef USE_TOAD_ALLOCATOR
#include <ArduinoJson.h>
#endif

#define LEDC_MAX_CHANS 16


//************************************** Stream Link **************************************//

/**
 * @class StreamLink
 * @brief Manages shared communication interfaces (Modbus, CAN, etc.).
 * @note Singleton pattern to prevent multiple instances.
 */
class StreamLink {
public:
  // Forbidded operations
  StreamLink(StreamLink& other) = delete;
  void operator=(const StreamLink&) = delete;
  // Get this StreamLink instance
  static StreamLink& Instance();
  
  /**
   * @brief Replaces current Modbus client.
   */
  void SetNew(ModbusClientRTU* modbusClient);

  /**
   * @brief Gets Modbus client instance.
   */
  ModbusClientRTU* Get();

private:
  ModbusClientRTU* m_mbcrtu;
  static xSemaphoreHandle m_mutex;

protected:
  StreamLink();
  ~StreamLink();
};


//************************************** Helper for Task Management **************************************//

/**
 * @class Task
 * @brief Base class for FreeRTOS tasks requiring custom logic via inheritance.
 * @note Inherit from this class to create tasks with encapsulated behavior.
 * @details Task class inspired on https://fjrg76.wordpress.com/2018/05/20/objectifying-task-creation-in-freertos/
 */
class Task {
  public:

  /**
   * @brief Constructs a FreeRTOS task (abstract base).
   * @param name Task identifier for debugging.
   * @param stackDepth Stack size in bytes (use 4096+ for complex tasks).
   * @param priority Task priority (0 = lowest, configMAX_PRIORITIES-1 = 24 = highest).
   */
  Task(const char * const name, const uint32_t stackDepth, UBaseType_t priority) {
    xTaskCreate(task, name, stackDepth, this, priority, &m_taskHandle);
  }
  
  /**
   * @brief Pure virtual method with task's main logic.
   * @note Implement in subclasses for custom behavior.
   */
  virtual void Main() = 0;

  /**
   * @brief Suspends task execution.
   * @note Preserves task state for later resume.
   */
  void suspend() { vTaskSuspend(m_taskHandle); }
  
  /**
   * @brief Resumes a suspended task.
   */
  void resume() { vTaskResume(m_taskHandle); }

  /**
   * @brief Forces task to exit blocking delay.
   * @details Useful for immediate response to commands.
   */
  void abortTaskDelay () { xTaskAbortDelay(m_taskHandle); }

protected:

  /**
   * @brief FreeRTOS task entry point (do not call directly).
   * @param hiddenTask Pointer to Task instance (via static cast).
   * @details Implementation detail: Bridges C-style FreeRTOS API to C++ object model.
   */
  static void task(void* hiddenTask) {
    Task* taskInstance = static_cast<Task*>(hiddenTask);
    taskInstance->Main();
    vTaskDelete(taskInstance->m_taskHandle);
  }

  TaskHandle_t m_taskHandle;
};


/**
 * @class extTask
 * @brief Ready-to-use task implementation with callback support.
 * @details Avoids subclassing by accepting function objects/lambdas.
 */
class extTask : public Task {

  /**
   * @brief Callback signature for task logic.
   */
  typedef std::function<void()> TaskCallback;

public:

  /**
   * @brief Constructs a task with immediate logic.
   * @param name Task identifier.
   * @param stackDepth Stack size in bytes.
   * @param priority FreeRTOS priority.
   * @param taskcb Callback/lambda with task loop logic.
   * @warning Callback must handle its own infinite loop!
   * @example 
   * extTask* newtask = new extTask("LED", 1024, 1, []{ 
   *     while(1) { toggleLED(); vTaskDelay(1000); } 
   * });
   * 
   * then, you can call newtask->suspend(), newtask->resume(), etc...
   */
  extTask(const char * const name, const uint32_t stackDepth, UBaseType_t priority, TaskCallback taskcb) :
    Task(name, stackDepth, priority), m_taskcb(taskcb) {}

private:

  /**
   * @brief Inherited from Task - delegates to m_taskcb.
   */
  void Main() override { m_taskcb(); }
  
  TaskCallback m_taskcb;
};


//************************************** Scoped Lockguard **************************************//

/**
 * @class LockGuard
 * @brief RAII wrapper for mutex/semaphore management.
 * @tparam SemaphT Type of semaphore (xSemaphoreHandle by default)
 * @details This lock guard will lock with semaphore SemaphT until LockGuard instance goes out of scope
 * @warning Please, avoid use of anonimous declaration for this class (declare "LockGuard<xSemaphoreHandle> lg(m_mutex);"
 * instead of "LockGuard<xSemaphoreHandle> (m_mutex);"), otherwise the scope will dissapear in the same line,
 * and also the mutex locking
 */
template<typename SemaphT>
class LockGuard {
public:

  /**
   * @brief Locks semaphore on creation.
   * @param s Reference or pointer to semaphore
   */
  LockGuard(SemaphT& s) : m_s(s) { lock(); }
  LockGuard(SemaphT* s) : m_s(*s) { lock(); }

  /**
   * @brief Unlocks semaphore on destruction.
   */
  ~LockGuard() { xSemaphoreGive(m_s); }
  //~LockGuard() { log_d("Unlocking"); xSemaphoreGive(m_s); }

private:
  //inline void lock() { xSemaphoreTake(m_s, portMAX_DELAY); log_d("Locked");  }
  inline void lock() { xSemaphoreTake(m_s, portMAX_DELAY); }
  LockGuard(const LockGuard&);
  SemaphT& m_s;
};


//************************************** Scoped Allocator **************************************//

/**
 * @class ScopedAllocator
 * @brief RAII-style memory allocator for fixed-size buffers.
 * @tparam typeT Data type to allocate.
 * @details This allocator deallocate memory heap of type typeT when its instance goes out of scope
 * @warning Memory is freed automatically when out of scope.
 * @warning Please, avoid to use anonimous declaration for this class (declare "ScopedAllocator<char> alloc(size);", then
 * "char* ptr = alloc.pointer();",instead of "char* ptr = ScopedAllocator<char>(size).pointer();"), otherwise the
 * scope will dissapear in the same line, and also the memory allocated
 */
template<typename typeT>
class ScopedAllocator {
public:

  /**
   * @param size Number of elements to allocate.
   */
  ScopedAllocator(size_t size) { m_ptr = malloc(size * sizeof(typeT)); }
  ~ScopedAllocator() {if (m_ptr != nullptr) { free(m_ptr); } }

  /**
   * @brief Access to allocated memory.
   */
  inline typeT* pointer() { return static_cast<typeT*>(m_ptr); }

private:
  void* m_ptr;
};


//************************************** A Gossip Allocator :) **************************************//

/**
 * @class Toadllocator
 * @brief Memory debugger for ESP32 heap management.
 * @details Enabled with -DUSE_TOAD_ALLOCATOR flag. Monitors:
 * - Heap integrity (corruption detection)
 * - Current/Maximum/Minimum free heap
 * - Allocation/Deallocation patterns
 * @example
 * In platformio.ini:
 * build_flags = -DUSE_TOAD_ALLOCATOR
 * 
 * In code:
 * Toadllocator::check(__FILE__, __LINE__); // Heap report
 */
#ifdef USE_TOAD_ALLOCATOR
class Toadllocator : public Allocator {
#else
class Toadllocator {
#endif
private:
  Toadllocator() = default;
  ~Toadllocator() = default;

public:
#ifdef USE_TOAD_ALLOCATOR

  static Allocator* instance(const char * srcFileName, int srcLineNumber);

  /**
   * @brief Tracks memory allocation for leak detection.
   * @param size Bytes requested
   * @param srcFile File where allocation occurred
   * @param srcLine Line number in source
   */
  void* allocate(size_t size) override;

  /**
   * @brief Verifies memory integrity on deallocation.
   * @param ptr Pointer to memory block
   */
  void deallocate(void* ptr) override;

  void* reallocate(void* ptr, size_t new_size) override;

#endif

  /**
   * @brief Generates heap status report with ESP32 functions.
   * @param srcFileName File requesting check
   * @param srcLineNumber Line number
   */
  static void check(const char * srcFileName, int srcLineNumber);
};


//************************************** Helper class for PWM channel assigment in ESP32 **************************************//

/**
 * @class LedcChan
 * @brief Manages ESP32 LEDC PWM channel allocation.
 * @warning Limited to 16 channels total (ESP32 hardware limit).
 */
class LedcChan {
public:
  static LedcChan& Instance();

  /**
   * @brief Acquires an available PWM channel.
   * @return Channel number (0-15) or -1 if full.
   */
  int addChannel();

  /**
   * @brief Releases a PWM channel for reuse.
   * @param channel Channel to free (0-15)
   */
  void removeChannel(int);
private:
  LedcChan();
  ~LedcChan() = default;
  uint32_t m_busyChans; // Bitmask for tracking (1 << channel)
  static xSemaphoreHandle m_mutex;  // Semaphore for thread safety
  // Delete the copy and move constructors
  LedcChan(const LedcChan&) = delete;
  LedcChan& operator=(const LedcChan&) = delete;
  LedcChan(LedcChan&&) = delete;
  LedcChan& operator=(LedcChan&&) = delete;
};


//************************************** Class to get Time in Seconds **************************************//

/**
 * @class TimeCounter
 * @brief Tracks elapsed time in seconds using FreeRTOS ticks.
 * @note Singleton pattern for system-wide time management.
 */
class TimeCounter {
public:
  // Forbidded operations
  TimeCounter(TimeCounter& other) = delete;
  void operator=(const TimeCounter&) = delete;

  /**
   * @brief Gets single TimeCounter instance.
   */
  static TimeCounter& Instance();

  /**
   * @brief Gets current uptime in seconds.
   */
  uint32_t getTimeSecs();

private:
  // Task to measure 1 second delay, then add it to a time counter
  static void TickTask (void*);
  TimeCounter();
  ~TimeCounter() {}

  static uint32_t m_timeInSecs;
  static xSemaphoreHandle m_mutex;
};


//************************************** Class for Send Arbitration Management **************************************//

/**
 * @class SendManager
 * @brief Base class for data transmission policies.
 * @details Determines when to send updates based on time/value changes.
 */
class SendManager {
public:

  /**
   * @brief Constructs a SendManager with time-based constraints.
   * @param min_send_time Minimum interval between sends (seconds).
   * @param max_send_time Maximum interval even with no changes (seconds).
   */
  SendManager(uint32_t min_send_time, u_int32_t max_send_time);

 protected:

 /**
   * @brief Checks if data should be sent.
   * @param force Bypass all checks if true.
   * @return true if transmission is required.
   */
  bool updateRequired(bool force);

  bool m_changed;

private:
  uint32_t m_min_time;
  uint32_t m_max_time;
  uint32_t m_last_time;
};

/**
 * @class AnSendManager
 * @brief Transmission policy for analog values (delta-based triggering).
 */
class AnSendManager : public SendManager {
public:

  /**
   * @brief Constructs an analog send manager.
   * @param min_send_time Minimum send interval (seconds).
   * @param max_send_time Maximum send interval (seconds).
   * @param delta Minimum value change to trigger send (e.g., 0.5°C).
   */
  AnSendManager(uint32_t min_send_time, u_int32_t max_send_time, float delta) :
  SendManager(min_send_time, max_send_time), m_var_delta(delta), m_last_value(0) {}

  /**
   * @brief Checks if analog value meets send criteria.
   * @param current_value Latest sensor reading.
   * @param force Force transmission if true.
   * @return true if:
   * - Value change >= delta
   * - max_send_time expired
   * - Force flag set
   */
  bool updateRequired(float value, bool force);

protected:
  float m_var_delta;
  float m_last_value;
};

/**
 * @class DgSendManager
 * @brief Transmission policy for digital values (state change triggering).
 */
class DgSendManager : public SendManager {
public:

  /**
   * @brief Constructs a digital send manager.
   * @param min_send_time Minimum send interval (seconds).
   * @param max_send_time Maximum send interval (seconds).
   */
  DgSendManager(uint32_t min_send_time, u_int32_t max_send_time) :
  SendManager(min_send_time, max_send_time), m_last_value(false) {}

  /**
   * @brief Checks if digital state changed.
   * @param current_value Current state (true/false).
   * @param force Force transmission if true.
   * @return true if:
   * - State changed
   * - max_send_time expired
   * - Force flag set
   */
  bool updateRequired(bool value, bool force);

protected:
  bool m_last_value;
};


//************************************** Helper for Analog Calculus **************************************//

/**
 * @class AnalogCalc
 * @brief Base class for analog value transformations. Converts between raw hardware values (e.g., ADC readings) and engineering units.
 * @details Designed for both linear and non-linear systems:
 * - Linear: @ref LinearCalc (y = gain * x + offset)
 * - Non-linear: Custom implementations (e.g., thermocouples, Hall sensors)
 * @note Subclasses must implement send() and receive() methods.
 */
class AnalogCalc {
public:

  /**
   * @brief Transforms raw value to physical unit, to send outside.
   * @param value Raw input (e.g., 12-bit ADC reading).
   * @return Converted value (e.g., 25.4°C).
   */
  virtual float send(int value) = 0;

  /**
   * @brief Transforms physical unit from outside to raw value.
   * @param value Engineering value (e.g., 100.5 RPM).
   * @return Raw output for hardware (e.g., Modbus register value).
   */
  virtual int receive(float value) = 0;

protected:

  /**
   * @brief Constructor for analog calculators with value limiting.
   * @param max Maximum allowed raw value (e.g., 4095 for 12-bit ADC).
   * @param min Minimum allowed raw value (usually 0).
   * @note For subclass use only.
   */
  AnalogCalc(int max, int min) : m_min(min), m_max(max) {}

  /**
   * @brief Clamps raw values to [m_min, m_max] range.
   * @param value Raw input from hardware.
   * @return Value constrained between min and max.
   */
  inline int limiter(int value) {
    return value > m_max ? m_max : value < m_min ? m_min : value;
  }

private:
  int m_min;
  int m_max;
};


//************************************** Helper for Analog Linear Calculus **************************************//

/**
 * @class LinearCalc
 * @brief Applies linear transformation: y = gain * x + offset
 */
class LinearCalc : public AnalogCalc {
public:

  /**
   * @brief Creates a linear converter.
   * @param offset Zero adjustment (e.g., sensor calibration).
   * @param gain Scaling factor (e.g., 0.1 for 10:1 scaling).
   * @param max Maximum raw value (default: max of data type).
   * @param min Minimum raw value (default: 0).
   * @example 
   * LinearCalc(2.5, 0.02)       // y = 0.02x + 2.5
   * LinearCalc(-40, 0.1, 1023)  // For 10-bit sensor (-40°C at 0)
   */
  LinearCalc(float offset, float gain, int max = INT_MAX, int min = 0) :
  m_offset(offset), m_gain(gain), AnalogCalc(max, min) {}

  virtual float send(int value) {
    return (value - m_offset) / m_gain;
  }

  virtual int receive(float value) {
    return limiter((value * m_gain) + m_offset);
  }

private:
  float m_offset;
  float m_gain;
};


//************************************** Helper for Digital Calculus **************************************//

/**
 * @class DigitalCalc
 * @brief Handles digital signal inversion and conversion.
 * @example
 * DigitalCalc calc(true); // Inverted logic
 * bool bVal = calc.send(HIGH); // Returns false
 */
class DigitalCalc {
public:

  /**
   * @param invert Enables logic inversion if true.
   */
  DigitalCalc (bool inv) : m_invert(inv) {}

  /**
   * @brief Converts digital state to boolean, to send outside.
   * @param value Hardware state (HIGH/LOW).
   */
  bool send(uint8_t value);

  /**
   * @brief Converts boolean from outside to hardware state.
   * @param value Logical state (true/false).
   */
  uint8_t receive(bool value);

private:
  inline bool condInverter(bool value) {
    return m_invert ? !value : value;
  }
  bool m_invert;
};


//************************************** Custom Variant **************************************//

//class JsonVariant;
#include <ArduinoJson.h>

/**
 * @class CustomVariant
 * @brief Safe container for multiple data types with type checking.
 * @warning Uses union for storage - do not use with complex types!
 */
class CustomVariant {
public:
  CustomVariant() : b(0), m_alternative(alt_t::t_boolean) {}
  CustomVariant( const CustomVariant& other ) { convertFromOther(other); }

  void operator=( const CustomVariant other ) { convertFromOther(other); }

  CustomVariant( const JsonVariant src ) { convertFromVariant(src); }

  void operator=( const JsonVariant src ) { convertFromVariant(src); }

  /**
   * @brief Checks if stored value can be converted to type T.
   * @tparam T Target type (bool, int, float, std::string)
   */
  template<typename T>
  bool can_convert();

  /**
   * @brief Converts stored value to type T with safety checks.
   * @tparam T Target type
   * @return Converted value or default if type mismatch.
   * @throws Internal error if type is invalid (debug builds only).
   */
  template<typename T>
  const T& convert();

  ~CustomVariant() { s.~basic_string(); }

private:
  void convertFromVariant( const JsonVariant src );
  void convertFromOther( const CustomVariant& other );
  union {
    bool b;
    int i;
    float f;
    std::string s;
  };

  enum class alt_t {
    t_boolean, t_float, t_string, t_notype
  };

  alt_t m_alternative;
};

/**
 * @brief Converts a hexadecimal string to byte array.
 * @param hexString Input string (e.g., "A1B2C3").
 * @param byteArray Output buffer for converted bytes.
 * @param size Maximum bytes to convert (buffer size).
 * @return true if conversion succeeded, false otherwise.
 * @details 
 * - Supports uppercase/lowercase hex (0-9, A-F, a-f)
 * - Ignores non-hex characters (colons, spaces, etc.)
 * - Fails if valid hex chars < 2*size or invalid chars present
 * @warning Ensure byteArray has >= size capacity.
 * @example 
 * uint8_t buf[3];
 * stringHex2ByteArray("A1:B2-C3", buf, 3); // buf = {0xA1, 0xB2, 0xC3}
 */
bool stringHex2ByteArray(String hexArray, uint8_t* byteArray, size_t size);

#endif // DIUTILS_H
