
#ifndef DIUTILS_H
#define DIUTILS_H

#include <limits>
#include <vector>
#include <unordered_map>
#include <type_traits>
#include <memory>
#include <string>
#include <ModbusClientRTU.h> // TODO: Delete this, when DsModbus updated with new functionality
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#ifdef USE_TOAD_ALLOCATOR
#include <ArduinoJson.h>
#endif

#define LEDC_MAX_CHANS 16


//************************************** Scoped Lockguard **************************************//

// Forward declaration
class LockGuard;

class Semaphore {
private:
  enum class SemaphoreType {
    MUTEX,
    RECURSIVE_MUTEX,
    BINARY_SEMAPHORE,
    COUNTING_SEMAPHORE
    // More types, if needed...
  };
  xSemaphoreHandle m_handle;
  SemaphoreType m_type;

  Semaphore(SemaphoreType type, UBaseType_t max_count = 0, UBaseType_t initial_count = 0);

public:

  // Static creation methods
  static Semaphore create_mutex() { return Semaphore(SemaphoreType::MUTEX); }
  static Semaphore create_recursive_mutex() { return Semaphore(SemaphoreType::RECURSIVE_MUTEX); }
  static Semaphore create_binary_semaphore() { return Semaphore(SemaphoreType::BINARY_SEMAPHORE); }
  static Semaphore create_counting_semaphore(UBaseType_t max_count, UBaseType_t initial_count) {
    return Semaphore(SemaphoreType::COUNTING_SEMAPHORE, max_count, initial_count);
  }

  // Destructor for semaphore
  ~Semaphore();

  // No copy
  Semaphore(const Semaphore&) = delete;
  Semaphore& operator=(const Semaphore&) = delete;

  // Move
  Semaphore(Semaphore&& other) noexcept;
  Semaphore &operator=(Semaphore &&other) noexcept;

  friend LockGuard;
};

/**
 * @class LockGuard
 * @brief RAII wrapper for mutex/semaphore management.
 * @details This lock guard will lock with semaphore SemaphT until LockGuard instance goes out of scope
 * @warning Please, avoid use of anonimous declaration for this class (declare "LockGuard lg(m_mutex);"
 * instead of "LockGuard (m_mutex);"), otherwise the scope will dissapear in the same line,
 * and also the locking
 */
class LockGuard {
private:
  Semaphore& m_s;

public:

  /**
   * @brief Locks semaphore on creation with optional timeout.
   * @param semaphore Reference to semaphore
   * @param timeout Maximum time to wait (portMAX_DELAY by default, which means wait for ever)
   */
  LockGuard(Semaphore& semaphore, TickType_t timeout = portMAX_DELAY);

  /**
   * @brief Unlocks semaphore on destruction.
   */
  ~LockGuard();

  // Avoids temporaries
  LockGuard(Semaphore&& semaphore, TickType_t timeout = portMAX_DELAY) = delete;
  // No copy/move operators
  LockGuard(const LockGuard&) = delete;
  LockGuard& operator=(const LockGuard&) = delete;
  LockGuard(LockGuard&&) = delete;
  LockGuard& operator=(LockGuard&&) = delete;
};


//************************************** Conversion templates **************************************//

template<typename T>
T convertTo(const std::string& value);

template<typename T>
const std::string convertFrom(T value);


// Forward declaration
class BaseProtocol;

// Default value for pair, meaning not existing
#define def_value {nullptr, false}


//************************************** ProtocolEntry **************************************//

/**
 * @struct ProtocolEntry
 * @brief Entry in the registry containing both configuration and protocol instance
 */
struct ProtocolEntry {
  std::unordered_map<std::string, std::string> config;  ///< Configuration parameters
  BaseProtocol* protocol;                               ///< Protocol instance (optional)
  uint16_t referenceCount;                              ///< Number of users
  bool autoManaged;                                     ///< If true, registry manages lifecycle

  ProtocolEntry() 
    : protocol(nullptr), referenceCount(0), autoManaged(false) {}

  explicit ProtocolEntry(BaseProtocol* proto, bool managed = false)
    : protocol(proto), referenceCount(1), autoManaged(managed) {}
};


//************************************** ConfigRegistry **************************************//

/**
 * @class ConfigRegistry
 * @brief Register and manages configuration parameters for different communication interfaces
 * (Modbus, CAN, etc.).
 * @note Singleton pattern to prevent multiple instances.
 */
class ConfigRegistry {
private:
  std::unordered_map<std::string, ProtocolEntry> registry; ///< Registry: protocolKey -> ProtocolEntry

  Semaphore m_mutex; ///< Mutex for thread safety

  // Private constructor/destructor for singleton
  ConfigRegistry() : m_mutex(Semaphore::create_mutex()) {};
  ~ConfigRegistry() {
    cleanup();
  }

  // Instance
  static ConfigRegistry& getInstance();

  // Cleanup managed protocols
  void cleanup();

public:
  // ========== CONFIGURATION MANAGEMENT ==========

  /**
   * @brief Register a protocol key without an instance
   */
  static bool registerProtocol(const std::string& protocolKey);

  /**
   * @brief Register or update a protocol with an instance
   * @param protocolKey The protocol identifier
   * @param protocol Pointer to protocol instance
   * @param autoManaged If true, registry takes ownership and deletes on cleanup
   * @return true if successful, false if key already exists with a different instance
   */
  static bool registerProtocol(const std::string& protocolKey, 
                               BaseProtocol* protocol, 
                               bool autoManaged = false);

  // ========== INSTANCE MANAGEMENT ==========

  /**
   * @brief Get protocol instance (nullptr if not registered)
   */
  static std::pair<BaseProtocol*, bool> getProtocol(const std::string& protocolKey);

  /**
   * @brief Check if a protocol instance is registered
   */
  static bool hasProtocol(const std::string& protocolKey);

  /**
   * @brief Release a reference to a protocol
   * @return true if protocol was removed (referenceCount == 0)
   */
  static bool releaseProtocol(const std::string& protocolKey);

  /**
   * @brief Remove a protocol entry completely
   */
  static bool removeProtocol(const std::string& protocolKey);

  // ========== CONFIGURATION METHODS ==========

  /**
   * @brief Register/change a configuration parameter
   */
  static bool setConfig(const std::string& protocolKey,
                       const std::string& paramKey,
                       const std::string& paramValue);

  template<typename T>
  static bool setConfigAs(const std::string& protocolKey, 
                       const std::string& paramKey, 
                       T paramValue) {
    return setConfig(protocolKey, paramKey, convertFrom<T>(paramValue));
  }

  static bool setConfig(const std::string& protocolKey,
                       const std::string& paramKey,
                       int paramValue) {
    return setConfigAs<int>(protocolKey, paramKey, paramValue);
  }

  static bool setConfig(const std::string& protocolKey, 
                       const std::string& paramKey, 
                       float paramValue) {
    return setConfigAs<float>(protocolKey, paramKey, paramValue);
  }

  static bool setConfig(const std::string& protocolKey, 
                       const std::string& paramKey, 
                       bool paramValue) {
    return setConfigAs<bool>(protocolKey, paramKey, paramValue);
  }
                       
  static std::string getConfig(const std::string& protocolKey, 
                              const std::string& paramKey, 
                              const std::string& defaultValue = "");

  template<typename T>
  static T getConfigAs(const std::string& protocolKey, 
                      const std::string& paramKey, 
                      T defaultValue = T()) {
    std::string value = getConfig(protocolKey, paramKey);
    if (value.empty()) return defaultValue;
  
    // Conversion based on type
    try {
      return convertTo<T>(value);
    } catch (...) {} // Do nothing, just return the default
    return defaultValue;
  }

  static bool hasConfig(const std::string& protocolKey, 
                       const std::string& paramKey = "");

  // ========== UTILITY METHODS ==========

  static std::vector<std::string> listProtocols();
  static void clear();
  static void debugPrint();
};


//************************************** BaseProtocol **************************************//

/**
 * @class BaseProtocol
 * @brief Base class to build multiple communication interfaces (Modbus, CAN, etc.).
 */
class BaseProtocol {
protected:
  std::string m_key;          ///< "[DATATYPE]@[Instance]", where DATATYPE = ("CANBUS", "RTU", "TCP", etc...) and [0 <= Instance < <uint8_t>::max()]
  bool m_configured = false;  ///< Protocol is configured?
  bool m_started = false;     ///< Protocol is started?
  Semaphore m_rec_mutex;      ///< Recursive mutex
  
  // Key parse methods
  static std::pair<std::string, uint8_t> parseKey(const std::string& key) {
    uint8_t no_value = std::numeric_limits<uint8_t>::max();
    size_t atPos = key.find('@');
    if (atPos == std::string::npos) {
      return {key, no_value}; // No instance, instance 0 by default
    }
    
    std::string type = key.substr(0, atPos);
    std::string instanceStr = key.substr(atPos + 1);
    
    try {
      uint8_t instance = static_cast<uint8_t>(std::stoi(instanceStr));
      return {type, instance};
    } catch (...) {
      return {type, no_value};
    }
  }
  
  // Protected constructor
  BaseProtocol(const std::string& key)
    : m_key(key), m_rec_mutex(Semaphore::create_recursive_mutex()) {
    
    // Auto registry with no instance
    ConfigRegistry::registerProtocol(key);
  }
  
public:
  virtual ~BaseProtocol() = default;
  
  // ========== TYPE AND VERIFICATION METHODS ==========
  
  /**
   * @brief Gets the type of protocol (ej: "twai", "rtu", "tcp")
   */
  const std::string getProtocolType() const { return parseKey(m_key).first; }
  
  /**
   * @brief Gets the instance number
   */
  uint8_t getInstanceNumber() const { return parseKey(m_key).second; }
  
  /**
   * @brief Verify if this protocol is compatible with given type
   * @param expectedType Expacted type (ej: "TWAI", "CANBUS", "MODBUS_RTU")
   */
  bool isType(const std::string& expectedType) const {
    return expectedType == getProtocolType();
  }

  // ========== PUBLIC SATIC METHODS ==========
  
  /**
   * @brief Gets an existing instance with type verification
   * @param key Protocol key (ej: "twai@0")
   * @param expectedType Expected type (for verification)
   * @return Protocol pointer or nullptr does not exits or wrong type
   */
  template<typename T>
  static std::pair<T*, bool> getExistingChecked(const std::string& key, 
                               const std::string& expectedType = "") {
    auto existing = getExisting(key);
    if (!existing.first) return def_value;
    
    // Verify type if specified
    if (!expectedType.empty() && !existing.first->isType(expectedType)) {
      log_e("Type mismatch for %s. Expected: %s, Got: %s\n",
                    key.c_str(), expectedType.c_str(), 
                    existing.first->getProtocolType().c_str());
      return def_value;
    }
    
    // Safe cast after verification
    return {static_cast<T*>(existing.first), existing.second};
  }

  static std::pair<BaseProtocol*, bool> getExisting(const std::string& key) {
    return ConfigRegistry::getProtocol(key);
  }
  
  static bool registerInstance(const std::string& key, 
                              BaseProtocol* instance, 
                              bool autoManaged = false) {
    return ConfigRegistry::registerProtocol(key, instance, autoManaged);
  }
  
  // ========== FACTORY METHODS SEGUROS ==========

  /**
   * @brief Creates or get a shared instance with verification
   */
  template<typename T>
  static T* createShared(const std::string& key,
                         std::function<T *()> new_func,
                         const std::string& expectedType = "") {

    // Verify if already exist
    auto existing = BaseProtocol::getExistingChecked<T>(key, expectedType);

    if (existing.first) {
      // Verify if protocol can be shared
      if (!existing.second) {
        log_e("Can't take protocol %s: is not shared", key.c_str());
        return nullptr;
      }
      return existing.first;
    }

    // Create new instance
    T* instance = new_func();
    // Register as auto-managed
    BaseProtocol::registerInstance(key, instance, true);
    return instance;
  }

  /**
   * @brief Creates unique instance (not shared)
   */
  template<typename T>
  static std::unique_ptr<T> createUnique(const std::string& key,
                                         std::function<T *()> new_func) {
    // Verify that does not exist
    auto existing = getExisting(key);

    if (existing.first) {
      log_e("Protocol %s already exists\n", key.c_str());
      return nullptr;
    }
  
    // Create new instance
    std::unique_ptr<T> instance(new_func());
    // Register as no auto-managed
    BaseProtocol::registerInstance(key, instance.get(), false);
  
    return instance;
  }
  
  // ========== PUBLIC INTERFACE ==========
  
  virtual bool configure() = 0;
  virtual bool begin() = 0;
  virtual void end() = 0;
  
  bool isReady() const { return m_configured && m_started; }
  
  // ====== Access to configuration ======

  template<typename T>
  T getConfig(const std::string& param, T defaultValue = T()) const {
    return ConfigRegistry::getConfigAs<T>(m_key, param, defaultValue);
  }
  
  std::string getString(const std::string& param, 
                       const std::string& def = "") const {
    return getConfig<std::string>(param, def);
  }
  
  int getInt(const std::string& param, int def = 0) const {
    return getConfig<int>(param, def);
  }
  
  float getFloat(const std::string& param, float def = 0.0f) const {
    return getConfig<float>(param, def);
  }

  float getDouble(const std::string& param, double def = 0.0f) const {
    return getConfig<double>(param, def);
  }
  
  bool getBool(const std::string& param, bool def = false) const {
    return getConfig<bool>(param, def);
  }
  
  bool hasParam(const std::string& param) const {
    return ConfigRegistry::hasConfig(m_key, param);
  }
  
  const std::string& getKey() const { return m_key; }
  
  friend class ConfigRegistry;
};


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
  static Semaphore m_mutex;

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

  /**
   * @brief Send a notification to this task, bringing it out of the Blocked state.
   * @details Use it from an external task when you invoked the function
   * `notifyTakeFromThisTask()` from this task to lock 
   */
  void notifyGive() { xTaskNotifyGive(m_taskHandle); }

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

  /**
   * @brief Static function for take notification from external task.
   * @param ticksToWait Time to wait before skip from this lock. Default portMAX_DELAY (wait forever)
   * @details Use this function inside the task implementation. Then, from an external task, call
   * `notifyGive()` to resume task operation. Unlike suspend()/resume() which suspend task operation
   * from outside, notifyTakeFromThisTask()/notifyGive() make possible to suspend from inside the
   * task to resume from outside. This is usefull for tasks synchronization.
   */
  static inline void notifyTakeFromThisTask(TickType_t ticksToWait = portMAX_DELAY) { ulTaskNotifyTake( pdTRUE, ticksToWait ); }

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
  static Semaphore m_mutex;  // Semaphore for thread safety
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
  static Semaphore m_mutex;
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
   * @tparam T Target type (bool, int, float, string)
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
