
#include "diutils.h"

#ifndef HIGH
#include <Arduino.h>
#endif // HIGH

#define USE_ARDUINO_JSON

#ifdef USE_ARDUINO_JSON
#include <ArduinoJson.h>
#endif // USE_ARDUINO_JSON


//************************************** Conversion templates **************************************//

template<>
int convertTo<int>(const std::string& value) { return std::stoi(value); };

template<>
long convertTo<long>(const std::string& value) { return std::stol(value); };

template<>
uint32_t convertTo<uint32_t>(const std::string& value) { return std::stoul(value); };

template<>
int8_t convertTo<int8_t>(const std::string& value) { return std::stoi(value); };

template<>
float convertTo<float>(const std::string& value) { return std::stof(value); };

template<>
double convertTo<double>(const std::string& value) { return std::stod(value); };

template<>
bool convertTo<bool>(const std::string& value) { return (value == "true" || value == "1" || value == "TRUE"); };

template<>
const std::string& convertTo<const std::string&>(const std::string& value) { return value; };

template<>
std::string convertTo<std::string>(const std::string& value) { return value; };


//************************************** Scoped Lockguard **************************************//

Semaphore::Semaphore(SemaphoreType type, UBaseType_t max_count, UBaseType_t initial_count) : m_type(type), m_handle(nullptr) {
  switch (type) {
  case SemaphoreType::COUNTING_SEMAPHORE:
    m_handle = xSemaphoreCreateCounting(max_count, initial_count);
    log_d("Counting Semaphore started at address %p", m_handle);
    break;
  case SemaphoreType::MUTEX:
    m_handle = xSemaphoreCreateMutex();
    log_d("Mutex started at address %p", m_handle);
    break;
  case SemaphoreType::RECURSIVE_MUTEX:
    m_handle = xSemaphoreCreateRecursiveMutex();
    log_d("Recursive Mutex started at address %p", m_handle);
    break;
  case SemaphoreType::BINARY_SEMAPHORE:
    m_handle = xSemaphoreCreateBinary();
    log_d("Binary Semaphore started at address %p", m_handle);
    break;
  // More types, if needed...
  }

  assert(m_handle != nullptr);
}

Semaphore::~Semaphore() {
  if (m_handle) {
    vSemaphoreDelete(m_handle);
  }
}

Semaphore::Semaphore(Semaphore&& other) noexcept : m_handle(other.m_handle), m_type(other.m_type) {
  other.m_handle = nullptr;
}

Semaphore &Semaphore::operator=(Semaphore &&other) noexcept {
  if (this != &other) {
    m_type = other.m_type;
    if (m_handle)
      vSemaphoreDelete(m_handle);
    m_handle = other.m_handle;
    other.m_handle = nullptr;
  }
  return *this;
}

LockGuard::LockGuard(Semaphore& semaphore, TickType_t timeout) : m_s(semaphore) {
  BaseType_t result;
  switch (m_s.m_type) {
  case Semaphore::SemaphoreType::RECURSIVE_MUTEX:
    result = xSemaphoreTakeRecursive(m_s.m_handle, timeout);
    break;
  case Semaphore::SemaphoreType::MUTEX:
  case Semaphore::SemaphoreType::BINARY_SEMAPHORE:
  case Semaphore::SemaphoreType::COUNTING_SEMAPHORE:
    result = xSemaphoreTake(m_s.m_handle, timeout);
    break;
  // More types, if needed...
  }
  // Barrier after take
  asm volatile("" : : : "memory"); // Compiler Memory Barrier
  #if CONFIG_FREERTOS_UNICORE == 0
    __sync_synchronize(); // Multicore hardware barrier
  #endif
  if (result == pdTRUE) {
  //log_d("Lock acquired %p", &m_s); // Uncomment only if suspect of bug caused by lockguard, because is too verbose
  } else {
    log_e("Failed to acquire lock %p", &m_s);
  }
}

LockGuard::~LockGuard() {
  //log_d("Releasing lock %p", &m_s); // Uncomment only if suspect of bug caused by lockguard, because is too verbose
  // Barrier before give
  #if CONFIG_FREERTOS_UNICORE == 0
    __sync_synchronize();
  #endif
  asm volatile("" : : : "memory");
  switch (m_s.m_type) {
  case Semaphore::SemaphoreType::RECURSIVE_MUTEX:
    xSemaphoreGiveRecursive(m_s.m_handle);
    break;
  case Semaphore::SemaphoreType::MUTEX:
  case Semaphore::SemaphoreType::BINARY_SEMAPHORE:
  case Semaphore::SemaphoreType::COUNTING_SEMAPHORE:
    xSemaphoreGive(m_s.m_handle);
    break;
  // More types, if needed...
  }
  // Barrier after give
  asm volatile("" : : : "memory"); // Compiler Memory Barrier
  #if CONFIG_FREERTOS_UNICORE == 0
    __sync_synchronize();
  #endif
}


//************************************** ConfigRegistry **************************************//

ConfigRegistry& ConfigRegistry::getInstance() {
  static ConfigRegistry instance;
  return instance;
}

void ConfigRegistry::cleanup() {
  LockGuard lg(m_mutex);
  
  for (auto& pair : registry) {
    if (pair.second.autoManaged && pair.second.protocol) {
      delete pair.second.protocol;
      pair.second.protocol = nullptr;
    }
  }
  registry.clear();
}

bool ConfigRegistry::registerProtocol(const std::string& protocolKey) {
  auto& instance = getInstance();
  LockGuard lg(instance.m_mutex);

  auto it = instance.registry.find(protocolKey);
  if (it == instance.registry.end()) {
    instance.registry[protocolKey] = ProtocolEntry();
    return true;
  }
  return false; // Already exists
}

bool ConfigRegistry::registerProtocol(const std::string& protocolKey, 
                                     BaseProtocol* protocol, 
                                     bool autoManaged) {
  auto& instance = getInstance();
  LockGuard lg(instance.m_mutex);

  auto it = instance.registry.find(protocolKey);
  if (it == instance.registry.end()) {
    // Create new entry
    instance.registry[protocolKey] = ProtocolEntry(protocol, autoManaged);
    return true;
  } else if (it->second.protocol == nullptr) {
    // Update existing entry without instance
    it->second.protocol = protocol;
    it->second.autoManaged = autoManaged;
    it->second.referenceCount = 1;
    return true;
  } else if (it->second.protocol == protocol) {
    // Same instance, increment reference count
    it->second.referenceCount++;
    return true;
  }
  // Different instance already registered
  return false;
}

std::pair<BaseProtocol*, bool> ConfigRegistry::getProtocol(const std::string& protocolKey) {
  auto& instance = getInstance();
  LockGuard lg(instance.m_mutex);

  auto it = instance.registry.find(protocolKey);
  if (it != instance.registry.end()) {
    return {it->second.protocol, it->second.autoManaged};
  }
  return def_value;
}

bool ConfigRegistry::hasProtocol(const std::string& protocolKey) {
  auto& instance = getInstance();
  LockGuard lg(instance.m_mutex);

  auto it = instance.registry.find(protocolKey);
  return it != instance.registry.end() && it->second.protocol != nullptr;
}

bool ConfigRegistry::releaseProtocol(const std::string& protocolKey) {
  auto& instance = getInstance();
  LockGuard lg(instance.m_mutex);
  
  auto it = instance.registry.find(protocolKey);
  if (it == instance.registry.end()) return false;
  
  if (it->second.protocol && it->second.referenceCount > 0) {
    it->second.referenceCount--;
    
    if (it->second.referenceCount == 0 && it->second.autoManaged) {
      // Last reference and auto-managed, delete instance
      delete it->second.protocol;
      it->second.protocol = nullptr;
      it->second.autoManaged = false;
      return true; // Instance removed
    }
  }
  return false; // Instance still exists
}

bool ConfigRegistry::removeProtocol(const std::string& protocolKey) {
  auto& instance = getInstance();
  LockGuard lg(instance.m_mutex);
  
  auto it = instance.registry.find(protocolKey);
  if (it == instance.registry.end()) return false;
  
  // Cleanup if auto-managed
  if (it->second.autoManaged && it->second.protocol) {
    delete it->second.protocol;
  }
  
  instance.registry.erase(it);
  return true;
}

// Configuration methods (similar to your existing implementation)
bool ConfigRegistry::setConfig(const std::string& protocolKey, 
                              const std::string& paramKey, 
                              const std::string& paramValue) {
  auto& instance = getInstance();
  LockGuard lg(instance.m_mutex);
  
  auto it = instance.registry.find(protocolKey);
  if (it != instance.registry.end()) {
    it->second.config[paramKey] = paramValue;
    return true;
  }
  return false;
}

std::string ConfigRegistry::getConfig(const std::string& protocolKey, 
                                     const std::string& paramKey, 
                                     const std::string& defaultValue) {
  auto& instance = getInstance();
  LockGuard lg(instance.m_mutex);
  
  auto it = instance.registry.find(protocolKey);
  if (it != instance.registry.end()) {
    auto paramIt = it->second.config.find(paramKey);
    if (paramIt != it->second.config.end()) {
      return paramIt->second;
    }
  }
  return defaultValue;
}

bool ConfigRegistry::hasConfig(const std::string& protocolKey,
                              const std::string& paramKey) {
  auto& instance = getInstance();
  LockGuard lg(instance.m_mutex);

  auto it = instance.registry.find(protocolKey);
  if (it == instance.registry.end())
    return false;
  auto paramIt = it->second.config.find(paramKey);
  return paramIt != it->second.config.end();
}

void ConfigRegistry::debugPrint() {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
  log_d("***************************** Show ConfigRegistry *****************************");
  auto& instance = getInstance();
  LockGuard lg(instance.m_mutex);
  int proto_index = 0;
  for (const auto& protoPair : instance.registry) {
    log_d("%d. Protocol: %s", proto_index++, protoPair.first.c_str());
    int prm_index = 0;
    for (const auto& paramPair : protoPair.second.config) {
      log_d("  %d) %s = %s", prm_index++, paramPair.first.c_str(), paramPair.second.c_str());
    }
  }
  log_d("***************************** End  ConfigRegistry *****************************");
#endif //ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
}

//************************************** A Gossip Allocator :) **************************************//

#ifdef USE_TOAD_ALLOCATOR

Allocator* Toadllocator::instance(const char * srcFileName, int srcLineNumber) {
  log_d("Allocator instanced from %s:%d", srcFileName, srcLineNumber);
  static Toadllocator allocator;
  return &allocator;
}

void* Toadllocator::allocate(size_t size) {
  log_d("allocate: %u  heap: %d", size, xPortGetFreeHeapSize());
  void* ptr = malloc(size);
  log_d("allocate: %u  addr: %p  heap: %d", size, ptr, xPortGetFreeHeapSize());
  return ptr;
}

void Toadllocator::deallocate(void* ptr) {
  log_d("deallocate  addr: %p  heap: %d", ptr, xPortGetFreeHeapSize());
  free(ptr);
  log_d("heap: %d  lower: %d  integrity: %s", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize(), heap_caps_check_integrity_all(true) ? "ok!" : "corrupted");
}

void* Toadllocator::reallocate(void* ptr, size_t new_size) {
  log_d("reallocate: %u  addr: %p  heap: %d", new_size, ptr, xPortGetFreeHeapSize());
  void* new_ptr = realloc(ptr, new_size);
  log_d("reallocate: %u  new addr: %p  heap: %d", new_size, new_ptr, xPortGetFreeHeapSize());
  return new_ptr;
}

#endif

void Toadllocator::check(const char * srcFileName, int srcLineNumber) {
  log_d("%s:%d -> Heap integrity: %s  free: %d  lower: %d", srcFileName, srcLineNumber,
    heap_caps_check_integrity_all(true) ? "ok!" : "corrupted",
    xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
}


//************************************** Helper for PWM channel in ESP32 **************************************//

Semaphore LedcChan::m_mutex(Semaphore::create_mutex());

LedcChan& LedcChan::Instance() {
  static LedcChan instance;
  return instance;
}

LedcChan::LedcChan() : m_busyChans(0x0) {}

int LedcChan::addChannel() {
  int channel;
  LockGuard lg(m_mutex);  // Thread-safe access
  uint32_t busyChans = m_busyChans;
  for (channel = 0; channel < LEDC_MAX_CHANS; ++channel) {
    if (!(busyChans & 0x1))
      break;
    busyChans >> 1;
  }
  if (channel >= LEDC_MAX_CHANS) {
    log_w("Channels already full");
    return -1;    // All channels are busy
  }
  log_d("Got channel %d", channel);
  m_busyChans |= ((uint32_t) 0x1) << channel;
  return channel; // Assigned channel
}

void LedcChan::removeChannel(int channel) {
  if (channel < 0 || channel >= LEDC_MAX_CHANS) {
    log_w("Channel outside range: %d", channel);
  } else {
    LockGuard lg(m_mutex);        // Thread-safe access
    m_busyChans &= ~(((uint32_t) 0x1) << channel);  // Release the bit
  }
}


//************************************** Helper for Digital Calculus **************************************//

bool DigitalCalc::send(uint8_t value) {
  bool bValue = value == HIGH ? true : false;
  return condInverter(bValue);
}

uint8_t DigitalCalc::receive(bool value) {
  bool bValue = condInverter(value);
  return bValue ? HIGH : LOW;
}


//************************************** Class to get Time in Seconds **************************************//

Semaphore TimeCounter::m_mutex(Semaphore::create_mutex());

uint32_t TimeCounter::m_timeInSecs(0);

void TimeCounter::TickTask (void*) {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
  // Checks for resource usage in debug mode
  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  log_d("Stack unused %d", uxHighWaterMark);
  UBaseType_t uxHighWaterMark2;
#endif
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  BaseType_t xWasDelayed;
  log_d("Starting timer counter task with %u ticks per second, current ticks %u", xFrequency, xLastWakeTime);
  while (true) {
    xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xFrequency );
    do { // To apply mutex just here
      LockGuard lg(m_mutex);
      m_timeInSecs++;
    } while(false);
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
}

TimeCounter& TimeCounter::Instance() {
  static TimeCounter instance;
  return instance;
}

TimeCounter::TimeCounter() {
  log_d("Starting single TimeCounter instance");
  xTaskCreate(TimeCounter::TickTask, "TickTask", 2048, nullptr, 3, nullptr);
}

uint32_t TimeCounter::getTimeSecs() {
  LockGuard lg(m_mutex);
  return m_timeInSecs;
}


//************************************** Sending Arbitrer **************************************//

SendManager::SendManager(uint32_t min_send_time, u_int32_t max_send_time) :
  m_min_time(min_send_time), m_max_time(max_send_time), m_changed(true) {
  m_last_time = TimeCounter::Instance().getTimeSecs();
}

 bool SendManager::updateRequired(bool force) {
  // Get time
  uint32_t now = TimeCounter::Instance().getTimeSecs();
  // If any of these 3 conditions apply, then update all and say that update is required
  if (force || ((now - m_last_time > m_min_time) && m_changed) || (now - m_last_time > m_max_time)) {
    m_last_time = now;
    m_changed = false;
    return true;
  }
  return false;
 }

bool AnSendManager::updateRequired(float value, bool force) {
  // Detect change
  float delta = value - m_last_value;
  if (delta >= m_var_delta || delta <= -m_var_delta)
    m_changed = true;
  // proccess if update required
  if (SendManager::updateRequired(force)) {
    m_last_value = value;
    return true;
  }
  return false;
}

bool DgSendManager::updateRequired(bool value, bool force) {
  // Detect change
  if (value != m_last_value)
    m_changed = true;
  // proccess if update required
  if (SendManager::updateRequired(force)) {
    m_last_value = value;
    return true;
  }
  return false;
}


//************************************** Custom Variant **************************************//

void CustomVariant::convertFromOther( const CustomVariant& src ) {
  m_alternative = src.m_alternative;
  switch (m_alternative)
  {
  case alt_t::t_boolean:
    b = src.b;
    break;
  case alt_t::t_float:
    f = src.f;
    break;
  case alt_t::t_string:
    s = src.s;
    break;
  default:
    break;
  }
}

void CustomVariant::convertFromVariant( const JsonVariant src ) {
  if (src.is<bool>()) {
    m_alternative = alt_t::t_boolean;
    b = src.as<bool>();
  } else if (src.is<float>()) {
    m_alternative = alt_t::t_float;
    f = src.as<float>();
  } else if (src.is<std::string>()) {
    m_alternative = alt_t::t_string;
    s = src.as<std::string>();
  } else {
    m_alternative = alt_t::t_notype;
  }
}

template<typename T>
bool CustomVariant::can_convert() {
  switch (m_alternative) {
  case alt_t::t_boolean:
    return std::is_same<bool, T>::value;
  case alt_t::t_float:
    return (std::is_same<float, T>::value || std::is_same<int, T>::value);
  case alt_t::t_string:
    return std::is_same<std::string, T>::value;
  default:
    return false;
  }
}

template bool CustomVariant::can_convert<bool>();
template bool CustomVariant::can_convert<int>();
template bool CustomVariant::can_convert<float>();
template bool CustomVariant::can_convert<std::string>();

template<typename T>
const T& CustomVariant::convert() {
  switch (m_alternative) {
  case alt_t::t_boolean:
    return *reinterpret_cast<T*>(&b);
  case alt_t::t_float:
    if (std::is_same<int, T>::value) {
      int itemp = (int) f;
      float roundDet = f - ((float) itemp);
      i = roundDet >= 0.5f ? itemp + 1 : roundDet < -0.5 ? itemp - 1 : itemp;
      return *reinterpret_cast<T*>(&i);
    }
    return *reinterpret_cast<T*>(&f);
  case alt_t::t_string:
    return *reinterpret_cast<T*>(&s);
  }
  log_e("Is getting here!");
  T *t = nullptr;
  return *t;
}

template const bool& CustomVariant::convert<bool>();
template const int& CustomVariant::convert<int>();
template const float& CustomVariant::convert<float>();
template const std::string& CustomVariant::convert<std::string>();


//************************************** String hex array to byte conversion **************************************//

bool stringHex2ByteArray(String hexArray, uint8_t* byteArray, size_t size) {
  if (byteArray == nullptr) return false;
  const String ref = "0123456789abcdefABCDEF";
  hexArray.replace(":", "");
  hexArray.replace(" ", "");
  int index1, index2;
  const unsigned int haSize = hexArray.length();
  for (unsigned int bytes = 0; bytes < size; ++bytes) {
    if (2*bytes + 1 >= haSize) return false;
    index1 = ref.indexOf(hexArray.charAt(2*bytes + 1));
    index2 = ref.indexOf(hexArray.charAt(2*bytes));
    if (index1 < 0 || index2 < 0) return false;
    byteArray[bytes] = ((index2 > 15 ? index2 - 6 : index2) << 4) + (index1 > 15 ? index1 - 6 : index1);
    log_d("Byte[%u]=0x%02X", bytes, byteArray[bytes]);
  }
  return true;
}
