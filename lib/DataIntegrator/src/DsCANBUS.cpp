#include "DsCANBUS.h"
#include "diutils.h"
#include <ArduinoJson.h>
#include <string.h>


//************************************** TwaiProtocol **************************************//

class TwaiProtocol : public BaseProtocol {
private:
  // Miembros específicos de TWAI
  void* twaiHandler = nullptr;

  // Constructor privado para forzar uso de factory
  TwaiProtocol(const std::string& key) : BaseProtocol(key) {};
  
  // Constructor privado para forzar uso de factory
  /*TwaiProtocol(const std::string& key) : BaseProtocol(key) {
    // Registro automático en el registry
    BaseProtocol::registerInstance(key, this, true);
  }*/
  
public:
  ~TwaiProtocol() {
    end();
  }
  
  // ========== FACTORY CON VERIFICACIÓN DE TIPO ==========
  
  /**
   * @brief Crea o obtiene una instancia compartida con verificación
   */
  static TwaiProtocol* createShared(const std::string& key, 
                                   DataSource* dataSource) {
    return BaseProtocol::createShared<TwaiProtocol>(key, [key] { return new TwaiProtocol(key); }, dataSource->type());
  }
  
  /**
   * @brief Crea una instancia única (no compartida)
   */
  static std::unique_ptr<TwaiProtocol> createUnique(const std::string& key) {
    return BaseProtocol::createUnique<TwaiProtocol>(key, [key] { return new TwaiProtocol(key); });
  }
  
  // ========== IMPLEMENTACIÓN DE MÉTODOS ==========
  
  bool configure() override {
    LockGuard lock(m_rec_mutex);
    
    // Leer configuración del registry
    int txPin = getInt("tx_pin", 5);
    int rxPin = getInt("rx_pin", 4);
    long baud = getInt("baud", 500000);
    
    log_d("[%s] Configuring TWAI: TX=%d, RX=%d, Baud=%ld",
                  m_key.c_str(), txPin, rxPin, baud);
    
    m_configured = true;
    return true;
  }
  
  bool begin() override {
    if (!m_configured && !configure()) {
      return false;
    }
    
    LockGuard lock(m_rec_mutex);
    
    log_d("[%s] Starting TWAI communication", m_key.c_str());
    
    m_started = true;
    return true;
  }
  
  void end() override {
    LockGuard lock(m_rec_mutex);
    
    if (m_started) {
      log_d("[%s] Finishing TWAI communication", m_key.c_str());
      m_started = false;
    }
  }
  
  // Métodos específicos de TWAI
  bool sendMessage(uint32_t id, const uint8_t* data, uint8_t length);
  bool receiveMessage(uint32_t& id, uint8_t* data, uint8_t& length);
};


#if 0
class TwaiProtocol : public BaseProtocol {
    TwaiProtocol(const string& key) : BaseProtocol(key) {}
public:
    ~TwaiProtocol() = default;

    // Factory methods
    static TwaiProtocol* createShared(const std::string& key) {
        return BaseProtocol::createShared<TwaiProtocol>(key);
    }

    static std::unique_ptr<TwaiProtocol> createUnique(const std::string& key) {
        return BaseProtocol::createUnique<TwaiProtocol>(key);
    }

    bool configure() override {return false;}
    bool begin() override {return false;}
    void end() override {};
};
#endif


// ************************************** CANSignalNode ************************************** //

uint32_t value2uint32_t(JsonVariant value) { // TODO: check for string
    const char *vs = value.as<const char*>();
    if (vs == nullptr) {
        return value.as<uint32_t>();
    }
    char *endPtr;
    // Assuming that, if string, is hex
    uint32_t ui32value = std::strtol(vs, &endPtr, 16);
    return ui32value;
}

twai_filter_config_t getFilter(JsonObject filter) { // TODO: interpret and fix
    if (filter.isNull())
        return TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (!filter["code"].isNull()) {
        uint32_t code = value2uint32_t(filter["code"]);
        uint32_t mask = value2uint32_t(filter["mask"]);
        return {.acceptance_code = code, .acceptance_mask = mask, .single_filter = true};
    }
    uint32_t code1 = value2uint32_t(filter["code1"]);
    uint32_t mask1 = value2uint32_t(filter["mask1"]);
    uint32_t code2 = value2uint32_t(filter["code2"]);
    uint32_t mask2 = value2uint32_t(filter["mask2"]);
    return {
        .acceptance_code = (code1 & 0xffff) | ((code2 << 16) & 0xffff0000),
        .acceptance_mask = (mask1 & 0xffff) | ((mask2 << 16) & 0xffff0000),
        .single_filter = false};
}

int DsCANBUS::build(JsonObject source, int index) {
    m_filter = getFilter(source["filter"]);
    log_d("Accep code: 0x%08X, mask: 0x%08X, single: %s", m_filter.acceptance_code, m_filter.acceptance_mask, m_filter.single_filter? "true" : "false");
    const String mode = source["mode"] | "";
    m_mode = TWAI_MODE_NORMAL;
    if (mode.isEmpty()) {
        if (mode == "listen")
            m_mode = TWAI_MODE_LISTEN_ONLY;
        else if (mode == "noack")
            m_mode = TWAI_MODE_NO_ACK;
    }
    log_d("TWAI mode: %d", m_mode);
    // 1. Instance twai protocol, to register name
    string name = string(type()) + "@0";
    m_twai = TwaiProtocol::createShared(name, this);
    log_d("%s protocol driver started at address %p", type(), m_twai);
    // 2. Set protocol parameters
    ConfigRegistry::setConfig(name, "tx_pin", 21);
    ConfigRegistry::setConfig(name, "rx_pin", 22);
    ConfigRegistry::setConfig(name, "baud", 500000);
    ConfigRegistry::setConfig(name, "mode", 1); // NORMAL mode
    // 3. Start configuration
    if (!m_twai->configure()) return DIError::BUS_REQ_ERROR; // TODO: create bus init error
    // Then, wait to one data node to start the protocol

    return DataSource::build(source, index);
}


// ************************************* CANBUSTaskQueue ************************************* //

CANBUSTaskQueue& CANBUSTaskQueue::Instance() {
  static CANBUSTaskQueue instance;
  return instance;
}

bool CANBUSTaskQueue::addId(uint32_t id, CANSignalNodeBase *node) {
    LockGuard lg(m_mutex);
    auto id_it = m_id_map.find(id);
    if (id_it != m_id_map.end()) {
        log_d("Can node for id(%d) already exist...", id);
        return false;
    }
    m_id_map[id] = node;
    log_d("Id(%d) added to node List! Addr: %p", id, node);
    return true;
}

CANBUSTaskQueue::CANBUSTaskQueue() : m_mutex(Semaphore::create_mutex()), m_task(nullptr), m_id_map() {
    LockGuard lg_ini1(m_mutex); // Mutex guard just started here!
    if (m_started) { // To avoid multiple extTask instances
        return;
    }
    m_task = new extTask("CAN_RX", 4096, tskIDLE_PRIORITY + 2, [this]{
        twai_message_t rx_msg;
        log_d("Starting Can Bus Receive Task...");
        do { // Start a temporal scope (to not affect lg)
            LockGuard lg_ini2(m_mutex); // Wait for delete of lg_ini1
            log_d("... then, internal task should get to this point later!");
        } while (false);

        while(true) {
            // Wait for message (blocking operation)
            log_d("Waiting for Can Bus message...");
            esp_err_t rx_error = twai_receive(&rx_msg, portMAX_DELAY);
            if (rx_error == ESP_OK) {
                log_d("Message received from id(%d)", rx_msg.identifier);
                LockGuard lg(m_mutex);
                auto id_it = m_id_map.find(rx_msg.identifier);
                if (id_it != m_id_map.end()) {
                    CANSignalNodeBase* node = id_it->second;
                    // Now, send the message to the node
                    node->processCanMessage(rx_msg);
                    log_d("Message sent to node %p", node);
                } else {
                    log_d("Id not found in list.");
                }
            } else {
                const char *tx_error_s = esp_err_to_name(rx_error);
                log_e("Error receiving message: %s", tx_error_s);
                vTaskDelay(pdMS_TO_TICKS(10000));
            }
        }
    });
    m_started = true;
    log_d("External task should get to this point first ...");
}


// ************************************ CANSignalNodeBase ************************************ //

CANSignalNodeBase::CANSignalNodeBase() : 
    m_canId(0), m_extendedId(false), m_isTx(false), m_offset(0), m_last_value(0),
    m_littleEndian(true), m_calc(nullptr), m_sendManager(nullptr), m_timeout(100) {}

CANSignalNodeBase::~CANSignalNodeBase() {
    delete m_sendManager;
    delete m_calc;
}

int CANSignalNodeBase::build(JsonObject node, int index) {
    // ID validation
    JsonVariant var;
    var = node["id"];
    if (var.isNull()) {
        return DIError::PARAM_NOT_INCLUDED;
    }

    // Get timeout
    m_timeout = node["timeout"] | 100;

    // Get ID, mask and extended
    m_canId = value2uint32_t(var);
    m_extendedId = node["ext"] | false;
    var = node["mask"];
    if (var.isNull())
        m_mask = 0x7FF;
    else
        m_mask = value2uint32_t(var);

    // Take kay parameters from derived classes
    typePrms type_prms = typeParams();

    // Get byte offset
    m_offset = node["byte_offset"] | 0;
    if (m_offset + type_prms.length > 8) {
        return DIError::PARAM_OUT_BOUNDS;
    }

    // Get direction and endianess
    m_isTx = (String(node["direction"] | "rx") == "tx");
    m_littleEndian = (String(node["byte_order"] | "little") == "little");

    log_d("Params type: %s", type());
    log_d("Length: %u, max: %d, min: %d, first byte: %u, endianess: %s", type_prms.length, type_prms.max, type_prms.min, m_offset, m_littleEndian ? "little" : "big");
    log_d("Id: 0x%08X, mask: 0x%08X, ext: %s, dir: %s", m_canId, m_mask, m_extendedId ? "yes" : "no", m_isTx ? "tx" : "rx");

    // Add linear calculator
    float gain = node["gain"] | 1.0f;
    float offset = node["off"] | 0.0f;
    m_calc = new LinearCalc(offset, gain, type_prms.max, type_prms.min);
    m_unit = node["unit"] | "";

    // add send manager
    uint32_t mintime = node["mintime"] | 1;
    uint32_t maxtime = node["maxtime"] | 60;
    float delta = node["delta"] | 0.1f;
    m_sendManager = new AnSendManager(mintime, maxtime, delta);
    
    return DataNode::build(node, index);
}

void CANSignalNodeBase::start() {
    if (!m_isTx) {
        CANBUSTaskQueue::Instance().addId(m_canId, this);
    }
}

void CANSignalNodeBase::request() {
    if (m_isTx && m_cmd == DataNode::WRITE) {
        if (!m_variant.can_convert<float>()) {
            sendError(DIError::PARAM_WORNG_TYPE);
            return;
        }
        
        float value = m_variant.convert<float>();
        twai_message_t message;
        packData(message, value);
        
        esp_err_t tx_error = twai_transmit(&message, pdMS_TO_TICKS(m_timeout));
        if ( tx_error != ESP_OK) {
            const char *tx_error_s = esp_err_to_name(tx_error);
            log_e("Value not sent, error: %s", tx_error_s);
            sendError(DIError::BUS_REQ_ERROR, tx_error_s);
        } else
            log_e("Value sent: %.2f", value);
    }
}

void CANSignalNodeBase::processCanMessage(const twai_message_t &message) {
    float value = unpackData(message);
    
    if (m_sendManager->updateRequired(value, false)) {
        m_last_value = value;
        bool found = m_source->onResponse(m_name, [value, this](JsonObject object) {
            object["value"] = value;
            if (!m_unit.empty()) {
                object["unit"] = m_unit.c_str();
            }
        });
        log_d("CANSignalNode %s updated value: %.2f %s", m_name.c_str(), value, m_unit.c_str());
        log_d("Add %s: %s", m_name.c_str(), found ? "exist" : "new");
    }
}

void CANSignalNodeBase::packData(twai_message_t &message, float value) {
    // Aplicar transformación lineal
    int irawValue = m_calc->receive(value);

    message.identifier = m_canId;
    message.extd = m_extendedId;
    typePrms type_prms = typeParams();
    message.data_length_code = m_offset + type_prms.length;

    
    // Convertir a entero según el tipo
    packData(&message.data[m_offset], irawValue);
}

float CANSignalNodeBase::unpackData(const twai_message_t &message) {
    // Extraer el entero segun el tipo
    int irawValue = unpackData(&message.data[m_offset]);

    // Convertir a flotante mediante transformación lineal inversa
    return m_calc->send(irawValue);
}


// ************************************** CANSignalNode ************************************** //

void swapEndianess(void *varp, size_t sz) {
    switch (sz) {
        case sizeof(int8_t):
            break;
        case sizeof(int16_t): {
            int16_t *varp16 = static_cast<int16_t *>(varp);
            *varp16 = __builtin_bswap16(*varp16);
            break;
        }
        case sizeof(int32_t): {
            int32_t *varp32 = static_cast<int32_t *>(varp);
            *varp32 = __builtin_bswap32(*varp32);
            break;
        }
        case sizeof(int64_t): {
            int64_t *varp64 = static_cast<int64_t *>(varp);
            *varp64 = __builtin_bswap64(*varp64);
            break;
        }
    }
}
