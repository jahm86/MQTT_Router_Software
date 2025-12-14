#include "DsCANBUS.h"
#include "diutils.h"
#include <ArduinoJson.h>
#include <string.h>


//************************************** TwaiProtocol **************************************//

/**
 * @class TwaiProtocol
 * @brief Handles TWAI protocol and interrupts for CANBUS nodes
 */
class TwaiProtocol : public BaseProtocol {
private:

  // ========== TWAI SPECIFIC MEMBERS ==========

  unordered_map<uint32_t, CANSignalNodeBase*> m_id_map;
  extTask* m_task;

  // Private constructor to force factory usage
  TwaiProtocol(const std::string& key) : BaseProtocol(key), m_task(nullptr), m_id_map() {};

  // New instance creation
  static TwaiProtocol* newInstance(const std::string& key) { return new TwaiProtocol(key); }
  
public:
  ~TwaiProtocol() {
    end();
  }
  
  // ========== FACTORY METHODS ==========
  
  /**
   * @brief Creates or gets a shared instance
   */
  static TwaiProtocol* createShared(const std::string& key, DataSource* dataSource) {
    return BaseProtocol::createShared<TwaiProtocol>(key, newInstance, dataSource->type());
  }
  
  /**
   * @brief Creates unique instance (not shared)
   */
  static std::unique_ptr<TwaiProtocol> createUnique(const std::string& key) {
    return BaseProtocol::createUnique<TwaiProtocol>(key, newInstance);
  }
  
  // ========== PUBLIC INTERFACE IMPLEMENTATION ==========
  
  bool configure() override {
    LockGuard lock(m_rec_mutex);
    
    // Leer configuración del registry
    int tx_pin = getInt("tx_pin", 21);
    int rx_pin = getInt("rx_pin", 22);
    int mode = getInt("mode");
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        static_cast<gpio_num_t>(tx_pin),
        static_cast<gpio_num_t>(rx_pin),
        static_cast<twai_mode_t>(mode));

    long baud = getLong("baud", 100000);
    twai_timing_config_t t_config = getTiming(baud);

    twai_filter_config_t f_config = {
        .acceptance_code = getU32("f_accept", 0),
        .acceptance_mask = getU32("f_mask", 0xFFFFFFFF),
        .single_filter = getBool("f_single", false)
    };
    
    log_d("[%s] Configuring TWAI: TX=%d, RX=%d, Baud=%ld",
                  m_key.c_str(), tx_pin, rx_pin, baud);

    esp_err_t error = twai_driver_install(&g_config, &t_config, &f_config);
    if (error != ESP_OK) {
        log_e("[%s] Can't configure TWAI: %s", m_key.c_str(), esp_err_to_name(error));
        return false;
    }
    
    m_configured = true;
    return true;
  }
  
  bool begin() override {
    // Make sure that m_task stops in lock_ini2 point until begin() exit
    LockGuard lock_ini1(m_rec_mutex);

    if (!m_configured) {
        log_d("[%s] Not configured", m_key.c_str());
        return false;
    }

    if (m_started) {
        log_d("[%s] Already started", m_key.c_str());
        return false;
    }

    esp_err_t error = twai_start();
    if (error != ESP_OK) {
        log_e("[%s] Can't start TWAI: ", m_key.c_str(), esp_err_to_name(error));
        return false;
    }
    
    log_d("[%s] Starting TWAI communication", m_key.c_str());
    m_task = new extTask("CAN_RX", 4096, tskIDLE_PRIORITY + 2, [this] {
        twai_message_t rx_msg;
        log_d("[%s] Starting Can Bus Receive Task...", m_key.c_str());
        do { // Start a temporal scope (to not affect lock)
            LockGuard lock_ini2(m_rec_mutex); // Wait for delete of lock_ini1
            log_d("[%s] ... then, internal task should get to this point later!", m_key.c_str());
        } while (false);

        while(true) {
            // Wait for message (blocking operation)
            log_d("[%s] Waiting for Can Bus message...", m_key.c_str());
            esp_err_t rx_error = twai_receive(&rx_msg, portMAX_DELAY);
            if (rx_error == ESP_OK) {
                log_d("[%s] Message received from id(%d)", m_key.c_str(), rx_msg.identifier);
                LockGuard lock(m_rec_mutex);
                auto id_it = m_id_map.find(rx_msg.identifier);
                if (id_it != m_id_map.end()) {
                    CANSignalNodeBase* node = id_it->second;
                    // Now, send the message to the node
                    node->processCanMessage(rx_msg);
                    log_d("[%s] Message sent to node %p", m_key.c_str(), node);
                } else {
                    log_d("[%s] Id not found in list.", m_key.c_str());
                }
            } else {
                log_e("[%s] Error receiving message: %s", m_key.c_str(), esp_err_to_name(rx_error));
                vTaskDelay(millis2ticks(10000));
            }
        }
    });
    
    m_started = true;
    log_d("[%s] External task should get to this point first ...", m_key.c_str());
    return true;
  }
  
  void end() override {
    LockGuard lock(m_rec_mutex, millis2ticks(20000));
    
    if (m_started) {
      log_d("[%s] Finishing TWAI communication", m_key.c_str());
      delete m_task;
      m_task = nullptr;
      esp_err_t error = twai_stop();
      log_d("[%s] TWAI stop status: %s", m_key.c_str(), esp_err_to_name(error));
      error = twai_driver_uninstall();
      log_d("[%s] TWAI uninstall status: %s", m_key.c_str(), esp_err_to_name(error));
      m_started = false;
    }
  }
  
  // ========== TWAI SPECIFIC METHODS ==========

  esp_err_t sendMessage(uint32_t id, const uint8_t* data, uint8_t length, int timeout_ms) {
    TickType_t timeout = millis2ticks(timeout_ms);
    twai_message_t tx_msg = {0};
    uint8_t secure_length = length < TWAI_FRAME_MAX_DLC ? length : TWAI_FRAME_MAX_DLC;
    // Configuring message
    tx_msg.identifier = id;
    tx_msg.extd = false;
    tx_msg.rtr = false;
    tx_msg.ss = false;
    tx_msg.self = false;
    tx_msg.dlc_non_comp = false;
    tx_msg.data_length_code = secure_length;
    memcpy(&tx_msg.data[0], data, secure_length);
    // Sending message
    esp_err_t error = twai_transmit(&tx_msg, timeout);
    log_d("[%s] Message id(%u) of size(%u) sent, status: %s", m_key.c_str(),
        tx_msg.identifier, tx_msg.data_length_code, esp_err_to_name(error));
    return error;
  }

  esp_err_t sendMessage(const twai_message_t &tx_msg, int timeout_ms) {
    TickType_t timeout = millis2ticks(timeout_ms);
    esp_err_t error = twai_transmit(&tx_msg, timeout);
    log_d("[%s] Message id(%u) of size(%u) sent, status: %s", m_key.c_str(),
        tx_msg.identifier, tx_msg.data_length_code, esp_err_to_name(error));
    return error;
  }

  //bool receiveMessage(uint32_t& id, uint8_t* data, uint8_t& length);

  bool addId(uint32_t id, CANSignalNodeBase *node) {
    LockGuard lock(m_rec_mutex);
    auto id_it = m_id_map.find(id);
    if (id_it != m_id_map.end()) {
        log_d("[%s] Can node for id(%d) already exist...", m_key.c_str(), id);
        return false;
    }
    m_id_map[id] = node;
    log_d("[%s] Id(%d) added to node List! Addr: %p", m_key.c_str(), id, node);
    return true;
  }

    twai_timing_config_t getTiming(long baudrate) {
        switch (baudrate) {
            case 25000: return TWAI_TIMING_CONFIG_25KBITS();
            case 50000: return TWAI_TIMING_CONFIG_50KBITS();
            case 125000: return TWAI_TIMING_CONFIG_125KBITS();
            case 250000: return TWAI_TIMING_CONFIG_250KBITS();
            case 500000: return TWAI_TIMING_CONFIG_500KBITS();
            case 800000: return TWAI_TIMING_CONFIG_800KBITS();
            case 1000000: return TWAI_TIMING_CONFIG_1MBITS();
        }
        long def_baudrate = 100000;
        twai_timing_config_t def_config = TWAI_TIMING_CONFIG_100KBITS();
        if (baudrate != def_baudrate) {
            log_w("[%s] Baudrate '%d' not found. Returning %d", m_key.c_str(), baudrate, def_baudrate);
        }
        return def_config;
    }
};


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
    twai_filter_config_t filter;
    twai_mode_t mode_t;
    filter = getFilter(source["filter"]);
    log_d("Accep code: 0x%08X, mask: 0x%08X, single: %s", filter.acceptance_code, filter.acceptance_mask, filter.single_filter? "true" : "false");
    const String mode = source["mode"] | "";
    mode_t = TWAI_MODE_NORMAL;
    if (mode.isEmpty()) {
        if (mode == "listen")
            mode_t = TWAI_MODE_LISTEN_ONLY;
        else if (mode == "noack")
            mode_t = TWAI_MODE_NO_ACK;
    }
    log_d("TWAI mode: %d", mode_t);
    // 1. Instance twai protocol, to register name
    std::string name = string(type()) + "@0";
    m_twai = TwaiProtocol::createShared(name, this);
    log_d("%s protocol driver started at address %p", type(), m_twai);
    // 2. Set protocol parameters
    ConfigRegistry::setConfig(name, "tx_pin", 21);
    ConfigRegistry::setConfig(name, "rx_pin", 22);
    ConfigRegistry::setConfig(name, "baud", 100000);
    ConfigRegistry::setConfig(name, "mode", mode_t);
    ConfigRegistry::setConfig(name, "f_accept", filter.acceptance_code);
    ConfigRegistry::setConfig(name, "f_mask", filter.acceptance_mask);
    ConfigRegistry::setConfig(name, "f_single", filter.single_filter);
    // 3. Start configuration
    if (!m_twai->configure()) {
        log_e("[%s] TWAI protocol driver not instanciated", name);
        return DIError::BUS_REQ_ERROR; // TODO: create bus init error
    }
    // Then, wait to one data node to start the protocol

    return DataSource::build(source, index);
}

bool DsCANBUS::addId(uint32_t id, CANSignalNodeBase *node) {
    if (!m_twai) {
        log_e("TWAI protocol driver lost instance");
        return false;
    }

    return m_twai->addId(id, node);
}

esp_err_t DsCANBUS::sendMessage(uint32_t id, const uint8_t* data, uint8_t length, int timeout_ms) {
    if (!m_twai) {
        log_e("TWAI protocol driver lost instance");
        return false;
    }

    return m_twai->sendMessage(id, data, length, timeout_ms);
}

bool DsCANBUS::runProtocol() {
    if (!m_twai) {
        log_e("TWAI protocol driver lost instance");
        return false;
    }
    return m_twai->begin();
}

TwaiProtocol *DsCANBUS::protocol() {
    return m_twai;
}

DsCANBUS::~DsCANBUS() {
    if (!m_twai)
        return;
    m_twai->release();
}


// ************************************ CANSignalNodeBase ************************************ //

CANSignalNodeBase::CANSignalNodeBase() : 
    m_canId(0), m_extendedId(false), m_dir(CommDir::rx), m_offset(0), m_last_value(0),
    m_littleEndian(true), m_sendManager(nullptr), m_timeout(100) {}

CANSignalNodeBase::~CANSignalNodeBase() {
    delete m_sendManager;
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

    // Take size from derived classes
    size_t tlen = typeLen();

    // Get byte offset
    m_offset = node["b_offset"] | 0;
    if (m_offset + tlen > 8) {
        return DIError::PARAM_OUT_BOUNDS;
    }

    // Get direction (no comparison to rx, as it is default)
    String dir = node["dir"] | "rx";
    if (dir == "rtx")
        m_dir = CommDir::rtx;
    else if (dir == "tx")
        m_dir = CommDir::tx;
    // Get endianess
    m_littleEndian = (String(node["b_order"] | "little") == "little");

    log_d("Params type: %s", type());
    log_d("Length: %u, first byte: %u, endianess: %s", tlen, m_offset, m_littleEndian ? "little" : "big");
    log_d("Id: 0x%08X, mask: 0x%08X, ext: %s, dir: %c", m_canId, m_mask, m_extendedId ? "yes" : "no", static_cast<char>(m_dir));

    // Add linear calculator
    float gain = node["gain"] | 1.0f;
    float offset = node["off"] | 0.0f;
    buildCalc(offset, gain);
    m_unit = node["unit"] | "";

    // add send manager
    uint32_t mintime = node["mintime"] | 1;
    uint32_t maxtime = node["maxtime"] | 60;
    float delta = node["delta"] | 0.1f;
    m_sendManager = new AnSendManager(mintime, maxtime, delta);
    
    return DataNode::build(node, index);
}

void CANSignalNodeBase::start() {
    DsCANBUS* source = static_cast<DsCANBUS*>(m_source);
    if (m_dir == CommDir::rtx || m_dir == CommDir::rx) {
        source->addId(m_canId, this);
    }
    source->runProtocol();
}

void CANSignalNodeBase::request() {
    if (m_cmd == DataNode::WRITE) {
        if (m_dir == CommDir::rx) {
            log_w("Trying to write an only-receive node");
            return;
        }
        if (!m_variant.can_convert<float>()) {
            sendError(DIError::PARAM_WORNG_TYPE);
            return;
        }

        DsCANBUS* source = static_cast<DsCANBUS*>(m_source);
        TwaiProtocol *protocol = source->protocol();
        float value = m_variant.convert<float>();
        twai_message_t message;
        packData(message, value);

        esp_err_t tx_error = ESP_ERR_NO_MEM;
        if (protocol != nullptr)
            tx_error = protocol->sendMessage(message, m_timeout);
        if ( tx_error != ESP_OK) {
            const char *tx_error_s = esp_err_to_name(tx_error);
            log_e("Value not sent, error: %s", tx_error_s);
            sendError(DIError::BUS_REQ_ERROR, tx_error_s);
        } else
            log_d("Value sent: %.2f %s", value, m_unit.c_str());
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

    message.identifier = m_canId;
    message.extd = m_extendedId;
    size_t tlen = typeLen();
    message.data_length_code = m_offset + tlen;

    
    // Convertir a entero según el tipo
    packData(&message.data[m_offset], value);
}

float CANSignalNodeBase::unpackData(const twai_message_t &message) {
    // Extraer el entero segun el tipo
    float value = unpackData(&message.data[m_offset]);

    // Convertir a flotante mediante transformación lineal inversa
    return value;
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
