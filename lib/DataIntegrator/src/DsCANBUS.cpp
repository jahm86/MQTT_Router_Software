#include "DsCANBUS.h"
#include "diutils.h"
#include <ArduinoJson.h>
#include <string.h> // Para memcpy
//#include <definitions.h>


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
    const char* mode = source["mode"].as<const char*>();
    m_mode = TWAI_MODE_NORMAL;
    if (mode != nullptr) {
        if (strcmp(mode, "listen") == 0)
            m_mode = TWAI_MODE_LISTEN_ONLY;
        else if (strcmp(mode, "noack") == 0)
            m_mode = TWAI_MODE_NO_ACK;
    }

    return DataSource::build(source, index);
}


// ************************************* CANBUSTaskQueue ************************************* //

CANBUSTaskQueue& CANBUSTaskQueue::Instance() {
  static CANBUSTaskQueue instance;
  return instance;
}

bool CANBUSTaskQueue::addId(uint32_t id, CANSignalNodeBase *node) {
    LockGuard<xSemaphoreHandle> lg(m_mutex);
    auto id_it = m_id_map.find(id);
    if (id_it != m_id_map.end())
        return false;
    m_id_map[id] = node;
    return true;
}

CANBUSTaskQueue::CANBUSTaskQueue() : m_mutex(xSemaphoreCreateMutex()), m_id_map() {
    m_task = new extTask("CAN_RX", 1024, tskIDLE_PRIORITY + 2, [this]{
        twai_message_t rx_msg;

        while(true) {
            // Wait for message (blocking operation)
            if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
                LockGuard<xSemaphoreHandle> lg(m_mutex); // change by this->m_mutex if error
                auto id_it = m_id_map.find(rx_msg.identifier);
                if (id_it != m_id_map.end()) {
                    CANSignalNodeBase* node = id_it->second;
                    // Now, send the message to the node
                    node->processCanMessage(rx_msg);
                }
            }
        }
    });
}


// ************************************ CANSignalNodeBase ************************************ //

CANSignalNodeBase::CANSignalNodeBase() : 
    m_canId(0), m_extendedId(false), m_isTx(false), m_offset(0), m_last_value(0),
    m_littleEndian(true), m_calc(nullptr), m_sendManager(nullptr) {}

CANSignalNodeBase::~CANSignalNodeBase() {
    delete m_sendManager;
    delete m_calc;
}

int CANSignalNodeBase::build(JsonObject node, int index) {
    // Validación básica
    if (!node.containsKey("id") || !node.containsKey("direction") || !node.containsKey("signals")) {
        return DIError::ErrCode::PARAM_NOT_INCLUDED;
    }

    m_canId = node["id"] | 0;
    m_extendedId = node["ext"] | false;
    m_mask = node["mask"] | 0x7FF;
    
    String direction = node["direction"] | "rx";
    m_isTx = (direction == "tx");
    
    JsonArray signals = node["signals"].as<JsonArray>();
    if (signals.isNull() || signals.size() == 0) {
        return DIError::ErrCode::PARAM_NOT_INCLUDED;
    }

    typePrms type_prms = typeParams();
    
    // Configuración de la primera señal (soporte para múltiples señales por mensaje podría añadirse)
    JsonObject signal = signals[0];
    
    m_offset = signal["offset"] | 0;
    if (m_offset + type_prms.length > 8) {
        return DIError::ErrCode::PARAM_OUT_BOUNDS;
    }
    
    m_littleEndian = (String(signal["byte_order"] | "little") == "little");
    float gain = signal["gain"] | 1.0f;
    float offset = signal["off"] | 0.0f;
    // TODO: Take calculus type from settings
    m_calc = new LinearCalc(offset, gain, type_prms.max, type_prms.min);
    m_unit = signal["unit"] | "";
    
    // Configurar el send manager
    uint32_t mintime = signal["mintime"] | 100;
    uint32_t maxtime = signal["maxtime"] | 1000;
    float delta = signal["delta"] | 0.1f;
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
        message.identifier = m_canId;
        message.extd = m_extendedId;
        typePrms type_prms = typeParams();
        message.data_length_code = m_offset + type_prms.length;
        
        packData(message, value);
        
        if (twai_transmit(&message, pdMS_TO_TICKS(100)) != ESP_OK) {
            sendError(DIError::BUS_REQ_ERROR, "CAN send failed");
        }
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
