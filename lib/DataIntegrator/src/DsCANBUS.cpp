#include "DsCANBUS.h"
#include "diutils.h"
#include <ArduinoJson.h>
#include <string.h> // Para memcpy
//#include <definitions.h>


// ************************************** CANSignalNode ************************************** //

TWAI_Object::twai_filter_type_t setType(String typeS) {
    if (typeS == "mask") {
        return TWAI_Object::twai_filter_type_t::TWAI_FILTER_TYPE_MASK;
    } else if (typeS == "list") {
        return TWAI_Object::twai_filter_type_t::TWAI_FILTER_TYPE_LIST;
    }
    return TWAI_Object::twai_filter_type_t::TWAI_FILTER_TYPE_RANGE;
}

int DsCANBUS::build(JsonObject source, int index) {
    TWAI_Object::twai_user_filter_t filter;

    filter.id = source["id"];
    filter.mask_or_end_id = source["mask"];
    filter.is_extended = source["ext"] | false;
    filter.type = setType(source["type"]);

    TWAI_Object::twai.set_filters(&filter, 1);

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
        TWAI_Object::can_event_t event;

        while(true) {
            // Wait for message (blocking operation)
            if (xQueueReceive(TWAI_Object::twai.get_event_queue(), &event, portMAX_DELAY)) {
                LockGuard<xSemaphoreHandle> lg(m_mutex); // change by this->m_mutex if error
                
                auto id_it = m_id_map.find(event.message.identifier);
                if (!event.is_error && id_it != m_id_map.end()) {
                    CANSignalNodeBase* node = id_it->second;
                    // Now, send the message to the node
                    node->processCanMessage(event.message);
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
        
        if (!TWAI_Object::twai.send(message)) {
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
