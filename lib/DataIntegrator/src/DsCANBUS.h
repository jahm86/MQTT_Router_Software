// DsCANBUS.h actualizado
#ifndef DS_CANBUS_H
#define DS_CANBUS_H

#include "DataIntegrator.h"
#include "diutils.h"
#include <driver/twai.h>


/**
 * @class DsCANBUS
 * @brief DataSource for CANBUS devices
 */
class DsCANBUS : public DataSource {
public:
    ~DsCANBUS() override = default;
    
    int build(JsonObject source, int index) override;
    const char* type() override { return "CANBUS"; }
    static DataSource* Create() { return new DsCANBUS(); }
    bool run() {return m_twai->begin(); }

private:
    BaseProtocol *m_twai;
    twai_filter_config_t m_filter;
    twai_mode_t m_mode;
};


// Forward declaration
class CANSignalNodeBase;

/**
 * @class CANBUSTaskQueue
 * @brief handles CANBUS interrupts to receiving CANBUS nodes
 */
class CANBUSTaskQueue
{
public:
    CANBUSTaskQueue(CANBUSTaskQueue& other) = delete;
    void operator=(const CANBUSTaskQueue&) = delete;
    // Get this CANBUSTaskQueue instance
    static CANBUSTaskQueue& Instance();

    bool addId(uint32_t id, CANSignalNodeBase *node);
private:
    Semaphore m_mutex;
    extTask* m_task;
    unordered_map<uint32_t, CANSignalNodeBase*> m_id_map;
    bool m_started = false;

    CANBUSTaskQueue();
    ~CANBUSTaskQueue() = default;
};


/**
 * @class CANSignalNodeBase
 * @brief DataNode for CAN signals (TX/RX)
 * This is the base class, without tempaltes
 */
class CANSignalNodeBase : public DataNode {
public:
    struct typePrms {
        uint8_t length;
        int max;
        int min;
    };

    CANSignalNodeBase();
    virtual ~CANSignalNodeBase();
    
    int build(JsonObject node, int index) override;
    void start() override;
    void request() override;

    // Procesa un mensaje CAN recibido
    void processCanMessage(const twai_message_t &message);

protected:
    virtual typePrms typeParams() = 0;
    virtual void packData(uint8_t *payload, int irawValue) = 0;
    virtual int unpackData(const uint8_t *payload) = 0;
    bool m_littleEndian;

private:
    uint32_t m_canId;
    bool m_extendedId;
    uint32_t m_mask;
    bool m_isTx; // true transmission, false for reception
    uint32_t m_timeout;
    uint8_t m_offset;
    AnalogCalc* m_calc;
    float m_last_value;
    
    AnSendManager* m_sendManager;
    string m_unit;
    
    void packData(twai_message_t &message, float value);
    float unpackData(const twai_message_t &message);
};


/**
 * @fn swapEndianess
 * @brief Helper function to change variable endianess
 * @param varp variable pointer
 * @param sz Size of the variable
 */
void swapEndianess(void *varp, size_t sz);

/**
 * @class CANSignalNode
 * @brief DataNode para señales CAN (TX/RX)
 * Esta lleva funciones plantilla para cada tipo
 */
template <typename Type>
class CANSignalNode : public CANSignalNodeBase {
public:

    typePrms typeParamsAsType() {
        return {
            sizeof(Type),
            std::numeric_limits<Type>::max(),
            std::numeric_limits<Type>::min()
        };
    }

    void packAsType(uint8_t *payload, int irawValue) {
        Type packed = static_cast<Type>(irawValue);
        if (!m_littleEndian) {
            swapEndianess(&packed, sizeof(Type));
        }
        memcpy(payload, &packed, sizeof(Type));
    }

    int unpackAsType(const uint8_t *payload) {
        Type unpacked;
        memcpy(&unpacked, payload, sizeof(Type));
        if (!m_littleEndian) {
            swapEndianess(&unpacked, sizeof(Type));
        }
        return static_cast<int>(unpacked);
    }
};

class CAN_UINT8 : public CANSignalNode<uint8_t> {
public:
    const char* type() override { return "c_uint8"; }
    static DataNode* Create() { return new CAN_UINT8(); }
private:
    typePrms typeParams() override { return typeParamsAsType(); }
    void packData(uint8_t *payload, int irawValue) override { packAsType(payload, irawValue); }
    int unpackData(const uint8_t *payload) override { return unpackAsType(payload); }
};


class CAN_UINT16 : public CANSignalNode<uint16_t> {
public:
    const char* type() override { return "c_uint16"; }
    static DataNode* Create() { return new CAN_UINT16(); }
private:
    typePrms typeParams() override { return typeParamsAsType(); }
    void packData(uint8_t *payload, int irawValue) override { packAsType(payload, irawValue); }
    int unpackData(const uint8_t *payload) override { return unpackAsType(payload); }
};

// TODO: fix float case
class CAN_FLOAT : public CANSignalNode<float> {
public:
    const char* type() override { return "c_float"; }
    static DataNode* Create() { return new CAN_FLOAT(); }
private:
    typePrms typeParams() override { return { sizeof(int), std::numeric_limits<int>::max(), std::numeric_limits<int>::min() }; }
    void packData(uint8_t *payload, int irawValue) override { packAsType(payload, irawValue); }
    int unpackData(const uint8_t *payload) override { return unpackAsType(payload); }
};

#endif // DS_CANBUS_H