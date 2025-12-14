// DsCANBUS.h actualizado
#ifndef DS_CANBUS_H
#define DS_CANBUS_H

#include "DataIntegrator.h"
#include "diutils.h"
#include <driver/twai.h>

// Forward declaration
class CANSignalNodeBase;
class TwaiProtocol;

/**
 * @class DsCANBUS
 * @brief DataSource for CANBUS devices
 */
class DsCANBUS : public DataSource {
public:
    ~DsCANBUS();
    
    int build(JsonObject source, int index) override;
    const char* type() override { return "CANBUS"; }
    static DataSource* Create() { return new DsCANBUS(); }
    bool runProtocol();
    bool addId(uint32_t id, CANSignalNodeBase *node);
    esp_err_t sendMessage(uint32_t id, const uint8_t* data, uint8_t length, int timeout_ms);
    TwaiProtocol *protocol();

private:
    TwaiProtocol *m_twai;
};


/**
 * @class CANSignalNodeBase
 * @brief DataNode for CAN signals (TX/RX)
 * This is the base class, without tempaltes
 */
class CANSignalNodeBase : public DataNode {
public:

    CANSignalNodeBase();
    virtual ~CANSignalNodeBase();
    
    int build(JsonObject node, int index) override;
    void start() override;
    void request() override;

    // Proccess tx CANBUS messagge
    void processCanMessage(const twai_message_t &message);

    // Enumerator for communication direction
    enum class CommDir : char {
        rx = 'R',   ///> Reception (from CAN bus to this board)
        tx = 'T',   ///> Transmission (from this board to CAN bus)
        rtx = 'X'   ///> Both directions
    };

protected:
    virtual size_t typeLen() = 0;
    virtual void packData(uint8_t *payload, float value) = 0;
    virtual float unpackData(const uint8_t *payload) = 0;
    virtual void buildCalc(float offset, float gain) = 0;
    bool m_littleEndian;

private:
    uint32_t m_canId;
    bool m_extendedId;
    uint32_t m_mask;
    CommDir m_dir;
    int m_timeout;
    uint8_t m_offset;
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
 * This leads template functions for every type
 */
template <typename Type>
class CANSignalNode : public CANSignalNodeBase {
public:

    CANSignalNode() : CANSignalNodeBase(), m_calc(nullptr) {}

    ~CANSignalNode() { delete m_calc; }

    size_t typeLen() override { return sizeof(Type); }
    
    void buildCalc(float offset, float gain) override { m_calc = new LinearCalc<Type>(offset, gain); }

    void packData(uint8_t *payload, float value) override {
        // Apply linear transformation
        Type packed = m_calc->receive(value);
        if (!m_littleEndian) {
            swapEndianess(&packed, sizeof(Type));
        }
        memcpy(payload, &packed, sizeof(Type));
    }

    float unpackData(const uint8_t *payload) override {
        Type unpacked;
        memcpy(&unpacked, payload, sizeof(Type));
        if (!m_littleEndian) {
            swapEndianess(&unpacked, sizeof(Type));
        }

        // Apply linear transformation
        float retval = m_calc->send(unpacked);
        return retval;
    }
    
private:
    LinearCalc<Type> *m_calc;
};

// Macro for easy management of CANSignalNode subclasses
#define CANSigNode(typeId) class CAN_##typeId : public CANSignalNode<typeId##_t> { \
public: \
    const char* type() override { return "c_"#typeId; } \
    static DataNode* Create() { return new CAN_##typeId(); } \
}

CANSigNode(uint8);

CANSigNode(uint16);

CANSigNode(uint32);

CANSigNode(int8);

CANSigNode(int16);

CANSigNode(int32);

CANSigNode(float);

#endif // DS_CANBUS_H
