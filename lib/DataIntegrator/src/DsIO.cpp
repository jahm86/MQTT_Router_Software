
#include "DsIO.h"
#include <Arduino.h>

DIError::ErrCode checkPin(JsonObject node, uint8_t& m_pin) {
  int pin = node["pin"] | -1;
  if (pin < 0 || pin > UINT8_MAX) {
    log_e("Pin outside range or not defined: %d", pin);
    return DIError::ErrCode::PARAM_OUT_BOUNDS;
  }
  m_pin = pin;
  return DIError::ErrCode::SUCCESS;
}


//************************************** Analog Input Node **************************************//

AnIn::~AnIn() {
  delete m_calc;
  delete m_sendman;
}

int AnIn::build(JsonObject node, int index) {
  DIError::ErrCode code = checkPin(node, m_pin);
  if (code != DIError::ErrCode::SUCCESS)
    return code;
  // Add linear calculator
  float offset = node["off"];
  float gain = node["gain"] | 1.0;
  m_bits = node["bits"] | UINT8_MAX;
  m_calc = new LinearCalc(offset, gain, (1UL << m_bits) - 1);
  // Add send manager
  uint32_t mintime = node["mintime"] | 1;
  uint32_t maxtime = node["maxtime"] | 60;
  float delta = node["delta"] | 0.1;
  m_sendman = new AnSendManager(mintime, maxtime, delta);

  m_unit = node["unit"].as<string>();
  return DataNode::build(node, index);
}

void AnIn::start() {
  if (m_bits != UINT8_MAX) {
    analogReadResolution(m_bits);
  }
  pinMode(m_pin, INPUT);
  request();
}

void AnIn::request() {
  if (m_cmd == DataNode::WRITE) {
    sendError(DIError::BUS_WR_DENIED);
    return;
  }
  uint16_t value = analogRead(m_pin);
  float fValue = m_calc->send(value);
  log_d("ADC: %u  Calc: %f", value, fValue);
  if (!m_sendman->updateRequired(fValue, m_cmd == Cmd::READ_FORCED || m_cmd == Cmd::WRITE))
    return;
  string& unit = m_unit;
  bool found = m_source->onResponse(m_name, [fValue, unit](JsonObject object) {
    object["value"] = fValue;
    if(!unit.empty())
      object["unit"] = unit;
  });
  log_d("Add %s: %s", m_name.c_str(), found ? "exist" : "new");
}


//************************************** Digital Input Node **************************************//

DgIn::~DgIn() {
  delete m_calc;
  delete m_sendman;
}

int DgIn::build(JsonObject node, int index) {
  DIError::ErrCode code = checkPin(node, m_pin);
  if (code != DIError::ErrCode::SUCCESS)
    return code;
  // Add digital calculator
  bool invert = node["inv"];
  m_calc = new DigitalCalc(invert);
  // Add send manager
  uint32_t mintime = node["mintime"] | 1;
  uint32_t maxtime = node["maxtime"] | 60;
  m_sendman = new DgSendManager(mintime, maxtime);

  return DataNode::build(node, index);
}

void DgIn::start() {
  pinMode(m_pin, INPUT);
  request();
}

void DgIn::request() {
  if (m_cmd == DataNode::WRITE) {
    sendError(DIError::BUS_WR_DENIED);
    return;
  }
  uint8_t value = digitalRead(m_pin);
  bool bValue = m_calc->send(value);
  log_d("GPIO In: %u  Calc: %s", value, bValue ? "true" : "false");
  if (!m_sendman->updateRequired(bValue, m_cmd == Cmd::READ_FORCED || m_cmd == Cmd::WRITE))
    return;
  bool found = m_source->onResponse(m_name, [bValue](JsonObject object) {
    object["value"] = bValue;
  });
  log_d("Add %s: %s", m_name.c_str(), found ? "exist" : "new");
}


//************************************** Digital Output Node **************************************//

DgOut::~DgOut() {
  delete m_calc;
  delete m_sendman;
}

int DgOut::build(JsonObject node, int index) {
  DIError::ErrCode code = checkPin(node, m_pin);
  if (code != DIError::ErrCode::SUCCESS)
    return code;
  // Add digial calculator
  bool invert = node["inv"];
  m_calc = new DigitalCalc(invert);
  // Add send manager
  uint32_t mintime = node["mintime"] | 1;
  uint32_t maxtime = node["maxtime"] | 60;
  m_sendman = new DgSendManager(mintime, maxtime);

  m_init_state = node["init"];
  return DataNode::build(node, index);
}

void DgOut::start() {
  pinMode(m_pin, OUTPUT);
  uint8_t value = m_calc->receive(m_init_state);
  digitalWrite(m_pin, value);
  request();
}

void DgOut::request() {
  if (m_cmd == DataNode::WRITE) {
    if (!m_variant.can_convert<bool>()) {
      sendError(DIError::PARAM_WORNG_TYPE);
      return;
    }
    bool bValue = m_variant.convert<bool>();
    uint8_t value = m_calc->receive(bValue);
    digitalWrite(m_pin, value);
  }
  uint8_t value = digitalRead(m_pin);
  bool bValue = m_calc->send(value);
  log_d("GPIO Out: %u  Calc: %s", value, bValue ? "true" : "false");
  if (!m_sendman->updateRequired(bValue, m_cmd == Cmd::READ_FORCED || m_cmd == Cmd::WRITE))
    return;
  bool found = m_source->onResponse(m_name, [bValue](JsonObject object) {
    object["value"] = bValue;
  });
  log_d("Add %s: %s", m_name.c_str(), found ? "exist" : "new");
}


//************************************** PWM Output Node **************************************//

PWM::~PWM() {
  delete m_calc;
  delete m_sendman;
}

int PWM::build(JsonObject node, int index) {
  DIError::ErrCode code = checkPin(node, m_pin);
  if (code != DIError::ErrCode::SUCCESS)
    return code;
  // Add linear calculator
  float offset = node["off"];
  float gain = node["gain"] | 1.0;
  m_bits = node["bits"] | 8;
  m_calc = new LinearCalc(offset, gain, (1UL << m_bits) - 1);
  // Add send manager
  uint32_t mintime = node["mintime"] | 1;
  uint32_t maxtime = node["maxtime"] | 60;
  float delta = node["delta"] | 0.1;
  m_sendman = new AnSendManager(mintime, maxtime, delta);
  // Add PWM parameters
  m_unit = node["unit"].as<string>();
  m_freq = node["freq"] | 5000;
  m_init_value = node["init"] | 0.0;
  m_channel = -1;
  return DataNode::build(node, index);
}

void PWM::start() {
  m_channel = LedcChan::Instance().addChannel();
  uint32_t duty = m_calc->receive(m_init_value);
  // configure LED PWM functionalitites
  ledcSetup(m_channel, m_freq, m_bits);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(m_pin, m_channel);
  // changing the PWM duty cycle
  ledcWrite(m_channel, duty);
  request();
}

void PWM::request() {
  if (m_cmd == DataNode::WRITE) {
    if (!m_variant.can_convert<float>()) {
      sendError(DIError::PARAM_WORNG_TYPE);
      return;
    }
    float fValue = m_variant.convert<float>();
    uint32_t duty = m_calc->receive(fValue);
    ledcWrite(m_channel, duty);
  }
  uint32_t duty = ledcRead(m_channel);
  float fValue = m_calc->send(duty);
  log_d("PWM: %u  Calc: %f", duty, fValue);
  if (!m_sendman->updateRequired(fValue, m_cmd == Cmd::READ_FORCED || m_cmd == Cmd::WRITE))
    return;
  string& unit = m_unit;
  bool found = m_source->onResponse(m_name, [fValue, unit](JsonObject object) {
    object["value"] = fValue;
    if(!unit.empty())
      object["unit"] = unit;
  });
  log_d("Add %s: %s", m_name.c_str(), found ? "exist" : "new");
}
