#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <climits>

#include "q7rf_tx.h"
#include "transmitter.h"
#include "message_encoder.h"

namespace esphome {
namespace q7rf_tx {

class Device {
 protected:
  const uint16_t device_id_;
  const uint32_t resend_interval_;
  const uint32_t watchdog_interval_;
  const std::vector<uint8_t> pair_message_;
  const std::vector<uint8_t> on_message_;
  const std::vector<uint8_t> off_message_;

  ThermostatSwitch *thermostat_switch_;
  Transmitter *transmitter_;

  bool state_;

  uint32_t last_switch_millis_;
  uint32_t last_resend_millis_;

  static constexpr const char *TAG = "q7rf_tx::device";

  static uint32_t elapsed(uint32_t since, uint32_t now);
  void send_state_();
  void set_state_(bool state);

 public:
  Device(uint16_t device_id, uint32_t resend_interval, uint32_t watchdog_interval, PairButton *pair_button,
         ThermostatSwitch *thermostat_switch, Transmitter *transmitter);

  void dump_config() const;
  void pair() const;
  void set_switch(bool state);
  void loop();
};

uint32_t Device::elapsed(uint32_t since, uint32_t now) {
  if (since > now) {
    // millis() overflows every ~50 days
    return (UINT32_MAX - since) + now;
  } else {
    return now - since;
  }
}

void Device::send_state_() {
  last_resend_millis_ = millis();
  ESP_LOGV(TAG, "0x%x started sending state %d", device_id_, state_);
  const bool state = state_;
  ThermostatSwitch *const thermostat_switch = thermostat_switch_;
  const uint16_t device_id = device_id_;
  transmitter_->send(state_ ? on_message_ : off_message_, [state, thermostat_switch, device_id](bool success) {
    if (success) {
      thermostat_switch->publish_state(state);
      ESP_LOGV(TAG, "0x%x succeeded sending state %d", device_id, state);
    } else {
      ESP_LOGV(TAG, "0x%x failed sending state %d", device_id, state);
    }
  });
}

void Device::set_state_(bool state) {
  if (this->state_ != state) {
    ESP_LOGV(TAG, "0x%x setting state %d", device_id_, state);
    this->state_ = state;
    send_state_();
  } else {
    ESP_LOGV(TAG, "0x%x state %d already set", device_id_, state);
  }
}

Device::Device(uint16_t device_id, uint32_t resend_interval, uint32_t watchdog_interval, PairButton *pair_button,
               ThermostatSwitch *thermostat_switch, Transmitter *transmitter)
    : device_id_(device_id),
      resend_interval_(resend_interval),
      watchdog_interval_(watchdog_interval),
      pair_message_(encode_message(device_id, Command::PAIR)),
      on_message_(encode_message(device_id, Command::ON)),
      off_message_(encode_message(device_id, Command::OFF)),
      thermostat_switch_(thermostat_switch),
      transmitter_(transmitter),
      state_(false),
      last_switch_millis_(0),
      last_resend_millis_(0) {
  pair_button->set_device(this);
  thermostat_switch_->set_device(this);

  ESP_LOGV(TAG, "Pair message: %d %s", pair_message_.size(), message_to_string(pair_message_).c_str());
  ESP_LOGV(TAG, "On message: %d %s", on_message_.size(), message_to_string(on_message_).c_str());
  ESP_LOGV(TAG, "Off message: %d %s", off_message_.size(), message_to_string(off_message_).c_str());
}

void Device::dump_config() const {
  ESP_LOGCONFIG(TAG, "  Device ID: 0x%x", device_id_);
  ESP_LOGCONFIG(TAG, "  Resend interval: %d", resend_interval_);
  ESP_LOGCONFIG(TAG, "  Watchdog interval: %d", watchdog_interval_);
}

void Device::pair() const { transmitter_->send(pair_message_, nullptr); }

void Device::set_switch(bool state) {
  last_switch_millis_ = millis();
  ESP_LOGV(TAG, "0x%x switch received new state %d", device_id_, state);
  set_state_(state);
}

void Device::loop() {
  const uint32_t now = millis();
  if (watchdog_interval_ != 0 && elapsed(last_switch_millis_, now) > watchdog_interval_) {
    ESP_LOGW(TAG, "0x%x watchdog timeout. Setting state to off.", device_id_);
    set_state_(false);
  }
  if (elapsed(last_resend_millis_, now) > resend_interval_) {
    ESP_LOGV(TAG, "0x%x resending state %d", device_id_, state_);
    send_state_();
  }
}

void PairButton::press_action() {
  ESP_LOGV(TAG, "Pressed");
  device_->pair();
}

void PairButton::set_device(Device *device) { device_ = device; }

void ThermostatSwitch::write_state(bool state) {
  ESP_LOGV(TAG, "switched");
  device_->set_switch(state);
}

void ThermostatSwitch::set_device(Device *device) { device_ = device; }

Q7RfTx::Q7RfTx() = default;
Q7RfTx::~Q7RfTx() = default;

void Q7RfTx::set_spi(uint8_t clk_pin, uint8_t miso_pin, uint8_t mosi_pin, uint8_t cs_pin) {
  transmitter_ = make_unique<Transmitter>(clk_pin, miso_pin, mosi_pin, cs_pin);
}

void Q7RfTx::add_device(uint16_t device_id_in, uint32_t resend_interval_in, uint32_t watchdog_interval_in,
                            PairButton *pair_button_in, ThermostatSwitch *thermostat_switch_in) {
  devices_.emplace_back(make_unique<Device>(device_id_in, resend_interval_in, watchdog_interval_in, pair_button_in,
                                            thermostat_switch_in, transmitter_.get()));
}

void Q7RfTx::setup() {
  transmitter_->setup();
  }

void Q7RfTx::dump_config() {
  transmitter_->dump_config();
  for (const auto &d : devices_) {
    d->dump_config();
  }
}

void Q7RfTx::loop() {
  for (const auto &d : devices_) {
    d->loop();
  }
  transmitter_->loop();
}

} // namespace q7rf_tx
} // namespace esphome
