#pragma once

#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace q7rf_tx {

class Device;
class Transmitter;

class PairButton : public esphome::button::Button {
 protected:
  Device *device_ = nullptr;
  void press_action() override;
  static constexpr const char *TAG = "q7rf_tx::pair_button";

 public:
  void set_device(Device *device);
};

class ThermostatSwitch : public esphome::switch_::Switch {
 protected:
  Device *device_ = nullptr;
  void write_state(bool state) override;
  static constexpr const char *TAG = "q7rf_tx::thermostat_switch";

 public:
  void set_device(Device *device);
};

class Q7RfTx : public esphome::Component {
 protected:
  std::unique_ptr<Transmitter> transmitter_;
  std::vector<std::unique_ptr<Device>> devices_;

  void loop() override;

 public:
  Q7RfTx();
  ~Q7RfTx();

  void set_spi(uint8_t clk_pin, uint8_t miso_pin, uint8_t mosi_pin, uint8_t cs_pin);

  void add_device(uint16_t device_id_in, uint32_t resend_interval_in, uint32_t watchdog_interval_in,
                  PairButton *pair_button_in, ThermostatSwitch *thermostat_switch_in);

  void setup() override;
  void dump_config() override;
};

} // namespace q7rf_tx
} // namespace esphome
