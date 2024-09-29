#pragma once

#include <queue>
#include <cstdint>
#include <functional>

namespace esphome {
namespace q7rf_tx {

class Transmitter {
 protected:
  static constexpr const char *TAG = "q7rf_tx::transmitter";

  uint8_t clk_pin_, miso_pin_, mosi_pin_, cs_pin_;
  size_t module_number_;
  int spi_status_;

  class SendTask;
  std::queue<SendTask> tasks_;

 public:
  Transmitter(uint8_t clk_pin, uint8_t miso_pin, uint8_t mosi_pin, uint8_t cs_pin);

  ~Transmitter();

  void setup();

  void loop();
  void dump_config() const;

  void send(const std::vector<uint8_t> &data, const std::function<void(bool)> &result_callback);
};

} // namespace q7rf_tx
} // namespace esphome
