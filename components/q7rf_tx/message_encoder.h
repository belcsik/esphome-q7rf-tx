#pragma once

#include <vector>
#include <cstdint>
#include <string>

namespace esphome {
namespace q7rf_tx {

enum class Command {
  PAIR = 0,
  ON,
  OFF,
};

std::vector<uint8_t> encode_message(uint16_t device_id, Command command);
std::string message_to_string(const std::vector<uint8_t> &bytes);

} // namespace q7rf_tx
} // namespace esphome
