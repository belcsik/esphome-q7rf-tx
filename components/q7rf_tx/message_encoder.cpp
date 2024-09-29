#include <array>
#include <sstream>
#include <iomanip>
#include <type_traits>

#include "message_encoder.h"

namespace esphome {
namespace q7rf_tx {

std::vector<bool> operator""_bits(const char *str, size_t size) {
  std::vector<bool> v;
  for (size_t i = 0; i < size; ++i) {
    v.push_back(str[i] != '0');
  }
  return v;
}

class MessageEncoder {
 protected:
  std::vector<bool> buffer_;
  std::array<std::vector<bool>, 2> bits_;  // PWM encoded value of 0 and 1 bit

  void set_raw_encoding_() {
    bits_[0] = "0"_bits;
    bits_[1] = "1"_bits;
  }

  void set_old_encoding_() {
    bits_[0] = "011"_bits;
    bits_[1] = "001"_bits;
  }

  void set_new_encoding_() {
    bits_[0] = "110000"_bits;
    bits_[1] = "111100"_bits;
  }

  size_t start_old_message_() {
    const size_t start = buffer_.size();
    set_raw_encoding_();
    append_("111000111"_bits);
    set_old_encoding_();
    return start;
  }

  size_t start_new_message_() {
    const size_t start = buffer_.size();
    set_raw_encoding_();
    append_("111111111000000000"_bits);
    set_new_encoding_();
    return start;
  }

  void repeat_(size_t from, size_t times) {
    const size_t to = buffer_.size();
    buffer_.reserve(to + (to - from) * times);
    for (size_t i = 0; i < times; ++i) {
      std::copy(buffer_.begin() + from, buffer_.begin() + to, std::back_inserter(buffer_));
    }
  }

  size_t append_(const std::vector<bool> &data) {
    const size_t start = buffer_.size();
    for (const auto b : data) {
      std::copy(bits_[b].begin(), bits_[b].end(), std::back_inserter(buffer_));
    }
    return start;
  }

  template<typename T> std::vector<bool> to_bits_(T v, uint8_t from = sizeof(T) * 8 - 1, uint8_t to = 0) {
    static_assert(std::is_integral<T>::value, "T should be integral");
    std::vector<bool> bits;
    for (uint8_t i = from + 1; i > to; --i) {
      bits.push_back((v >> (i - 1)) & 1);
    }
    return bits;
  }

  std::vector<uint8_t> to_bytes_() {
    std::vector<bool> buffer_copy = this->buffer_;
    buffer_copy.resize(((buffer_copy.size() + 7) / 8) * 8);
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i < buffer_copy.size();) {
      uint8_t b = 0;
      for (size_t j = 0; j < 8; ++j) {
        b |= (buffer_copy[i] << (7 - j));
        ++i;
      }
      bytes.push_back(b);
    }
    return bytes;
  }

 public:
  MessageEncoder() {}

  std::vector<uint8_t> compile_message(uint16_t device_id, Command command) {
    buffer_.clear();
    set_raw_encoding_();
    {
      // preamble
      const size_t preamble_start = append_("10"_bits);
      repeat_(preamble_start, 42);
    }
    {
      // newMessage
      const size_t sync_start = start_new_message_();
      uint8_t c;
      switch (command) {
        case Command::PAIR:
          c = 0x01;
          break;
        case Command::ON:
          c = 0x80;
          break;
        default:
        case Command::OFF:
          c = 0x00;
          break;
      }

      std::array<uint8_t, 5> msg_bytes;
      msg_bytes[0] = (device_id >> 8) & 0xFF;
      msg_bytes[1] = (device_id) &0xFF;
      msg_bytes[2] = 0x80;  // ? zone
      msg_bytes[3] = c;
      msg_bytes[4] = msg_bytes[0] ^ msg_bytes[1] ^ msg_bytes[2] ^ msg_bytes[3];

      for (const auto &b : msg_bytes) {
        append_(to_bits_(b));
      }
      repeat_(sync_start, 3);
    }
    {
      // old message
      const size_t sync_start = start_old_message_();

      const size_t message_start = append_(to_bits_(device_id, 11, 8));
      append_(to_bits_(device_id, 15, 12));
      append_(to_bits_(device_id, 3, 0));
      append_(to_bits_(device_id, 7, 4));
      append_("1000"_bits);  // ? zone

      uint8_t c;
      switch (command) {
        case Command::PAIR:
          c = 0x00;
          break;
        case Command::ON:
          c = 0xFF;
          break;
        default:
        case Command::OFF:
          c = 0x0F;
          break;
      }

      append_(to_bits_(c));
      repeat_(message_start, 1);

      repeat_(sync_start, 3);
    }

    return to_bytes_();
  }
};

std::vector<uint8_t> encode_message(uint16_t device_id, Command command) {
  return MessageEncoder().compile_message(device_id, command);
}

std::string message_to_string(const std::vector<uint8_t> &bytes) {
  std::ostringstream ss;
  ss << std::hex << std::setfill('0');
  for (const auto b : bytes) {
    ss << std::setw(2) << static_cast<unsigned int>(b);
  }
  return ss.str();
}

} // namespace q7rf_tx
} // namespace esphome
