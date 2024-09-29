#include "debug_logger.h"

namespace esphome {
namespace q7rf_tx {
const bool DebugLogger::ENABLED = (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE);

void DebugLogger::flush_() {
  if (ENABLED) {
    ESP_LOGV(tag_, "%s", ss_.str().c_str());
    ss_.str("");
    ss_.clear();
  }
}

void DebugLogger::end_of_line_() {
  if (mode_ == FLUSH_LINE) {
    flush_();
  }
}

DebugLogger::LineScope::LineScope(DebugLogger *logger) : logger_(logger) {}

DebugLogger::LineScope::LineScope(LineScope &&s) noexcept : logger_(s.logger_) { s.logger_ = nullptr; }

DebugLogger::LineScope::~LineScope() {
  if (logger_) {
    logger_->end_of_line_();
  }
}

DebugLogger::DebugLogger(Mode mode, const char *tag) : mode_(mode), tag_(tag) {}

DebugLogger::~DebugLogger() {
  if (mode_ == FLUSH_COMPLETE) {
    flush_();
  }
}

} // namespace q7rf_tx
} // namespace esphome
