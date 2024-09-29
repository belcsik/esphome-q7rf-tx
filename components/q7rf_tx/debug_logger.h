#pragma once

#include <sstream>
#include "esphome/core/log.h"

namespace esphome {
namespace q7rf_tx {

class DebugLogger {
 public:
  enum Mode { FLUSH_LINE, FLUSH_COMPLETE };

 protected:
  static const bool ENABLED;

  std::ostringstream ss_;
  const Mode mode_;
  const char *tag_;

  void flush_();
  void end_of_line_();

 public:
  class LineScope {
   protected:
    DebugLogger *logger_;

   public:
    LineScope(DebugLogger *logger);
    LineScope(LineScope &&s) noexcept;

    template<typename T> LineScope &operator<<(T &&v) {
      if (ENABLED) {
        logger_->ss_ << std::forward<T>(v);
      }
      return *this;
    }

    ~LineScope();
  };

  DebugLogger(Mode mode, const char *tag);

  DebugLogger(const DebugLogger &) = delete;
  DebugLogger &operator=(const DebugLogger &) = delete;

  ~DebugLogger();

  template<typename T> LineScope operator<<(T &&v) {
    LineScope s(this);
    s << std::forward<T>(v);
    return s;
  }
};

} // namespace q7rf_tx
} // namespace esphome
