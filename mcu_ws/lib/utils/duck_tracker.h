#pragma once
/**
 * @file duck_tracker.h
 * @author Rafeed Khan
 * @brief Tracks collected ducks during a match (bounded counter)
 */

#include <cstdint>

namespace secbot::utils {

class DuckTracker {
 public:
  explicit constexpr DuckTracker(uint8_t max_capacity = 6)
      : count_(0), max_(max_capacity) {}

  constexpr void collect() {
    if (count_ < max_) ++count_;
  }
  constexpr void drop() {
    if (count_ > 0) --count_;
  }
  constexpr void reset() { count_ = 0; }

  constexpr uint8_t count() const { return count_; }
  constexpr uint8_t capacity() const { return max_; }
  constexpr uint8_t remaining() const { return max_ - count_; }
  constexpr bool full() const { return count_ >= max_; }
  constexpr bool empty() const { return count_ == 0; }

 private:
  uint8_t count_;
  uint8_t max_;
};

}  // namespace secbot::utils