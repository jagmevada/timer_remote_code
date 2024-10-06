#pragma once
#include <Arduino.h>

class ConfigurableDownCounter {
   public:
    ConfigurableDownCounter() : minutes_(42), seconds_(42) {}

    ConfigurableDownCounter(uint8_t minutes, uint8_t seconds)
        : minutes_(min(minutes, kMaxMinutes)),
          seconds_(min(seconds, kMaxSeconds)) {}

    void IncrementTensMinuteDigit() {
        if (minutes_ / 10 < kMaxDigit) {
            minutes_ += 10;
        }
    }
    void DecrementTensMinuteDigit() {
        if (minutes_ / 10 > kMinDigit) {
            minutes_ -= 10;
        }
    }

    void IncrementOnesMinuteDigit() {
        if (minutes_ % 10 < kMaxDigit) {
            ++minutes_;
        }
    }
    void DecrementOnesMinuteDigit() {
        if (minutes_ % 10 > kMinDigit) {
            --minutes_;
        }
    }

    void IncrementTensSecondDigit() {
        if (seconds_ / 10 < kMaxTensSecondDigit) {
            seconds_ += 10;
        }
    }
    void DecrementTensSecondDigit() {
        if (seconds_ / 10 > kMinDigit) {
            seconds_ -= 10;
        }
    }

    void IncrementOnesSecondDigit() {
        if (seconds_ % 10 < kMaxDigit) {
            ++seconds_;
        }
    }
    void DecrementOnesSecondDigit() {
        if (seconds_ % 10 > kMinDigit) {
            --seconds_;
        }
    }

    /// @brief Decrements the timer by one second.
    void CountDownByOneSecond() {
        if (not IsComplete()) {
            if (seconds_ == kMinSeconds) {
                seconds_ = kMaxSeconds;
                --minutes_;
            } else {
                --seconds_;
            }
        }
    }

    /// @brief Decrements the timer by the given number of seconds.
    void CountDownBySeconds(uint64_t num_seconds) {
        if (not IsComplete()) {
            uint64_t current_seconds = (minutes_ * 60) + seconds_;
            current_seconds = max(0, current_seconds - num_seconds);
            minutes_ = min(current_seconds / 60, kMaxMinutes);
            seconds_ = min(current_seconds % 60, kMaxSeconds);
        }
    }

    /// @brief Writes count to the output array.
    /// @param output Output array to write to. MUST have
    /// size 6.
    void SetOutputString(char *output) {
        snprintf(output, 6, "%d%d:%d%d", minutes_ / 10, minutes_ % 10,
                 seconds_ / 10, seconds_ % 10);
    }

    uint8_t GetMinutes() const { return minutes_; }

    uint8_t GetSeconds() const { return seconds_; }

   private:
    static const uint8_t kMaxMinutes = 99;
    static const uint8_t kMinMinutes = 0;

    static const uint8_t kMaxSeconds = 59;
    static const uint8_t kMinSeconds = 0;

    static const uint8_t kMinDigit = 0;
    static const uint8_t kMaxDigit = 9;
    static const uint8_t kMaxTensSecondDigit = 5;

    inline bool IsComplete() {
        return seconds_ == kMinSeconds && minutes_ == kMinMinutes;
    }

    uint8_t seconds_ = 0;
    uint8_t minutes_ = 0;
};