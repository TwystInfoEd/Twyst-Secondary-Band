#pragma once
#include <Arduino.h>

// ===== CONFIGURE THESE FOR YOUR WIRING =====
#define BATTERY_ADC_PIN    4      // secondary band battery divider tap
#define DIVIDER_RATIO      2.0f   // 10k/10k divider -> Vbat = 2 * Vadc

// Master switch: flip to false at compile time for boards that don't have
// the divider wired at all. Keeps the pin from being read/attenuated and
// guarantees battery fields never appear in the frame.
#define BATTERY_MONITORING_ENABLED true

struct BatteryPoint { float voltage; float percent; };
static const BatteryPoint BATTERY_CURVE[] = {
    {3.00f,   0.0f},
    {3.40f,   5.0f},
    {3.60f,  10.0f},
    {3.70f,  20.0f},
    {3.75f,  30.0f},
    {3.80f,  40.0f},
    {3.85f,  50.0f},
    {3.90f,  60.0f},
    {3.95f,  70.0f},
    {4.00f,  80.0f},
    {4.10f,  90.0f},
    {4.20f, 100.0f},
};
static const int BATTERY_CURVE_LEN = sizeof(BATTERY_CURVE) / sizeof(BATTERY_CURVE[0]);

// A floating (disconnected) ADC pin typically settles either near 0V or at
// noisy rail-level readings — well outside any real single-cell LiPo's
// resting range. Used as a runtime sanity check independent of the
// compile-time switch above, in case a connector works loose mid-session.
static const float PLAUSIBLE_MIN_V = 2.5f;
static const float PLAUSIBLE_MAX_V = 4.4f;

namespace Battery {

inline float voltageToPercent(float v) {
    if (v <= BATTERY_CURVE[0].voltage) return 0.0f;
    if (v >= BATTERY_CURVE[BATTERY_CURVE_LEN - 1].voltage) return 100.0f;

    for (int i = 0; i < BATTERY_CURVE_LEN - 1; ++i) {
        const BatteryPoint &a = BATTERY_CURVE[i];
        const BatteryPoint &b = BATTERY_CURVE[i + 1];
        if (v >= a.voltage && v <= b.voltage) {
            const float t = (v - a.voltage) / (b.voltage - a.voltage);
            return a.percent + t * (b.percent - a.percent);
        }
    }
    return 0.0f;
}

static float s_smoothedVoltage = -1.0f; // -1 = "not yet initialised"
static const float EMA_ALPHA = 0.15f;

inline bool isEnabled() {
    return BATTERY_MONITORING_ENABLED;
}

inline void begin() {
    if (!isEnabled()) return;
    analogReadResolution(12);
#if defined(ARDUINO_ARCH_ESP32)
    analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
#endif
}

inline float readVoltage() {
    if (!isEnabled()) return -1.0f;

    const uint32_t mv = analogReadMilliVolts(BATTERY_ADC_PIN);
    const float vAdc = mv / 1000.0f;
    const float vBat = vAdc * DIVIDER_RATIO;

    if (s_smoothedVoltage < 0.0f) {
        s_smoothedVoltage = vBat;
    } else {
        s_smoothedVoltage = EMA_ALPHA * vBat + (1.0f - EMA_ALPHA) * s_smoothedVoltage;
    }
    return s_smoothedVoltage;
}

inline float readPercent() {
    return voltageToPercent(s_smoothedVoltage < 0.0f ? readVoltage() : s_smoothedVoltage);
}
// True only if monitoring is compiled in AND the last reading looks like a
// real, connected LiPo rather than a floating pin.
inline bool isAvailable() {
    if (!isEnabled()) return false;
    return s_smoothedVoltage >= PLAUSIBLE_MIN_V && s_smoothedVoltage <= PLAUSIBLE_MAX_V;
}

} // namespace Battery