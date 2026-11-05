#pragma once

// =============================================================================
// units.h
// Unit conversion functions for Feetech HL-3606-C001 servo
//
// All "raw" values are the integers returned by / passed to the HLSCL library.
// All "real" values are in SI or clearly stated units.
//
// Servo specs (at 6V):
//   Encoder:        12-bit (0 - 4095 ticks per revolution)
//   Stall torque:   6 kg/cm = 0.5884 Nm
//   No-load speed:  111 RPM
//   Stall current:  1.3 A
//   Gear ratio:     1/205 (output shaft)
// =============================================================================

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Physical constants and servo parameters
// ---------------------------------------------------------------------------

// Position: 4096 ticks = 360 degrees = 2*PI radians
static constexpr float TICKS_PER_REV      = 4096.0f;
static constexpr float DEG_PER_TICK       = 360.0f / TICKS_PER_REV;   // 0.08789 deg/tick
static constexpr float RAD_PER_TICK       = (2.0f * PI) / TICKS_PER_REV; // 0.001534 rad/tick
static constexpr float TICKS_PER_DEG      = TICKS_PER_REV / 360.0f;   // 11.378 ticks/deg
static constexpr float TICKS_PER_RAD      = TICKS_PER_REV / (2.0f * PI); // 651.9 ticks/rad

// Speed: 1 raw unit = 0.732 RPM (Feetech's unit)
static constexpr float RPM_PER_SPEED_UNIT = 0.732f;
static constexpr float RADPS_PER_SPEED_UNIT = RPM_PER_SPEED_UNIT * (2.0f * PI) / 60.0f; // 0.07665 rad/s per unit
static constexpr float SPEED_UNITS_PER_RADPS = 1.0f / RADPS_PER_SPEED_UNIT;              // 13.045 units per rad/s

// Acceleration: 1 raw unit = 8.7 deg/s^2 (Feetech's figure, not measured here)
static constexpr float DEGPS2_PER_ACC_UNIT  = 8.7f;
static constexpr float RADPS2_PER_ACC_UNIT  = DEGPS2_PER_ACC_UNIT * (PI / 180.0f); // 0.1518 rad/s^2 per unit
static constexpr float ACC_UNITS_PER_RADPS2 = 1.0f / RADPS2_PER_ACC_UNIT;           // 6.587 units per rad/s^2

// Torque / Load:
//   Load raw range: -1000 to +1000 (percentage of stall torque)
//   Stall torque at 6V: 6.0 kg/cm = 0.5884 Nm
static constexpr float STALL_TORQUE_KGCM  = 6.0f;
static constexpr float KGCM_TO_NM         = 0.0980665f;
static constexpr float STALL_TORQUE_NM    = STALL_TORQUE_KGCM * KGCM_TO_NM; // 0.5884 Nm
static constexpr float NM_PER_LOAD_UNIT   = STALL_TORQUE_NM / 1000.0f;       // 0.0005884 Nm per raw unit
static constexpr float LOAD_UNITS_PER_NM  = 1.0f / NM_PER_LOAD_UNIT;         // 1699.0 units per Nm

// Current: 1 raw unit = 3.25 mA, measured on the arm (Feetech's documents say 6.5 mA)
static constexpr float AMPS_PER_CURRENT_UNIT = 0.00325f;
static constexpr float STALL_CURRENT_A       = 1.3f;
// Raw stall current: 1.3 A / 3.25 mA = 400 units
static constexpr int   STALL_CURRENT_RAW     = 400;

// Voltage: 1 raw unit = 0.1 V (standard Feetech protocol)
static constexpr float VOLTS_PER_VOLTAGE_UNIT = 0.1f;

// Temperature: raw value is directly in degrees Celsius, no conversion needed

// ---------------------------------------------------------------------------
// Position conversions
// ---------------------------------------------------------------------------

// Raw ticks (0-4095, as returned by ReadPos) -> radians
inline float ticksToRad(int ticks)
{
    return (float)ticks * RAD_PER_TICK;
}

// Raw ticks -> degrees
inline float ticksToDeg(int ticks)
{
    return (float)ticks * DEG_PER_TICK;
}

// Radians -> raw ticks (clamped to 0-4095)
inline int radToTicks(float radians)
{
    int ticks = (int)(radians * TICKS_PER_RAD);
    if (ticks < 0)    ticks = 0;
    if (ticks > 4095) ticks = 4095;
    return ticks;
}

// Degrees -> raw ticks (clamped to 0-4095)
inline int degToTicks(float degrees)
{
    int ticks = (int)(degrees * TICKS_PER_DEG);
    if (ticks < 0)    ticks = 0;
    if (ticks > 4095) ticks = 4095;
    return ticks;
}

// ---------------------------------------------------------------------------
// Speed conversions
// ---------------------------------------------------------------------------

// Raw speed units (signed, as returned by ReadSpeed) -> rad/s
// Sign is preserved: positive = CCW, negative = CW (per HLSCL convention)
inline float speedToRadPerSec(int rawSpeed)
{
    return (float)rawSpeed * RADPS_PER_SPEED_UNIT;
}

// Raw speed units -> RPM
inline float speedToRPM(int rawSpeed)
{
    return (float)rawSpeed * RPM_PER_SPEED_UNIT;
}

// rad/s -> raw speed units (for WriteSpe / WritePosEx)
// Returns a signed integer; caller should pass abs value + direction bit if needed
inline int radPerSecToSpeed(float radps)
{
    return (int)(radps * SPEED_UNITS_PER_RADPS);
}

// RPM -> raw speed units
inline int rpmToSpeed(float rpm)
{
    return (int)(rpm / RPM_PER_SPEED_UNIT);
}

// ---------------------------------------------------------------------------
// Acceleration conversions
// ---------------------------------------------------------------------------

// Raw acceleration units -> rad/s^2
inline float accToRadPerSec2(uint8_t rawAcc)
{
    return (float)rawAcc * RADPS2_PER_ACC_UNIT;
}

// rad/s^2 -> raw acceleration units (clamped to 0-255 for uint8_t)
inline uint8_t radPerSec2ToAcc(float radps2)
{
    int acc = (int)(radps2 * ACC_UNITS_PER_RADPS2);
    if (acc < 0)   acc = 0;
    if (acc > 255) acc = 255;
    return (uint8_t)acc;
}

// ---------------------------------------------------------------------------
// Torque / Load conversions
// ---------------------------------------------------------------------------

// Raw load value (signed -1000..+1000, as returned by ReadLoad) -> Nm
// Sign indicates direction: positive = CW load, negative = CCW load
inline float loadToNm(int rawLoad)
{
    return (float)rawLoad * NM_PER_LOAD_UNIT;
}

// Nm -> raw torque limit command value for WritePosEx / WriteSpe (unsigned, 0-1000)
// Input should be positive (magnitude only)
inline uint16_t nmToTorqueCmd(float nm)
{
    int cmd = (int)(nm * LOAD_UNITS_PER_NM);
    if (cmd < 0)    cmd = 0;
    if (cmd > 1000) cmd = 1000;
    return (uint16_t)cmd;
}

// ---------------------------------------------------------------------------
// Current conversions
// ---------------------------------------------------------------------------

// Raw current (as returned by ReadCurrent) -> Amperes
inline float currentToAmps(int rawCurrent)
{
    return (float)rawCurrent * AMPS_PER_CURRENT_UNIT;
}

// Amperes -> raw current units
inline int ampsToCurrentRaw(float amps)
{
    return (int)(amps / AMPS_PER_CURRENT_UNIT);
}

// Returns true if raw current indicates a stall condition (>= stall threshold)
inline bool isCurrentStalled(int rawCurrent)
{
    return (rawCurrent >= STALL_CURRENT_RAW);
}

// ---------------------------------------------------------------------------
// Voltage conversions
// ---------------------------------------------------------------------------

// Raw voltage (as returned by ReadVoltage) -> Volts
inline float voltageToVolts(int rawVoltage)
{
    return (float)rawVoltage * VOLTS_PER_VOLTAGE_UNIT;
}

// ---------------------------------------------------------------------------
// Temperature
// ---------------------------------------------------------------------------

// Raw temperature is already in degrees Celsius - provided for completeness
// and to keep a uniform access pattern across all values.
inline float temperatureToC(int rawTemp)
{
    return (float)rawTemp;
}

// ---------------------------------------------------------------------------
// Movement time estimation
// Estimate how long a position move will take given distance and speed setting.
// Useful to know how long to wait before reading the final position.
// ---------------------------------------------------------------------------

// Returns estimated move duration in milliseconds given:
//   distanceTicks: abs(targetTick - currentTick)
//   speedRaw:      speed command value (raw units)
//   marginFactor:  safety margin multiplier (default 1.2 = 20% extra)
inline uint32_t estimateMoveDurationMs(int distanceTicks, int speedRaw, float marginFactor = 1.2f)
{
    if (speedRaw <= 0) return 0;
    float seconds = (float)distanceTicks / (float)speedRaw;
    return (uint32_t)(seconds * 1000.0f * marginFactor);
}
