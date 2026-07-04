// ======================================================================
// Legacy deck robot: ESP32 + DRV8871 x2 + hall encoders + GY-9250.
// DRV8871 IN1/IN2 mode: PWM on one input, other held LOW (sign-magnitude).
// Verify pins against your 38-pin ESP32 dev board silkscreen.
// ======================================================================
#pragma once

// Right/left encoder inputs. Right=GPIO34, left=GPIO32 (spread apart on 38-pin
// boards; avoid adjacent 32/33 or 34/35 pairs). Keep OUT wires away from motor
// power; 3.3V logic only.
static const int PIN_ENC_RIGHT = 34;
static const int PIN_ENC_LEFT  = 32;

// ---------------- DRV8871 x2 (one chip per wheel) -------------------------
// Forward: IN1=PWM, IN2=LOW   |   Reverse: IN1=LOW, IN2=PWM
static const int PIN_R_IN1 = 26;
static const int PIN_R_IN2 = 27;
static const int PIN_L_IN1 = 16;
static const int PIN_L_IN2 = 17;

// ---------------- I2C — GY-9250 (MPU9250) + VL53L1X x3 -------------------
static const int PIN_I2C_SDA = 21;
static const int PIN_I2C_SCL = 22;
static const uint32_t I2C_FREQ = 100000;   // 100 kHz: safer with MPU9250 + 3x ToF on dupont wires

// VL53LXX-V2 modules are usually VL53L0X (not L1X). Firmware tries L0X first, then L1X.
static const int PIN_XSHUT_FRONT = 4;
static const int PIN_XSHUT_LEFT  = 5;
static const int PIN_XSHUT_RIGHT = 23;
static const uint8_t TOF_ADDR_FRONT = 0x30;
static const uint8_t TOF_ADDR_LEFT  = 0x31;
static const uint8_t TOF_ADDR_RIGHT = 0x32;
static const float TOF_FOV_RAD      = 0.47f;   // ~27 deg
static const float TOF_MIN_RANGE_M  = 0.04f;
static const float TOF_MAX_RANGE_M  = 4.0f;
static const uint32_t TOF_HZ        = 10;      // read in controlTask (100 ms loop)

// ---------------- Wheel geometry (from mobile_base/config/base.yaml) -------
static const float WHEEL_DIAMETER_M       = 0.0646f;
static const float WHEEL_SEPARATION_M     = 0.208f;
static const float COUNTS_PER_WHEEL_REV   = 564.0f;
static const float M_PER_COUNT =
    3.1415926f * WHEEL_DIAMETER_M / COUNTS_PER_WHEEL_REV;

// Flip if a wheel encoder counts backward vs motion.
static const bool R_ENC_INVERT = false;
static const bool L_ENC_INVERT = false;

// ---------------- Speed loop (100 ms, same as Arduino firmware) ------------
static const uint32_t CONTROL_INTERVAL_MS = 100;

// Target unit: encoder counts per CONTROL_INTERVAL_MS (signed).
static const float TARGET_MAX      = 80.0f;
static const float TARGET_DEADBAND = 10.0f;

static const float KP_RIGHT = 2.5f;
static const float KP_LEFT  = 2.5f;

static const int   PWM_MIN = 0;
static const int   PWM_MAX = 255;
static const float PWM_STEP_LIMIT = 15.0f;

static const int   STALL_PWM_THRESHOLD = 90;   // was 240 (L298N); keep above FF table when tuning
static const float STALL_SPEED_RATIO   = 0.7f;
static const int   STALL_LIMIT_COUNT   = 10;   // 10 x 100ms = 1s

// Set false while debugging wiring/encoders (Arduino serial firmware had auto-clear
// from the host; stall on bad encoder feedback stops one side after ~1s).
// Set false while tuning PWM on the bench (encoder stall latch causes buzz-stop cycles).
static const bool STALL_PROTECTION_ENABLE = false;

// Do not drive motors until micro-ROS agent is connected (prevents run on boot).
static const bool MOTORS_REQUIRE_AGENT = true;

// Closed-loop: encoder PID on top of feedforward table below.
static const bool OPEN_LOOP_MOTOR = false;

// Keyboard tune session (./run_motor_pwm_tune.sh):
// Host publishes /motor_ff_pwm (0-255) to override getBasePWM() live; PID + stall stay on.
static const uint32_t FF_OVERRIDE_TIMEOUT_MS = 3000;

// DRV8871 feedforward PWM (0-255) vs |target| in counts/100ms.
// LE_30 calibrated: speed=30, min PWM=60, margin +10 -> 70 (USB serial, closed-loop, 16V).
// Re-tune with ./run_motor_pwm_tune.sh; then set STALL_PROTECTION_ENABLE=true.
static const int PWM_FF_LE_12 = 75;
static const int PWM_FF_LE_20 = 72;
static const int PWM_FF_LE_30 = 70;
static const int PWM_FF_LE_40 = 78;
static const int PWM_FF_LE_55 = 88;
static const int PWM_FF_LE_70 = 100;
static const int PWM_FF_LE_80 = 115;
static const int PWM_FF_MAX   = 130;

// Auto-clear stall latch (only used when STALL_PROTECTION_ENABLE is true).
static const uint32_t STALL_AUTO_CLEAR_MS = 1000;

// ---------------- WiFi / micro-ROS agent ---------------------------------
#define MICROROS_WIFI_SSID  "NETGEAR71"
#define MICROROS_WIFI_PASS  "melodicdaisy353"
static const char AGENT_IP[]      = "192.168.1.12";
static const uint16_t AGENT_PORT  = 8888;

// ---------------- ROS / micro-ROS ----------------------------------------
static const uint32_t CMD_TIMEOUT_MS = 2000;
static const uint32_t PUB_ODOM_HZ    = 50;
static const uint32_t PUB_IMU_HZ     = 100;
static const uint32_t PUB_TOF_HZ     = 10;

#define FRAME_ODOM  "odom"
#define FRAME_BASE  "base_footprint"
#define FRAME_IMU   "imu_link"
#define FRAME_TOF_FRONT "tof_front_link"
#define FRAME_TOF_LEFT  "tof_left_link"
#define FRAME_TOF_RIGHT "tof_right_link"
