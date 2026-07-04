// ======================================================================
// Hardware / robot configuration for the ESP32 base.
// >>> VERIFY EVERY PIN against your specific 28-pin ESP32 WROOM board <<<
// Avoid input-only GPIO 34-39 for outputs; avoid strapping pins 0/2/12/15.
// ======================================================================
#pragma once

// ---------------- Stepper drivers (D556, step/dir) ----------------
// Right = motor A, Left = motor B.
static const int PIN_R_STEP = 25;
static const int PIN_R_DIR  = 26;
static const int PIN_L_STEP = 27;
static const int PIN_L_DIR  = 14;
static const int PIN_EN     = 13;   // shared enable (many D556 are active-LOW); -1 if unused
static const bool EN_ACTIVE_LOW = true;

// Flip these if a wheel drives the wrong way.
static const bool R_DIR_INVERT = false;
static const bool L_DIR_INVERT = true;   // left motor usually mirrored

// ---------------- I2C buses ----------------
// Bus 0 (Wire):  AS5600 #right (0x36) + MPU9250 (0x68) + VL53L1X x3 (re-addressed)
// Bus 1 (Wire1): AS5600 #left  (0x36)  -- separate bus because AS5600 addr is fixed
static const int PIN_I2C0_SDA = 21;
static const int PIN_I2C0_SCL = 22;
static const int PIN_I2C1_SDA = 18;
static const int PIN_I2C1_SCL = 19;
static const uint32_t I2C_FREQ = 400000;

// ---------------- VL53L1X ToF x3 ----------------
// All power up at 0x29; we hold them in reset via XSHUT, then bring up one at a
// time and assign a unique address. Order: front, left, right.
static const int PIN_XSHUT_FRONT = 4;
static const int PIN_XSHUT_LEFT  = 5;
static const int PIN_XSHUT_RIGHT = 23;
static const uint8_t TOF_ADDR_FRONT = 0x30;
static const uint8_t TOF_ADDR_LEFT  = 0x31;
static const uint8_t TOF_ADDR_RIGHT = 0x32;
static const float TOF_FOV_RAD      = 0.47f;   // ~27 deg
static const float TOF_MIN_RANGE_M  = 0.04f;
static const float TOF_MAX_RANGE_M  = 4.0f;

// ---------------- Wheel / odometry ----------------
// TODO: measure on the real chassis. Must match indoor_robot.urdf + ekf.
static const float WHEEL_DIAMETER_M    = 0.100f;   // TODO
static const float WHEEL_SEPARATION_M  = 0.300f;   // TODO (track width)

// AS5600 is 12-bit absolute (4096 counts / encoder-shaft revolution).
// If the encoder sits directly on the WHEEL axle, counts/wheel-rev = 4096.
// If on the motor shaft before a gearbox, multiply by the gear ratio.
static const float ENC_COUNTS_PER_WHEEL_REV = 4096.0f;   // TODO if geared
static const bool  R_ENC_INVERT = false;   // flip if encoder counts opposite to motion
static const bool  L_ENC_INVERT = false;

// ---------------- Stepper motion (for commanding speed) ----------------
static const int   MOTOR_FULL_STEPS_PER_REV = 200;   // 1.8 deg motor
static const int   MICROSTEPS               = 8;     // D556 DIP setting (TODO match)
static const float GEAR_RATIO               = 1.0f;  // motor:wheel (TODO)
// steps per wheel revolution:
static const float STEPS_PER_WHEEL_REV =
    MOTOR_FULL_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO;

static const float MAX_WHEEL_SPEED_MPS = 0.45f;   // clamp commanded wheel speed
static const float STEP_ACCEL_HZ_S     = 4000.0f; // stepper accel (steps/s^2)

// ---------------- Loop rates ----------------
static const uint32_t CONTROL_HZ   = 100;   // odom integration + motor update
static const uint32_t IMU_HZ       = 100;
static const uint32_t TOF_HZ       = 30;
static const uint32_t PUB_ODOM_HZ  = 50;
static const uint32_t PUB_IMU_HZ   = 100;
static const uint32_t PUB_TOF_HZ   = 15;

static const uint32_t CMD_TIMEOUT_MS = 500;   // stop if no /cmd_vel within this

// ---------------- WiFi / micro-ROS agent ---------------------------------
#define MICROROS_WIFI_SSID  "NETGEAR71"
#define MICROROS_WIFI_PASS  "melodicdaisy353"
static const char AGENT_IP[]      = "192.168.1.12";
static const uint16_t AGENT_PORT  = 8888;

// ---------------- Frames ----------------
#define FRAME_ODOM        "odom"
#define FRAME_BASE        "base_footprint"
#define FRAME_IMU         "imu_link"
#define FRAME_TOF_FRONT   "tof_front_link"
#define FRAME_TOF_LEFT    "tof_left_link"
#define FRAME_TOF_RIGHT   "tof_right_link"
