// ======================================================================
// ESP32 micro-ROS base for the legacy deck robot (DRV8871 + hall encoders).
// Ported from Adruino_PID-full/PID-full.ino + IMU from esp32_base/main.cpp.
//
// Publishes: /odom, /imu/data_raw, /tof_front, /tof_left, /tof_right
// Subscribes: /cmd_vel, /motor_ff_pwm (live feedforward override for keyboard tune)
//
// Flash (USB serial):  pio run -e esp32dev_l298n -t upload
// Flash (WiFi):       pio run -e esp32dev_l298n_wifi -t upload
// Agent (serial):     scripts/run_microros_agent.sh /dev/ttyUSB0
// Agent (WiFi):       scripts/run_microros_agent_wifi.sh
// ======================================================================
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <MPU9250.h>
#include <VL53L0X.h>
#include <VL53L1X.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/u_int8_multi_array.h>

#include "config_l298n.h"
#include "microros_qos.h"
#include "microros_transport_setup.h"

// ======================================================================
// Encoder ISRs
// ======================================================================
static volatile long g_encRight = 0;
static volatile long g_encLeft  = 0;
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR isrEncRight() {
  portENTER_CRITICAL_ISR(&encMux);
  g_encRight++;
  portEXIT_CRITICAL_ISR(&encMux);
}

static void IRAM_ATTR isrEncLeft() {
  portENTER_CRITICAL_ISR(&encMux);
  g_encLeft++;
  portEXIT_CRITICAL_ISR(&encMux);
}

static long readEncRight() {
  long v;
  portENTER_CRITICAL(&encMux);
  v = g_encRight;
  portEXIT_CRITICAL(&encMux);
  return v;
}

static long readEncLeft() {
  long v;
  portENTER_CRITICAL(&encMux);
  v = g_encLeft;
  portEXIT_CRITICAL(&encMux);
  return v;
}

// ======================================================================
// Motor control (from PID-full.ino)
// ======================================================================
static portMUX_TYPE motorMux = portMUX_INITIALIZER_UNLOCKED;

static float targetRight = 0.0f;
static float targetLeft  = 0.0f;
static float pwmRight    = 0.0f;
static float pwmLeft     = 0.0f;
static int rightStallCounter = 0;
static int leftStallCounter  = 0;
static bool rightFault = false;
static bool leftFault  = false;
static uint32_t lastFaultClearMs = 0;

// Live feedforward override from scripts/motor_pwm_tune.py (/motor_ff_pwm).
static portMUX_TYPE tuneMux = portMUX_INITIALIZER_UNLOCKED;
static float g_ffOverride = -1.0f;
static uint32_t g_ffOverrideMs = 0;

static float getBasePWM(float targetAbs) {
  if (targetAbs <= 0) return 0;

  float ov = -1.0f;
  uint32_t ovMs = 0;
  portENTER_CRITICAL(&tuneMux);
  ov = g_ffOverride;
  ovMs = g_ffOverrideMs;
  portEXIT_CRITICAL(&tuneMux);
  if (ov >= 0.0f && (millis() - ovMs) < FF_OVERRIDE_TIMEOUT_MS) {
    return ov;
  }

  if (targetAbs <= 12) return (float)PWM_FF_LE_12;
  if (targetAbs <= 20) return (float)PWM_FF_LE_20;
  if (targetAbs <= 30) return (float)PWM_FF_LE_30;
  if (targetAbs <= 40) return (float)PWM_FF_LE_40;
  if (targetAbs <= 55) return (float)PWM_FF_LE_55;
  if (targetAbs <= 70) return (float)PWM_FF_LE_70;
  if (targetAbs <= 80) return (float)PWM_FF_LE_80;
  return (float)PWM_FF_MAX;
}

static void setupMotorPwm() {
  ledcSetup(0, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
  ledcSetup(1, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
  ledcSetup(2, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
  ledcSetup(3, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
  ledcAttachPin(PIN_R_IN1, 0);
  ledcAttachPin(PIN_R_IN2, 1);
  ledcAttachPin(PIN_L_IN1, 2);
  ledcAttachPin(PIN_L_IN2, 3);
}

// DRV8871 IN1/IN2: PWM on active leg, other leg LOW (3.3V logic OK).
static void driveDrv8871(int pinIn1, int pinIn2, int pwm, int direction) {
  pwm = constrain(pwm, 0, 255);
  if (direction > 0) {
    analogWrite(pinIn1, pwm);
    analogWrite(pinIn2, 0);
  } else if (direction < 0) {
    analogWrite(pinIn1, 0);
    analogWrite(pinIn2, pwm);
  } else {
    analogWrite(pinIn1, 0);
    analogWrite(pinIn2, 0);
  }
}

static void setRightMotor(int pwm, int direction) {
  driveDrv8871(PIN_R_IN1, PIN_R_IN2, pwm, direction);
}

static void setLeftMotor(int pwm, int direction) {
  driveDrv8871(PIN_L_IN1, PIN_L_IN2, pwm, direction);
}

static void stopMotors() {
  targetRight = 0;
  targetLeft  = 0;
  pwmRight    = 0;
  pwmLeft     = 0;
  setRightMotor(0, 0);
  setLeftMotor(0, 0);
}

static float clampTarget(float speed) {
  float a = fabsf(speed);
  if (a < TARGET_DEADBAND) return 0.0f;
  if (a > TARGET_MAX) a = TARGET_MAX;
  return (speed >= 0) ? a : -a;
}

static float mpsToCountsPer100ms(float v_mps) {
  if (M_PER_COUNT <= 0) return 0.0f;
  return v_mps / M_PER_COUNT * 0.1f;
}

static void setTargetsFromCmdVel(float v, float w) {
  float half = WHEEL_SEPARATION_M * 0.5f;
  float vR = v + w * half;
  float vL = v - w * half;
  targetRight = clampTarget(mpsToCountsPer100ms(vR));
  targetLeft  = clampTarget(mpsToCountsPer100ms(vL));
}

// Immediate feedforward kick — PWM only; caller must set targets first.
static void kickMotorsPwm() {
  auto kickOne = [](float target, float &pwm, bool isRight) {
    float absT = fabsf(target);
    if (absT <= 0.0f) {
      pwm = 0.0f;
      if (isRight) setRightMotor(0, 0);
      else setLeftMotor(0, 0);
      return;
    }
    int dir = (target > 0.0f) ? 1 : -1;
    float base = getBasePWM(absT);
    if (base < (float)PWM_START_FLOOR) base = (float)PWM_START_FLOOR;
    pwm = base;
    if (isRight) setRightMotor((int)pwm, dir);
    else setLeftMotor((int)pwm, dir);
  };

  kickOne(targetRight, pwmRight, true);
  kickOne(targetLeft, pwmLeft, false);
}

static bool cmdVelNeedsKick(float v, float w, float lastV, float lastW) {
  bool moving = fabsf(v) >= 1e-6f || fabsf(w) >= 1e-6f;
  bool wasMoving = fabsf(lastV) >= 1e-6f || fabsf(lastW) >= 1e-6f;
  if (!moving) return false;
  if (!wasMoving) return true;
  if (v * lastV < 0.0f || w * lastW < 0.0f) return true;
  return false;
}

static void updateOneWheel(
    float targetSigned, float actualCount, float &pwmOutput, float Kp,
    int &stallCounter, bool &fault, bool isRight) {
  float targetAbs = fabsf(targetSigned);
  int direction = 0;
  if (targetSigned > 0) direction = 1;
  else if (targetSigned < 0) direction = -1;

  if (targetAbs <= 0.0f) {
    pwmOutput = 0.0f;
    stallCounter = 0;
    if (isRight) setRightMotor(0, 0);
    else setLeftMotor(0, 0);
    return;
  }

  if (fault) {
    pwmOutput = 0;
    if (isRight) setRightMotor(0, 0);
    else setLeftMotor(0, 0);
    return;
  }

  float basePWM = getBasePWM(targetAbs);
  float intervalScale = (float)CONTROL_INTERVAL_MS / 100.0f;
  float targetThisInterval = targetAbs * intervalScale;
  float error = targetThisInterval - actualCount;
  if (fabsf(error) < 1.0f) error = 0.0f;
  float targetPWM = OPEN_LOOP_MOTOR ? basePWM : (basePWM + Kp * error);
  targetPWM = constrain(targetPWM, (float)PWM_MIN, (float)PWM_MAX);

  if (OPEN_LOOP_MOTOR) {
    pwmOutput = targetPWM;
  } else {
    float diff = targetPWM - pwmOutput;
    if (diff > PWM_STEP_LIMIT) diff = PWM_STEP_LIMIT;
    if (diff < -PWM_STEP_LIMIT) diff = -PWM_STEP_LIMIT;
    pwmOutput += diff;
    pwmOutput = constrain(pwmOutput, (float)PWM_MIN, (float)PWM_MAX);
  }

  if (isRight) setRightMotor((int)pwmOutput, direction);
  else setLeftMotor((int)pwmOutput, direction);

#if STALL_PROTECTION_ENABLE
  if (pwmOutput > STALL_PWM_THRESHOLD &&
      actualCount < targetThisInterval * STALL_SPEED_RATIO) {
    stallCounter++;
  } else {
    stallCounter = 0;
  }

  if (stallCounter >= STALL_LIMIT_COUNT) {
    fault = true;
    pwmOutput = 0;
    if (isRight) setRightMotor(0, 0);
    else setLeftMotor(0, 0);
  }
#endif
}

#if STALL_PROTECTION_ENABLE
static void maybeAutoClearFault() {
  if (!rightFault && !leftFault) return;
  uint32_t now = millis();
  if (now - lastFaultClearMs < STALL_AUTO_CLEAR_MS) return;
  lastFaultClearMs = now;
  rightFault = false;
  leftFault  = false;
  rightStallCounter = 0;
  leftStallCounter  = 0;
}
#else
static void maybeAutoClearFault() {}
#endif

// ======================================================================
// VL53L0X / VL53L1X x3 (XSHUT address sequencing on shared I2C bus)
// ======================================================================
enum class TofKind : uint8_t { None = 0, L0X = 1, L1X = 2 };

struct TofUnit {
  VL53L0X l0x;
  VL53L1X l1x;
  TofKind kind = TofKind::None;
};

static TofUnit tofFront, tofLeft, tofRight;
static bool tofFrontOk = false, tofLeftOk = false, tofRightOk = false;
static uint8_t g_tofStatus[3] = {0, 0, 0};   // TofKind per sensor, for /tof_status

static bool bringUpTofUnit(TofUnit &unit, int xshut, uint8_t addr) {
  digitalWrite(xshut, HIGH);
  delay(10);

  // VL53LXX-V2 breakouts are usually VL53L0X clones — try L0X before L1X.
  unit.l0x.setBus(&Wire);
  unit.l0x.setTimeout(500);
  if (unit.l0x.init()) {
    unit.l0x.setAddress(addr);
    unit.l0x.startContinuous(1000 / TOF_HZ);
    unit.kind = TofKind::L0X;
    return true;
  }

  unit.l1x.setBus(&Wire);
  unit.l1x.setTimeout(500);
  if (unit.l1x.init()) {
    unit.l1x.setAddress(addr);
    unit.l1x.setDistanceMode(VL53L1X::Short);
    unit.l1x.setMeasurementTimingBudget(20000);
    unit.l1x.startContinuous(1000 / TOF_HZ);
    unit.kind = TofKind::L1X;
    return true;
  }

  unit.kind = TofKind::None;
  return false;
}

static void setupTofTrio() {
  pinMode(PIN_XSHUT_FRONT, OUTPUT);
  pinMode(PIN_XSHUT_LEFT, OUTPUT);
  pinMode(PIN_XSHUT_RIGHT, OUTPUT);
  digitalWrite(PIN_XSHUT_FRONT, LOW);
  digitalWrite(PIN_XSHUT_LEFT, LOW);
  digitalWrite(PIN_XSHUT_RIGHT, LOW);
  delay(20);

  tofFrontOk = bringUpTofUnit(tofFront, PIN_XSHUT_FRONT, TOF_ADDR_FRONT);
  tofLeftOk  = bringUpTofUnit(tofLeft, PIN_XSHUT_LEFT, TOF_ADDR_LEFT);
  tofRightOk = bringUpTofUnit(tofRight, PIN_XSHUT_RIGHT, TOF_ADDR_RIGHT);

  g_tofStatus[0] = static_cast<uint8_t>(tofFront.kind);
  g_tofStatus[1] = static_cast<uint8_t>(tofLeft.kind);
  g_tofStatus[2] = static_cast<uint8_t>(tofRight.kind);
}

static float readTofUnit(TofUnit &unit, bool ok) {
  if (!ok || unit.kind == TofKind::None) return -1.0f;

  if (unit.kind == TofKind::L0X) {
    uint16_t mm = unit.l0x.readRangeContinuousMillimeters();
    if (unit.l0x.timeoutOccurred() || mm == 0) return -1.0f;
    return mm / 1000.0f;
  }

  if (unit.l1x.dataReady()) {
    uint16_t mm = unit.l1x.read(false);
    if (mm > 0) return mm / 1000.0f;
  }
  return -1.0f;
}

static void readTofTrio(float &tofF, float &tofL, float &tofR) {
  tofF = readTofUnit(tofFront, tofFrontOk);
  tofL = readTofUnit(tofLeft, tofLeftOk);
  tofR = readTofUnit(tofRight, tofRightOk);
}

// ======================================================================
// Shared state (cross-core)
// ======================================================================
static portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

struct SharedState {
  float cmd_v = 0.0f;
  float cmd_w = 0.0f;
  uint32_t cmd_stamp_ms = 0;
  bool motors_enabled = false;
  float x = 0, y = 0, yaw = 0;
  float vx = 0, vyaw = 0;
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  float tof_m[3] = {-1, -1, -1};
};
static SharedState g;

static MPU9250 mpu;

// ======================================================================
// micro-ROS
// ======================================================================
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_timer_t timer;

static rcl_publisher_t pub_odom, pub_imu, pub_tof_f, pub_tof_l, pub_tof_r, pub_tof_status;
static rcl_subscription_t sub_cmd, sub_ff;

static nav_msgs__msg__Odometry odom_msg;
static sensor_msgs__msg__Imu imu_msg;
static sensor_msgs__msg__Range tof_f_msg, tof_l_msg, tof_r_msg;
static geometry_msgs__msg__Twist cmd_msg;
static std_msgs__msg__Float32 ff_msg;
static std_msgs__msg__UInt8MultiArray tof_status_msg;

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static AgentState agent_state = WAITING_AGENT;

static void setMotorsEnabled(bool on) {
  portENTER_CRITICAL(&stateMux);
  g.motors_enabled = on;
  portEXIT_CRITICAL(&stateMux);
  if (!on) {
    portENTER_CRITICAL(&motorMux);
    stopMotors();
    rightFault = leftFault = false;
    rightStallCounter = leftStallCounter = 0;
    portEXIT_CRITICAL(&motorMux);
  }
}

#define RCCHECK(fn) { rcl_ret_t _rc = (fn); if (_rc != RCL_RET_OK) { return false; } }
#define RCSOFT(fn)  { (void)(fn); }

static void set_string(rosidl_runtime_c__String *s, const char *txt) {
  s->data = (char *)txt;
  s->size = strlen(txt);
  s->capacity = s->size + 1;
}

static void fill_stamp(builtin_interfaces__msg__Time *stamp) {
  int64_t ns = rmw_uros_epoch_nanos();
  stamp->sec = (int32_t)(ns / 1000000000LL);
  stamp->nanosec = (uint32_t)(ns % 1000000000LL);
}

static float wheelDistM(float targetSigned, float deltaCounts) {
  if (deltaCounts == 0.0f) return 0.0f;
  if (targetSigned != 0.0f) {
    // Motor commanded: single-edge hall only counts up — use command sign for direction.
    float sign = (targetSigned > 0) ? 1.0f : -1.0f;
    return sign * fabsf(deltaCounts) * M_PER_COUNT;
  }
  // Hand-turn / coasting: no command sign; treat each pulse as forward for that wheel.
  return deltaCounts * M_PER_COUNT;
}

static void integrateOdom(
    float targetR, float targetL, float deltaR, float deltaL, float dt) {
  float dist_r = wheelDistM(targetR, deltaR);
  float dist_l = wheelDistM(targetL, deltaL);
  float ds = 0.5f * (dist_l + dist_r);
  float dyaw = (dist_r - dist_l) / WHEEL_SEPARATION_M;

  float x = g.x + ds * cosf(g.yaw + dyaw * 0.5f);
  float y = g.y + ds * sinf(g.yaw + dyaw * 0.5f);
  float yaw = g.yaw + dyaw;
  yaw = atan2f(sinf(yaw), cosf(yaw));

  g.x = x;
  g.y = y;
  g.yaw = yaw;
  g.vx = (dt > 0) ? ds / dt : 0.0f;
  g.vyaw = (dt > 0) ? dyaw / dt : 0.0f;
}

static float g_lastKickV = 0.0f;
static float g_lastKickW = 0.0f;

static void cmd_vel_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  float v = m->linear.x;
  float w = m->angular.z;
  bool motors_on;
  bool is_stop = fabsf(v) < 1e-6f && fabsf(w) < 1e-6f;

  portENTER_CRITICAL(&stateMux);
  g.cmd_v = v;
  g.cmd_w = w;
  g.cmd_stamp_ms = millis();
  motors_on = g.motors_enabled;
  portEXIT_CRITICAL(&stateMux);

  portENTER_CRITICAL(&motorMux);
  if (is_stop) {
    stopMotors();
    g_lastKickV = 0.0f;
    g_lastKickW = 0.0f;
  } else if (motors_on) {
    if (cmdVelNeedsKick(v, w, g_lastKickV, g_lastKickW)) {
      setTargetsFromCmdVel(v, w);
      kickMotorsPwm();
    }
    g_lastKickV = v;
    g_lastKickW = w;
  }
  portEXIT_CRITICAL(&motorMux);
}

static void ff_pwm_cb(const void *msgin) {
  const std_msgs__msg__Float32 *m = (const std_msgs__msg__Float32 *)msgin;
  portENTER_CRITICAL(&tuneMux);
  if (m->data <= 0.0f) {
    g_ffOverride = -1.0f;
  } else {
    g_ffOverride = constrain(m->data, 0.0f, 255.0f);
    g_ffOverrideMs = millis();
  }
  portEXIT_CRITICAL(&tuneMux);
}

static void fill_range(sensor_msgs__msg__Range *m, float meters) {
  fill_stamp(&m->header.stamp);
  m->range = (meters >= 0.0f) ? meters : INFINITY;
}

static void timer_cb(rcl_timer_t *t, int64_t) {
  if (t == nullptr) return;
  static uint32_t tick = 0;
  tick++;

  SharedState s;
  portENTER_CRITICAL(&stateMux);
  s = g;
  portEXIT_CRITICAL(&stateMux);

  fill_stamp(&imu_msg.header.stamp);
  imu_msg.linear_acceleration.x = s.ax;
  imu_msg.linear_acceleration.y = s.ay;
  imu_msg.linear_acceleration.z = s.az;
  imu_msg.angular_velocity.x = s.gx;
  imu_msg.angular_velocity.y = s.gy;
  imu_msg.angular_velocity.z = s.gz;
  imu_msg.orientation_covariance[0] = -1.0;
  RCSOFT(rcl_publish(&pub_imu, &imu_msg, NULL));

  if (tick % (PUB_IMU_HZ / PUB_ODOM_HZ) == 0) {
    fill_stamp(&odom_msg.header.stamp);
    odom_msg.pose.pose.position.x = s.x;
    odom_msg.pose.pose.position.y = s.y;
    odom_msg.pose.pose.orientation.z = sinf(s.yaw * 0.5f);
    odom_msg.pose.pose.orientation.w = cosf(s.yaw * 0.5f);
    odom_msg.twist.twist.linear.x = s.vx;
    odom_msg.twist.twist.angular.z = s.vyaw;
    RCSOFT(rcl_publish(&pub_odom, &odom_msg, NULL));
  }

  if (tick % (PUB_IMU_HZ / PUB_TOF_HZ) == 0) {
    fill_range(&tof_f_msg, s.tof_m[0]);
    RCSOFT(rcl_publish(&pub_tof_f, &tof_f_msg, NULL));
    fill_range(&tof_l_msg, s.tof_m[1]);
    RCSOFT(rcl_publish(&pub_tof_l, &tof_l_msg, NULL));
    fill_range(&tof_r_msg, s.tof_m[2]);
    RCSOFT(rcl_publish(&pub_tof_r, &tof_r_msg, NULL));

    tof_status_msg.data.data[0] = g_tofStatus[0];
    tof_status_msg.data.data[1] = g_tofStatus[1];
    tof_status_msg.data.data[2] = g_tofStatus[2];
    RCSOFT(rcl_publish(&pub_tof_status, &tof_status_msg, NULL));
  }
}

static void init_messages() {
  nav_msgs__msg__Odometry__init(&odom_msg);
  set_string(&odom_msg.header.frame_id, FRAME_ODOM);
  set_string(&odom_msg.child_frame_id, FRAME_BASE);

  sensor_msgs__msg__Imu__init(&imu_msg);
  set_string(&imu_msg.header.frame_id, FRAME_IMU);

  sensor_msgs__msg__Range *ranges[3] = {&tof_f_msg, &tof_l_msg, &tof_r_msg};
  const char *range_frames[3] = {FRAME_TOF_FRONT, FRAME_TOF_LEFT, FRAME_TOF_RIGHT};
  for (int i = 0; i < 3; i++) {
    sensor_msgs__msg__Range__init(ranges[i]);
    ranges[i]->radiation_type = sensor_msgs__msg__Range__INFRARED;
    ranges[i]->field_of_view = TOF_FOV_RAD;
    ranges[i]->min_range = TOF_MIN_RANGE_M;
    ranges[i]->max_range = TOF_MAX_RANGE_M;
    set_string(&ranges[i]->header.frame_id, range_frames[i]);
  }

  std_msgs__msg__UInt8MultiArray__init(&tof_status_msg);
  tof_status_msg.data.capacity = 3;
  tof_status_msg.data.size = 3;
  tof_status_msg.data.data = g_tofStatus;
}

static bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_l298n_base", "", &support));

  static rmw_qos_profile_t qos_reliable = microros_qos_reliable_depth1();

  RCCHECK(microros_publisher_init_best_effort_depth1(
      &pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
  RCCHECK(microros_publisher_init_best_effort_depth1(
      &pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));
  RCCHECK(rclc_publisher_init(
      &pub_tof_f, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "tof_front",
      &qos_reliable));
  RCCHECK(rclc_publisher_init(
      &pub_tof_l, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "tof_left",
      &qos_reliable));
  RCCHECK(rclc_publisher_init(
      &pub_tof_r, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "tof_right",
      &qos_reliable));
  RCCHECK(rclc_publisher_init(
      &pub_tof_status, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray), "tof_status", &qos_reliable));

  RCCHECK(rclc_subscription_init_best_effort(
      &sub_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init(
      &sub_ff, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_ff_pwm",
      &qos_reliable));

  const unsigned int timer_period = 1000 / PUB_IMU_HZ;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_period), timer_cb));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &cmd_msg, &cmd_vel_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_ff, &ff_msg, &ff_pwm_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  RCSOFT(rmw_uros_sync_session(1000));
  return true;
}

static void destroy_entities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_publisher_fini(&pub_odom, &node);
  rcl_publisher_fini(&pub_imu, &node);
  rcl_publisher_fini(&pub_tof_f, &node);
  rcl_publisher_fini(&pub_tof_l, &node);
  rcl_publisher_fini(&pub_tof_r, &node);
  rcl_publisher_fini(&pub_tof_status, &node);
  rcl_subscription_fini(&sub_cmd, &node);
  rcl_subscription_fini(&sub_ff, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ======================================================================
// Core 0: micro-ROS (+ WiFi stack on WiFi builds)
// ======================================================================
static void microRosTask(void *arg) {
  (void)arg;
  init_messages();
  uint32_t lastPingMs = 0;
  for (;;) {
    switch (agent_state) {
      case WAITING_AGENT:
        agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
        break;
      case AGENT_AVAILABLE:
        agent_state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
        if (agent_state == AGENT_CONNECTED) {
#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
          Serial.println("micro-ROS agent connected");
#endif
          setMotorsEnabled(true);
          portENTER_CRITICAL(&stateMux);
          g.cmd_stamp_ms = millis();
          portEXIT_CRITICAL(&stateMux);
          lastPingMs = millis();
        } else if (agent_state == WAITING_AGENT) {
          destroy_entities();
        }
        break;
      case AGENT_CONNECTED: {
        uint32_t nowMs = millis();
        if (nowMs - lastPingMs >= 500) {
          lastPingMs = nowMs;
          if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
            agent_state = AGENT_DISCONNECTED;
            break;
          }
        }
        for (int i = 0; i < 8; ++i) {
          rclc_executor_spin_some(&executor, 0);
        }
        continue;
      }
      case AGENT_DISCONNECTED:
        setMotorsEnabled(false);
        destroy_entities();
        agent_state = WAITING_AGENT;
        break;
    }
    vTaskDelay(1);
  }
}

// ======================================================================
// Core 1: motor PID + encoders + IMU + odom
// ======================================================================
static void controlTask(void *arg) {
  (void)arg;
  long lastRightEnc = readEncRight();
  long lastLeftEnc  = readEncLeft();
  uint32_t lastControlMs = millis();
  uint32_t lastImuMs = millis();

  for (;;) {

    uint32_t now = millis();
    if (now - lastControlMs >= CONTROL_INTERVAL_MS) {
      float dt = (now - lastControlMs) / 1000.0f;
      lastControlMs = now;

      float cmd_v, cmd_w;
      bool motors_on;
      uint32_t cmd_ts;
      portENTER_CRITICAL(&stateMux);
      cmd_v = g.cmd_v;
      cmd_w = g.cmd_w;
      cmd_ts = g.cmd_stamp_ms;
      motors_on = g.motors_enabled;
      portEXIT_CRITICAL(&stateMux);

      long curR = readEncRight();
      long curL = readEncLeft();
      float deltaR = (float)(curR - lastRightEnc);
      float deltaL = (float)(curL - lastLeftEnc);
      lastRightEnc = curR;
      lastLeftEnc  = curL;
      if (R_ENC_INVERT) deltaR = -deltaR;
      if (L_ENC_INVERT) deltaL = -deltaL;

      float odomTargetR = 0.0f;
      float odomTargetL = 0.0f;

      portENTER_CRITICAL(&motorMux);
      if (!motors_on) {
        stopMotors();
      } else {
        if (cmd_ts != 0 && (now - cmd_ts) > CMD_TIMEOUT_MS) {
          cmd_v = 0.0f;
          cmd_w = 0.0f;
        }
        setTargetsFromCmdVel(cmd_v, cmd_w);
        odomTargetR = targetRight;
        odomTargetL = targetLeft;
        maybeAutoClearFault();

        updateOneWheel(targetRight, deltaR, pwmRight, KP_RIGHT,
                       rightStallCounter, rightFault, true);
        updateOneWheel(targetLeft, deltaL, pwmLeft, KP_LEFT,
                       leftStallCounter, leftFault, false);
      }
      portEXIT_CRITICAL(&motorMux);

      portENTER_CRITICAL(&stateMux);
      integrateOdom(odomTargetR, odomTargetL, deltaR, deltaL, dt);
      portEXIT_CRITICAL(&stateMux);
    }

    static uint32_t lastTofMs = 0;
    if (now - lastTofMs >= (1000 / TOF_HZ)) {
      lastTofMs = now;
      float tofF, tofL, tofR;
      readTofTrio(tofF, tofL, tofR);
      portENTER_CRITICAL(&stateMux);
      g.tof_m[0] = tofF;
      g.tof_m[1] = tofL;
      g.tof_m[2] = tofR;
      portEXIT_CRITICAL(&stateMux);
    }

    if (now - lastImuMs >= 50) {
      lastImuMs = now;
      mpu.update_accel_gyro();
      portENTER_CRITICAL(&stateMux);
      g.ax = mpu.getAccX() * 9.80665f;
      g.ay = mpu.getAccY() * 9.80665f;
      g.az = mpu.getAccZ() * 9.80665f;
      g.gx = mpu.getGyroX() * (float)M_PI / 180.0f;
      g.gy = mpu.getGyroY() * (float)M_PI / 180.0f;
      g.gz = mpu.getGyroZ() * (float)M_PI / 180.0f;
      portEXIT_CRITICAL(&stateMux);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void setup() {
  // Buffer expansion must be before Serial.begin (inside init_microros_transport).
  Serial.setRxBufferSize(2048);
  Serial.setTxBufferSize(2048);

  char wifi_ssid[] = MICROROS_WIFI_SSID;
  char wifi_pass[] = MICROROS_WIFI_PASS;
  init_microros_transport(wifi_ssid, wifi_pass, AGENT_IP, AGENT_PORT);

  pinMode(PIN_ENC_RIGHT, INPUT_PULLUP);
  pinMode(PIN_ENC_LEFT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT), isrEncRight, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT), isrEncLeft, RISING);

  pinMode(PIN_R_IN1, OUTPUT);
  pinMode(PIN_R_IN2, OUTPUT);
  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);
  setupMotorPwm();
  digitalWrite(PIN_R_IN1, LOW);
  digitalWrite(PIN_R_IN2, LOW);
  digitalWrite(PIN_L_IN1, LOW);
  digitalWrite(PIN_L_IN2, LOW);
  stopMotors();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ);
  mpu.setup(0x68, MPU9250Setting(), Wire);
  setupTofTrio();

  xTaskCreatePinnedToCore(controlTask, "control", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(microRosTask, "microros", 8192, NULL, 5, NULL, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
