// ======================================================================
// ESP32 base firmware (micro-ROS) for the indoor inspection robot.
//
// Core 0 (microRosTask): micro-ROS (+ WiFi on WiFi builds). Publishes /odom,
//   /imu/data_raw, /tof_front|left|right; subscribes /cmd_vel. Agent (re)connect.
// Core 1 (controlTask): real-time. Reads AS5600 wheel encoders -> integrates
//   differential-drive odometry; reads MPU9250 + 3x VL53L1X; converts the
//   latest /cmd_vel into stepper speeds (D556 step/dir via FastAccelStepper).
//
// Shared state between cores is guarded by a portMUX spinlock.
// NOTE: serial is owned by the micro-ROS transport -> no Serial.print debugging.
// This is a SKELETON: pins/params in config.h are placeholders; needs on-hardware
// calibration. Cannot be compiled in this environment.
// ======================================================================
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <FastAccelStepper.h>
#include <AS5600.h>
#include <MPU9250.h>
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

#include "config.h"
#include "microros_qos.h"
#include "microros_transport_setup.h"

// ======================================================================
// Hardware objects
// ======================================================================
static FastAccelStepperEngine engine = FastAccelStepperEngine();
static FastAccelStepper *stepR = nullptr;
static FastAccelStepper *stepL = nullptr;

static AS5600 encR(&Wire);    // right encoder on I2C bus 0
static AS5600 encL(&Wire1);   // left encoder on I2C bus 1
static MPU9250 mpu;           // on I2C bus 0 (0x68)

static VL53L1X tofFront, tofLeft, tofRight;
static bool tofFrontOk = false, tofLeftOk = false, tofRightOk = false;

// ======================================================================
// Shared state (cross-core)
// ======================================================================
static portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

struct SharedState {
  // host -> motor
  float cmd_v = 0.0f;        // m/s
  float cmd_w = 0.0f;        // rad/s
  uint32_t cmd_stamp_ms = 0;
  // motor -> host (odom)
  float x = 0, y = 0, yaw = 0;
  float vx = 0, vyaw = 0;
  // imu (SI: m/s^2, rad/s)
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  // tof (meters; <0 => no reading)
  float tof_m[3] = {-1, -1, -1};
};
static SharedState g;

// ======================================================================
// micro-ROS entities
// ======================================================================
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_timer_t timer;

static rcl_publisher_t pub_odom, pub_imu, pub_tof_f, pub_tof_l, pub_tof_r;
static rcl_subscription_t sub_cmd;

static nav_msgs__msg__Odometry odom_msg;
static sensor_msgs__msg__Imu imu_msg;
static sensor_msgs__msg__Range tof_f_msg, tof_l_msg, tof_r_msg;
static geometry_msgs__msg__Twist cmd_msg;

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static AgentState agent_state = WAITING_AGENT;

#define RCCHECK(fn) { rcl_ret_t _rc = (fn); if (_rc != RCL_RET_OK) { return false; } }
#define RCSOFT(fn)  { (void)(fn); }

// ======================================================================
// Helpers
// ======================================================================
static void set_string(rosidl_runtime_c__String *s, const char *txt) {
  s->data = (char *)txt;            // string literals have static storage
  s->size = strlen(txt);
  s->capacity = s->size + 1;
}

static void fill_stamp(builtin_interfaces__msg__Time *stamp) {
  int64_t ns = rmw_uros_epoch_nanos();   // 0 until time is synced
  stamp->sec = (int32_t)(ns / 1000000000LL);
  stamp->nanosec = (uint32_t)(ns % 1000000000LL);
}

static inline int16_t enc_delta(uint16_t now, uint16_t last) {
  int16_t d = (int16_t)now - (int16_t)last;   // 0..4095
  if (d > 2048) d -= 4096;
  if (d < -2048) d += 4096;
  return d;
}

// ======================================================================
// VL53L1X bring-up (XSHUT address sequencing on shared bus 0)
// ======================================================================
static bool bringUpTof(VL53L1X &dev, int xshut, uint8_t addr) {
  digitalWrite(xshut, HIGH);
  delay(10);
  dev.setBus(&Wire);
  dev.setTimeout(100);
  if (!dev.init()) return false;
  dev.setAddress(addr);
  dev.setDistanceMode(VL53L1X::Long);
  dev.setMeasurementTimingBudget(33000);
  dev.startContinuous(1000 / TOF_HZ);
  return true;
}

static void setupTofTrio() {
  pinMode(PIN_XSHUT_FRONT, OUTPUT);
  pinMode(PIN_XSHUT_LEFT, OUTPUT);
  pinMode(PIN_XSHUT_RIGHT, OUTPUT);
  // Hold all in reset, then enable one at a time.
  digitalWrite(PIN_XSHUT_FRONT, LOW);
  digitalWrite(PIN_XSHUT_LEFT, LOW);
  digitalWrite(PIN_XSHUT_RIGHT, LOW);
  delay(20);
  tofFrontOk = bringUpTof(tofFront, PIN_XSHUT_FRONT, TOF_ADDR_FRONT);
  tofLeftOk  = bringUpTof(tofLeft, PIN_XSHUT_LEFT, TOF_ADDR_LEFT);
  tofRightOk = bringUpTof(tofRight, PIN_XSHUT_RIGHT, TOF_ADDR_RIGHT);
}

// ======================================================================
// Motor command: (v, w) -> per-wheel step rate
// ======================================================================
static void applyWheelSpeeds(float v, float w) {
  float half = WHEEL_SEPARATION_M * 0.5f;
  float vR = v + w * half;
  float vL = v - w * half;
  vR = constrain(vR, -MAX_WHEEL_SPEED_MPS, MAX_WHEEL_SPEED_MPS);
  vL = constrain(vL, -MAX_WHEEL_SPEED_MPS, MAX_WHEEL_SPEED_MPS);

  float circ = (float)M_PI * WHEEL_DIAMETER_M;     // m per wheel rev
  float hzR = (vR / circ) * STEPS_PER_WHEEL_REV;   // steps/s (signed)
  float hzL = (vL / circ) * STEPS_PER_WHEEL_REV;

  if (stepR) {
    if (fabsf(hzR) < 1.0f) { stepR->stopMove(); }
    else { stepR->setSpeedInHz((uint32_t)fabsf(hzR)); (hzR > 0) ? stepR->runForward() : stepR->runBackward(); }
  }
  if (stepL) {
    if (fabsf(hzL) < 1.0f) { stepL->stopMove(); }
    else { stepL->setSpeedInHz((uint32_t)fabsf(hzL)); (hzL > 0) ? stepL->runForward() : stepL->runBackward(); }
  }
}

// ======================================================================
// Core 1: real-time control + sensing
// ======================================================================
static void controlTask(void *arg) {
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_HZ);
  TickType_t last_wake = xTaskGetTickCount();

  uint16_t lastRaw_R = encR.rawAngle();
  uint16_t lastRaw_L = encL.rawAngle();
  float x = 0, y = 0, yaw = 0;
  uint32_t tof_div = 0;
  const float dt = 1.0f / (float)CONTROL_HZ;
  const float m_per_count = ((float)M_PI * WHEEL_DIAMETER_M) / ENC_COUNTS_PER_WHEEL_REV;

  for (;;) {
    // ---- Encoders -> odometry ----
    uint16_t rawR = encR.rawAngle();
    uint16_t rawL = encL.rawAngle();
    int16_t dR = enc_delta(rawR, lastRaw_R);
    int16_t dL = enc_delta(rawL, lastRaw_L);
    lastRaw_R = rawR;
    lastRaw_L = rawL;
    if (R_ENC_INVERT) dR = -dR;
    if (L_ENC_INVERT) dL = -dL;

    float distR = dR * m_per_count;
    float distL = dL * m_per_count;
    float ds = 0.5f * (distR + distL);
    float dyaw = (distR - distL) / WHEEL_SEPARATION_M;
    yaw += dyaw;
    yaw = atan2f(sinf(yaw), cosf(yaw));
    x += ds * cosf(yaw - dyaw * 0.5f);
    y += ds * sinf(yaw - dyaw * 0.5f);
    float vx = ds / dt;
    float vyaw = dyaw / dt;

    // ---- IMU ----
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    mpu.update_accel_gyro();
    ax = mpu.getAccX() * 9.80665f;
    ay = mpu.getAccY() * 9.80665f;
    az = mpu.getAccZ() * 9.80665f;
    gx = mpu.getGyroX() * (float)M_PI / 180.0f;
    gy = mpu.getGyroY() * (float)M_PI / 180.0f;
    gz = mpu.getGyroZ() * (float)M_PI / 180.0f;

    // ---- ToF (lower rate) ----
    float tofF = -1, tofL = -1, tofR = -1;
    if (++tof_div >= (CONTROL_HZ / TOF_HZ)) {
      tof_div = 0;
      if (tofFrontOk && tofFront.dataReady()) tofF = tofFront.read(false) / 1000.0f;
      if (tofLeftOk && tofLeft.dataReady())   tofL = tofLeft.read(false) / 1000.0f;
      if (tofRightOk && tofRight.dataReady()) tofR = tofRight.read(false) / 1000.0f;
    }

    // ---- Pull command, apply (with timeout) ----
    float cmd_v, cmd_w;
    uint32_t cmd_ts;
    portENTER_CRITICAL(&stateMux);
    cmd_v = g.cmd_v; cmd_w = g.cmd_w; cmd_ts = g.cmd_stamp_ms;
    // publish odom/imu into shared
    g.x = x; g.y = y; g.yaw = yaw; g.vx = vx; g.vyaw = vyaw;
    g.ax = ax; g.ay = ay; g.az = az; g.gx = gx; g.gy = gy; g.gz = gz;
    if (tof_div == 0) { g.tof_m[0] = tofF; g.tof_m[1] = tofL; g.tof_m[2] = tofR; }
    portEXIT_CRITICAL(&stateMux);

    if ((millis() - cmd_ts) > CMD_TIMEOUT_MS) { cmd_v = 0; cmd_w = 0; }
    applyWheelSpeeds(cmd_v, cmd_w);

    vTaskDelayUntil(&last_wake, period);
  }
}

// ======================================================================
// micro-ROS callbacks
// ======================================================================
static void cmd_vel_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  portENTER_CRITICAL(&stateMux);
  g.cmd_v = m->linear.x;
  g.cmd_w = m->angular.z;
  g.cmd_stamp_ms = millis();
  portEXIT_CRITICAL(&stateMux);
}

static void fill_range(sensor_msgs__msg__Range *m, float meters) {
  fill_stamp(&m->header.stamp);
  m->range = (meters >= 0) ? meters : INFINITY;   // out of range -> +inf per REP
}

static void timer_cb(rcl_timer_t *t, int64_t) {
  if (t == nullptr) return;
  static uint32_t tick = 0;
  tick++;

  SharedState s;
  portENTER_CRITICAL(&stateMux);
  s = g;
  portEXIT_CRITICAL(&stateMux);

  // ---- IMU every tick (timer runs at PUB_IMU_HZ) ----
  fill_stamp(&imu_msg.header.stamp);
  imu_msg.linear_acceleration.x = s.ax;
  imu_msg.linear_acceleration.y = s.ay;
  imu_msg.linear_acceleration.z = s.az;
  imu_msg.angular_velocity.x = s.gx;
  imu_msg.angular_velocity.y = s.gy;
  imu_msg.angular_velocity.z = s.gz;
  imu_msg.orientation_covariance[0] = -1.0;   // no orientation from raw IMU
  RCSOFT(rcl_publish(&pub_imu, &imu_msg, NULL));

  // ---- Odom decimated ----
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

  // ---- ToF decimated ----
  if (tick % (PUB_IMU_HZ / PUB_TOF_HZ) == 0) {
    fill_range(&tof_f_msg, s.tof_m[0]); RCSOFT(rcl_publish(&pub_tof_f, &tof_f_msg, NULL));
    fill_range(&tof_l_msg, s.tof_m[1]); RCSOFT(rcl_publish(&pub_tof_l, &tof_l_msg, NULL));
    fill_range(&tof_r_msg, s.tof_m[2]); RCSOFT(rcl_publish(&pub_tof_r, &tof_r_msg, NULL));
  }
}

// ======================================================================
// micro-ROS entity lifecycle
// ======================================================================
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
}

static bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_base", "", &support));

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

  RCCHECK(rclc_subscription_init(
      &sub_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel",
      &qos_reliable));

  const unsigned int timer_period = 1000 / PUB_IMU_HZ;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_period), timer_cb));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &cmd_msg, &cmd_vel_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  RCSOFT(rmw_uros_sync_session(1000));   // sync clock for timestamps
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
  rcl_subscription_fini(&sub_cmd, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ======================================================================
// Core 0: micro-ROS (+ WiFi on WiFi builds)
// ======================================================================
static void microRosTask(void *arg) {
  (void)arg;
  init_messages();
  for (;;) {
    switch (agent_state) {
      case WAITING_AGENT:
        agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
        break;
      case AGENT_AVAILABLE:
        agent_state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
        if (agent_state == WAITING_AGENT) destroy_entities();
        break;
      case AGENT_CONNECTED:
        if (RMW_RET_OK != rmw_uros_ping_agent(200, 3)) {
          agent_state = AGENT_DISCONNECTED;
        } else {
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        }
        break;
      case AGENT_DISCONNECTED:
        destroy_entities();
        agent_state = WAITING_AGENT;
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ======================================================================
// Setup
// ======================================================================
void setup() {
  char wifi_ssid[] = MICROROS_WIFI_SSID;
  char wifi_pass[] = MICROROS_WIFI_PASS;
  init_microros_transport(wifi_ssid, wifi_pass, AGENT_IP, AGENT_PORT);

  // I2C buses
  Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_FREQ);
  Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, I2C_FREQ);

  // Encoders
  encR.begin();
  encL.begin();

  // IMU (MPU9250 on bus 0)
  mpu.setup(0x68, MPU9250Setting(), Wire);   // TODO: handle return / retries

  // ToF trio
  setupTofTrio();

  // Stepper enable
  if (PIN_EN >= 0) {
    pinMode(PIN_EN, OUTPUT);
    digitalWrite(PIN_EN, EN_ACTIVE_LOW ? LOW : HIGH);   // enable drivers
  }

  // Steppers
  engine.init();
  stepR = engine.stepperConnectToPin(PIN_R_STEP);
  stepL = engine.stepperConnectToPin(PIN_L_STEP);
  if (stepR) {
    stepR->setDirectionPin(PIN_R_DIR, !R_DIR_INVERT);
    stepR->setAcceleration((uint32_t)STEP_ACCEL_HZ_S);
  }
  if (stepL) {
    stepL->setDirectionPin(PIN_L_DIR, !L_DIR_INVERT);
    stepL->setAcceleration((uint32_t)STEP_ACCEL_HZ_S);
  }

  // Tasks: micro-ROS on core 0, control on core 1.
  xTaskCreatePinnedToCore(controlTask, "control", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(microRosTask, "microros", 8192, NULL, 5, NULL, 0);
}

void loop() {
  // Everything runs in the FreeRTOS tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
