// ESP32-S3 micro-ROS firmware for the obstacle-avoider rover.
//
// Exposes the same ROS 2 interface as the sim:
//     publishes: /ultrasonic/front|left|right   (sensor_msgs/Range)   <- 3x HC-SR04
//     publishes: /joint_states                  (sensor_msgs/JointState) <- wheel encoders
//     subscribes: /cmd_vel                       (geometry_msgs/Twist) -> L298N motors
//
// Transport: WiFi/UDP to the micro-ROS Agent. Flash over USB, then it runs untethered.
//
// >>> EDIT THE CONFIG BLOCK BELOW for your WiFi, your Agent's IP, and your wiring. <<<
// Hardware: ESP32-S3 + L298N dual H-bridge (skid-steer: left pair / right pair)
// + 3x HC-SR04. If you use a different motor driver, only drive_side()/setup change.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_system.h>            // esp_reset_reason() -> tells brownout from power-on
#include <soc/gpio_reg.h>          // GPIO_IN_REG: IRAM-safe pin reads inside the ISRs
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/twist.h>

// ============================ CONFIG — EDIT ME ============================
// WiFi + Agent
static char     WIFI_SSID[] = "YOUR_WIFI_SSID";
static char     WIFI_PASS[] = "YOUR_WIFI_PASSWORD";
static uint8_t  AGENT_IP[4] = {192, 168, 0, 100};   // the PC running micro_ros_agent
static uint16_t    AGENT_PORT   = 8888;

// HC-SR04 ultrasonics: {trig, echo} pins  (ESP32-S3 safe GPIOs)
// NOTE: HC-SR04 echo is 5V - use a level shifter / voltage divider to the S3 (3.3V).
static const int US_FRONT[2] = {10, 11};
static const int US_LEFT[2]  = {12, 13};
static const int US_RIGHT[2] = {14, 21};

// L298N motor driver — left channel (both left wheels) / right channel (both right wheels)
static const int L_EN = 6,  L_IN1 = 4, L_IN2 = 5;      // ENA, IN1, IN2
static const int R_EN = 16, R_IN1 = 7, R_IN2 = 15;     // ENB, IN3, IN4

// --- ENCODER CONFIG ---
// Use interrupt-capable pins.
static const int ENC_L_A = 17, ENC_L_B = 18;
static const int ENC_R_A = 8, ENC_R_B = 9;
// Adjust this based on your exact JGB37-520 gearbox (e.g., 11 PPR * 30:1 ratio = 330)
static const float TICKS_PER_REV = 330.0f;

// Kinematics / tuning
static const float WHEEL_SEP   = 0.251f;   // m, left-right track width
static const float WHEEL_RADIUS = 0.0325f; // m (65 mm wheel) — sets m/s <-> ticks/sec
static const float MAX_LIN     = 0.25f;   // m/s (open-loop fallback only, see below)
static const int   PWM_FREQ    = 1000;    // Hz
static const int   US_MAX_M    = 2;       // clamp ultrasonic range (m)

// ===================== CLOSED-LOOP WHEEL VELOCITY PID =====================
// Ported from test/PID.cpp, which was tuned and verified on THIS robot. Each wheel runs
// its own velocity PID: the encoder measures actual wheel speed (ticks/sec) and the
// controller trims PWM to hit the target. /cmd_vel is converted to per-wheel ticks/sec,
// so the vehicle now tracks the COMMANDED speed instead of an open-loop PWM guess.
//
// Why this matters beyond straight-line accuracy:
//   * it removes the PWM deadband problem — the integrator simply raises PWM until the
//     wheel actually turns, so slow speeds are reachable
//   * it removes the mechanical right-drift, because each wheel is regulated to its own
//     target rather than both getting the same PWM
// HOW THE WHEELS ARE REGULATED — pick one:
//   VEL_OPEN = no encoder feedback at all, /cmd_vel mapped straight to PWM (original)
//   VEL_BANG = bang-bang on encoder feedback (DEFAULT, no gains, best at slow speeds)
//   VEL_PID  = closed-loop velocity PID from test/PID.cpp (needs speeds >= ~0.07 m/s or it
//              stalls/lurches, because the motor cannot hold a slower continuous PWM)
#define VEL_OPEN 0
#define VEL_BANG 1
#define VEL_PID  2
#define VELOCITY_MODE VEL_BANG

// convenience: any mode that reads the encoders and runs the fixed-rate control loop
#define VELOCITY_CLOSED_LOOP (VELOCITY_MODE == VEL_BANG || VELOCITY_MODE == VEL_PID)

// --- bang-bang knobs (used when VELOCITY_MODE == VEL_BANG) ---
// BANG_PWM must comfortably exceed the ~70 PWM stiction point or the wheel never starts.
// Lower it for gentler pulsing, raise it if the wheel is slow to break away.
static int   BANG_PWM      = 105;
// Tolerance around the target speed, in ticks/sec. Wider = slower pulsing, coarser speed.
static float BANG_HYST_TPS = 18.0f;

static const int   CONTROL_HZ  = 50;                    // PID rate (Hz)
static const float CONTROL_DT  = 1.0f / CONTROL_HZ;     // s per control step
// SAFETY cap on PID output (0..255). Raised from 110: with the MEASURED KFF below, 110 PWM
// only reaches ~170 tps (~0.11 m/s), so a 0.12 m/s command would sit saturated forever.
// 200 PWM allows ~310 tps (~0.19 m/s), which covers the follower's speed range.
static int         MAX_PWM     = 200;

// Gains from the verified bench tuning. Domain: error in ticks/sec -> output in PWM.
// KD stays 0: at low speed the encoder yields only a few ticks per 20 ms window, so the
// speed reading is quantized in ~50 tps steps and D just amplifies that into PWM thrash.
static float KP = 0.14f;      // softened from 0.20 after a step test showed ~30% overshoot
static float KI = 1.00f;      // per second; removes steady-state error
static float KD = 0.0f;       // leave at 0 unless encoder resolution/speed goes up
// Feedforward: PWM needed per tick/sec. MEASURED with test/MainFinal.cpp: PWM 180 produced
// ~280 tps on both wheels -> 180/280 = 0.64. (test/PID.cpp assumed 0.11 from "110 PWM ->
// 1000 tps"; that is 5.8x off and left the feedforward contributing almost nothing, so the
// integrator had to do all the work and the loop sat saturated. This is very likely why
// PID.cpp only worked "to an extent".)
static float KFF = 0.64f;
static float MEAS_ALPHA = 0.30f;      // EMA low-pass on measured speed (tames quantization)
static float MAX_ACCEL_TPS = 800.0f;  // setpoint ramp (ticks/sec^2), avoids startup lurch

// Encoder feedback sign: measured speed MUST read POSITIVE when the wheel is driven
// FORWARD, otherwise the loop becomes positive feedback and that wheel runs away.
//
// MEASURED with test/MainFinal.cpp (same pins, same ISR, same RISING edge as here),
// both motors driven forward at PWM 180:
//     left  ticks: -56 per 200 ms  -> ~-280 tps  -> counts BACKWARD -> sign -1
//     right ticks: +57 per 200 ms  -> ~+285 tps  -> counts FORWARD  -> sign +1
// Equal magnitude, OPPOSITE sense, so the two signs must differ.
// (test/PID.cpp's older note claimed BOTH were backward. That predates the ESP32-S3
//  swap and the 5V-logic rewiring, which evidently flipped the right encoder's A/B.
//  A wrong ENC_R_SIGN made the right wheel saturate while the runaway guard kept
//  resetting both PIDs, so the left wheel never spun up.)
static const int ENC_L_SIGN = -1;
static const int ENC_R_SIGN = +1;

// Motor direction flags (mirrored 2WD mounting). Both false on this robot.
static const bool INVERT_LEFT_MOTOR  = false;
static const bool INVERT_RIGHT_MOTOR = false;

// Ceiling implied by the safety cap: MAX_PWM/KFF ticks/sec.
static const float MAX_TPS = 300.0f;     // ~0.91 rev/s ~ 0.19 m/s at r=0.0325
// Stop the wheels if no /cmd_vel arrives for this long (the follower publishes at camera
// rate; if it dies the robot must not keep rolling on the last command).
static const uint32_t CMD_TIMEOUT_MS = 500;
// =========================================================================

// --- WiFi robustness / boot diagnostics ---
// The micro-ROS library's set_microros_wifi_transports() ends in an UNBOUNDED
//   while (WiFi.status() != WL_CONNECTED) delay(500);
// so if the join ever fails the board hangs in setup() forever: no encoder prints, no
// state prints, looks bricked (this is what a sagging supply looked like on battery).
// We therefore join WiFi ourselves with a timeout + retries and reboot instead of
// hanging; by the time the library runs, WiFi is already up so its wait returns at once.
static const uint32_t WIFI_ATTEMPT_MS     = 12000;  // per-attempt connect timeout
static const int      WIFI_MAX_ATTEMPTS   = 3;      // then reboot and start clean
static const uint32_t WIFI_LOSS_REBOOT_MS = 20000;  // link lost this long at run time -> reboot
// Boot-time network scan is a ~7 s, full-band, max-TX current spike. Off by default now;
// set to 1 when you actually need to see which APs are visible.
#define WIFI_SCAN_ON_BOOT 0
// Cap TX power to cut peak current on a marginal supply (costs some range).
#define WIFI_LOW_TX_POWER 0
// ========================================================================

// ---- micro-ROS handles ----
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_publisher_t pub_front, pub_left, pub_right, pub_joints;
rcl_subscription_t sub_cmd;
rcl_timer_t timer;
sensor_msgs__msg__Range range_front, range_left, range_right;
geometry_msgs__msg__Twist cmd_msg;
sensor_msgs__msg__JointState joint_msg;

// Memory allocation for the JointState strings and arrays
rosidl_runtime_c__String joint_names[2];
double joint_positions[2];

// ---- agent connection state machine (standard micro-ROS reconnect pattern) ----
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState state = WAITING_AGENT;

#define RCCHECK(fn)    { if ((fn) != RCL_RET_OK) return false; }
#define EXEC_EVERY(MS, X)  do { static volatile int64_t t=-1; \
  if (t==-1) t=uxr_millis(); if ((int32_t)(uxr_millis()-t) > (MS)) { X; t=uxr_millis(); } } while (0)

// ---- Hardware Encoder Interrupts ----
volatile int32_t enc_left_ticks = 0;
volatile int32_t enc_right_ticks = 0;
// Quadrature form from test/PID.cpp: comparing A against B is more robust to edge timing
// than sampling B alone.
//
// IRAM SAFETY — why these read the GPIO register instead of calling digitalRead():
// this Arduino core is built WITHOUT CONFIG_ARDUINO_ISR_IRAM, so its ARDUINO_ISR_ATTR
// expands to nothing and digitalRead() is placed in FLASH. Marking the ISR IRAM_ATTR is
// then pointless: the moment it calls digitalRead() it jumps back into flash. If an
// encoder edge lands while the flash cache is disabled (normal during SPI-flash access)
// the CPU faults with "Cache disabled but cached memory region accessed" and the chip
// reboots — which looks exactly like the micro-ROS session churn we were chasing, and is
// the same failure mode as the known Wire-plus-interrupts crash on ESP32.
// REG_READ is a macro over a volatile load, so it inlines and never leaves IRAM.
// Bonus: one register read samples A and B at the SAME instant, which is more correct
// than two sequential digitalRead() calls.
// (All four encoder pins are < 32, so GPIO_IN_REG covers them.)
void IRAM_ATTR isr_enc_left() {
  uint32_t g = REG_READ(GPIO_IN_REG);
  if (((g >> ENC_L_B) & 1U) == ((g >> ENC_L_A) & 1U)) enc_left_ticks++;
  else                                                enc_left_ticks--;
}
void IRAM_ATTR isr_enc_right() {
  uint32_t g = REG_READ(GPIO_IN_REG);
  if (((g >> ENC_R_B) & 1U) == ((g >> ENC_R_A) & 1U)) enc_right_ticks++;
  else                                                enc_right_ticks--;
}

// ---- HC-SR04: one blocking ping -> metres ----
float read_ultrasonic(const int pins[2]) {
  digitalWrite(pins[0], LOW);  delayMicroseconds(2);
  digitalWrite(pins[0], HIGH); delayMicroseconds(10);
  digitalWrite(pins[0], LOW);
  long us = pulseIn(pins[1], HIGH, US_MAX_M * 6000);   // timeout ~ round trip for US_MAX_M
  if (us == 0) return (float)US_MAX_M;                 // no echo -> max range
  float m = (us * 0.000343f) / 2.0f;
  return m > US_MAX_M ? (float)US_MAX_M : m;
}

// ---- L298N: signed [-1,1] command per side ----
void drive_side(int en, int in1, int in2, float cmd) {
  cmd = constrain(cmd, -1.0f, 1.0f);
  digitalWrite(in1, cmd >= 0);
  digitalWrite(in2, cmd < 0);
  analogWrite(en, (int)(fabs(cmd) * 255));
}

// ---- raw PWM out, honouring the mirrored-mounting inversion flags ----
void set_motor_pwm(int en, int in1, int in2, int pwm, bool inverted) {
  if (inverted) pwm = -pwm;
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  analogWrite(en, pwm); }
  else if (pwm < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); analogWrite(en, -pwm); }
  else              { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  analogWrite(en, 0); }
}

// ================= BANG-BANG per-wheel velocity control (NO PID) =================
// Why bang-bang and not PID for this drivetrain:
// these motors need ~70 PWM to break stiction, which is already ~110 ticks/s (~0.07 m/s).
// Anything slower cannot be held with a CONTINUOUS PWM, so a PID either stalls or lurches
// as its integrator winds up and collapses. Bang-bang sidesteps that entirely: kick a
// healthy PWM to get the wheel moving, coast once it passes the target, repeat. The wheel
// pulses, but the AVERAGE speed can be far below the motor's minimum continuous speed,
// which is exactly what we need for a slow tag follower.
//
// Per wheel, working in the commanded direction:
//     speed_along = sign(target) * measured        (positive = moving the way we asked)
//     speed_along < |target| - hyst  -> DRIVE at BANG_PWM
//     speed_along > |target| + hyst  -> COAST (0), let friction bleed it off
//     in between                      -> hold whatever we were doing (adds hysteresis)
// There are no gains. The only knobs are the drive level and the tolerance.
struct Bang {
  int   pwm = 0;          // last output, so the middle band can hold it
};
Bang leftBang, rightBang;

int bang_step(Bang &b, float target, float measured) {
  if (fabsf(target) < 1.0f) {            // asked to stop
    b.pwm = 0;
    return 0;
  }
  float dir = (target > 0.0f) ? 1.0f : -1.0f;
  float speed_along = dir * measured;    // how fast we are going the commanded way
  float mag = fabsf(target);

  if (speed_along < mag - BANG_HYST_TPS)        b.pwm = (int)(dir * BANG_PWM);
  else if (speed_along > mag + BANG_HYST_TPS)   b.pwm = 0;
  // else: leave b.pwm untouched -> hysteresis, no chatter at the setpoint
  return b.pwm;
}

void bang_reset(Bang &b) { b.pwm = 0; }

// ---- per-wheel velocity PID (kept, but OFF by default; see VELOCITY_MODE) ----
struct PID {
  float integral   = 0.0f;
  float prevError  = 0.0f;
  float rampTarget = 0.0f;    // eased setpoint the PID actually chases
};
PID leftPID, rightPID;

// target/measured in ticks/sec -> returns PWM, clamped to +/-MAX_PWM
float pid_step(PID &c, float target, float measured, float dt) {
  // ease the working setpoint so there is no instant full-speed demand
  float maxStep = MAX_ACCEL_TPS * dt;
  c.rampTarget += constrain(target - c.rampTarget, -maxStep, maxStep);
  float sp = c.rampTarget;

  float error = sp - measured;
  float derivative = (error - c.prevError) / dt;
  c.prevError = error;

  float feedforward = KFF * sp;      // does most of the work; PID only trims

  // anti-windup: clamp the integral so KI*integral + FF cannot exceed the safety cap
  c.integral += error * dt;
  if (KI > 0.0001f) {
    float iMax = ( (float)MAX_PWM - feedforward) / KI;
    float iMin = (-(float)MAX_PWM - feedforward) / KI;
    c.integral = constrain(c.integral, iMin, iMax);
  }

  float out = feedforward + KP * error + KI * c.integral + KD * derivative;
  return constrain(out, -(float)MAX_PWM, (float)MAX_PWM);
}

void pid_reset(PID &c) { c.integral = 0.0f; c.prevError = 0.0f; c.rampTarget = 0.0f; }

// ---- commanded wheel speeds (ticks/sec), written by /cmd_vel ----
volatile float g_target_l_tps = 0.0f;
volatile float g_target_r_tps = 0.0f;
volatile uint32_t g_last_cmd_ms = 0;

// m/s at the wheel -> ticks/sec:  (v / circumference) rev/s * TICKS_PER_REV
static inline float mps_to_tps(float v) {
  return (v / (2.0f * PI * WHEEL_RADIUS)) * TICKS_PER_REV;
}

// ---- /cmd_vel -> differential mixing ----
void on_cmd(const void* msgin) {
  const geometry_msgs__msg__Twist* m = (const geometry_msgs__msg__Twist*)msgin;
  float v = m->linear.x, w = m->angular.z;
  float vl = v - w * WHEEL_SEP / 2.0f;         // m/s at the left wheel
  float vr = v + w * WHEEL_SEP / 2.0f;         // m/s at the right wheel
  g_last_cmd_ms = millis();
#if VELOCITY_CLOSED_LOOP
  g_target_l_tps = constrain(mps_to_tps(vl), -MAX_TPS, MAX_TPS);
  g_target_r_tps = constrain(mps_to_tps(vr), -MAX_TPS, MAX_TPS);
#else
  drive_side(L_EN, L_IN1, L_IN2, vl / MAX_LIN);
  drive_side(R_EN, R_IN1, R_IN2, vr / MAX_LIN);
#endif
}

#if VELOCITY_CLOSED_LOOP
// One closed-loop update for both wheels. Call at exactly CONTROL_HZ.
void velocity_control_step() {
  static int32_t lastLeft = 0, lastRight = 0;
  static float leftMeasured = 0.0f, rightMeasured = 0.0f;

  // STARVATION GUARD: if loop() was blocked (a ping can stall it for up to ~1.5 s), the
  // motors were still holding their last PWM the whole time — with bang-bang that means
  // driving blind at BANG_PWM. Detect the gap, cut the outputs and reset the controllers
  // so we restart from a known state instead of continuing an unsupervised command.
  static uint32_t last_step_ms = 0;
  uint32_t now_ms = millis();
  if (last_step_ms != 0 && (now_ms - last_step_ms) > 200) {
    set_motor_pwm(L_EN, L_IN1, L_IN2, 0, INVERT_LEFT_MOTOR);
    set_motor_pwm(R_EN, R_IN1, R_IN2, 0, INVERT_RIGHT_MOTOR);
    bang_reset(leftBang);
    bang_reset(rightBang);
    pid_reset(leftPID);
    pid_reset(rightPID);
    leftMeasured = rightMeasured = 0.0f;
    // resync the tick baseline so the stall does not look like a huge speed spike
    noInterrupts();
    lastLeft = enc_left_ticks;
    lastRight = enc_right_ticks;
    interrupts();
    last_step_ms = now_ms;
    Serial.println("[vel] control loop was starved -> motors cut, controllers reset");
    return;
  }
  last_step_ms = now_ms;

  // atomic read of the ISR-updated counters
  int32_t lc, rc;
  noInterrupts();
  lc = enc_left_ticks;
  rc = enc_right_ticks;
  interrupts();

  // ENC_*_SIGN makes "driven forward" read POSITIVE, which is what keeps the feedback
  // negative instead of runaway.
  float leftRaw  = ENC_L_SIGN * (lc - lastLeft)  / CONTROL_DT;
  float rightRaw = ENC_R_SIGN * (rc - lastRight) / CONTROL_DT;
  lastLeft = lc;
  lastRight = rc;

  // EMA low-pass before the PID sees it (encoder speed is coarsely quantized at low rpm)
  leftMeasured  += MEAS_ALPHA * (leftRaw  - leftMeasured);
  rightMeasured += MEAS_ALPHA * (rightRaw - rightMeasured);

  // safety: stale /cmd_vel -> command zero speed
  float tl = g_target_l_tps, tr = g_target_r_tps;
  if (millis() - g_last_cmd_ms > CMD_TIMEOUT_MS) { tl = 0.0f; tr = 0.0f; }

  // fully stopped and asked to stay stopped -> cut PWM and clear the integrators, so the
  // controller cannot buzz or creep while holding still
  if (tl == 0.0f && tr == 0.0f) {
    pid_reset(leftPID);
    pid_reset(rightPID);
    bang_reset(leftBang);
    bang_reset(rightBang);
    set_motor_pwm(L_EN, L_IN1, L_IN2, 0, INVERT_LEFT_MOTOR);
    set_motor_pwm(R_EN, R_IN1, R_IN2, 0, INVERT_RIGHT_MOTOR);
    return;
  }

#if VELOCITY_MODE == VEL_BANG
  int lpwm = bang_step(leftBang,  tl, leftMeasured);
  int rpwm = bang_step(rightBang, tr, rightMeasured);
#else
  int lpwm = (int)pid_step(leftPID,  tl, leftMeasured,  CONTROL_DT);
  int rpwm = (int)pid_step(rightPID, tr, rightMeasured, CONTROL_DT);
#endif

  // ---- runaway watchdog: if we drive hard but the wheel spins the OPPOSITE way, the
  //      feedback sign is inverted (positive feedback). Stop and say which sign to flip.
  static int badL = 0, badR = 0;
#if VELOCITY_MODE == VEL_BANG
  const int PWM_ACTIVE = BANG_PWM / 2;
#else
  const int PWM_ACTIVE = MAX_PWM / 2;
#endif
  badL = (abs(lpwm) > PWM_ACTIVE && leftMeasured  * tl < 0) ? badL + 1 : 0;
  badR = (abs(rpwm) > PWM_ACTIVE && rightMeasured * tr < 0) ? badR + 1 : 0;
  if (badL > 15 || badR > 15) {                 // ~0.3 s of consistent wrong-way motion
    bool tL = badL > 15, tR = badR > 15;
    set_motor_pwm(L_EN, L_IN1, L_IN2, 0, INVERT_LEFT_MOTOR);
    set_motor_pwm(R_EN, R_IN1, R_IN2, 0, INVERT_RIGHT_MOTOR);
    pid_reset(leftPID);
    pid_reset(rightPID);
    bang_reset(leftBang);
    bang_reset(rightBang);
    badL = badR = 0;
    Serial.println("!! RUNAWAY GUARD: wheel spinning opposite to command -> encoder sign inverted.");
    if (tL) Serial.println("!!  Flip ENC_L_SIGN (left):  +1 <-> -1");
    if (tR) Serial.println("!!  Flip ENC_R_SIGN (right): +1 <-> -1");
    return;
  }

  set_motor_pwm(L_EN, L_IN1, L_IN2, lpwm, INVERT_LEFT_MOTOR);
  set_motor_pwm(R_EN, R_IN1, R_IN2, rpwm, INVERT_RIGHT_MOTOR);

  // tuning telemetry: watch meas converge on tgt
  EXEC_EVERY(500, Serial.printf("[vel] tgtL:%d tgtR:%d measL:%d measR:%d pwmL:%d pwmR:%d\n",
                                (int)tl, (int)tr, (int)leftMeasured, (int)rightMeasured,
                                lpwm, rpwm));
}
#endif  // VELOCITY_CLOSED_LOOP

void fill_range(sensor_msgs__msg__Range* r, const char* frame) {
  r->radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  r->field_of_view = 0.26f;
  r->min_range = 0.04f;
  r->max_range = (float)US_MAX_M;
  r->header.frame_id = micro_ros_string_utilities_set(r->header.frame_id, frame);
}

// ---- timer: read the ultrasonics and encoders, then publish ----
void on_timer(rcl_timer_t*, int64_t) {
  // 1. Ultrasonics — ROUND ROBIN, one sensor per tick.
  //    pulseIn BLOCKS for up to US_MAX_M*6000 us (~12 ms) when there is no echo. Reading
  //    all three back-to-back stalled this callback for ~36 ms every 100 ms, starving the
  //    micro-ROS executor and adding enough latency to make agent pings time out (which
  //    showed up as endless create/destroy session churn in the agent log).
  //    One per tick = ~12 ms worst case, and each sensor still refreshes at ~3.3 Hz.
  //    (Same pattern used in test/PID.cpp, for the same reason.)
  static uint8_t us_idx = 0;
  if (us_idx == 0)      range_front.range = read_ultrasonic(US_FRONT);
  else if (us_idx == 1) range_left.range  = read_ultrasonic(US_LEFT);
  else                  range_right.range = read_ultrasonic(US_RIGHT);
  us_idx = (us_idx + 1) % 3;

  // publish all three every tick; the two not just re-read keep their last value
  rcl_publish(&pub_front, &range_front, NULL);
  rcl_publish(&pub_left,  &range_left,  NULL);
  rcl_publish(&pub_right, &range_right, NULL);

  // 2. Publish encoders as JointStates: ticks -> radians = (ticks / ticks_per_rev) * 2*pi
  //    ENC_*_SIGN is applied HERE too, so the published angle INCREASES when the robot
  //    drives forward. The firmware knows its own wiring, so ROS does not have to guess:
  //    wheel_odometry can therefore run with encoder_direction = +1.
  joint_positions[0] = (ENC_L_SIGN * (double)enc_left_ticks  / TICKS_PER_REV) * 2.0 * PI;
  joint_positions[1] = (ENC_R_SIGN * (double)enc_right_ticks / TICKS_PER_REV) * 2.0 * PI;
  int64_t time_ns = rmw_uros_epoch_nanos();
  joint_msg.header.stamp.sec     = time_ns / 1000000000;
  joint_msg.header.stamp.nanosec = time_ns % 1000000000;
  rcl_publish(&pub_joints, &joint_msg, NULL);
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "microbot_esp32", "", &support));

  RCCHECK(rclc_publisher_init_default(&pub_front, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "/ultrasonic/front"));
  RCCHECK(rclc_publisher_init_default(&pub_left, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "/ultrasonic/left"));
  RCCHECK(rclc_publisher_init_default(&pub_right, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "/ultrasonic/right"));
  RCCHECK(rclc_publisher_init_default(&pub_joints, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/joint_states"));
  RCCHECK(rclc_subscription_init_default(&sub_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));

  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), on_timer));
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &cmd_msg, &on_cmd, ON_NEW_DATA));

  fill_range(&range_front, "us_front");
  fill_range(&range_left,  "us_left");
  fill_range(&range_right, "us_right");

  // Configure JointState message memory (names + position arrays)
  joint_msg.header.frame_id = micro_ros_string_utilities_set(joint_msg.header.frame_id, "base_link");
  joint_msg.name.data = joint_names;
  joint_msg.name.size = 2;
  joint_msg.name.capacity = 2;
  joint_msg.name.data[0] = micro_ros_string_utilities_set(joint_msg.name.data[0], "base_back_left_wheel_joint");
  joint_msg.name.data[1] = micro_ros_string_utilities_set(joint_msg.name.data[1], "base_back_right_wheel_joint");
  joint_msg.position.data = joint_positions;
  joint_msg.position.size = 2;
  joint_msg.position.capacity = 2;

  // Sync time with the agent so JointState timestamps are valid
  rmw_uros_sync_session(1000);
  return true;
}

void destroy_entities() {
  rmw_context_t* rmw = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw, 0);
  rcl_publisher_fini(&pub_front, &node);
  rcl_publisher_fini(&pub_left, &node);
  rcl_publisher_fini(&pub_right, &node);
  rcl_publisher_fini(&pub_joints, &node);
  rcl_subscription_fini(&sub_cmd, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ---- boot diagnostics + WiFi supervision ----
// RTC_DATA_ATTR survives a software reset (ESP.restart) but not a power cycle, so these
// counters reveal a reboot LOOP without needing anything logged off-board.
RTC_DATA_ATTR static int rtc_boot_count = 0;
RTC_DATA_ATTR static int rtc_wifi_fail_streak = 0;
static uint32_t g_last_wifi_ok_ms = 0;

static const char* reset_reason_str(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON (clean power-up)";
    case ESP_RST_EXT:       return "EXT (reset pin)";
    case ESP_RST_SW:        return "SW (ESP.restart from our own code)";
    case ESP_RST_PANIC:     return "PANIC (firmware crashed)";
    case ESP_RST_INT_WDT:   return "INT_WDT (interrupt watchdog)";
    case ESP_RST_TASK_WDT:  return "TASK_WDT (task watchdog)";
    case ESP_RST_WDT:       return "WDT (other watchdog)";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT  <<< THE SUPPLY RAIL COLLAPSED";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

// One bounded join attempt. Returns true only if associated within timeout_ms.
static bool wifi_join(uint32_t timeout_ms) {
  WiFi.persistent(false);          // don't wear flash rewriting credentials
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);            // no power-save: steadier latency for micro-ROS
#if WIFI_LOW_TX_POWER
  WiFi.setTxPower(WIFI_POWER_11dBm);
#endif
  WiFi.disconnect(true);           // clear any half-open state from a previous attempt
  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis(), last = 0;
  while (millis() - t0 < timeout_ms) {
    if (WiFi.status() == WL_CONNECTED) return true;
    if (millis() - last >= 1000) {         // status codes: 0=IDLE 1=NO_SSID 4=FAIL 6=DISCONNECTED
      last = millis();
      Serial.printf("[microbot]   ...status=%d  (%lu ms)\n",
                    (int)WiFi.status(), (unsigned long)(millis() - t0));
    }
    delay(50);
  }
  return WiFi.status() == WL_CONNECTED;
}

void setup() {
  // motor + sensor GPIO
  for (int p : {L_EN, L_IN1, L_IN2, R_EN, R_IN1, R_IN2,
                US_FRONT[0], US_LEFT[0], US_RIGHT[0]}) pinMode(p, OUTPUT);
  for (int p : {US_FRONT[1], US_LEFT[1], US_RIGHT[1]}) pinMode(p, INPUT);
  // encoders get pull-ups (open-collector hall outputs), as in the tested test/PID.cpp
  for (int p : {ENC_L_A, ENC_L_B, ENC_R_A, ENC_R_B}) pinMode(p, INPUT_PULLUP);
  drive_side(L_EN, L_IN1, L_IN2, 0);
  drive_side(R_EN, R_IN1, R_IN2, 0);
  pid_reset(leftPID);
  pid_reset(rightPID);

  // Attach hardware interrupts for the wheel encoders
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_enc_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_enc_right, RISING);

  // === CHECKPOINT: serial_diag ===
  // Boot/WiFi/Agent diagnostics on the USB serial console (115200). Comment out this
  // block to run fully headless. `pio device monitor` shows the IP + connection state.
  Serial.begin(115200);
  delay(300);
  Serial.println();

  // --- why did we just boot? POWERON vs BROWNOUT vs PANIC is the whole ballgame when
  //     chasing a supply problem, and it costs nothing to print. ---
  rtc_boot_count++;
  esp_reset_reason_t why = esp_reset_reason();
  Serial.printf("[microbot] boot #%d   reset reason: %s\n",
                rtc_boot_count, reset_reason_str(why));
  if (why == ESP_RST_BROWNOUT)
    Serial.println("[microbot] !! BROWNOUT -> the 3V3/5V rail sagged. Fix the supply "
                   "(bulk caps at the board, buck current rating, wire gauge) first.");
  if (rtc_wifi_fail_streak > 0)
    Serial.printf("[microbot] NOTE: %d previous boot(s) rebooted because WiFi would not join\n",
                  rtc_wifi_fail_streak);

#if WIFI_SCAN_ON_BOOT
  // Diagnostic only: ~7 s and a big current spike, so it is off by default.
  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks();
  Serial.printf("[microbot] scan found %d networks:\n", n);
  for (int i = 0; i < n; i++)
    Serial.printf("    '%s'  rssi=%d  ch=%d  enc=%d\n",
                  WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i), WiFi.encryptionType(i));
#endif

  // --- bounded join: timeout + retries, then reboot. Never hang silently. ---
  bool joined = false;
  for (int a = 1; a <= WIFI_MAX_ATTEMPTS && !joined; a++) {
    Serial.printf("[microbot] WiFi attempt %d/%d -> SSID='%s'\n",
                  a, WIFI_MAX_ATTEMPTS, WIFI_SSID);
    joined = wifi_join(WIFI_ATTEMPT_MS);
    if (!joined)
      Serial.printf("[microbot] attempt %d FAILED (status=%d)\n", a, (int)WiFi.status());
  }

  if (!joined) {
    rtc_wifi_fail_streak++;
    Serial.printf("[microbot] WiFi unreachable after %d attempts -> REBOOTING (streak=%d)\n",
                  WIFI_MAX_ATTEMPTS, rtc_wifi_fail_streak);
    Serial.flush();
    delay(200);
    ESP.restart();                    // start clean rather than block forever
  }
  rtc_wifi_fail_streak = 0;
  g_last_wifi_ok_ms = millis();

  Serial.printf("[microbot] WiFi OK   IP=%s  RSSI=%d dBm\n",
                WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
  Serial.printf("[microbot] agent target = %d.%d.%d.%d:%u\n",
                AGENT_IP[0], AGENT_IP[1], AGENT_IP[2], AGENT_IP[3], AGENT_PORT);
  // === END CHECKPOINT: serial_diag ===

  // WiFi is already associated here, so the library's internal
  //   while (WiFi.status() != WL_CONNECTED) delay(500);
  // falls straight through instead of trapping us.
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
}

void loop() {
  // === CHECKPOINT: serial_diag ===
  // Print agent-connection state whenever it changes (heartbeat every 2 s while WAITING).
  static AgentState last_state = AGENT_DISCONNECTED;
  static const char* NM[] = {"WAITING_AGENT", "AGENT_AVAILABLE", "AGENT_CONNECTED", "AGENT_DISCONNECTED"};
  if (state != last_state) { Serial.printf("[microbot] state -> %s\n", NM[state]); last_state = state; }
  if (state == WAITING_AGENT) EXEC_EVERY(2000, Serial.println("[microbot] waiting for agent ping..."));

  // WiFi link heartbeat every 3 s so you can confirm the connection stays up.
  EXEC_EVERY(3000, {
    if (WiFi.status() == WL_CONNECTED)
      Serial.printf("[microbot] WiFi CONNECTED  IP=%s  RSSI=%d dBm\n",
                    WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
    else
      Serial.printf("[microbot] WiFi NOT connected (status=%d)\n", (int)WiFi.status());
  });

  // --- closed-loop wheel velocity: fixed-rate, non-blocking, independent of the agent
  //     so the controller keeps regulating (to zero) even while micro-ROS reconnects. ---
#if VELOCITY_CLOSED_LOOP
  {
    static uint32_t last_ctrl_us = 0;
    const uint32_t period_us = 1000000UL / CONTROL_HZ;
    uint32_t now_us = micros();
    if (now_us - last_ctrl_us >= period_us) {
      last_ctrl_us += period_us;
      if (now_us - last_ctrl_us > 4 * period_us) last_ctrl_us = now_us;  // resync if starved
      velocity_control_step();
    }
  }
#endif

  // --- WiFi supervision: a dead link means no /cmd_vel can ever arrive, so stop the
  //     motors immediately and reboot if it does not come back. Prevents both a runaway
  //     robot and an endless silent sulk. ---
  if (WiFi.status() == WL_CONNECTED) {
    g_last_wifi_ok_ms = millis();
  } else {
    g_target_l_tps = 0.0f;                    // let the PID brake to a stop
    g_target_r_tps = 0.0f;
    drive_side(L_EN, L_IN1, L_IN2, 0);
    drive_side(R_EN, R_IN1, R_IN2, 0);
    if (millis() - g_last_wifi_ok_ms > WIFI_LOSS_REBOOT_MS) {
      Serial.printf("[microbot] WiFi down >%lu ms -> REBOOTING\n",
                    (unsigned long)WIFI_LOSS_REBOOT_MS);
      Serial.flush();
      delay(200);
      ESP.restart();
    }
  }

  // Encoder heartbeat every 250 ms: raw tick counts + wheel angle (rad). Spin a wheel by
  // hand and watch the ticks move (and the sign) — quick way to verify wiring/direction.
  EXEC_EVERY(250, {
    Serial.printf("[enc] L_ticks=%ld  R_ticks=%ld  |  L=%.2f rad  R=%.2f rad\n",
                  (long)enc_left_ticks, (long)enc_right_ticks,
                  (double)enc_left_ticks  / TICKS_PER_REV * 2.0 * PI,
                  (double)enc_right_ticks / TICKS_PER_REV * 2.0 * PI);
  });
  // === END CHECKPOINT: serial_diag ===

  // standard micro-ROS reconnect lifecycle: survive the Agent restarting
  switch (state) {
    // PING TOLERANCE: this used to be ping(100 ms, 3) checked every 200 ms. A 100 ms
    // round-trip budget is far too tight on 2.4 GHz shared with the ESP32-CAM's MJPEG
    // stream — ordinary latency spikes failed all 3 attempts, so the firmware tore down
    // its own session and rebuilt it, over and over (the create/destroy churn in the agent
    // log). The CAM firmware has always used 1000 ms here and never churned.
    // 500 ms x 3 is 5x more tolerant, and checking every second keeps the worst-case
    // blocking (1.5 s, only when the link really is dead) bounded.
    case WAITING_AGENT:
      EXEC_EVERY(500, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 1))
                              ? AGENT_AVAILABLE : WAITING_AGENT);
      break;
    case AGENT_AVAILABLE:
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroy_entities();
      break;
    case AGENT_CONNECTED:
      EXEC_EVERY(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 3))
                               ? AGENT_CONNECTED : AGENT_DISCONNECTED);
      if (state == AGENT_CONNECTED)
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
      break;
    case AGENT_DISCONNECTED:
      g_target_l_tps = 0.0f;                 // safety: stop on link loss
      g_target_r_tps = 0.0f;
      drive_side(L_EN, L_IN1, L_IN2, 0);
      drive_side(R_EN, R_IN1, R_IN2, 0);
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}
