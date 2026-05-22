#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose2_d.h>

// ===================== WIFI + AGENT =====================
char WIFI_SSID[] = "iPhoneDanae";
char WIFI_PASS[] = "daanaecas";
char AGENT_IP[]  = "172.20.10.4";
uint16_t AGENT_PORT = 8888;

// ===================== PINES =====================
// RUEDA A (Izquierda)  L298N
const int pwm1_A = 32, pwm2_A = 33;
const int pinA_A = 25, pinB_A = 26;
volatile long cont_A = 0;

// RUEDA B (Derecha)    L298N
const int pwm1_B = 18, pwm2_B = 19;
const int pinA_B = 27, pinB_B = 14;
volatile long cont_B = 0;

// ===== LEDC (PWM) =====
const int CH_A1 = 0;
const int CH_A2 = 1;
const int CH_B1 = 2;
const int CH_B2 = 3;

const int PWM_FREQ = 1000;     // 1 kHz
const int PWM_RES  = 10;       // 10 bits -> 0..1023

// ===================== FÍSICA =====================
const float D_RUEDA = 0.067f;
const float C_RUEDA = (float)M_PI * D_RUEDA;

// 44 cuentas/vuelta encoder * 35 vueltas encoder por vuelta rueda = 1540 cuentas/vuelta rueda
const float CUENTAS_VUELTA = 1540.0f;
const float METROS_POR_CUENTA = C_RUEDA / CUENTAS_VUELTA;

// Distancia entre ruedas (m)
const float L_RUEDAS = 0.25f;

// ===================== CONTROL =====================
const float Ts = 0.1f;
const unsigned long CTRL_MS = 100UL;

unsigned long lastTime = 0;
long last_cont_A = 0, last_cont_B = 0;

// ===================== PID RUEDAS =====================
float Kp_A = 0.06013f, Ki_A = 2.617f, Kd_A = 0.0003453f;
float e_prev_A = 0.0f, s_A = 0.0f;

float Kp_B = 0.06071f, Ki_B = 2.649f, Kd_B = 0.0003479f;
float e_prev_B = 0.0f, s_B = 0.0f;

// ===================== ODOMETRÍA =====================
float x_robot = 0.0f, y_robot = 0.0f, theta = 0.0f;

// Si el theta gira al revés, cambia a true
const bool INVERT_B_ODOM = false;

// ===================== LIMITES SEGURIDAD =====================
const float V_MAX = 0.12f;    // m/s
const float W_MAX = 0.80f;    // rad/s  
const float ALPHA_CMD = 0.25f;

float v_f = 0.0f, w_f = 0.0f;

// watchdog cmd_vel
volatile uint32_t last_cmd_ms = 0;
const uint32_t CMD_TIMEOUT_MS = 500;

const long MAX_DELTA_COUNTS = 500;

// ===================== micro-ROS =====================
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

rcl_subscription_t sub_cmdvel;
geometry_msgs__msg__Twist msg_cmdvel;

rcl_publisher_t pub_pose;
geometry_msgs__msg__Pose2D msg_pose;

rcl_timer_t timer_pose;

// cmd_vel deseado (callback)
volatile float cmd_v = 0.0f;   // m/s
volatile float cmd_w = 0.0f;   // rad/s

// ===================== MACROS =====================
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// ===================== CRITICAL =====================
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ===================== ISR =====================
void IRAM_ATTR isrA_A() { if (digitalRead(pinA_A) == digitalRead(pinB_A)) cont_A++; else cont_A--; }
void IRAM_ATTR isrB_A() { if (digitalRead(pinA_A) != digitalRead(pinB_A)) cont_A++; else cont_A--; }
void IRAM_ATTR isrA_B() { if (digitalRead(pinA_B) == digitalRead(pinB_B)) cont_B--; else cont_B++; }
void IRAM_ATTR isrB_B() { if (digitalRead(pinA_B) != digitalRead(pinB_B)) cont_B--; else cont_B++; }

// ===================== UTILIDADES =====================
float normalizar_angulo(float a) {
  while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}
float mps_to_countsps(float v_mps) {
  return (v_mps / C_RUEDA) * CUENTAS_VUELTA; // (m/s) -> (cuentas/s)
}

void error_loop() {
  // Para motores por seguridad
  ledcWrite(CH_A1, 0); ledcWrite(CH_A2, 0);
  ledcWrite(CH_B1, 0); ledcWrite(CH_B2, 0);
  while (1) { delay(100); }
}

// ===================== PID =====================
int PID_A(float ref, float vel) {
  float e  = (ref - vel);
  float de = e - e_prev_A;

  float u_unsat = Kp_A * e + Ki_A * s_A * Ts + Kd_A * (de / Ts);
  float u_sat   = constrain(u_unsat, -1023.0f, 1023.0f);

  if (fabsf(u_sat) < 1023.0f) s_A += e;
  u_unsat = Kp_A * e + Ki_A * s_A * Ts + Kd_A * (de / Ts);

  e_prev_A = e;
  return (int)constrain(u_unsat, -1023.0f, 1023.0f);
}

int PID_B(float ref, float vel) {
  float e  = (ref - vel);
  float de = e - e_prev_B;

  float u_unsat = Kp_B * e + Ki_B * s_B * Ts + Kd_B * (de / Ts);
  float u_sat   = constrain(u_unsat, -1023.0f, 1023.0f);

  if (fabsf(u_sat) < 1023.0f) s_B += e;
  u_unsat = Kp_B * e + Ki_B * s_B * Ts + Kd_B * (de / Ts);

  e_prev_B = e;
  return (int)constrain(u_unsat, -1023.0f, 1023.0f);
}

void resetPID() {
  s_A = s_B = 0;
  e_prev_A = e_prev_B = 0;
}

// ===================== MOTORES =====================
void setMotorA(int u) {
  u = constrain(u, -1023, 1023);
  if (u > 0) { ledcWrite(CH_A1, u); ledcWrite(CH_A2, 0); }
  else if (u < 0) { ledcWrite(CH_A1, 0); ledcWrite(CH_A2, -u); }
  else { ledcWrite(CH_A1, 0); ledcWrite(CH_A2, 0); }
}
void setMotorB(int u) {
  u = constrain(u, -1023, 1023);
  if (u > 0) { ledcWrite(CH_B1, u); ledcWrite(CH_B2, 0); }
  else if (u < 0) { ledcWrite(CH_B1, 0); ledcWrite(CH_B2, -u); }
  else { ledcWrite(CH_B1, 0); ledcWrite(CH_B2, 0); }
}
void pararMotores() {
  ledcWrite(CH_A1, 0); ledcWrite(CH_A2, 0);
  ledcWrite(CH_B1, 0); ledcWrite(CH_B2, 0);
}

// ===================== micro-ROS callbacks =====================
void cmdvel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * m = (const geometry_msgs__msg__Twist *)msgin;
  cmd_v = m->linear.x;
  cmd_w = m->angular.z;
  last_cmd_ms = millis();
}

void timer_pose_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer == NULL) return;

  msg_pose.x = x_robot;
  msg_pose.y = y_robot;
  msg_pose.theta = theta;

  RCSOFTCHECK(rcl_publish(&pub_pose, &msg_pose, NULL));
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(1500);

  // PWM
  ledcSetup(CH_A1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_A2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B2, PWM_FREQ, PWM_RES);
  ledcAttachPin(pwm1_A, CH_A1);
  ledcAttachPin(pwm2_A, CH_A2);
  ledcAttachPin(pwm1_B, CH_B1);
  ledcAttachPin(pwm2_B, CH_B2);
  pararMotores();

  // Encoders
  pinMode(pinA_A, INPUT_PULLUP); pinMode(pinB_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA_A), isrA_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB_A), isrB_A, CHANGE);

  pinMode(pinA_B, INPUT_PULLUP); pinMode(pinB_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA_B), isrA_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB_B), isrB_B, CHANGE);

  // WiFi
  Serial.println("\n[ESP32] Conectando WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    if (millis() - t0 > 15000) {
      Serial.println("\n[ESP32] ERROR WiFi (timeout)");
      return;
    }
  }
  Serial.print("\n[ESP32] WiFi OK. IP: ");
  Serial.println(WiFi.localIP());

  // micro-ROS transport
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
  delay(2000);

  // micro-ROS init
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_diffdrive", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &sub_cmdvel, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  RCCHECK(rclc_publisher_init_default(
    &pub_pose, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D),
    "/pose2d"));

  RCCHECK(rclc_timer_init_default(
    &timer_pose, &support, RCL_MS_TO_NS(100),
    timer_pose_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmdvel, &msg_cmdvel, &cmdvel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_pose));

  // init control
  lastTime = millis();
  last_cmd_ms = millis();

  // leer contadores iniciales de forma atómica
  portENTER_CRITICAL(&mux);
  last_cont_A = cont_A;
  last_cont_B = cont_B;
  portEXIT_CRITICAL(&mux);

  resetPID();

  Serial.println("[ESP32] micro-ROS listo ✅");
}

// ===================== LOOP =====================
void loop() {
  // 1) micro-ROS tick
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // 2) Control cada 100 ms
  unsigned long now = millis();
  if (now - lastTime < CTRL_MS) return;
  lastTime += CTRL_MS;

  // ----- WATCHDOG -----
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    cmd_v = 0.0f;
    cmd_w = 0.0f;
  }

  // ----- LIMITAR y FILTRAR cmd_vel -----
  float v = cmd_v;
  float w = cmd_w;

  v = constrain(v, -V_MAX, V_MAX);
  w = constrain(w, -W_MAX, W_MAX);

  v_f = v_f + ALPHA_CMD * (v - v_f);
  w_f = w_f + ALPHA_CMD * (w - w_f);

  v = v_f;
  w = w_f;

  // ----- Leer encoders ATÓMICO -----
  long contA, contB;
  portENTER_CRITICAL(&mux);
  contA = cont_A;
  contB = cont_B;
  portEXIT_CRITICAL(&mux);

  long delta_A = contA - last_cont_A;
  long delta_B = contB - last_cont_B;
  last_cont_A = contA;
  last_cont_B = contB;

  // Anti-picos
  if (labs(delta_A) > MAX_DELTA_COUNTS || labs(delta_B) > MAX_DELTA_COUNTS) {
    pararMotores();
    return;
  }

  float vel_A = (float)delta_A / Ts; // cuentas/s
  float vel_B = (float)delta_B / Ts; // cuentas/s

  // ----- Odometría -----
  float dsA = ((float)delta_A) * METROS_POR_CUENTA;
  float dsB = ((float)delta_B) * METROS_POR_CUENTA;
  if (INVERT_B_ODOM) dsB = -dsB;

  float ds = 0.5f * (dsA + dsB);
  float dtheta = (dsB - dsA) / L_RUEDAS;

  float th_mid = theta + 0.5f * dtheta;
  x_robot += ds * cosf(th_mid);
  y_robot += ds * sinf(th_mid);
  theta = normalizar_angulo(theta + dtheta);

  // ----- cmd_vel -> ruedas -----
  float v_r = v + w * (L_RUEDAS * 0.5f);
  float v_l = v - w * (L_RUEDAS * 0.5f);

  float refA = mps_to_countsps(v_l);
  float refB = mps_to_countsps(v_r);

  // ----- PID -----
  int uA = PID_A(refA, vel_A);
  int uB = PID_B(refB, vel_B);

  setMotorA(uA);
  setMotorB(uB);
}