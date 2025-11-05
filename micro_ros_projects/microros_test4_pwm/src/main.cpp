#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#define LED_PIN_R 4
#define LED_PIN_L 23
#define LED_PIN_LPWM 5
#define LED_PIN_RPWM 21

#define PWM_R_CHANNEL 0
#define PWM_L_CHANNEL 1
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

int R_pwm_value = 0;
int L_pwm_value = 0;

rcl_subscription_t subscriber_led;
rcl_subscription_t subscriber_L_motor;
rcl_subscription_t subscriber_R_motor;
std_msgs__msg__String led_pwr;
std_msgs__msg__String motor_msg_R;
std_msgs__msg__String motor_msg_L;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define STRING_BUFFER_SIZE 100
#define BLINK_INTERVAL 500

// Variables de estado globales
enum LedMode {
  MODE_OFF,
  MODE_RIGHT,
  MODE_LEFT,
  MODE_STOP
};

LedMode current_mode = MODE_OFF;
unsigned long last_blink_time = 0;
bool blink_state = false;
bool Rmotor_state = false;
bool Lmotor_state = false;

void error_loop() {
  while(1) {
    delay(100);
  }
}

void led_callback(const void *msgin) {
  const std_msgs__msg__String *msg_in = (const std_msgs__msg__String *)msgin;

  char msg_buffer[STRING_BUFFER_SIZE];
  size_t len = msg_in->data.size;
  
  if (len > STRING_BUFFER_SIZE - 1) {
    len = STRING_BUFFER_SIZE - 1;
  }
  
  memcpy(msg_buffer, msg_in->data.data, len);
  msg_buffer[len] = '\0';

  if (strcmp(msg_buffer, "R") == 0) {
    current_mode = MODE_RIGHT;
  } 
  else if (strcmp(msg_buffer, "L") == 0) {
    current_mode = MODE_LEFT;
  }
  else if (strcmp(msg_buffer, "F") == 0) {
    current_mode = MODE_OFF;
    digitalWrite(LED_PIN_R, LOW);
    digitalWrite(LED_PIN_L, LOW);
  } 
  else if (strcmp(msg_buffer, "S") == 0) {
    current_mode = MODE_STOP;
  }
}

void motor_L_callback(const void *msgin) {
  const std_msgs__msg__String *msg_in = (const std_msgs__msg__String *)msgin;

  char msg_buffer[STRING_BUFFER_SIZE];
  size_t len = msg_in->data.size;
  
  if (len > STRING_BUFFER_SIZE - 1) {
    len = STRING_BUFFER_SIZE - 1;
  }
  
  memcpy(msg_buffer, msg_in->data.data, len);
  msg_buffer[len] = '\0';
  
  // Convertir el mensaje a PWM
  int new_pwm = atoi(msg_buffer);
  
  // Si el motor derecho está activo, apagarlo primero
  if (Rmotor_state) {
    R_pwm_value = 0;
    ledcWrite(PWM_R_CHANNEL, 0);
    Rmotor_state = false;
    delay(50); // Pequeña pausa para asegurar que el motor se detenga
  }
  
  // Activar motor izquierdo con el nuevo PWM
  L_pwm_value = new_pwm;
  ledcWrite(PWM_L_CHANNEL, L_pwm_value);
  Lmotor_state = (new_pwm > 0); // Solo marcar como activo si PWM > 0
}

void motor_R_callback(const void *msgin) {
  const std_msgs__msg__String *msg_in = (const std_msgs__msg__String *)msgin;

  char msg_buffer[STRING_BUFFER_SIZE];
  size_t len = msg_in->data.size;
  
  if (len > STRING_BUFFER_SIZE - 1) {
    len = STRING_BUFFER_SIZE - 1;
  }
  
  memcpy(msg_buffer, msg_in->data.data, len);
  msg_buffer[len] = '\0';
  
  // Convertir el mensaje a PWM
  int new_pwm = atoi(msg_buffer);
  
  // Si el motor izquierdo está activo, apagarlo primero
  if (Lmotor_state) {
    L_pwm_value = 0;
    ledcWrite(PWM_L_CHANNEL, 0);
    Lmotor_state = false;
    delay(50); // Pequeña pausa para asegurar que el motor se detenga
  }
  
  // Activar motor derecho con el nuevo PWM
  R_pwm_value = new_pwm;
  ledcWrite(PWM_R_CHANNEL, R_pwm_value);
  Rmotor_state = (new_pwm > 0); // Solo marcar como activo si PWM > 0
}

void handle_led_blinking() {
  unsigned long current_time = millis();
  
  if (current_time - last_blink_time >= BLINK_INTERVAL) {
    last_blink_time = current_time;
    blink_state = !blink_state;
    
    switch (current_mode) {
      case MODE_RIGHT:
        digitalWrite(LED_PIN_L, LOW);
        digitalWrite(LED_PIN_R, blink_state ? HIGH : LOW);
        break;
        
      case MODE_LEFT:
        digitalWrite(LED_PIN_R, LOW);
        digitalWrite(LED_PIN_L, blink_state ? HIGH : LOW);
        break;
        
      case MODE_STOP:
        digitalWrite(LED_PIN_R, blink_state ? HIGH : LOW);
        digitalWrite(LED_PIN_L, blink_state ? HIGH : LOW);
        break;
        
      case MODE_OFF:
        digitalWrite(LED_PIN_R, LOW);
        digitalWrite(LED_PIN_L, LOW);
        break;
    }
  }
}

void setup() {
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(LED_PIN_L, OUTPUT);

  pinMode(LED_PIN_RPWM, OUTPUT);
  pinMode(LED_PIN_LPWM, OUTPUT);

  ledcSetup(PWM_R_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_L_CHANNEL, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(LED_PIN_RPWM, PWM_R_CHANNEL);
  ledcAttachPin(LED_PIN_LPWM, PWM_L_CHANNEL);

  digitalWrite(LED_PIN_R, LOW);
  digitalWrite(LED_PIN_L, LOW);
  
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_led_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_led,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "led_power"));
  
  RCCHECK(rclc_subscription_init_default(
    &subscriber_L_motor,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "L_motor_pwm"));
  
  RCCHECK(rclc_subscription_init_default(
    &subscriber_R_motor,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "R_motor_pwm"));

  std_msgs__msg__String__init(&led_pwr);
  std_msgs__msg__String__init(&motor_msg_R);
  std_msgs__msg__String__init(&motor_msg_L);
  
  led_pwr.data.data = (char *)allocator.allocate(STRING_BUFFER_SIZE, allocator.state);
  led_pwr.data.size = 0;
  led_pwr.data.capacity = STRING_BUFFER_SIZE;
  
  motor_msg_R.data.data = (char *)allocator.allocate(STRING_BUFFER_SIZE, allocator.state);
  motor_msg_R.data.size = 0;
  motor_msg_R.data.capacity = STRING_BUFFER_SIZE;

  motor_msg_L.data.data = (char *)allocator.allocate(STRING_BUFFER_SIZE, allocator.state);
  motor_msg_L.data.size = 0;
  motor_msg_L.data.capacity = STRING_BUFFER_SIZE;

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_led, &led_pwr, &led_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_L_motor, &motor_msg_L, &motor_L_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_R_motor, &motor_msg_R, &motor_R_callback, ON_NEW_DATA));
}

void loop() {
  handle_led_blinking();
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}