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

rcl_subscription_t subscriber;
std_msgs__msg__String led_pwr;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define STRING_BUFFER_SIZE 100
#define BLINK_INTERVAL 500  // Intervalo de parpadeo

// Variables de estado globales
enum LedMode {
  MODE_OFF,        // F - Apagado
  MODE_RIGHT,      // R - Parpadea derecho
  MODE_LEFT,       // L - Parpadea izquierdo
  MODE_STOP        // S - Parpadean ambos
};

LedMode current_mode = MODE_OFF;
unsigned long last_blink_time = 0;
bool blink_state = false;

void error_loop() {
  while(1) {
    delay(100);
  }
}

void subscription_callback(const void *msgin) {
  const std_msgs__msg__String *msg_in = (const std_msgs__msg__String *)msgin;

  // Crear un buffer temporal para comparar el mensaje
  char msg_buffer[STRING_BUFFER_SIZE];
  size_t len = msg_in->data.size;
  
  if (len > STRING_BUFFER_SIZE - 1) {
    len = STRING_BUFFER_SIZE - 1;
  }
  
  memcpy(msg_buffer, msg_in->data.data, len);
  msg_buffer[len] = '\0'; // asegurar null-terminated

  // Actualizar el modo según el mensaje recibido
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

void handle_led_blinking() {
  unsigned long current_time = millis();
  
  // Verificar si es tiempo de cambiar el estado del parpadeo
  if (current_time - last_blink_time >= BLINK_INTERVAL) {
    last_blink_time = current_time;
    blink_state = !blink_state;
    
    // Aplicar el estado según el modo actual
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
  digitalWrite(LED_PIN_R, LOW);
  digitalWrite(LED_PIN_L, LOW);
  
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_led_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "led_power"));

  std_msgs__msg__String__init(&led_pwr);

  led_pwr.data.data = (char *)allocator.allocate(STRING_BUFFER_SIZE, allocator.state);
  led_pwr.data.size = 0;
  led_pwr.data.capacity = STRING_BUFFER_SIZE;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &led_pwr, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  // Manejar el parpadeo de los LEDs
  handle_led_blinking();
  
  // Procesar mensajes de micro-ROS
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);  // Pequeña pausa para no saturar el CPU
}