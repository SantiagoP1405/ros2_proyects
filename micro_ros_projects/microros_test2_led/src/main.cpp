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

#define LED_PIN 4

rcl_subscription_t subscriber;
std_msgs__msg__String led_pwr;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define STRING_BUFFER_SIZE 100


void error_loop() {
  while(1) {
    delay(100);
  }
}

char reenvio_buffer[256];

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

  // Comparar el mensaje y controlar el LED
  if (strcmp(msg_buffer, "ON") == 0) {
    digitalWrite(LED_PIN, HIGH);
    //Serial.println("LED encendido");
  } 
  else if (strcmp(msg_buffer, "OFF") == 0) {
    digitalWrite(LED_PIN, LOW);
    //Serial.println("LED apagado");
  }
}

// put function declarations here:
int myFunction(int, int);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
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
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}