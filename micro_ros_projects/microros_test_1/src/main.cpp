#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_subscription_t subscriber;
std_msgs__msg__String sub_msg;

rcl_publisher_t publisher_string;
std_msgs__msg__String pub_msg_string;

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

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

char reenvio_buffer[256];

void subscription_callback(const void *msgin) {
  const std_msgs__msg__String *msg_in = (const std_msgs__msg__String *)msgin;

  // Copiamos el mensaje entrante al buffer estático
  size_t len = msg_in->data.size;
  if (len > 255) {
    len = 255;
  }
  memcpy(reenvio_buffer, msg_in->data.data, len);
  reenvio_buffer[len] = '\0'; // asegurar null-terminated

  // Asignar el buffer al mensaje de publicación
  pub_msg_string.data.data = reenvio_buffer;
  pub_msg_string.data.size = len;
  pub_msg_string.data.capacity = len + 1;

  // Publicar el mensaje
  rcl_ret_t ret = rcl_publish(&publisher_string, &pub_msg_string, NULL);
  if (ret != RCL_RET_OK) {
    Serial.println("Error al publicar en reenvio_chatter");
  } else {
    Serial.println("Mensaje reenviado");
  }
}


void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Publisher Int32
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  // Subscriber String
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "chatter"));

  // Publisher String
  RCCHECK(rclc_publisher_init_default(
    &publisher_string,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "reenvio_chatter"));
    
  // Timer
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Inicializar mensajes
  std_msgs__msg__String__init(&sub_msg);
  std_msgs__msg__String__init(&pub_msg_string);
  
  // Reservar espacio para el buffer de recepción
  sub_msg.data.data = (char *)allocator.allocate(STRING_BUFFER_SIZE, allocator.state);
  sub_msg.data.size = 0;
  sub_msg.data.capacity = STRING_BUFFER_SIZE;

  msg.data = 0;

  // Executor con 2 handles (1 timer + 1 subscription)
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}