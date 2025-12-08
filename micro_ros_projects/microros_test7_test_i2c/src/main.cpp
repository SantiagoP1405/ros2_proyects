#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <Wire.h>
#include <WireSlaveRequest.h>

// Configuración I2C
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_SLAVE_ADDR 0x04
#define I2C_SLAVE_ADDR_2 0x05
#define MAX_SLAVE_RESPONSE_LENGTH 32

// Configuración micro-ROS - Dos publicadores
rcl_publisher_t publisher_slave1;
rcl_publisher_t publisher_slave2;
std_msgs__msg__String msg_slave1;
std_msgs__msg__String msg_slave2;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Buffers para mensajes
char msg_buffer_slave1[128];
char msg_buffer_slave2[128];

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100);
  }
}

bool readI2CSlave(uint8_t slaveAddr, char* buffer, size_t bufferSize) {
  WireSlaveRequest slaveReq(Wire, slaveAddr, MAX_SLAVE_RESPONSE_LENGTH);
  slaveReq.setRetryDelay(5);
  
  bool success = slaveReq.request();
  
  if (success) {
    size_t index = 0;
    memset(buffer, 0, bufferSize);
    
    while (slaveReq.available() > 1 && index < bufferSize - 10) {
      char c = slaveReq.read();
      buffer[index++] = c;
    }
    
    if (slaveReq.available()) {
      int x = slaveReq.read();
      sprintf(&buffer[index], "%d", x);
    }
    return true;
  } else {
    strncpy(buffer, slaveReq.lastStatusToString().c_str(), bufferSize - 1);
    return false;
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    char temp_data[100];
    bool success;
    
    // ========== ESCLAVO 1 ==========
    success = readI2CSlave(I2C_SLAVE_ADDR, temp_data, sizeof(temp_data));
    
    // Formatear mensaje para esclavo 1
    snprintf(msg_buffer_slave1, sizeof(msg_buffer_slave1), 
      "{\"timestamp\":%lu,\"addr\":\"0x%02X\",\"success\":%s,\"data\":\"%s\"}",
      millis(), I2C_SLAVE_ADDR, success ? "true" : "false", temp_data);
    
    // Publicar datos del esclavo 1
    msg_slave1.data.data = msg_buffer_slave1;
    msg_slave1.data.size = strlen(msg_buffer_slave1);
    msg_slave1.data.capacity = sizeof(msg_buffer_slave1);
    RCSOFTCHECK(rcl_publish(&publisher_slave1, &msg_slave1, NULL));
    
    // ========== ESCLAVO 2 ==========
    success = readI2CSlave(I2C_SLAVE_ADDR_2, temp_data, sizeof(temp_data));
    
    // Formatear mensaje para esclavo 2
    snprintf(msg_buffer_slave2, sizeof(msg_buffer_slave2), 
      "{\"timestamp\":%lu,\"addr\":\"0x%02X\",\"success\":%s,\"data\":\"%s\"}",
      millis(), I2C_SLAVE_ADDR_2, success ? "true" : "false", temp_data);
    
    // Publicar datos del esclavo 2
    msg_slave2.data.data = msg_buffer_slave2;
    msg_slave2.data.size = strlen(msg_buffer_slave2);
    msg_slave2.data.capacity = sizeof(msg_buffer_slave2);
    RCSOFTCHECK(rcl_publish(&publisher_slave2, &msg_slave2, NULL));
    
    // Debug
    Serial.println("Slave 1: " + String(msg_buffer_slave1));
    Serial.println("Slave 2: " + String(msg_buffer_slave2));
  }
}

void setup() {
  Serial.begin(115200);
  
  // Inicializar I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("I2C Master initialized");
  
  // Configurar micro-ROS
  set_microros_serial_transports(Serial);
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "i2c_master_node", "", &support));
  
  // Crear dos publicadores separados
  RCCHECK(rclc_publisher_init_default(
    &publisher_slave1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "i2c/slave_0x04"));
  
  RCCHECK(rclc_publisher_init_default(
    &publisher_slave2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "i2c/slave_0x05"));
  
  // Crear timer
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  // Crear executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  // Inicializar mensajes
  msg_slave1.data.data = msg_buffer_slave1;
  msg_slave1.data.capacity = sizeof(msg_buffer_slave1);
  msg_slave1.data.size = 0;
  
  msg_slave2.data.data = msg_buffer_slave2;
  msg_slave2.data.capacity = sizeof(msg_buffer_slave2);
  msg_slave2.data.size = 0;
  
  Serial.println("micro-ROS initialized with 2 publishers");
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
