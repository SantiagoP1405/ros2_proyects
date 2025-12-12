#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>

#include <Wire.h>

// Configuración I2C
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_SLAVE_ADDR 0x04

// Identificadores para los datos
#define ID_DIRECTION 0x01
#define ID_THROTTLE 0x02

// Configuración micro-ROS
rcl_subscription_t subscriber_direction;
rcl_subscription_t subscriber_throttle;
std_msgs__msg__String direction_msg;
std_msgs__msg__String throttle_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Variables globales
uint16_t direction_value = 0;
uint16_t throttle_value = 0;
bool new_direction_data = false;
bool new_throttle_data = false;

#define STRING_BUFFER_SIZE 100

// Macro para manejo de errores
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
    while(1) {
        delay(100);
    }
}

// Función para enviar datos por I2C con identificador
void sendI2CDataWithID(uint16_t value, uint8_t id) {
    // Iniciar transmisión
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    
    // Primero enviar el identificador
    Wire.write(id);
    
    // Luego enviar los dos bytes del valor
    byte highByte = (byte)(value >> 8);      // Bits 15-8
    byte lowByte = (byte)(value & 0xFF);     // Bits 7-0
    
    Wire.write(highByte);
    Wire.write(lowByte);
    
    // Finalizar transmisión
    Wire.endTransmission();
    
}

// Callback para direction_servo
void direction_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    
    // Buffer para el mensaje
    char msg_buffer[STRING_BUFFER_SIZE];
    size_t len = msg->data.size;
    
    if (len > STRING_BUFFER_SIZE - 1) {
        len = STRING_BUFFER_SIZE - 1;
    }
    
    memcpy(msg_buffer, msg->data.data, len);
    msg_buffer[len] = '\0';
    
    // Convertir string a uint16_t
    char* endptr;
    unsigned long temp = strtoul(msg_buffer, &endptr, 10);
    
    if (*endptr == '\0' && temp <= 65535) {
        direction_value = (uint16_t)temp;
        new_direction_data = true;
    }
}

// Callback para throttle_motor
void throttle_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    
    // Buffer para el mensaje
    char msg_buffer[STRING_BUFFER_SIZE];
    size_t len = msg->data.size;
    
    if (len > STRING_BUFFER_SIZE - 1) {
        len = STRING_BUFFER_SIZE - 1;
    }
    
    memcpy(msg_buffer, msg->data.data, len);
    msg_buffer[len] = '\0';
    
    // Convertir string a uint16_t
    char* endptr;
    unsigned long temp = strtoul(msg_buffer, &endptr, 10);
    
    if (*endptr == '\0' && temp <= 65535) {
        throttle_value = (uint16_t)temp;
        new_throttle_data = true;
    }
}

void setup() {
    // Inicializar Serial
    Serial.begin(115200);
    delay(2000);
    

    // Inicializar I2C como Master
    Wire.begin(SDA_PIN, SCL_PIN);
    
    // Configurar transporte micro-ROS
    set_microros_serial_transports(Serial);
    delay(2000);
    
    // Inicializar micro-ROS
    allocator = rcl_get_default_allocator();
    
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "i2c_master_node", "", &support));
    
    // Crear suscriptor para direction_servo
    RCCHECK(rclc_subscription_init_default(
        &subscriber_direction,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "direction_servo"
    ));
    
    // Crear suscriptor para throttle_motor
    RCCHECK(rclc_subscription_init_default(
        &subscriber_throttle,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "throttle_motor"
    ));
    
    // Inicializar mensajes
    std_msgs__msg__String__init(&direction_msg);
    std_msgs__msg__String__init(&throttle_msg);
    
    direction_msg.data.data = (char *)allocator.allocate(STRING_BUFFER_SIZE, allocator.state);
    direction_msg.data.size = 0;
    direction_msg.data.capacity = STRING_BUFFER_SIZE;
    
    throttle_msg.data.data = (char *)allocator.allocate(STRING_BUFFER_SIZE, allocator.state);
    throttle_msg.data.size = 0;
    throttle_msg.data.capacity = STRING_BUFFER_SIZE;
    
    // Crear executor con 2 suscripciones
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_direction, 
            &direction_msg, &direction_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_throttle, 
            &throttle_msg, &throttle_callback, ON_NEW_DATA));
}

void loop() {
    // Procesar callbacks de micro-ROS
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
    
    // Enviar datos de dirección si hay nuevos
    if (new_direction_data) {
        sendI2CDataWithID(direction_value, ID_DIRECTION);
        new_direction_data = false;
    }
    
    // Enviar datos de throttle si hay nuevos
    if (new_throttle_data) {
        sendI2CDataWithID(throttle_value, ID_THROTTLE);
        new_throttle_data = false;
    }
    
    delay(10);
}