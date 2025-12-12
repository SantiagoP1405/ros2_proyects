#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

#include <Wire.h>
#include <WirePacker.h>

// Configuración I2C
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_SLAVE_ADDR 0x04

// Configuración micro-ROS
rcl_subscription_t subscriber;
std_msgs__msg__String msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Variables globales
uint16_t value_to_send = 0;
bool new_data_available = false;

// Macro para manejo de errores
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    delay(100);
  }
}

// Función para enviar datos por I2C
void sendI2CData(uint16_t value) {
    // Iniciar transmisión
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    
    // Separar en dos bytes
    byte highByte = (byte)(value >> 8);      // Bits 15-8
    byte lowByte = (byte)(value & 0xFF);     // Bits 7-0
    
    // Enviar bytes
    Wire.write(highByte);
    Wire.write(lowByte);
    
    // Finalizar transmisión
    int result = Wire.endTransmission();
    
}

// Callback para cuando se recibe un mensaje
void subscription_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    // Intentar convertir el string a uint16_t
    char* endptr;
    unsigned long temp = strtoul(msg->data.data, &endptr, 10);
    
    // Verificar si la conversión fue exitosa
    if (*endptr == '\0' && temp <= 65535) {
        value_to_send = (uint16_t)temp;
        new_data_available = true;
    } 
}

void setup() {
    
    // Inicializar Serial
    Serial.begin(115200);
    delay(2000);
    
    // Inicializar I2C como Master
    Wire.begin(SDA_PIN, SCL_PIN);
    
    // Configurar transporte micro-ROS (ajustar según tu configuración)
    set_microros_serial_transports(Serial);
    
    delay(2000);
    
    // Inicializar micro-ROS
    allocator = rcl_get_default_allocator();
    
    // Crear init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    // Crear nodo
    RCCHECK(rclc_node_init_default(&node, "i2c_master_node", "", &support));
    
    // Crear suscriptor
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "master_info"
    ));
    
    // Inicializar mensaje
    static char string_buffer[256];
    msg.data.data = string_buffer;
    msg.data.size = 0;
    msg.data.capacity = sizeof(string_buffer);
    
    // Crear executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
    // Procesar callbacks de micro-ROS
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    
    // Si hay nuevos datos disponibles, enviarlos por I2C
    if (new_data_available) {
        sendI2CData(value_to_send);
        new_data_available = false;
    }
    
    delay(10);
}