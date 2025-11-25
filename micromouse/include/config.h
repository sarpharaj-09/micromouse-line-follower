#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration (optional for remote control)
const char* SSID = "YourWiFiSSID";
const char* PASSWORD = "YourWiFiPassword";

// Motor Configuration
struct MotorConfig {
    int in1_pin;
    int in2_pin;
    int pwm_channel;
    int max_speed;
};

// Sensor Configuration
struct SensorConfig {
    int pin;
    int index;
    int weight;
    bool inverted;
};

// PID Configuration
struct PIDConfig {
    float kp;
    float ki;
    float kd;
    float max_integral;
};

#endif