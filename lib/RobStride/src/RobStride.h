#ifndef ROBSTRIDE_H
#define ROBSTRIDE_H

#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>

// Limits
#define RS_P_MIN -12.57f
#define RS_P_MAX 12.57f
#define RS_V_MIN -50.0f
#define RS_V_MAX 50.0f
#define RS_KP_MIN 0.0f
#define RS_KP_MAX 500.0f
#define RS_KD_MIN 0.0f
#define RS_KD_MAX 5.0f
#define RS_T_MIN -6.0f
#define RS_T_MAX 6.0f

// Parameter Indices
#define RS_IDX_RUN_MODE 0x7005
#define RS_IDX_IQ_REF 0x7006
#define RS_IDX_SPD_REF 0x700A
#define RS_IDX_LIMIT_TORQUE 0x700B
#define RS_IDX_CUR_KP 0x7010
#define RS_IDX_CUR_KI 0x7011
#define RS_IDX_LOC_REF 0x7016
#define RS_IDX_LIMIT_SPD 0x7017
#define RS_IDX_LIMIT_CUR 0x7018
#define RS_IDX_ACC_RAD 0x7022
#define RS_IDX_VEL_MAX 0x7024
#define RS_IDX_ACC_SET 0x7025

// Run Modes
#define RS_MODE_MOTION 0
#define RS_MODE_POS_PP 1
#define RS_MODE_SPEED 2
#define RS_MODE_CURRENT 3
#define RS_MODE_POS_CSP 5

// Motor Status (from feedback frame - Communication Type 2)
struct RobStrideStatus
{
    float position;      // Current position (rad)
    float velocity;      // Current velocity (rad/s)
    float torque;        // Current torque (Nm)
    float temperature;   // Temperature (Celsius)
    uint8_t mode;        // Motor mode: 0=Reset, 1=Cali, 2=Motor
    uint8_t fault;       // Fault code (0 = no fault)
    bool valid;          // true if status was successfully read
};

class RobStrideMotor
{
private:
    uint8_t motor_id;
    uint8_t master_id;

    // Helper functions for value conversion
    int float_to_uint(float x, float x_min, float x_max, int bits);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);

public:
    RobStrideMotor(uint8_t id, uint8_t master = 0xFD);

    // Basic Commands
    void enable();
    void stop();
    void clear_fault();
    void set_zero_position();

    // Parameter Access
    void write_parameter(uint16_t index, float value);
    void write_parameter(uint16_t index, uint8_t value);

    // Control Commands
    void send_motion_command(float position, float velocity, float kp, float kd, float torque);
    void set_current_reference(float current);
    void set_speed_reference(float speed);
    void set_position_reference(float position);

    // Mode Configuration Helpers (Atomic - No Delays)
    void set_run_mode(uint8_t mode);
    
    // Limits Configuration Helpers (Atomic)
    void set_limit_current(float current);
    void set_limit_speed(float speed);
    void set_limit_accel_rad(float accel); // For Speed Mode
    void set_pp_limits(float velocity, float accel); // For PP Mode

    // Status Reading (Communication Type 2 feedback frame)
    RobStrideStatus read_status(uint32_t timeout_ms = 10);

    // Auto Report Mode (Communication Type 24)
    // Enables automatic status reporting at specified interval
    // Motor will continuously send feedback frames at the given interval
    void enable_auto_report(uint16_t interval_ms = 10);
    void disable_auto_report();
};

#endif // ROBSTRIDE_H
