#include "RobStride.h"

RobStrideMotor::RobStrideMotor(uint8_t id, uint8_t master) : motor_id(id), master_id(master) {}

int RobStrideMotor::float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float RobStrideMotor::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}

void RobStrideMotor::enable()
{
    CanFrame frame;
    frame.identifier = ((uint32_t)0x03 << 24) | ((uint32_t)master_id << 8) | motor_id;
    frame.extd = true;
    frame.data_length_code = 8;
    memset(frame.data, 0, 8);
    ESP32Can.writeFrame(&frame);
}

void RobStrideMotor::stop()
{
    CanFrame frame;
    frame.identifier = ((uint32_t)0x04 << 24) | ((uint32_t)master_id << 8) | motor_id;
    frame.extd = true;
    frame.data_length_code = 8;
    memset(frame.data, 0, 8);
    ESP32Can.writeFrame(&frame);
}

void RobStrideMotor::clear_fault()
{
    CanFrame frame;
    frame.identifier = ((uint32_t)0x11 << 24) | ((uint32_t)master_id << 8) | motor_id;
    frame.extd = true;
    frame.data_length_code = 8;
    memset(frame.data, 0, 8);
    frame.data[0] = 1; // Clear Fault Command
    ESP32Can.writeFrame(&frame);
}

void RobStrideMotor::set_zero_position()
{
    CanFrame frame;
    frame.identifier = ((uint32_t)0x06 << 24) | ((uint32_t)master_id << 8) | motor_id;
    frame.extd = true;
    frame.data_length_code = 0;
    ESP32Can.writeFrame(&frame);
}

void RobStrideMotor::write_parameter(uint16_t index, float value)
{
    CanFrame frame;
    frame.identifier = ((uint32_t)0x12 << 24) | ((uint32_t)master_id << 8) | motor_id;
    frame.extd = true;
    frame.data_length_code = 8;

    frame.data[0] = index & 0xFF;
    frame.data[1] = (index >> 8) & 0xFF;
    frame.data[2] = 0;
    frame.data[3] = 0;
    memcpy(&frame.data[4], &value, 4);

    ESP32Can.writeFrame(&frame);
}

void RobStrideMotor::write_parameter(uint16_t index, uint8_t value)
{
    CanFrame frame;
    frame.identifier = ((uint32_t)0x12 << 24) | ((uint32_t)master_id << 8) | motor_id;
    frame.extd = true;
    frame.data_length_code = 8;

    frame.data[0] = index & 0xFF;
    frame.data[1] = (index >> 8) & 0xFF;
    frame.data[2] = 0;
    frame.data[3] = 0;
    frame.data[4] = value;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;

    ESP32Can.writeFrame(&frame);
}

void RobStrideMotor::send_motion_command(float position, float velocity, float kp, float kd, float torque)
{
    CanFrame frame;
    uint16_t torque_int = float_to_uint(torque, RS_T_MIN, RS_T_MAX, 16);

    frame.identifier = ((uint32_t)0x01 << 24) | ((uint32_t)torque_int << 8) | motor_id;
    frame.extd = true;
    frame.data_length_code = 8;

    uint16_t p_int = float_to_uint(position, RS_P_MIN, RS_P_MAX, 16);
    uint16_t v_int = float_to_uint(velocity, RS_V_MIN, RS_V_MAX, 16);
    uint16_t kp_int = float_to_uint(kp, RS_KP_MIN, RS_KP_MAX, 16);
    uint16_t kd_int = float_to_uint(kd, RS_KD_MIN, RS_KD_MAX, 16);

    frame.data[0] = p_int >> 8;
    frame.data[1] = p_int & 0xFF;
    frame.data[2] = v_int >> 8;
    frame.data[3] = v_int & 0xFF;
    frame.data[4] = kp_int >> 8;
    frame.data[5] = kp_int & 0xFF;
    frame.data[6] = kd_int >> 8;
    frame.data[7] = kd_int & 0xFF;

    ESP32Can.writeFrame(&frame);
}

void RobStrideMotor::set_current_reference(float current)
{
    write_parameter(RS_IDX_IQ_REF, current);
}

void RobStrideMotor::set_speed_reference(float speed)
{
    write_parameter(RS_IDX_SPD_REF, speed);
}

void RobStrideMotor::set_position_reference(float position)
{
    write_parameter(RS_IDX_LOC_REF, position);
}

void RobStrideMotor::set_run_mode(uint8_t mode)
{
    write_parameter(RS_IDX_RUN_MODE, mode);
}

void RobStrideMotor::set_limit_current(float current)
{
    write_parameter(RS_IDX_LIMIT_CUR, current);
}

void RobStrideMotor::set_limit_speed(float speed)
{
    write_parameter(RS_IDX_LIMIT_SPD, speed);
}

void RobStrideMotor::set_limit_accel_rad(float accel)
{
    write_parameter(RS_IDX_ACC_RAD, accel);
}

void RobStrideMotor::set_pp_limits(float velocity, float accel)
{
    write_parameter(RS_IDX_VEL_MAX, velocity);
    write_parameter(RS_IDX_ACC_SET, accel);
}

RobStrideStatus RobStrideMotor::read_status(uint32_t timeout_ms)
{
    RobStrideStatus status = {0};
    status.valid = false;

    CanFrame frame;
    uint32_t start_time = millis();

    // Keep checking until timeout or match found
    // In non-blocking mode (timeout_ms==0), check all frames in buffer
    while (true)
    {
        if (ESP32Can.readFrame(&frame, 0))
        {
            // Check if this is an extended frame
            if (!frame.extd)
                continue;

            // Extract communication type (bit28~24)
            uint8_t comm_type = (frame.identifier >> 24) & 0x1F;

            // Check for feedback frame (type 2)
            if (comm_type != 0x02)
                continue;

            // Extract motor ID (bit15~8)
            uint8_t rx_motor_id = (frame.identifier >> 8) & 0xFF;
            if (rx_motor_id != motor_id)
                continue;

            // Extract mode (bit23~22) and fault (bit21~16)
            status.mode = (frame.identifier >> 22) & 0x03;
            status.fault = (frame.identifier >> 16) & 0x3F;

            // Parse data bytes (high byte first)
            uint16_t pos_int = ((uint16_t)frame.data[0] << 8) | frame.data[1];
            uint16_t vel_int = ((uint16_t)frame.data[2] << 8) | frame.data[3];
            uint16_t torque_int = ((uint16_t)frame.data[4] << 8) | frame.data[5];
            uint16_t temp_int = ((uint16_t)frame.data[6] << 8) | frame.data[7];

            // Convert to float values
            status.position = uint_to_float(pos_int, RS_P_MIN, RS_P_MAX, 16);
            status.velocity = uint_to_float(vel_int, RS_V_MIN, RS_V_MAX, 16);
            status.torque = uint_to_float(torque_int, RS_T_MIN, RS_T_MAX, 16);
            status.temperature = (float)temp_int / 10.0f;

            status.valid = true;
            return status;
        }
        else
        {
            // No frame available in buffer
            if (timeout_ms == 0) {
                // Non-blocking mode: buffer is empty, exit
                break;
            }

            // Blocking mode: check if timeout reached
            if ((millis() - start_time) >= timeout_ms) {
                break;
            }

            // Wait a bit before retrying
            delay(1);
        }
    }

    return status;
}

void RobStrideMotor::enable_auto_report(uint16_t interval_ms)
{
    CanFrame frame;
    frame.identifier = ((uint32_t)0x18 << 24) | ((uint32_t)master_id << 8) | motor_id;
    frame.extd = true;
    frame.data_length_code = 8;

    // Fixed bytes (protocol requirement)
    frame.data[0] = 0x01;
    frame.data[1] = 0x02;
    frame.data[2] = 0x03;
    frame.data[3] = 0x04;
    frame.data[4] = 0x05;
    frame.data[5] = 0x06;

    // Enable auto report (Byte6: 0x01=ON)
    frame.data[6] = 0x01;

    // Reserved/unused
    frame.data[7] = 0x00;

    ESP32Can.writeFrame(&frame);
}

void RobStrideMotor::disable_auto_report()
{
    CanFrame frame;
    frame.identifier = ((uint32_t)0x18 << 24) | ((uint32_t)master_id << 8) | motor_id;
    frame.extd = true;
    frame.data_length_code = 8;

    // Fixed bytes (protocol requirement)
    frame.data[0] = 0x01;
    frame.data[1] = 0x02;
    frame.data[2] = 0x03;
    frame.data[3] = 0x04;
    frame.data[4] = 0x05;
    frame.data[5] = 0x06;

    // Disable auto report (Byte6: 0x00=OFF)
    frame.data[6] = 0x00;

    // Reserved/unused
    frame.data[7] = 0x00;

    ESP32Can.writeFrame(&frame);
}
