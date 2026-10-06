#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <RobStride.h>

RobStrideMotor motor1(1);
RobStrideMotor motor2(2);

void setup()
{
  Serial.begin(115200);

  // Initialize CAN bus globally once
  // GPIO1=RX, GPIO2=TX, 1Mbps
  ESP32Can.begin(TWAI_SPEED_1000KBPS, 2, 1);

  delay(1000);
  Serial.println("Starting Demo Loop...");
}

enum DemoMode {
  MODE_MOTION_CW_CCW,
  MODE_CURRENT,
  MODE_SPEED,
  MODE_POS_PP,
  MODE_POS_CSP,
  MODE_MAX
};

DemoMode current_mode = MODE_MAX; // Force init
unsigned long mode_start_time = 0;
const unsigned long RAMP_DOWN_TIME = 1000; // Last 1 second to ramp down

// Helper function to handle safe mode switching with delays (Application Layer)
void switch_mode_safely(DemoMode mode)
{
  // 1. Stop Motors
  motor1.stop();
  motor2.stop();
  delay(50);

  // 2. Clear Faults (Twice for safety)
  motor1.clear_fault();
  motor2.clear_fault();
  delay(10);
  motor1.clear_fault();
  motor2.clear_fault();
  delay(50);

  // 3. Set Parameters & Mode
  switch (mode)
  {
  case MODE_MOTION_CW_CCW: // MIT
    motor1.set_run_mode(RS_MODE_MOTION);
    motor2.set_run_mode(RS_MODE_MOTION);
    break;

  case MODE_CURRENT: // Torque
    motor1.set_run_mode(RS_MODE_CURRENT);
    motor2.set_run_mode(RS_MODE_CURRENT);
    break;

  case MODE_SPEED:
    // Set Limits first
    motor1.set_limit_current(3.0f);
    motor2.set_limit_current(3.0f);
    delay(10);
    motor1.set_limit_accel_rad(10.0f);
    motor2.set_limit_accel_rad(10.0f);
    delay(10);
    // Set Mode
    motor1.set_run_mode(RS_MODE_SPEED);
    motor2.set_run_mode(RS_MODE_SPEED);
    break;

  case MODE_POS_PP:
    // Set Limits first
    motor1.set_limit_current(3.0f);
    motor2.set_limit_current(3.0f);
    delay(10);
    motor1.set_pp_limits(5.0f, 5.0f); // Vel, Accel
    motor2.set_pp_limits(5.0f, 5.0f);
    delay(10);
    // Set Mode
    motor1.set_run_mode(RS_MODE_POS_PP);
    motor2.set_run_mode(RS_MODE_POS_PP);
    break;

  case MODE_POS_CSP:
    // Set Limits first
    motor1.set_limit_speed(5.0f);
    motor2.set_limit_speed(5.0f);
    delay(10);
    motor1.set_limit_current(3.0f);
    motor2.set_limit_current(3.0f);
    delay(10);
    // Set Mode
    motor1.set_run_mode(RS_MODE_POS_CSP);
    motor2.set_run_mode(RS_MODE_POS_CSP);
    break;

  default:
    break;
  }

  delay(20);

  // 4. Enable Motors
  motor1.enable();
  motor2.enable();
}

void loop()
{
  unsigned long now = millis();
  static float phase = 0.0;

  // Initialize first mode safely
  if (current_mode == MODE_MAX) {
     mode_start_time = now;
     current_mode = MODE_MOTION_CW_CCW;
     Serial.println("Init: Motion (CW/CCW)");
     switch_mode_safely(MODE_MOTION_CW_CCW);
  }

  unsigned long elapsed = now - mode_start_time;
  // Extend duration slightly for CW/CCW mode to allow full cycles
  unsigned long duration = (current_mode == MODE_MOTION_CW_CCW) ? 15000 : 10000;

  // --- Mode Switching Logic ---
  if (elapsed > duration) {
    mode_start_time = now;
    elapsed = 0;
    phase = 0.0;

    // Switch to next mode
    current_mode = (DemoMode)((current_mode + 1) % MODE_MAX);

    Serial.print("Switching to Mode: ");

    switch (current_mode) {
      case MODE_MOTION_CW_CCW:
        Serial.println("Motion (CW/CCW)");
        break;
      case MODE_CURRENT:
        Serial.println("Current (Torque)");
        break;
      case MODE_SPEED:
        Serial.println("Speed");
        break;
      case MODE_POS_PP:
        Serial.println("Position (PP)");
        break;
      case MODE_POS_CSP:
        Serial.println("Position (CSP)");
        break;
      default:
        break;
    }

    switch_mode_safely(current_mode);
  }

  // --- Control Logic with Ramp Down ---

  bool ramping_down = (elapsed > (duration - RAMP_DOWN_TIME));
  float ramp_factor = 1.0f;

  if (ramping_down) {
      ramp_factor = 1.0f - ((float)(elapsed - (duration - RAMP_DOWN_TIME)) / (float)RAMP_DOWN_TIME);
      if (ramp_factor < 0.0f) ramp_factor = 0.0f;
  }

  switch (current_mode) {
    case MODE_MOTION_CW_CCW: {
      // 6000ms cycle: 2s Move CW, 1s Wait, 2s Move CCW, 1s Wait
      unsigned long cycle_time = elapsed % 6000;
      float target_pos = 0.0f;
      float PI_VAL = 3.14159f;

      // Soft Start (Ramp Up) - Extended to 3 seconds
      float ramp_up_factor = 1.0f;
      if (elapsed < 3000) {
          ramp_up_factor = (float)elapsed / 3000.0f;
      }

      if (cycle_time < 2000) {
          // Moving CW (-PI to +PI)
          float progress = (float)cycle_time / 2000.0f;
          // Map 0..1 to -PI..PI via Cosine
          target_pos = -PI_VAL * cos(progress * PI_VAL);
      } else if (cycle_time < 3000) {
          // Wait CW (+PI)
          target_pos = PI_VAL;
      } else if (cycle_time < 5000) {
          // Moving CCW (+PI to -PI)
          float progress = (float)(cycle_time - 3000) / 2000.0f;
          // Map 0..1 to PI..-PI via Cosine
          target_pos = PI_VAL * cos(progress * PI_VAL);
      } else {
          // Wait CCW (-PI)
          target_pos = -PI_VAL;
      }

      // Apply ramp down (return to 0 at the very end of mode)
      if (ramping_down) target_pos *= ramp_factor;

      // Apply Ramp Up/Down to Gains as well for smooth stiffness transition
      float final_kp = 10.0f * ramp_up_factor * ramp_factor;
      float final_kd = 0.5f * ramp_up_factor * ramp_factor;

      motor1.send_motion_command(target_pos, 0, final_kp, final_kd, 0);
      motor2.send_motion_command(-target_pos, 0, final_kp, final_kd, 0);
      break;
    }

    case MODE_CURRENT: {
      float current = (sin(phase) * 0.15) * ramp_factor;
      motor1.set_current_reference(current);
      motor2.set_current_reference(-current);
      phase += 0.005;
      break;
    }

    case MODE_SPEED: {
      float speed = (sin(phase) * 3.0) * ramp_factor;
      motor1.set_speed_reference(speed);
      motor2.set_speed_reference(-speed);
      phase += 0.005;
      break;
    }

    case MODE_POS_PP: {
      if (ramping_down) {
          motor1.set_position_reference(0.0f);
          motor2.set_position_reference(0.0f);
      } else {
          // Move every 3 seconds
          unsigned long cycle_time = elapsed % 6000;
          float target = (cycle_time < 3000) ? 1.0f : -1.0f;
          motor1.set_position_reference(target);
          motor2.set_position_reference(-target);
      }
      break;
    }

    case MODE_POS_CSP: {
      float target = (sin(phase) * 1.0) * ramp_factor;
      motor1.set_position_reference(target);
      motor2.set_position_reference(-target);
      phase += 0.005;
      break;
    }

    default:
      break;
  }

  delay(10);
}
