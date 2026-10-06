/*
  BasicControl - Simple example for RobStride motor control

  This example shows basic motor control in MIT motion mode.
  The motor moves smoothly between two positions.

  Hardware:
  - M5Stack AtomS3 (or other ESP32-S3 board)
  - RobStride motor (ID: 1)
  - CAN transceiver connected to GPIO1 (RX) and GPIO2 (TX)
*/

#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <RobStride.h>

// Create motor instance with ID 1
RobStrideMotor motor(1);

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("RobStride Basic Control Example");

  // Initialize CAN bus: 1Mbps, TX=GPIO2, RX=GPIO1
  ESP32Can.begin(TWAI_SPEED_1000KBPS, 2, 1);

  delay(100);

  // Initialize motor
  motor.stop();
  delay(50);
  motor.clear_fault();
  delay(50);

  // Set to motion control mode (MIT mode)
  motor.set_run_mode(RS_MODE_MOTION);
  delay(20);

  // Enable motor
  motor.enable();
  delay(50);

  Serial.println("Motor initialized. Starting control...");
}

void loop()
{
  static unsigned long last_time = 0;
  unsigned long now = millis();

  // Switch position every 2 seconds
  if (now - last_time > 2000) {
    last_time = now;
  }

  // Calculate smooth position using sine wave
  float elapsed = (now - last_time) / 1000.0f; // 0.0 to 2.0 seconds
  float target_position = sin(elapsed * 3.14159f) * 1.5f; // -1.5 to +1.5 radians

  // Control parameters
  float kp = 10.0f;  // Position stiffness
  float kd = 0.5f;   // Damping

  // Send motion command: position, velocity, kp, kd, feedforward_torque
  motor.send_motion_command(target_position, 0, kp, kd, 0);

  delay(10);
}
