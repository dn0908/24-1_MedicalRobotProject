#include <Adafruit_AHRS.h>
#include <Adafruit_AHRS_FusionInterface.h>
#include <Adafruit_AHRS_Madgwick.h>
#include <Adafruit_AHRS_Mahony.h>
#include <Adafruit_AHRS_NXPFusion.h>
#include <Adafruit_Sensor_Set.h>
#include <Adafruit_Simple_AHRS.h>

#include <MadgwickAHRS.h>

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
// #include <Adafruit_Madgwick.h>

Adafruit_MPU6050 mpu;
Adafruit_Madgwick filter;

const float dt = 0.0067;    // Loop time in sec (approximately 150 Hz)
unsigned long lastUpdate = 0;

float ax, ay, az;
float gx, gy, gz;
float vx = 0, vy = 0, vz = 0;
float x = 0, y = 0, z = 0;
float alpha = 0.95; // High-pass filter constant

void setup() {
  Serial.begin(115200); // Higher baud rate for faster data transmission
  Wire.begin();
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  filter.begin(150); // Sample rate (Hz)
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastUpdate >= dt * 1000) {
    lastUpdate = currentTime;

    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Extract accelerometer data
    ax = accel.acceleration.x;
    ay = accel.acceleration.y;
    az = accel.acceleration.z;

    // Extract gyro data
    gx = gyro.gyro.x;
    gy = gyro.gyro.y;
    gz = gyro.gyro.z;

    // Update the filter, converting gyroscope data to radians/s
    filter.updateIMU(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, ay, az);

    // Get orientation quaternion
    float qw, qx, qy, qz;
    filter.getQuaternion(&qw, &qx, &qy, &qz);

    // Convert quaternion to rotation matrix
    float R[3][3];
    R[0][0] = 1 - 2*qy*qy - 2*qz*qz;
    R[0][1] = 2*qx*qy - 2*qw*qz;
    R[0][2] = 2*qx*qz + 2*qw*qy;
    R[1][0] = 2*qx*qy + 2*qw*qz;
    R[1][1] = 1 - 2*qx*qx - 2*qz*qz;
    R[1][2] = 2*qy*qz - 2*qw*qx;
    R[2][0] = 2*qx*qz - 2*qw*qy;
    R[2][1] = 2*qy*qz + 2*qw*qx;
    R[2][2] = 1 - 2*qx*qx - 2*qy*qy;

    // Calculate 'tilt-compensated' accelerometer (acceleration in the Earth frame)
    float tcAccX = R[0][0] * ax + R[0][1] * ay + R[0][2] * az;
    float tcAccY = R[1][0] * ax + R[1][1] * ay + R[1][2] * az;
    float tcAccZ = R[2][0] * ax + R[2][1] * ay + R[2][2] * az;

    // Subtract gravity (assume gravity is 1g in the Z direction)
    tcAccZ -= 9.81;

    // Integrate acceleration to get velocity
    vx = alpha * (vx + tcAccX * dt);
    vy = alpha * (vy + tcAccY * dt);
    vz = alpha * (vz + tcAccZ * dt);

    // Integrate velocity to get position
    x += vx * dt;
    y += vy * dt;
    z += vz * dt;

    // Print the position
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.println(z);
  }
}