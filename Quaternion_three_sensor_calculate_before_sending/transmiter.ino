#include "BluetoothSerial.h"
#include "I2Cdev.h"
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

// Connection MPU6050=========================
#define MPU_DEFAULT 0x68
#define MPU_ALT     0x69
MPU6050 mpu1(MPU_DEFAULT, &Wire);
MPU6050 mpu2(MPU_ALT, &Wire);
MPU6050 mpu3(MPU_DEFAULT, &Wire1);

// Bluetooth Module===========================
BluetoothSerial SerialBT;
uint8_t serverMAC[] = {0xCC, 0x7B, 0x5C, 0xF1, 0x90, 0xF6};

// Queue =====================================
struct MotionData {
  int16_t ax1, ay1, az1, gx1, gy1, gz1;
  int16_t ax2, ay2, az2, gx2, gy2, gz2;
  int16_t ax3, ay3, az3, gx3, gy3, gz3;
};
#define QUEUE_SIZE 10
MotionData queue[QUEUE_SIZE];
int front = 0, rear = 0, count = 0;
bool enqueue(MotionData d) {
  if (count < QUEUE_SIZE) {
    queue[rear] = d;
    rear = (rear + 1) % QUEUE_SIZE;
    count++;
    return true;
  }
  return false;
}
bool dequeue(MotionData &d) {
  if (count > 0) {
    d = queue[front];
    front = (front + 1) % QUEUE_SIZE;
    count--;
    return true;
  }
  return false;
}
MotionData newData;

struct DualQuaternion {
  float w1, x1, y1, z1;  // MPU1
  float w2, x2, y2, z2;  // MPU2
  float w3, x3, y3, z3;  // MPU3
};
DualQuaternion qData;
float q1[4] = {1, 0, 0, 0};
float q2[4] = {1, 0, 0, 0};
float q3[4] = {1, 0, 0, 0};

unsigned long lastUpdate1 = 0;
unsigned long lastUpdate2 = 0;
unsigned long lastUpdate3 = 0;

void convertToQuaternion(int16_t gx, int16_t gy, int16_t gz, float q[4], unsigned long &lastUpdate) {
  float gx_r = gx / 131.0f * (M_PI / 180.0f);
  float gy_r = gy / 131.0f * (M_PI / 180.0f);
  float gz_r = gz / 131.0f * (M_PI / 180.0f);

  unsigned long now = micros();
  float dt = (now - lastUpdate) / 1000000.0f;
  lastUpdate = now;

  float qw = q[0], qx = q[1], qy = q[2], qz = q[3];

  float dq_w = 0.5f * (-qx * gx_r - qy * gy_r - qz * gz_r);
  float dq_x = 0.5f * ( qw * gx_r + qy * gz_r - qz * gy_r);
  float dq_y = 0.5f * ( qw * gy_r - qx * gz_r + qz * gx_r);
  float dq_z = 0.5f * ( qw * gz_r + qx * gy_r - qy * gx_r);

  q[0] += dq_w * dt;
  q[1] += dq_x * dt;
  q[2] += dq_y * dt;
  q[3] += dq_z * dt;

  float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (norm > 0.0f) {
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire1.begin(25, 26);

  Serial.println("Initializing MPU1, MPU2, and MPU3...");
  mpu1.initialize();
  Serial.println(mpu1.testConnection() ? "MPU1 OK" : "MPU1 FAIL");

  mpu2.initialize();
  Serial.println(mpu2.testConnection() ? "MPU2 OK" : "MPU2 FAIL");

  mpu3.initialize();
  Serial.println(mpu3.testConnection() ? "MPU3 OK" : "MPU3 FAIL");

  SerialBT.begin("ESP32_Client", true);
  (SerialBT.connect(serverMAC))?Serial.println("Connected to server"):Serial.println("Failed to connect");

  for(int i = 0; i< 5; i++){
    mpu1.getMotion6(&newData.ax1, &newData.ay1, &newData.az1, &newData.gx1, &newData.gy1, &newData.gz1);
    mpu2.getMotion6(&newData.ax2, &newData.ay2, &newData.az2, &newData.gx2, &newData.gy2, &newData.gz2);
    mpu3.getMotion6(&newData.ax3, &newData.ay3, &newData.az3, &newData.gx3, &newData.gy3, &newData.gz3);
    enqueue(newData);
  }
}

void loop() {

  while(SerialBT.connected()){
    mpu1.getMotion6(&newData.ax1, &newData.ay1, &newData.az1, &newData.gx1, &newData.gy1, &newData.gz1);
    mpu2.getMotion6(&newData.ax2, &newData.ay2, &newData.az2, &newData.gx2, &newData.gy2, &newData.gz2);
    mpu3.getMotion6(&newData.ax3, &newData.ay3, &newData.az3, &newData.gx3, &newData.gy3, &newData.gz3);
    enqueue(newData);

    MotionData sendData;
    if (dequeue(sendData)) {
      convertToQuaternion(sendData.gx1, sendData.gy1, sendData.gz1, q1, lastUpdate1);
      convertToQuaternion(sendData.gx2, sendData.gy2, sendData.gz2, q2, lastUpdate2);
      convertToQuaternion(sendData.gx3, sendData.gy3, sendData.gz3, q3, lastUpdate3);

      qData.w1 = q1[0]; qData.x1 = q1[1]; qData.y1 = q1[2]; qData.z1 = q1[3];
      qData.w2 = q2[0]; qData.x2 = q2[1]; qData.y2 = q2[2]; qData.z2 = q2[3];
      qData.w3 = q3[0]; qData.x3 = q3[1]; qData.y3 = q3[2]; qData.z3 = q3[3];

      uint8_t buffer[sizeof(DualQuaternion)];

      memcpy(buffer, &qData, sizeof(DualQuaternion));
      SerialBT.write(buffer, sizeof(DualQuaternion));

      Serial.printf("Sent MPU1: W=%.4f X=%.4f Y=%.4f Z=%.4f | ", qData.w1, qData.x1, qData.y1, qData.z1);
      Serial.printf("MPU2: W=%.4f X=%.4f Y=%.4f Z=%.4f | ", qData.w2, qData.x2, qData.y2, qData.z2);
      Serial.printf("MPU2: W=%.4f X=%.4f Y=%.4f Z=%.4f\n", qData.w3, qData.x3, qData.y3, qData.z3);
    }
  }
}
