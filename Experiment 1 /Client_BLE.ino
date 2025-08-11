#include "BluetoothSerial.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>

BluetoothSerial SerialBT;
MPU6050 mpu;
// CC:7B:5C:F1:90:F6
uint8_t serverMAC[] = {0xCC, 0x7B, 0x5C, 0xF1, 0x90, 0xF6};

struct MotionData {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
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

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  SerialBT.begin("ESP32_Client", true);
  if (SerialBT.connect(serverMAC)) Serial.println("Connected to server"); 
  else Serial.println("Failed to connect");

  // Preload FIFO with initial readings
  for (int i = 0; i < 5; i++) {
    MotionData data;
    mpu.getMotion6(&data.ax, &data.ay, &data.az, &data.gx, &data.gy, &data.gz);
    enqueue(data);
  }
}

void loop() {
  // Always get the latest reading and enqueue it
  MotionData newData;
  mpu.getMotion6(&newData.ax, &newData.ay, &newData.az, &newData.gx, &newData.gy, &newData.gz);
  enqueue(newData);

  if (SerialBT.connected()) {
    MotionData sendData;
    if (dequeue(sendData)) {
      uint8_t buf[sizeof(MotionData)];
      memcpy(buf, &sendData, sizeof(MotionData));
      SerialBT.write(buf, sizeof(MotionData));
      Serial.printf("Sent -> ACC(%d,%d,%d) GYRO(%d,%d,%d)\n",
        sendData.ax, sendData.ay, sendData.az,
        sendData.gx, sendData.gy, sendData.gz);
    }
  }
  delay(10);
}
