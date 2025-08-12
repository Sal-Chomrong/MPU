#include "BluetoothSerial.h"
#include "I2Cdev.h"
#include <Wire.h>
#include <MPU6050.h>

BluetoothSerial SerialBT;
// MPU6050 default address
#define MPU_ADDR 0x68

MPU6050 mpu1(MPU_ADDR, &Wire);   // MPU1 on Wire bus
MPU6050 mpu2(MPU_ADDR, &Wire1);  // MPU2 on Wire1 bus

uint8_t serverMAC[] = {0xCC, 0x7B, 0x5C, 0xF1, 0x90, 0xF6};

struct MotionData {       // create structure 
  int16_t ax1, ay1, az1;
  int16_t gx1, gy1, gz1;
  int16_t ax2, ay2, az2;
  int16_t gx2, gy2, gz2;
};

#define QUEUE_SIZE 10     // size of Queue
MotionData queue[QUEUE_SIZE];
int front = 0, rear = 0, count = 0; // parameter of queue

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

  Wire.begin();          // Initialize default I2C bus
  Wire1.begin(25, 26);   // Initialize second I2C bus (pins for ESP32, adjust for your board)

  Serial.println("Initializing MPU1...");
  mpu1.initialize();
  if (mpu1.testConnection()) {
    Serial.println("MPU1 connection successful");
  } else {
    Serial.println("MPU1 connection failed");
  }

  Serial.println("Initializing MPU2...");
  mpu2.initialize();
  if (mpu2.testConnection()) {
    Serial.println("MPU2 connection successful");
  } else {
    Serial.println("MPU2 connection failed");
  }
  SerialBT.begin("ESP32_Client", true);
  if (SerialBT.connect(serverMAC)) Serial.println("Connected to server"); 
  else Serial.println("Failed to connect");
  for (int i = 0; i < 6; i++) {
    MotionData data;
    mpu1.getMotion6(&data.ax1, &data.ay1, &data.az1, &data.gx1, &data.gy1, &data.gz1);
    mpu2.getMotion6(&data.ax2, &data.ay2, &data.az2, &data.gx2, &data.gy2, &data.gz2);
    enqueue(data);
  }
}

void loop() {
  
  MotionData newData;
  mpu1.getMotion6(&newData.ax1, &newData.ay1, &newData.az1, &newData.gx1, &newData.gy1, &newData.gz1);
  mpu2.getMotion6(&newData.ax2, &newData.ay2, &newData.az2, &newData.gx2, &newData.gy2, &newData.gz2);
  enqueue(newData);
  if (SerialBT.connected()) {
    MotionData sendData;
    if (dequeue(sendData)) {
      uint8_t buf[sizeof(MotionData)];
      memcpy(buf, &sendData, sizeof(MotionData));
      SerialBT.write(buf, sizeof(MotionData));
      Serial.printf("Sent -> ACC(%d,%d,%d) GYRO(%d,%d,%d) ACC(%d,%d,%d) GYRO(%d,%d,%d)\n",
        sendData.ax1, sendData.ay1, sendData.az1,
        sendData.gx1, sendData.gy1, sendData.gz1,
        sendData.ax2, sendData.ay2, sendData.az2,
        sendData.gx2, sendData.gy2, sendData.gz2);
    }
  }


}
