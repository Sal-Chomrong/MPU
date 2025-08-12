#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include <math.h>

BluetoothSerial SerialBT;

struct rawData {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

struct TwoData {
  rawData sensor1;
  rawData sensor2;
};

#define QUEUE_SIZE 10
rawData queue[QUEUE_SIZE];
int front = 0, rear = 0, count = 0;

bool enqueue(rawData d) { 
  if (count < QUEUE_SIZE) { 
    queue[rear] = d; 
    rear = (rear + 1) % QUEUE_SIZE; 
    count++; 
    return true; 
  } 
  return false; 
}

bool dequeue(rawData &d) { 
  if (count > 0) { 
    d = queue[front]; 
    front = (front + 1) % QUEUE_SIZE; 
    count--; 
    return true; 
  } 
  return false; 
}

Servo servoRoll;  // Servo for roll control
Servo servoPitch; // Servo for pitch control

// Servo pins
const int servoRollPin = 18;
const int servoPitchPin = 19;
float roll1, pitch1;
float roll2, pitch2;

void calculateRollPitch(const rawData& data, float &roll, float &pitch) {
  float ax = data.ax / 16384.0f;
  float ay = data.ay / 16384.0f;
  float az = data.az / 16384.0f;

  roll  = atan2(ay, az) * 180.0f / PI;
  pitch = atan(-ax / sqrt(ay*ay + az*az)) * 180.0f / PI;
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Server");
  Serial.println("Bluetooth Server Ready. Waiting for client...");

  servoRoll.attach(servoRollPin);
  servoPitch.attach(servoPitchPin);
}

void loop() {
  if (SerialBT.available() >= sizeof(TwoData)) {
    uint8_t buf[sizeof(TwoData)];
    SerialBT.readBytes(buf, sizeof(TwoData));

    TwoData receivedData;
    memcpy(&receivedData, buf, sizeof(TwoData));

    calculateRollPitch(receivedData.sensor1, roll1, pitch1);
    calculateRollPitch(receivedData.sensor2, roll2, pitch2);

    // For example, use sensor 1 for servo control
    int servoRollAngle = map((int)roll1, -90, 90, 0, 180);
    int servoPitchAngle = map((int)pitch1, -90, 90, 0, 180);

    servoRollAngle = constrain(servoRollAngle, 0, 180);
    servoPitchAngle = constrain(servoPitchAngle, 0, 180);

    servoRoll.write(servoRollAngle);
    servoPitch.write(servoPitchAngle);

    Serial.printf("Sensor1 Roll: %.2f°, Pitch: %.2f° | Sensor2 Roll: %.2f°, Pitch: %.2f° | Servo Roll: %d, Servo Pitch: %d\n",
                  roll1, pitch1, roll2, pitch2, servoRollAngle, servoPitchAngle);
  }

  delay(10);
}
