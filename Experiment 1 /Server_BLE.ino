#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include <math.h>

BluetoothSerial SerialBT;

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

Servo servoRoll;  // Servo for roll control
Servo servoPitch; // Servo for pitch control

// Servo pins
const int servoRollPin = 18;
const int servoPitchPin = 19;

float yaw = 0.0f;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Server");
  Serial.println("Bluetooth Server Ready. Waiting for client...");

  // Attach servos to pins
  servoRoll.attach(servoRollPin);
  servoPitch.attach(servoPitchPin);

  lastTime = millis();
}

void loop() {
  if (SerialBT.available() >= sizeof(MotionData)) {
    uint8_t buf[sizeof(MotionData)];
    SerialBT.readBytes(buf, sizeof(MotionData));

    MotionData receivedData;
    memcpy(&receivedData, buf, sizeof(MotionData));

    float ax = receivedData.ax / 16384.0f;
    float ay = receivedData.ay / 16384.0f;
    float az = receivedData.az / 16384.0f;

    float gx = receivedData.gx / 131.0f;
    float gy = receivedData.gy / 131.0f;
    float gz = receivedData.gz / 131.0f;

    float roll  = atan2(ay, az);
    float pitch = atan(-ax / sqrt(ay*ay + az*az));

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0f;
    lastTime = currentTime;
    yaw += gz * dt;

    enqueue(receivedData);

    // Convert radians to degrees
    roll  = roll * 180.0f / PI;
    pitch = pitch * 180.0f / PI;

    // Map roll and pitch angles (-90 to 90) to servo angle range (0 to 180)
    int servoRollAngle = map((int)roll, -90, 90, 0, 180);
    int servoPitchAngle = map((int)pitch, -90, 90, 0, 180);

    // Constrain servo angles (optional, just in case)
    servoRollAngle = constrain(servoRollAngle, 0, 180);
    servoPitchAngle = constrain(servoPitchAngle, 0, 180);

    // Move servos
    servoRoll.write(servoRollAngle);
    servoPitch.write(servoPitchAngle);

    Serial.printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f° | Servo Roll: %d, Servo Pitch: %d\n",
                  roll, pitch, yaw, servoRollAngle, servoPitchAngle);
  }
}
