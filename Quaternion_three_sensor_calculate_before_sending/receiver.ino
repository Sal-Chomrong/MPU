#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// --- Struct definition (must match client) ---
struct Quaternion_data {
  float w1, x1, y1, z1;  // MPU1
  float w2, x2, y2, z2;  // MPU2
  float w3, x3, y3, z3;  // MPU3
};

// --- Queue ---
#define QUEUE_SIZE 10
Quaternion_data queue[QUEUE_SIZE];
int front = 0, rear = 0, count = 0;

bool enqueue(Quaternion_data d) {
  if (count < QUEUE_SIZE) {
    queue[rear] = d;
    rear = (rear + 1) % QUEUE_SIZE;
    count++;
  } else {
    // Overwrite oldest if full
    queue[rear] = d;
    rear = (rear + 1) % QUEUE_SIZE;
    front = (front + 1) % QUEUE_SIZE;
  }
  return true;
}

bool dequeue(Quaternion_data &d) {
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
  SerialBT.begin("ESP32_Server"); // Bluetooth server mode
  Serial.println("Bluetooth server started. Waiting for client...");
}

void loop() {
  // --- Receive data from Bluetooth ---
  if (SerialBT.available() >= sizeof(Quaternion_data)) {
    uint8_t buf[sizeof(Quaternion_data)];
    size_t bytesRead = 0;

    // Ensure full struct is received
    while (bytesRead < sizeof(Quaternion_data)) {
      bytesRead += SerialBT.readBytes(buf + bytesRead, sizeof(Quaternion_data) - bytesRead);
    }

    Quaternion_data receivedData;
    memcpy(&receivedData, buf, sizeof(Quaternion_data));

    enqueue(receivedData); // store in FIFO

    Serial.printf("📥 Received -> Q1(%f,%f,%f,%f) Q2(%f,%f,%f,%f) Q3(%f,%f,%f,%f)\n",
      receivedData.w1, receivedData.x1, receivedData.y1, receivedData.z1,
      receivedData.w2, receivedData.x2, receivedData.y2, receivedData.z2,
      receivedData.w3, receivedData.x3, receivedData.y3, receivedData.z3);
  }

  // --- Process oldest data from queue ---
  Quaternion_data q;
  if (dequeue(q)) {
    Serial.printf("📤 Processed -> Q1(%f,%f,%f,%f) Q2(%f,%f,%f,%f) Q3(%f,%f,%f,%f)\n",
      q.w1, q.x1, q.y1, q.z1,
      q.w2, q.x2, q.y2, q.z2,
      q.w3, q.x3, q.y3, q.z3);
  }

  delay(10);
}
