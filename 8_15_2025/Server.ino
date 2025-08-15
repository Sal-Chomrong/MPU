#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// --- Struct definition (must match client EXACTLY) ---
struct Quaternion_data {
  float w1, x1, y1, z1;  // MPU1
  float w2, x2, y2, z2;  // MPU2
  float w3, x3, y3, z3;  // MPU3
};

// --- Queue ---
#define QUEUE_SIZE 10
Quaternion_data queue[QUEUE_SIZE];
int front = 0, rear = 0, count = 0;

bool enqueue(const Quaternion_data &d) {
  if (count < QUEUE_SIZE) {
    queue[rear] = d;
    rear = (rear + 1) % QUEUE_SIZE;
    count++;
    return true;
  }
  return false;
}

bool dequeue(Quaternion_data &d) {
  if (count == 0) return false;
  d = queue[front];
  front = (front + 1) % QUEUE_SIZE;
  count--;
  return true;
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Server"); // Bluetooth server mode
  Serial.println("Bluetooth server started. Waiting for client...");
}

void loop() {
  // --- Receive data from Bluetooth ---
  // Read ONLY when a full struct is available
  if (SerialBT.available() >= sizeof(Quaternion_data)) {
    Quaternion_data receivedData;
    size_t n = SerialBT.readBytes((uint8_t*)&receivedData, sizeof(receivedData));
    if (n == sizeof(receivedData)) {
      enqueue(receivedData);
      // Uncomment to see raw reception:
      // Serial.printf("📥 Rx -> Q1(%.4f,%.4f,%.4f,%.4f) Q2(%.4f,%.4f,%.4f,%.4f) Q3(%.4f,%.4f,%.4f,%.4f)\n",
      //   receivedData.w1, receivedData.x1, receivedData.y1, receivedData.z1,
      //   receivedData.w2, receivedData.x2, receivedData.y2, receivedData.z2,
      //   receivedData.w3, receivedData.x3, receivedData.y3, receivedData.z3);
    } else {
      // Partial read (should be rare because of the available() guard)
      // Optionally: flush or resync if you see framing issues.
      Serial.println("⚠️ Partial packet received, ignoring.");
    }
  }

  // --- Process oldest data from queue ---
  Quaternion_data q;
  if (dequeue(q)) {
    Serial.printf("📤 Q1: W=%.4f X=%.4f Y=%.4f Z=%.4f | ", q.w1, q.x1, q.y1, q.z1);
    Serial.printf("Q2: W=%.4f X=%.4f Y=%.4f Z=%.4f | ", q.w2, q.x2, q.y2, q.z2);
    Serial.printf("Q3: W=%.4f X=%.4f Y=%.4f Z=%.4f\n",  q.w3, q.x3, q.y3, q.z3);
  }

  // Small yield is optional:
  // delay(1);
}
