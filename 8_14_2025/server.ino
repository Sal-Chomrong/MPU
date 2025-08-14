#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// --- Struct definition (must match client) ---
struct DualQuaternion {
  float w1, x1, y1, z1;  // MPU1
  float w2, x2, y2, z2;  // MPU2
};

DualQuaternion qData;

void setup() {
  Serial.begin(115200);
  
  // Start Bluetooth server
  SerialBT.begin("ESP32_Server"); 
  Serial.println("Bluetooth server started. Waiting for client...");
}

void loop() {
  // Check if connected
  if (SerialBT.hasClient()) {
    // Ensure enough bytes are available for the struct
    if (SerialBT.available() >= sizeof(qData)) {
      // Read struct directly into qData
      SerialBT.readBytes((uint8_t*)&qData, sizeof(qData));

      // Print received data
      Serial.printf("MPU1 Quaternion: W=%.4f X=%.4f Y=%.4f Z=%.4f | ",
                    qData.w1, qData.x1, qData.y1, qData.z1);
      Serial.printf("MPU2 Quaternion: W=%.4f X=%.4f Y=%.4f Z=%.4f\n",
                    qData.w2, qData.x2, qData.y2, qData.z2);
    }
  }
}
