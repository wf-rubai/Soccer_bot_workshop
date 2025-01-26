# ESP32 RC Car Project

This repository contains the Arduino code and setup instructions for an RC car using an ESP32, two BTS7960 motor drivers, and Bluetooth for control.

## Pin Connections

| **Component**      | **ESP32 Pin** | **BTS7960 Pin** | **Purpose**                 |
|--------------------|---------------|-----------------|-----------------------------|
| Left Motor RPWM    | GPIO 25       | RPWM-L          | Control speed and direction |
| Left Motor LPWM    | GPIO 26       | LPWM-L          | Control speed and direction |
| Right Motor RPWM   | GPIO 27       | RPWM-R          | Control speed and direction |
| Right Motor LPWM   | GPIO 14       | LPWM-R          | Control speed and direction |
| Left Motor Enable  | GPIO 12       | REN-L, LEN-L    | Enable motor driver         |
| Right Motor Enable | GPIO 33       | REN-R, LEN-R    | Enable motor driver         |
| GND                | GND           | GND             | Common ground connection    |
| VCC                | 5V            | VCC             | Power supply for logic      |

## Steps to Upload the Code to ESP32

### 1. Install Arduino IDE

Download and install the [Arduino IDE](https://www.arduino.cc/en/software) if you haven’t already.

### 2. Install ESP32 Board Support

1. Open the Arduino IDE.
2. Go to **File > Preferences**.
3. In the "Additional Board Manager URLs" field, add the following URL:
   ```
   https://dl.espressif.com/dl/package_esp32_index.json
   ```
4. Click **OK**.
5. Go to **Tools > Board > Boards Manager**.
6. Search for "ESP32" and install the **ESP32 by Espressif Systems** package.

### 3. Connect Your ESP32

Use a USB cable to connect the ESP32 to your computer. Ensure the cable supports data transfer (not just charging).

### 4. Select the Board and Port

1. Go to **Tools > Board** and select your ESP32 board model (e.g., **ESP32 Dev Module**).
2. Go to **Tools > Port** and select the correct port (e.g., `COM3` on Windows or `/dev/ttyUSB0` on Linux/macOS).

### 5. Install BluetoothSerial Library

The **BluetoothSerial** library is included with the ESP32 core. Ensure you've installed the ESP32 core correctly; no additional steps are needed.

### 6. Upload the Code

1. Copy the code into the Arduino IDE.
2. Click the **Verify** button (checkmark icon) to compile the code.
3. If there are no errors, click the **Upload** button (right-arrow icon).
4. Wait for the IDE to upload the code. You’ll see the message **"Done uploading"** when it’s complete.

### 7. Test the Code

1. Open the **Serial Monitor** (Tools > Serial Monitor) to debug or see logs.
2. Pair the ESP32 with your phone via Bluetooth (e.g., device name: `RC_Car`).

Now, your code should be running on the ESP32 and the RC car is ready for use!

## Arduino Code

```cpp
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

// Motor driver pins
const int RPWM_L = 25; // Left motor RPWM
const int LPWM_L = 26; // Left motor LPWM
const int RPWM_R = 27; // Right motor RPWM
const int LPWM_R = 14; // Right motor LPWM
const int REN_L = 12;  // Left motor enable
const int LEN_L = 13;  // Left motor enable
const int REN_R = 33;  // Right motor enable
const int LEN_R = 32;  // Right motor enable

int speedLevel = 0; // Speed (0-255)

void setup() {
  Serial.begin(115200);
  SerialBT.begin("soccer_bot_"); // Bluetooth device name
  pinMode(RPWM_L, OUTPUT);
  pinMode(LPWM_L, OUTPUT);
  pinMode(RPWM_R, OUTPUT);
  pinMode(LPWM_R, OUTPUT);
  pinMode(REN_L, OUTPUT);
  pinMode(LEN_L, OUTPUT);
  pinMode(REN_R, OUTPUT);
  pinMode(LEN_R, OUTPUT);

  digitalWrite(REN_L, HIGH);
  digitalWrite(LEN_L, HIGH);
  digitalWrite(REN_R, HIGH);
  digitalWrite(LEN_R, HIGH);
}

void loop() {
  if (SerialBT.available()) {
    char command = SerialBT.read();
    switch (command) {
      case 'F': controlMotors(speedLevel, speedLevel, 0, 0); break; // Forward
      case 'B': controlMotors(0, 0, speedLevel, speedLevel); break; // Backward
      case 'L': controlMotors(0, speedLevel, speedLevel, 0); break; // Left
      case 'R': controlMotors(speedLevel, 0, 0, speedLevel); break; // Right
      case 'FL': controlMotors(speedLevel / 2, speedLevel, speedLevel, 0); break; // Forward Left
      case 'FR': controlMotors(speedLevel, speedLevel / 2, 0, speedLevel); break; // Forward Right
      case 'BL': controlMotors(0, speedLevel / 2, speedLevel, speedLevel); break; // Backward Left
      case 'BR': controlMotors(0, 0, speedLevel / 2, speedLevel); break; // Backward Right
      case '0' ... '9': speedLevel = map(command - '0', 0, 9, 0, 255); break; // Speed control
      case 'S': stopMotors(); break; // Stop
    }
  }
}

// Common function for motor control
void controlMotors(int leftFwd, int leftBwd, int rightFwd, int rightBwd) {
  analogWrite(RPWM_L, leftFwd);
  analogWrite(LPWM_L, leftBwd);
  analogWrite(RPWM_R, rightFwd);
  analogWrite(LPWM_R, rightBwd);
}

// Stop all motors
void stopMotors() {
  controlMotors(0, 0, 0, 0);
}
```

---
## Some Special Notes
### Prompt Writing on ChatGPT
- Write discriptive prompt.
- You can see [This chat]() on ChatGPT as an example.