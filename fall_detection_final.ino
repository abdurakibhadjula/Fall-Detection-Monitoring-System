#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

// =====================================================
// HARDWARE PINS
// =====================================================
const int BUZZER_PIN = 4;
const int BUTTON_PIN = 5;

// =====================================================
// FALL DETECTION PARAMETERS (>= 0.4m Drop Logic)
// =====================================================
// Acceleration magnitude during weightless free-fall is near 0 m/s^2.
const float FREEFALL_ACCEL_THRESHOLD = 3.0; // m/s^2
const unsigned long FREEFALL_MIN_DURATION = 285; // ms (calculated for >= 0.4m drop)

// Backup Instant Spike Trigger
const float Z_SPIKE_THRESHOLD = -4.0; 

// =====================================================
// BLE SERVICES & CHARACTERISTICS
// =====================================================
BLEService fallService("19B10000-E8F2-537E-4F6C-D104768A1214");

// Transmit IMU values: ax,ay,az,gx,gy,gz
BLEStringCharacteristic dataChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 40);

// Transmit Fall State: "NORMAL", "DANGER"
BLEStringCharacteristic stateChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 15);

// Receive Cancel command from HTML (Value = 1)
BLEByteCharacteristic commandChar("19B10003-E8F2-537E-4F6C-D104768A1214", BLEWrite);

// =====================================================
// STATE VARIABLES
// =====================================================
bool fallConfirmed = false;
unsigned long freefallStartTime = 0;
bool inFreefall = false;
String currentState = "NORMAL";

float ax, ay, az, gx, gy, gz;
unsigned long lastReadTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT); // Change to INPUT_PULLUP if no external resistor

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  BLE.setLocalName("FallMonitor");
  BLE.setAdvertisedService(fallService);
  
  fallService.addCharacteristic(dataChar);
  fallService.addCharacteristic(stateChar);
  fallService.addCharacteristic(commandChar);
  BLE.addService(fallService);
  
  stateChar.writeValue("NORMAL");
  BLE.advertise();
}

void loop() {
  BLEDevice central = BLE.central();
  
  if (central) {
    while (central.connected()) {
      processSensors();
      checkPhysicalButton();
      checkWebCommand();
      handleBuzzer();
    }
    digitalWrite(BUZZER_PIN, LOW);
  } else {
    // Keep processing and alarm active even if BLE disconnects
    processSensors();
    checkPhysicalButton();
    handleBuzzer();
  }
}

void processSensors() {
  if (millis() - lastReadTime >= 20) { // ~50 Hz update rate
    lastReadTime = millis();
    
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);

      // Convert Gs to m/s^2
      ax *= 9.81; 
      ay *= 9.81; 
      az *= 9.81;

      float rawAccelMag = sqrt((ax * ax) + (ay * ay) + (az * az));

      // Stream data over BLE
      String imuStr = String(ax,1) + "," + String(ay,1) + "," + String(az,1) + "," + 
                      String(gx,1) + "," + String(gy,1) + "," + String(gz,1);
      dataChar.writeValue(imuStr);
      
      detectFall(rawAccelMag);
    }
  }
}

void detectFall(float accelMag) {
  if (fallConfirmed) return;

  // 1. FREE-FALL DETECTION FOR >= 0.4 METERS DROP
  if (accelMag < FREEFALL_ACCEL_THRESHOLD) {
    if (!inFreefall) {
      inFreefall = true;
      freefallStartTime = millis();
    } else {
      // If weightlessness persists longer than 285ms, it is a 0.4m+ fall
      if (millis() - freefallStartTime >= FREEFALL_MIN_DURATION) {
        confirmFall();
        return;
      }
    }
  } else {
    inFreefall = false; // Reset free-fall timer if object regains weight
  }

  // 2. BACKUP Z-AXIS NEGATIVE SPIKE TRIGGER
  if (az <= Z_SPIKE_THRESHOLD) {
    confirmFall();
    return;
  }
}

void confirmFall() {
  fallConfirmed = true;
  inFreefall = false;
  updateState("DANGER");
  Serial.println(">>> FALL DETECTED (>= 0.4m Drop) <<<");
}

void cancelFall() {
  fallConfirmed = false;
  inFreefall = false;
  digitalWrite(BUZZER_PIN, LOW); 
  updateState("NORMAL");
  Serial.println(">>> ALARM CANCELLED <<<");
}

void updateState(String newState) {
  if (currentState != newState) {
    currentState = newState;
    stateChar.writeValue(currentState);
  }
}

void checkPhysicalButton() {
  if (digitalRead(BUTTON_PIN) == HIGH) {
    if (fallConfirmed) {
      cancelFall();
      delay(200); // Simple debounce
    }
  }
}

void checkWebCommand() {
  if (commandChar.written()) {
    if (commandChar.value() == 1) {
      cancelFall(); 
    }
  }
}

void handleBuzzer() {
  if (fallConfirmed) {
    digitalWrite(BUZZER_PIN, (millis() / 250) % 2 == 0 ? HIGH : LOW);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}
#include <Arduino_LSM9DS1.h>
// #include <ArduinoBLE.h> // Uncomment if you are actively using BLE

// =====================================================
// HARDWARE PINS
// =====================================================
const int BUZZER_PIN = 4;        // Connect Buzzer to D4
const int BUTTON_PIN = 5;        // Connect Cancel Button to D5 (with pull-down resistor)

// =====================================================
// FALL DETECTION THRESHOLDS
// =====================================================
const float Z_SPIKE_THRESHOLD = -5.0;       // Instant fall trigger if Z drops below this (m/s²)
const float IMPACT_THRESHOLD = 24.5;        // ~2.5 Gs sudden impact (m/s²)
const float GYRO_THRESHOLD = 150.0;         // Rapid rotation (°/s)
const float LOW_MOVEMENT_THRESHOLD = 12.0;  // Resting threshold (m/s²)

const unsigned long CONFIRMATION_TIME = 1000; // Wait 1 sec before checking for inactivity
const unsigned long INACTIVITY_TIME = 2000;   // Must stay still for 2 secs to confirm
const unsigned long FALL_TIMEOUT = 5000;      // Reset possible fall after 5 secs

// =====================================================
// STATE VARIABLES
// =====================================================
bool fallConfirmed = false;
bool possibleFall = false;
unsigned long possibleFallTime = 0;
unsigned long inactivityStart = 0;

// Sensor Data
float ax, ay, az, gx, gy, gz;
float rawAccelerationMagnitude = 0;
float filteredAccelerationMagnitude = 0;
float gyroMagnitude = 0;

// Filter constant
const float ALPHA = 0.2; 
unsigned long lastReadTime = 0;

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT); // Use INPUT_PULLUP if not using a physical pull-down resistor

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop() {
  // 1. Read IMU Data (At roughly 100Hz)
  if (millis() - lastReadTime >= 10) {
    lastReadTime = millis();
    
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);

      // Convert Gs to m/s² (Most Arduino IMU libraries output in Gs)
      ax *= 9.81; 
      ay *= 9.81; 
      az *= 9.81;

      // Calculate Magnitudes
      rawAccelerationMagnitude = sqrt((ax * ax) + (ay * ay) + (az * az));
      gyroMagnitude = sqrt((gx * gx) + (gy * gy) + (gz * gz));
      
      // Apply Low-Pass Filter
      if (filteredAccelerationMagnitude == 0) {
        filteredAccelerationMagnitude = rawAccelerationMagnitude;
      } else {
        filteredAccelerationMagnitude = (ALPHA * rawAccelerationMagnitude) + ((1.0 - ALPHA) * filteredAccelerationMagnitude);
      }

      // Send Data to HTML Web Dashboard
      Serial.print("DATA,");
      Serial.print(ax); Serial.print(",");
      Serial.print(ay); Serial.print(",");
      Serial.print(az); Serial.print(",");
      Serial.print(gx); Serial.print(",");
      Serial.print(gy); Serial.print(",");
      Serial.println(gz);
      
      // Run Fall Logic
      detectFall();
    }
  }

  // 2. Check Physical Button
  checkButton();

  // 3. Listen for Web Dashboard Commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "CANCEL") {
      if (fallConfirmed || possibleFall) {
        cancelFall();
      }
    }
  }
  
  // 4. Handle Buzzer Alarm State
  if (fallConfirmed) {
    // Flash buzzer on and off every 250ms
    if ((millis() / 250) % 2 == 0) {
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// =====================================================
// FALL DETECTION LOGIC
// =====================================================
void detectFall() {
  if (fallConfirmed) return;

  // -------------------------------------------------
  // INSTANT FALL TRIGGER: Z-Axis Negative Spike
  // -------------------------------------------------
  if (az <= Z_SPIKE_THRESHOLD) {
    Serial.println();
    Serial.println(">>> INSTANT Z-AXIS NEGATIVE SPIKE DETECTED <<<");
    confirmFall();
    return; // Exit function immediately
  }

  // -------------------------------------------------
  // STAGE 1: Standard Impact + Rotation Check
  // -------------------------------------------------
  bool highImpact = rawAccelerationMagnitude >= IMPACT_THRESHOLD;
  bool highRotation = gyroMagnitude >= GYRO_THRESHOLD;

  if (!possibleFall && highImpact && highRotation) {
    possibleFall = true;
    possibleFallTime = millis();
    inactivityStart = 0;

    Serial.println();
    Serial.println(">>> POSSIBLE FALL DETECTED <<<");
    
    // fallCharacteristic.writeValue("POSSIBLE FALL"); // Uncomment for BLE
  }

  // -------------------------------------------------
  // STAGE 2: Wait for Inactivity (Confirmation)
  // -------------------------------------------------
  if (possibleFall) {
    if (millis() - possibleFallTime >= CONFIRMATION_TIME) {
      
      if (filteredAccelerationMagnitude < LOW_MOVEMENT_THRESHOLD) {
        if (inactivityStart == 0) {
          inactivityStart = millis();
        }

        if (millis() - inactivityStart >= INACTIVITY_TIME) {
          confirmFall();
        }
      } else {
        inactivityStart = 0; // Movement continues, reset inactivity timer
      }
    }

    // -------------------------------------------------
    // TIMEOUT: Cancel false alarm if user keeps moving
    // -------------------------------------------------
    if (millis() - possibleFallTime > FALL_TIMEOUT) {
      possibleFall = false;
      inactivityStart = 0;
      Serial.println("Fall candidate rejected. User resumed movement.");
      
      // fallCharacteristic.writeValue("FALL REJECTED"); // Uncomment for BLE
    }
  }
}

// =====================================================
// CONFIRM FALL
// =====================================================
void confirmFall() {
  fallConfirmed = true;
  possibleFall = false;
  Serial.println(">>> FALL CONFIRMED <<<");
  
  // fallCharacteristic.writeValue("FALL CONFIRMED"); // Uncomment for BLE
}

// =====================================================
// CANCEL FALL
// =====================================================
void cancelFall() {
  fallConfirmed = false;
  possibleFall = false;
  inactivityStart = 0;
  
  digitalWrite(BUZZER_PIN, LOW); // Silence buzzer immediately
  Serial.println(">>> ALARM CANCELLED <<<");
  
  // fallCharacteristic.writeValue("CANCELLED"); // Uncomment for BLE
}

// =====================================================
// CHECK PHYSICAL BUTTON
// =====================================================
void checkButton() {
  // Assuming a HIGH signal when pressed. Change to LOW if using INPUT_PULLUP
  if (digitalRead(BUTTON_PIN) == HIGH) {
    if (fallConfirmed || possibleFall) {
      cancelFall();
      delay(200); // Simple debounce
    }
  }
}