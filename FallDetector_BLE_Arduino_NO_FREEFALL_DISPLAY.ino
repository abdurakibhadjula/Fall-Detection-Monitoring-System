#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>
#include <math.h>

// ============================================================
// FALL DETECTOR - ARDUINO NANO 33 BLE REV2
// ============================================================
//
// Outputs:
//   D5 = vibration module input
//   D6 = active buzzer
//   D7 = LED (through 220 ohm resistor)
//   D8 = push button to GND
//
// BLE:
//   Device: FallDetector
//   Service:       19b10000-e8f2-537e-4f6c-d104768a1214
//   Alert:         19b10001-e8f2-537e-4f6c-d104768a1214
//   Data:          19b10002-e8f2-537e-4f6c-d104768a1214
//   Command:       19b10003-e8f2-537e-4f6c-d104768a1214
//
// IMPORTANT:
// This version uses RAW acceleration for the free-fall test.
// The alpha filter is used ONLY for graph values so it does
// not hide the short free-fall event.
// ============================================================


// -------------------- PINS --------------------
const int VIBRATION_PIN = 5;
const int BUZZER_PIN    = 6;
const int LED_PIN       = 7;
const int BUTTON_PIN    = 8;


// -------------------- BLE --------------------
const char* DEVICE_NAME = "FallDetector";

BLEService fallService(
  "19b10000-e8f2-537e-4f6c-d104768a1214"
);

BLEStringCharacteristic alertCharacteristic(
  "19b10001-e8f2-537e-4f6c-d104768a1214",
  BLERead | BLENotify,
  30
);

BLECharacteristic dataCharacteristic(
  "19b10002-e8f2-537e-4f6c-d104768a1214",
  BLERead | BLENotify,
  80
);

BLEStringCharacteristic commandCharacteristic(
  "19b10003-e8f2-537e-4f6c-d104768a1214",
  BLEWrite,
  20
);


// -------------------- IMU --------------------
float ax, ay, az;
float gx, gy, gz;

// Graph-only smoothing
float smoothAX, smoothAY, smoothAZ;
float smoothGX, smoothGY, smoothGZ;

const float GRAPH_ALPHA = 0.05f;
bool firstReading = true;


// -------------------- FREE-FALL TEST --------------------
// At this prototype stage, a fall is triggered when the
// acceleration magnitude stays below the threshold long
// enough to represent free fall.
//
// 2.5 m/s^2 is about 0.255 g.
// 60 ms is approximately 3 samples at 50 Hz.

const float FREE_FALL_THRESHOLD = 3.5f;    // m/s^2
const unsigned long FREE_FALL_TIME = 70;   // ms

bool inFreeFall = false;
unsigned long freeFallStart = 0;


// -------------------- FALL STATE --------------------
bool fallConfirmed = false;


// -------------------- BUTTON --------------------
bool previousButtonState = HIGH;
unsigned long lastButtonPress = 0;
const unsigned long BUTTON_DEBOUNCE = 50;


// -------------------- DATA TIMING --------------------
unsigned long lastDataTime = 0;
const unsigned long DATA_INTERVAL = 20;    // ~50 Hz BLE stream


// ============================================================
// HELPERS
// ============================================================

void sendAlert(const char* message) {
  alertCharacteristic.writeValue(message);

  Serial.print("ALERT,");
  Serial.println(message);
}

void stopAlarm() {
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(VIBRATION_PIN, LOW);
}

void startAlarm() {
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(VIBRATION_PIN, HIGH);
}

void confirmFall() {
  inFreeFall = false;
  freeFallStart = 0;

  fallConfirmed = true;

  startAlarm();
  sendAlert("FALL DETECTED");
}

void cancelFall() {
  inFreeFall = false;
  freeFallStart = 0;
  fallConfirmed = false;

  stopAlarm();
  sendAlert("FALL CANCELLED");
}


// ============================================================
// BUTTON
// ============================================================

void checkButton() {
  bool currentButtonState = digitalRead(BUTTON_PIN);

  if (currentButtonState == LOW &&
      previousButtonState == HIGH &&
      millis() - lastButtonPress > BUTTON_DEBOUNCE) {

    lastButtonPress = millis();

    if (fallConfirmed || inFreeFall) {
      cancelFall();
    }
  }

  previousButtonState = currentButtonState;
}


// ============================================================
// BLE COMMANDS
// ============================================================

void checkBluetoothCommand() {
  if (!commandCharacteristic.written()) {
    return;
  }

  String command = commandCharacteristic.value();
  command.trim();

  if (command == "CANCEL") {
    if (fallConfirmed || inFreeFall) {
      cancelFall();
    } else {
      sendAlert("NO ACTIVE FALL");
    }
  }
}


// ============================================================
// FREE-FALL DETECTION
// ============================================================

void detectFreeFall(float accelerationMagnitude) {

  // Do not retrigger while alarm is active.
  if (fallConfirmed) {
    return;
  }

  bool belowThreshold =
    accelerationMagnitude < FREE_FALL_THRESHOLD;

  if (belowThreshold) {

    if (!inFreeFall) {
      inFreeFall = true;
      freeFallStart = millis();
    }

    if (millis() - freeFallStart >= FREE_FALL_TIME) {
      confirmFall();
    }

  } else {

    // Free-fall event ended before confirmation.
    // Clear the internal state and return the phone UI to normal.
    if (inFreeFall) {
      inFreeFall = false;
      freeFallStart = 0;
      sendAlert("NORMAL");
    }
  }
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  pinMode(VIBRATION_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Button to GND, internal pull-up enabled.
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  stopAlarm();


  // -------------------- IMU --------------------
  if (!IMU.begin()) {
    Serial.println("IMU initialization failed!");
    while (true) {}
  }


  // -------------------- BLE --------------------
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (true) {}
  }

  BLE.setLocalName(DEVICE_NAME);
  BLE.setDeviceName(DEVICE_NAME);
  BLE.setAdvertisedService(fallService);

  fallService.addCharacteristic(alertCharacteristic);
  fallService.addCharacteristic(dataCharacteristic);
  fallService.addCharacteristic(commandCharacteristic);

  BLE.addService(fallService);

  alertCharacteristic.writeValue("SYSTEM READY");
  dataCharacteristic.writeValue("DATA,0,0,0,0,0,0");

  BLE.advertise();


  // -------------------- STATUS --------------------
  Serial.println();
  Serial.println("--------------------------------");
  Serial.println("FALL DETECTION SYSTEM READY");
  Serial.println("--------------------------------");
  Serial.println("BLE: FallDetector");
  Serial.println("Graph alpha: 0.005");
  Serial.println("Sample output: ~50 Hz");
  Serial.println("Free-fall threshold: 2.5 m/s2");
  Serial.println("Free-fall time: 60 ms");
  Serial.println("--------------------------------");
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  BLE.poll();
  checkBluetoothCommand();
  checkButton();


  if (IMU.accelerationAvailable() &&
      IMU.gyroscopeAvailable()) {

    // -------------------- READ IMU --------------------
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);


    // Arduino IMU acceleration is returned in g.
    // Convert to m/s^2.
    ax *= 9.80665f;
    ay *= 9.80665f;
    az *= 9.80665f;


    // -------------------- FREE-FALL DATA --------------------
    // Use RAW acceleration for the fall detector.
    float accelerationMagnitude = sqrtf(
      ax * ax +
      ay * ay +
      az * az
    );

    detectFreeFall(accelerationMagnitude);


    // -------------------- GRAPH SMOOTHING --------------------
    // Alpha = 0.005 gives a very smooth display.
    // This does NOT affect the fall detector above.

    if (firstReading) {
      smoothAX = ax;
      smoothAY = ay;
      smoothAZ = az;

      smoothGX = gx;
      smoothGY = gy;
      smoothGZ = gz;

      firstReading = false;
    }

    smoothAX += GRAPH_ALPHA * (ax - smoothAX);
    smoothAY += GRAPH_ALPHA * (ay - smoothAY);
    smoothAZ += GRAPH_ALPHA * (az - smoothAZ);

    smoothGX += GRAPH_ALPHA * (gx - smoothGX);
    smoothGY += GRAPH_ALPHA * (gy - smoothGY);
    smoothGZ += GRAPH_ALPHA * (gz - smoothGZ);


    // -------------------- SEND DATA --------------------
    if (millis() - lastDataTime >= DATA_INTERVAL) {

      lastDataTime = millis();

      char packet[80];

      snprintf(
        packet,
        sizeof(packet),
        "DATA,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
        smoothAX,
        smoothAY,
        smoothAZ,
        smoothGX,
        smoothGY,
        smoothGZ
      );

      // BLE notification for the web app.
      dataCharacteristic.writeValue(packet);

      // USB serial remains available for debugging.
      Serial.println(packet);
    }
  }

  delay(2);
}
