/*
  3-Zone Soil Moisture -> Solenoid Valve Controller with Flow Check
  - Arduino UNO (or compatible)
  - 3 analog moisture sensors (A0, A1, A2)
  - 3 solenoid valves driven by appropriate driver/relay/MOSFET on D3, D4, D5
  - Flow sensor (pulse) on D2 (interrupt 0)
  - Prints values, states, and liters to Serial

  NOTES:
  - Adjust thresholds and pulsesPerLiter for your sensors/hardware.
  - Ensure valve driver (relay module or MOSFET+) is used; do NOT drive solenoids directly from Arduino pins.
  - This sketch assumes "higher analog reading => wetter". If your sensors are opposite, invert logic or change SENSOR_WET_HIGH to false.
*/

//////////////////////
// Hardware pins
//////////////////////
const int moisturePin[3] = {A0, A1, A2};     // analog inputs for moisture sensors
const int valvePin[3]    = {5, 6, 7};        // digital outputs for valve drivers (use relay/MOSFET)
const int flowPin        = 2;                // flow sensor pulse (must be interrupt pin)

//////////////////////
// Calibration
//////////////////////
const bool SENSOR_WET_HIGH = true;   // true = higher analog value means wetter. Set false if opposite.

// The (dry and Wet)threshold for each of the three sensors
int dryThreshold[3] = {400, 400, 400};   
int wetThreshold[3] = {600, 600, 600}; 

// Flow calibration
volatile unsigned long pulseCount = 0;
const float pulsesPerLiter = 450.0;   

// Flow timeout when opening valve: if no pulses detected within this many ms after opening, consider NO_FLOW
const unsigned long FLOW_TIMEOUT_MS = 5000UL; // 5 seconds

// Serial print interval
const unsigned long PRINT_INTERVAL_MS = 1000UL;

//////////////////////
// Internal state
//////////////////////
unsigned long lastPrint = 0;
unsigned long lastPulseCheckMillis = 0;

// Per-zone boolean tracking with hysteresis
bool zoneIsWet[3] = {false, false, false};
bool valveIsOpen[3] = {false, false, false};

// For measuring pulses-per-second if desired
unsigned long lastPulseCountSnapshot = 0;
unsigned long lastFlowRateMillis = 0;

//////////////////////
// Flow ISR
//////////////////////
void flowISR() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);
  // while (!Serial && millis() < 2000) { ; } // wait briefly for Serial on some boards

  // pins
  for (int i = 0; i < 3; ++i) {
    pinMode(moisturePin[i], INPUT);
    pinMode(valvePin[i], OUTPUT);
    digitalWrite(valvePin[i], LOW); // ensure valves start closed (assumes HIGH = open). Adjust if inverted.
    valveIsOpen[i] = false;
  }

  pinMode(flowPin, INPUT_PULLUP); // flow sensors usually need pullup; check your sensor
  attachInterrupt(digitalPinToInterrupt(flowPin), flowISR, RISING);

  lastPrint = millis();
  lastPulseCountSnapshot = pulseCount;
  lastFlowRateMillis = millis();

  Serial.println(F("3-Zone Soil Moisture + Flow Controller starting..."));
  Serial.println(F("Adjust thresholds and pulsesPerLiter for your hardware as needed."));
}

//////////////////////
// Helpers
//////////////////////
float litersFromPulses(unsigned long pulses) {
  return pulses / pulsesPerLiter;
}

bool ensureFlowWhenOpening(int zoneIndex) {
  // Open valve, wait for pulses for up to FLOW_TIMEOUT_MS
  // Caller should have already set valve pin HIGH.
  unsigned long start = millis();
  unsigned long seenAtStart = pulseCount;
  while (millis() - start < FLOW_TIMEOUT_MS) {
    if (pulseCount > seenAtStart) {
      // Flow detected
      return true;
    }
    delay(100); // small sleep to avoid busy looping
  }
  // No pulses detected within timeout
  return false;
}

void openValve(int i) {
  digitalWrite(valvePin[i], HIGH); // set to HIGH to open (change if your driver is inverted)
  valveIsOpen[i] = true;
}

void closeValve(int i) {
  digitalWrite(valvePin[i], LOW);
  valveIsOpen[i] = false;
}

//////////////////////
// Main loop
//////////////////////
void loop() {
  unsigned long now = millis();

  // Read moisture sensors and update zone state
  for (int i = 0; i < 3; ++i) {
    int raw = analogRead(moisturePin[i]);

    bool previouslyWet = zoneIsWet[i];
    bool nowWet;

    if (SENSOR_WET_HIGH) {
      if (raw >= wetThreshold[i]) {
        nowWet = true;
      } else if (raw <= dryThreshold[i]) {
        nowWet = false;
      } else {
        nowWet = previouslyWet; // in hysteresis band, keep previous
      }
    } else {
      // sensor gives lower value when wet
      if (raw <= wetThreshold[i]) {
        nowWet = true;
      } else if (raw >= dryThreshold[i]) {
        nowWet = false;
      } else {
        nowWet = previouslyWet;
      }
    }

    zoneIsWet[i] = nowWet;

    // 2) Decide valve action
    if (!nowWet) {
      // soil is DRY and wants water. Attempt to open and ensure flow.
      if (!valveIsOpen[i]) {
        openValve(i);
        // check for flow; if no flow, close valve and note error
        bool flowOk = ensureFlowWhenOpening(i);
        if (!flowOk) {
          // Close valve and report
          // closeValve(i);
          Serial.print(F("ERROR: No flow detected when opening valve for zone "));
          Serial.println(i + 1);
        } else {
          Serial.print(F("Valve opened for zone "));
          Serial.print(i + 1);
          Serial.println(F(" (flow OK)"));
        }
      }
    } else {
      // WET -> ensure valve closed
      if (valveIsOpen[i]) {
        closeValve(i);
        Serial.print(F("Valve closed for zone "));
        Serial.println(i + 1);
      }
    }
  }

  // Print status periodically
  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;
    unsigned long pulses = pulseCount;
    float liters = litersFromPulses(pulses);

    // compute flow state: is there movement in last interval
    unsigned long newSnapshot = pulseCount;
    unsigned long pulsesDuringInterval = newSnapshot - lastPulseCountSnapshot;
    lastPulseCountSnapshot = newSnapshot;

    bool flowing = (pulsesDuringInterval > 0);

    Serial.println(F("---- STATUS ----"));
    for (int i = 0; i < 3; ++i) {
      int val = analogRead(moisturePin[i]); // read again so printed values match what you care about
      Serial.print(F("Zone "));
      Serial.print(i + 1);
      Serial.print(F(": raw="));
      Serial.print(val);
      Serial.print(F(" state="));
      Serial.print(zoneIsWet[i] ? F("WET") : F("DRY"));
      Serial.print(F(" valve="));
      Serial.println(valveIsOpen[i] ? F("OPEN") : F("CLOSED"));
    }
    Serial.print(F("Flow pulses total: "));
    Serial.println(pulses);
    Serial.print(F("Litres total: "));
    Serial.println(liters, 3);
    Serial.print(F("Flowing (pulses in last sec): "));
    Serial.println(pulsesDuringInterval);
    Serial.print(F("Flow state: "));
    Serial.println(flowing ? F("FLOWING") : F("STOPPED"));
    Serial.println(F("----------------"));
  }

  // short sleep to allow background tasks; loop runs frequently
  delay(50);
}
