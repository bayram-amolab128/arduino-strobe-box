// ============================================================
// FAN CONTROL WITH TWO MODES + FINE PWM CONTROL
// Arduino UNO R3 / Nano (ATmega328P)
//
// D7 = blinking LED
// D9 = high-resolution Timer1 PWM output to BC547 -> IRF520 -> fan
// A0 = photodiode RPM signal
//
// Serial commands (115200 baud, Newline ending):
//   C          -> Calibration mode
//   N          -> Normal/manual mode
//   16.25      -> Set fan duty to 16.25% in Normal mode
//   D 16.25    -> Same thing
//   OFF        -> Fan off
//   H          -> Help
//
// IMPORTANT:
//   The BC547 stage inverts the PWM.
//   Therefore the Arduino PWM duty is (100% - desired fan duty).
//
// This version uses Timer1 on D9 at ~500 Hz with TOP = 31999.
// That gives ~32000 PWM steps instead of analogWrite()'s 256 steps.
// Nominal duty resolution ~= 100/32000 = 0.003125%.
// ============================================================

const int LED_PIN = 7;
const int PWM_PIN = 9;      // MUST be D9 for Timer1 OC1A
const int RPM_PIN = A0;

// ============================================================
// USER SETTINGS
// ============================================================

float blinksPerSecond = 34.0;

// Calibration sweep
const int DUTY_START = 10;
const int DUTY_END   = 100;
const int DUTY_STEP  = 10;

const unsigned long SETTLE_TIME_MS = 10000;
const unsigned long SAMPLE_TIME_MS = 1000;
const int NUM_OMEGA_SAMPLES = 10;

// Photodiode thresholds using INTERNAL ~1.1 V reference
const int THRESHOLD_HIGH = 135;
const int THRESHOLD_LOW  = 126;

const float PULSES_PER_REV = 1.0;
const unsigned long MIN_PULSE_INTERVAL_US = 5000;

// ============================================================
// HIGH-RESOLUTION TIMER1 PWM SETTINGS
// ============================================================
// Arduino clock = 16 MHz
// Fast PWM, TOP = ICR1, prescaler = 1
// f_PWM = 16e6 / (1 * (TOP + 1))
//       = 16e6 / 32000
//       = 500 Hz

const uint16_t PWM_TOP = 31999;

// ============================================================
// OPERATING MODES
// ============================================================

enum OperatingMode {
  WAITING_FOR_MODE,
  CALIBRATION_MODE,
  NORMAL_MODE
};

OperatingMode operatingMode = WAITING_FOR_MODE;

// ============================================================
// CALIBRATION STATE
// ============================================================

enum SweepState {
  SETTLING,
  MEASURING,
  FINISHED
};

SweepState sweepState = FINISHED;

// ============================================================
// VARIABLES
// ============================================================

unsigned long previousBlinkTime = 0;
bool ledState = LOW;

bool signalHigh = false;
unsigned long pulseCount = 0;
unsigned long lastPulseTimeUs = 0;

float currentDuty = 0.0;
uint16_t currentTimerCount = PWM_TOP;

unsigned long stateStartTime = 0;
unsigned long previousOmegaSampleTime = 0;
unsigned long previousNormalSampleTime = 0;

float omegaSum = 0.0;
int omegaSampleCount = 0;

const int NUM_POINTS =
  ((DUTY_END - DUTY_START) / DUTY_STEP) + 1;

float averageOmegaResults[NUM_POINTS];
float averageRPMResults[NUM_POINTS];
float dutyResults[NUM_POINTS];
uint16_t pwmResults[NUM_POINTS];
int resultIndex = 0;

// Serial command buffer
const int COMMAND_BUFFER_SIZE = 32;
char commandBuffer[COMMAND_BUFFER_SIZE];
int commandLength = 0;

// ============================================================
// LED
// ============================================================

void updateLED() {
  unsigned long now = millis();
  unsigned long halfPeriod =
    (unsigned long)(1000.0 / (2.0 * blinksPerSecond));

  if (now - previousBlinkTime >= halfPeriod) {
    previousBlinkTime = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }
}

// ============================================================
// TIMER1 PWM
// ============================================================

void setupFinePWM() {
  pinMode(PWM_PIN, OUTPUT);

  // Stop/reset Timer1 while configuring.
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  ICR1 = PWM_TOP;
  OCR1A = PWM_TOP;

  // Fast PWM, mode 14: WGM13:0 = 1110
  // TOP = ICR1
  TCCR1A = _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);  // prescaler = 1

  // Start safely with fan OFF.
  // BC547 inversion means Arduino HIGH -> BC547 ON -> MOSFET gate LOW.
  TCCR1A &= ~(_BV(COM1A1) | _BV(COM1A0));
  digitalWrite(PWM_PIN, HIGH);
}

uint16_t dutyToTimerCount(float fanDutyPercent) {
  if (fanDutyPercent <= 0.0) return PWM_TOP;
  if (fanDutyPercent >= 100.0) return 0;

  float fanFraction = fanDutyPercent / 100.0;

  // BC547 inverts the signal, so Arduino HIGH fraction is 1 - fanFraction.
  float arduinoHighFraction = 1.0 - fanFraction;

  long count = lround((double)PWM_TOP * arduinoHighFraction);

  if (count < 0) count = 0;
  if (count > PWM_TOP) count = PWM_TOP;

  return (uint16_t)count;
}

void setFanDuty(float fanDutyPercent) {
  if (fanDutyPercent < 0.0) fanDutyPercent = 0.0;
  if (fanDutyPercent > 100.0) fanDutyPercent = 100.0;

  currentDuty = fanDutyPercent;
  currentTimerCount = dutyToTimerCount(currentDuty);

  // Handle the two endpoints as true DC levels so there is no tiny PWM pulse.
  if (currentDuty <= 0.0) {
    // Fan OFF: Arduino HIGH constantly -> BC547 pulls MOSFET gate LOW.
    TCCR1A &= ~(_BV(COM1A1) | _BV(COM1A0));
    digitalWrite(PWM_PIN, HIGH);
  }
  else if (currentDuty >= 100.0) {
    // Fan FULL ON: Arduino LOW constantly -> BC547 OFF -> gate pulled to +10 V.
    TCCR1A &= ~(_BV(COM1A1) | _BV(COM1A0));
    digitalWrite(PWM_PIN, LOW);
  }
  else {
    OCR1A = currentTimerCount;

    // Non-inverting OC1A output on D9.
    // BC547 then performs the required hardware inversion.
    TCCR1A &= ~_BV(COM1A0);
    TCCR1A |= _BV(COM1A1);
  }

  float effectiveDuty;
  if (currentDuty <= 0.0) {
    effectiveDuty = 0.0;
  }
  else if (currentDuty >= 100.0) {
    effectiveDuty = 100.0;
  }
  else {
    effectiveDuty = 100.0 *
      (1.0 - ((float)currentTimerCount / (float)PWM_TOP));
  }

  Serial.print("Requested fan duty = ");
  Serial.print(currentDuty, 3);
  Serial.print(" %   Timer1 count = ");
  Serial.print(currentTimerCount);
  Serial.print("   Effective duty ~= ");
  Serial.print(effectiveDuty, 4);
  Serial.println(" %");
}

void fanOff() {
  setFanDuty(0.0);
}

// ============================================================
// PHOTODIODE / RPM
// ============================================================

void samplePhotodiode() {
  int sensorValue = analogRead(RPM_PIN);

  if (!signalHigh && sensorValue >= THRESHOLD_HIGH) {
    unsigned long nowUs = micros();

    if (nowUs - lastPulseTimeUs >= MIN_PULSE_INTERVAL_US) {
      bool shouldCount =
        (operatingMode == NORMAL_MODE) ||
        (operatingMode == CALIBRATION_MODE && sweepState == MEASURING);

      if (shouldCount) {
        pulseCount++;
      }

      lastPulseTimeUs = nowUs;
    }

    signalHigh = true;
  }

  if (signalHigh && sensorValue <= THRESHOLD_LOW) {
    signalHigh = false;
  }
}

float calculateRPM(unsigned long pulses, unsigned long elapsedMs) {
  if (elapsedMs == 0) return 0.0;

  float revolutions = pulses / PULSES_PER_REV;
  return revolutions * (60000.0 / elapsedMs);
}

float rpmToOmega(float rpm) {
  return rpm * 2.0 * PI / 60.0;
}

// ============================================================
// NORMAL MODE
// ============================================================

void startNormalMode() {
  operatingMode = NORMAL_MODE;
  sweepState = FINISHED;

  pulseCount = 0;
  previousNormalSampleTime = millis();

  Serial.println();
  Serial.println("======================================");
  Serial.println("NORMAL FAN CONTROL MODE");
  Serial.println("======================================");
  Serial.println("Enter decimal fan duty from 0 to 100.");
  Serial.println("Examples: 16.25   16.5   17.125   D 42.75");
  Serial.println("C = calibration mode");
  Serial.println("OFF = fan off");
  Serial.println();

  fanOff();
}

void updateNormalMode() {
  unsigned long now = millis();

  if (now - previousNormalSampleTime >= SAMPLE_TIME_MS) {
    unsigned long elapsed = now - previousNormalSampleTime;
    previousNormalSampleTime = now;

    unsigned long pulses = pulseCount;
    pulseCount = 0;

    float rpm = calculateRPM(pulses, elapsed);
    float omega = rpmToOmega(rpm);

    Serial.print("Duty = ");
    Serial.print(currentDuty, 3);
    Serial.print(" %   RPM = ");
    Serial.print(rpm, 1);
    Serial.print("   omega = ");
    Serial.print(omega, 3);
    Serial.println(" rad/s");
  }
}

// ============================================================
// CALIBRATION MODE
// ============================================================

void startCalibrationMode() {
  operatingMode = CALIBRATION_MODE;

  resultIndex = 0;
  currentDuty = (float)DUTY_START;
  pulseCount = 0;
  omegaSum = 0.0;
  omegaSampleCount = 0;

  Serial.println();
  Serial.println("======================================");
  Serial.println("CALIBRATION MODE");
  Serial.println("======================================");
  Serial.println("Sweep: 10% to 100% in 10% steps");
  Serial.println("Each point: 10 s settle + 10 s measure");
  Serial.println("Enter N at any time to abort to Normal mode.");
  Serial.println();

  setFanDuty(currentDuty);
  Serial.println("Stabilizing for 10 seconds...");

  stateStartTime = millis();
  sweepState = SETTLING;
}

void startMeasurement() {
  pulseCount = 0;
  omegaSum = 0.0;
  omegaSampleCount = 0;

  previousOmegaSampleTime = millis();
  sweepState = MEASURING;

  Serial.println("Starting 10 second measurement...");
}

void takeOmegaSample() {
  unsigned long now = millis();
  unsigned long elapsed = now - previousOmegaSampleTime;
  previousOmegaSampleTime = now;

  unsigned long pulses = pulseCount;
  pulseCount = 0;

  float rpm = calculateRPM(pulses, elapsed);
  float omega = rpmToOmega(rpm);

  omegaSum += omega;
  omegaSampleCount++;

  Serial.print("Sample ");
  Serial.print(omegaSampleCount);
  Serial.print("/");
  Serial.print(NUM_OMEGA_SAMPLES);
  Serial.print("   RPM = ");
  Serial.print(rpm, 1);
  Serial.print("   omega = ");
  Serial.print(omega, 3);
  Serial.println(" rad/s");
}

void printFinalResults() {
  Serial.println();
  Serial.println("============================================================");
  Serial.println("FINAL FAN PWM -> ANGULAR SPEED CALIBRATION");
  Serial.println("============================================================");
  Serial.println("FanDutyPercent,Timer1Count,AverageRPM,AverageOmega_rad_s");

  for (int i = 0; i < resultIndex; i++) {
    Serial.print(dutyResults[i], 3);
    Serial.print(",");
    Serial.print(pwmResults[i]);
    Serial.print(",");
    Serial.print(averageRPMResults[i], 2);
    Serial.print(",");
    Serial.println(averageOmegaResults[i], 3);
  }

  Serial.println("============================================================");
}

void finishMeasurement() {
  if (omegaSampleCount == 0) return;

  float averageOmega = omegaSum / omegaSampleCount;
  float averageRPM = averageOmega * 60.0 / (2.0 * PI);
  uint16_t timerCount = dutyToTimerCount(currentDuty);

  dutyResults[resultIndex] = currentDuty;
  pwmResults[resultIndex] = timerCount;
  averageOmegaResults[resultIndex] = averageOmega;
  averageRPMResults[resultIndex] = averageRPM;
  resultIndex++;

  Serial.println();
  Serial.println("--------------------------------------");
  Serial.print("Fan duty = ");
  Serial.print(currentDuty, 3);
  Serial.println(" %");
  Serial.print("AVERAGE RPM = ");
  Serial.println(averageRPM, 2);
  Serial.print("AVERAGE omega = ");
  Serial.print(averageOmega, 3);
  Serial.println(" rad/s");
  Serial.println("--------------------------------------");

  if (currentDuty + DUTY_STEP <= DUTY_END + 0.001) {
    currentDuty += DUTY_STEP;
    setFanDuty(currentDuty);
    Serial.println("Stabilizing for 10 seconds...");

    stateStartTime = millis();
    sweepState = SETTLING;
  }
  else {
    sweepState = FINISHED;
    printFinalResults();
    fanOff();

    Serial.println();
    Serial.println("Calibration complete. Fan OFF.");
    Serial.println("Enter N for Normal mode or C to calibrate again.");
  }
}

void updateCalibrationMode() {
  unsigned long now = millis();

  if (sweepState == SETTLING) {
    if (now - stateStartTime >= SETTLE_TIME_MS) {
      startMeasurement();
    }
  }
  else if (sweepState == MEASURING) {
    if (now - previousOmegaSampleTime >= SAMPLE_TIME_MS) {
      takeOmegaSample();

      if (omegaSampleCount >= NUM_OMEGA_SAMPLES) {
        finishMeasurement();
      }
    }
  }
}

// ============================================================
// SERIAL COMMANDS
// ============================================================

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  C          -> Calibration mode");
  Serial.println("  N          -> Normal/manual mode");
  Serial.println("  16.25      -> Set fan duty to 16.25% in Normal mode");
  Serial.println("  D 16.25    -> Same thing");
  Serial.println("  OFF        -> Fan off");
  Serial.println("  H          -> Show this help");
  Serial.println();
}

void processCommand(char *cmd) {
  while (*cmd == ' ' || *cmd == '\t') cmd++;

  int len = strlen(cmd);
  while (len > 0 && (cmd[len - 1] == ' ' || cmd[len - 1] == '\t')) {
    cmd[len - 1] = '\0';
    len--;
  }

  if (len == 0) return;

  for (int i = 0; cmd[i] != '\0'; i++) {
    if (cmd[i] >= 'a' && cmd[i] <= 'z') {
      cmd[i] = cmd[i] - 'a' + 'A';
    }
  }

  if (strcmp(cmd, "C") == 0 ||
      strcmp(cmd, "CAL") == 0 ||
      strcmp(cmd, "CALIBRATION") == 0) {
    startCalibrationMode();
    return;
  }

  if (strcmp(cmd, "N") == 0 ||
      strcmp(cmd, "NORMAL") == 0) {
    startNormalMode();
    return;
  }

  if (strcmp(cmd, "H") == 0 ||
      strcmp(cmd, "HELP") == 0) {
    printHelp();
    return;
  }

  if (strcmp(cmd, "OFF") == 0) {
    if (operatingMode == NORMAL_MODE) {
      fanOff();
    }
    else {
      Serial.println("Switch to Normal mode first with N.");
    }
    return;
  }

  if (operatingMode != NORMAL_MODE) {
    Serial.println("Manual duty control is available only in Normal mode.");
    Serial.println("Enter N first.");
    return;
  }

  char *numberStart = cmd;

  // Also accept: D 16.25 or DUTY 16.25
  if (cmd[0] == 'D') {
    char *space = strchr(cmd, ' ');
    if (space == NULL) {
      Serial.println("Use D <0..100>, for example D 16.25");
      return;
    }
    numberStart = space + 1;
  }

  char *endPtr;
  double duty = strtod(numberStart, &endPtr);

  while (*endPtr == ' ' || *endPtr == '\t') endPtr++;

  if (*endPtr != '\0' || duty < 0.0 || duty > 100.0) {
    Serial.println("Invalid duty. Enter a decimal number from 0 to 100.");
    return;
  }

  setFanDuty((float)duty);
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      commandBuffer[commandLength] = '\0';
      processCommand(commandBuffer);
      commandLength = 0;
    }
    else if (commandLength < COMMAND_BUFFER_SIZE - 1) {
      commandBuffer[commandLength++] = c;
    }
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  pinMode(LED_PIN, OUTPUT);

  setupFinePWM();

  analogReference(INTERNAL);
  Serial.begin(115200);

  delay(100);

  // Throw away first ADC readings after switching reference.
  analogRead(RPM_PIN);
  analogRead(RPM_PIN);
  analogRead(RPM_PIN);

  Serial.println();
  Serial.println("Fine-resolution Fan Controller");
  Serial.println("Timer1 PWM on D9, approximately 500 Hz");
  Serial.println("Nominal PWM resolution approximately 0.003125%");
  Serial.println("ADC reference = INTERNAL approximately 1.1 V");
  Serial.println();

  printHelp();
  Serial.println("Select mode: C = Calibration, N = Normal");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  handleSerial();
  samplePhotodiode();
  updateLED();

  if (operatingMode == NORMAL_MODE) {
    updateNormalMode();
  }
  else if (operatingMode == CALIBRATION_MODE) {
    updateCalibrationMode();
  }
}
