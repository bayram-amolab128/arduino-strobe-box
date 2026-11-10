// ============================================================
// FAN CONTROL WITH POTENTIOMETER + STARTUP KICK
// Arduino UNO R3 / Nano (ATmega328P)
//
// D7 = blinking LED
// D9 = high-resolution Timer1 PWM output to BC547 -> IRF520 -> fan
// A0 = photodiode RPM signal
// A1 = potentiometer wiper (0-5 V)
//
// Behavior:
//   - Pot controls requested fan duty from 0% to 100%.
//   - Whenever the fan starts from OFF, it runs at 50% duty
//     for 3 seconds, then drops to the potentiometer-requested duty.
//   - If the pot is turned back to 0, the fan turns OFF and the
//     startup kick is re-armed for the next start.
//
// IMPORTANT:
//   The BC547 stage inverts the PWM.
//   Therefore the Arduino PWM duty is (100% - desired fan duty).
//
// Timer1:
//   D9 = OC1A
//   Fast PWM at ~500 Hz
//   TOP = 31999
//   Resolution ~= 100/32000 = 0.003125%
// ============================================================

const int LED_PIN = 7;
const int PWM_PIN = 9;
const int RPM_PIN = A0;
const int POT_PIN = A1;

// USER SETTINGS
float blinksPerSecond = 34.0;

const float STARTUP_DUTY = 50.0;
const unsigned long STARTUP_TIME_MS = 3000;

const float ADC_REFERENCE_V = 5.0;
const int POT_OFF_ADC = 5;
const unsigned long POT_SAMPLE_TIME_MS = 20;

const unsigned long RPM_SAMPLE_TIME_MS = 1000;

// Photodiode thresholds recalculated for DEFAULT ~5 V ADC reference:
// 145 mV -> ~30 counts, 135 mV -> ~28 counts.
const int THRESHOLD_HIGH = 30;
const int THRESHOLD_LOW  = 28;

// Two optical crossings per revolution.
const float PULSES_PER_REV = 2.0;

const unsigned long MIN_PULSE_INTERVAL_US = 5000;

// TIMER1 PWM
const uint16_t PWM_TOP = 31999;

// LED
unsigned long previousBlinkTime = 0;
bool ledState = LOW;

// RPM
bool signalHigh = false;
unsigned long pulseCount = 0;
unsigned long lastPulseTimeUs = 0;
unsigned long previousRPMSampleTime = 0;

// Pot
unsigned long previousPotSampleTime = 0;
int potADC = 0;
float potVoltage = 0.0;
float requestedDuty = 0.0;

// Fan output
float currentDuty = 0.0;
uint16_t currentTimerCount = PWM_TOP;

enum FanState {
  FAN_STOPPED,
  FAN_STARTUP,
  FAN_RUNNING
};

FanState fanState = FAN_STOPPED;
unsigned long startupStartTime = 0;

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

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  ICR1 = PWM_TOP;
  OCR1A = PWM_TOP;

  // Fast PWM mode 14, TOP = ICR1, prescaler = 1
  TCCR1A = _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);

  // Fan OFF:
  // Arduino HIGH -> BC547 ON -> MOSFET gate LOW.
  TCCR1A &= ~(_BV(COM1A1) | _BV(COM1A0));
  digitalWrite(PWM_PIN, HIGH);
}

uint16_t dutyToTimerCount(float fanDutyPercent) {
  if (fanDutyPercent <= 0.0) return PWM_TOP;
  if (fanDutyPercent >= 100.0) return 0;

  float fanFraction = fanDutyPercent / 100.0;
  float arduinoHighFraction = 1.0 - fanFraction;

  long count = lround((double)PWM_TOP * arduinoHighFraction);

  if (count < 0) count = 0;
  if (count > PWM_TOP) count = PWM_TOP;

  return (uint16_t)count;
}

void setFanDuty(float fanDutyPercent) {
  fanDutyPercent = constrain(fanDutyPercent, 0.0, 100.0);

  currentDuty = fanDutyPercent;
  currentTimerCount = dutyToTimerCount(currentDuty);

  if (currentDuty <= 0.0) {
    TCCR1A &= ~(_BV(COM1A1) | _BV(COM1A0));
    digitalWrite(PWM_PIN, HIGH);
  }
  else if (currentDuty >= 100.0) {
    TCCR1A &= ~(_BV(COM1A1) | _BV(COM1A0));
    digitalWrite(PWM_PIN, LOW);
  }
  else {
    OCR1A = currentTimerCount;
    TCCR1A &= ~_BV(COM1A0);
    TCCR1A |= _BV(COM1A1);
  }
}

void fanOff() {
  setFanDuty(0.0);
  fanState = FAN_STOPPED;
}

// ============================================================
// POTENTIOMETER CONTROL
// ============================================================

void updatePotentiometerControl() {
  unsigned long now = millis();

  if (now - previousPotSampleTime < POT_SAMPLE_TIME_MS) {
    return;
  }

  previousPotSampleTime = now;

  potADC = analogRead(POT_PIN);

  potVoltage =
    ((float)potADC / 1023.0) * ADC_REFERENCE_V;

  if (potADC <= POT_OFF_ADC) {
    requestedDuty = 0.0;
  }
  else {
    requestedDuty =
      ((float)potADC / 1023.0) * 100.0;
  }

  // Pot at zero -> fan OFF
  if (requestedDuty <= 0.0) {
    if (fanState != FAN_STOPPED) {
      fanOff();
      Serial.println("Fan OFF. Startup kick re-armed.");
    }
    return;
  }

  // Start from OFF -> 50% for 3 s
  if (fanState == FAN_STOPPED) {
    fanState = FAN_STARTUP;
    startupStartTime = now;

    setFanDuty(STARTUP_DUTY);

    Serial.println();
    Serial.print("Startup kick: ");
    Serial.print(STARTUP_DUTY, 1);
    Serial.print("% for ");
    Serial.print(STARTUP_TIME_MS / 1000.0, 1);
    Serial.println(" s");
    return;
  }

  // Hold startup duty; after 3 s apply latest pot command.
  if (fanState == FAN_STARTUP) {
    if (now - startupStartTime >= STARTUP_TIME_MS) {
      fanState = FAN_RUNNING;
      setFanDuty(requestedDuty);

      Serial.print("Startup complete. Pot duty = ");
      Serial.print(requestedDuty, 2);
      Serial.println(" %");
    }
    return;
  }

  // Normal running
  if (fanState == FAN_RUNNING) {
    if (fabs(requestedDuty - currentDuty) >= 0.02) {
      setFanDuty(requestedDuty);
    }
  }
}

// ============================================================
// PHOTODIODE / RPM
// ============================================================

void samplePhotodiode() {
  int sensorValue = analogRead(RPM_PIN);

  if (!signalHigh && sensorValue >= THRESHOLD_HIGH) {
    unsigned long nowUs = micros();

    if (nowUs - lastPulseTimeUs >= MIN_PULSE_INTERVAL_US) {
      pulseCount++;
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

void updateRPMReport() {
  unsigned long now = millis();

  if (now - previousRPMSampleTime < RPM_SAMPLE_TIME_MS) {
    return;
  }

  unsigned long elapsed = now - previousRPMSampleTime;
  previousRPMSampleTime = now;

  unsigned long pulses = pulseCount;
  pulseCount = 0;

  float rpm = calculateRPM(pulses, elapsed);
  float rps = rpm / 60.0;
  float omega = rpmToOmega(rpm);

  Serial.print("Pot = ");
  Serial.print(potVoltage, 3);
  Serial.print(" V");

  Serial.print("   Requested = ");
  Serial.print(requestedDuty, 2);
  Serial.print(" %");

  Serial.print("   Applied = ");
  Serial.print(currentDuty, 2);
  Serial.print(" %");

  if (fanState == FAN_STARTUP) {
    Serial.print("   [STARTUP]");
  }
  else if (fanState == FAN_STOPPED) {
    Serial.print("   [OFF]");
  }
  else {
    Serial.print("   [RUNNING]");
  }

  Serial.print("   RPS = ");
  Serial.print(rps, 2);

  Serial.print("   RPM = ");
  Serial.print(rpm, 1);

  Serial.print("   omega = ");
  Serial.print(omega, 2);
  Serial.println(" rad/s");
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);

  setupFinePWM();

  // Pot is 0-5 V, so use AVcc (~5 V) as ADC reference.
  analogReference(DEFAULT);

  Serial.begin(115200);
  delay(100);

  // Throw away initial ADC conversions.
  analogRead(RPM_PIN);
  analogRead(POT_PIN);
  analogRead(RPM_PIN);
  analogRead(POT_PIN);

  fanOff();

  previousRPMSampleTime = millis();
  previousPotSampleTime = millis();

  Serial.println();
  Serial.println("==============================================");
  Serial.println("POTENTIOMETER FAN CONTROLLER");
  Serial.println("==============================================");
  Serial.println("D9 = fine PWM -> BC547 -> IRF520 -> fan");
  Serial.println("A0 = photodiode RPM input");
  Serial.println("A1 = potentiometer wiper, 0-5 V");
  Serial.println("Pot 0-5 V maps to fan duty 0-100%");
  Serial.println("Startup kick = 50% for 3 seconds");
  Serial.println("Timer1 PWM ~= 500 Hz");
  Serial.println();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  samplePhotodiode();
  updatePotentiometerControl();
  updateRPMReport();
  updateLED();
}
