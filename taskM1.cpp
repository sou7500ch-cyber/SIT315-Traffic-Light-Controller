/*
 * =====================================================================
 *  Arduino Uno - Traffic Light Controller with FSM
 *  Pin Change Interrupts (PCINT0 group) + Timer1 CTC Interrupt
 * =====================================================================
 *
 *  Hardware:
 *   D2  - North Green
 *   D3  - North Yellow
 *   D4  - North Red
 *   D5  - East Green
 *   D6  - East Yellow
 *   D7  - East Red
 *   D8  - Pedestrian LED
 *   D9  - Vehicle Button   (INPUT_PULLUP, active LOW)
 *   D10 - Emergency Button (INPUT_PULLUP, active LOW)
 *   D11 - Pedestrian Button(INPUT_PULLUP, active LOW)
 *   D12 - Buzzer
 *
 *  Timing is derived from a software tick counter incremented every
 *  1 ms by Timer1 (CTC mode). millis() and attachInterrupt() are not
 *  used anywhere in this program. delay() is used only inside the
 *  startup self-test routine.
 * =====================================================================
 */

#include <avr/io.h>
#include <avr/interrupt.h>

// ---------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------
const byte PIN_NORTH_GREEN   = 2;
const byte PIN_NORTH_YELLOW  = 3;
const byte PIN_NORTH_RED     = 4;
const byte PIN_EAST_GREEN    = 5;
const byte PIN_EAST_YELLOW   = 6;
const byte PIN_EAST_RED      = 7;
const byte PIN_PEDESTRIAN_LED= 8;
const byte PIN_VEHICLE_BTN   = 9;
const byte PIN_EMERGENCY_BTN = 10;
const byte PIN_PEDESTRIAN_BTN= 11;
const byte PIN_BUZZER        = 12;

// ---------------------------------------------------------------------
// FSM states (const byte instead of enum)
// ---------------------------------------------------------------------
const byte STATE_NS_GREEN   = 0;
const byte STATE_NS_YELLOW  = 1;
const byte STATE_EW_GREEN   = 2;
const byte STATE_EW_YELLOW  = 3;
const byte STATE_PEDESTRIAN = 4;
const byte STATE_EMERGENCY  = 5;

// ---------------------------------------------------------------------
// Timing constants (all in milliseconds, using our own tick counter)
// ---------------------------------------------------------------------
const unsigned long GREEN_DURATION      = 5000UL;
const unsigned long YELLOW_DURATION     = 2000UL;
const unsigned long PEDESTRIAN_DURATION = 5000UL;
const unsigned long EMERGENCY_DURATION  = 5000UL;
const unsigned long VEHICLE_EXTENSION   = 2000UL;
const unsigned long DEBOUNCE_MS         = 50UL;

// ---------------------------------------------------------------------
// Volatile globals shared with ISRs
// ---------------------------------------------------------------------
volatile unsigned long tickCounter = 0;     // incremented every 1 ms by Timer1
volatile byte lastPINB = 0;                 // previous state of PORTB for edge detection

volatile bool vehicleRequest    = false;
volatile bool emergencyRequest  = false;
volatile bool pedestrianRequest = false;

volatile unsigned long lastVehicleInterrupt    = 0;
volatile unsigned long lastEmergencyInterrupt  = 0;
volatile unsigned long lastPedestrianInterrupt = 0;

// ---------------------------------------------------------------------
// FSM control variables (main-loop only, not touched by ISRs)
// ---------------------------------------------------------------------
byte currentState;
byte savedState;          // state to resume after emergency/pedestrian phase
unsigned long stateStartTick;
unsigned long stateDuration;

// =====================================================================
// Helper: atomic read of tickCounter
// =====================================================================
unsigned long getTicks() {
  unsigned long t;
  noInterrupts();
  t = tickCounter;
  interrupts();
  return t;
}

// =====================================================================
// Pin setup
// =====================================================================
void setupPins() {
  pinMode(PIN_NORTH_GREEN, OUTPUT);
  pinMode(PIN_NORTH_YELLOW, OUTPUT);
  pinMode(PIN_NORTH_RED, OUTPUT);
  pinMode(PIN_EAST_GREEN, OUTPUT);
  pinMode(PIN_EAST_YELLOW, OUTPUT);
  pinMode(PIN_EAST_RED, OUTPUT);
  pinMode(PIN_PEDESTRIAN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  pinMode(PIN_VEHICLE_BTN, INPUT_PULLUP);
  pinMode(PIN_EMERGENCY_BTN, INPUT_PULLUP);
  pinMode(PIN_PEDESTRIAN_BTN, INPUT_PULLUP);

  digitalWrite(PIN_NORTH_GREEN, LOW);
  digitalWrite(PIN_NORTH_YELLOW, LOW);
  digitalWrite(PIN_NORTH_RED, LOW);
  digitalWrite(PIN_EAST_GREEN, LOW);
  digitalWrite(PIN_EAST_YELLOW, LOW);
  digitalWrite(PIN_EAST_RED, LOW);
  digitalWrite(PIN_PEDESTRIAN_LED, LOW);
  digitalWrite(PIN_BUZZER, LOW);
}

// =====================================================================
// Pin Change Interrupt setup (PCINT0 group covers D8-D13 / PORTB)
// =====================================================================
void setupPCI() {
  noInterrupts();

  // Record initial button pin states so the first ISR call has a
  // valid reference for edge detection.
  lastPINB = PINB;

  PCICR  |= (1 << PCIE0);   // Enable PCINT0..PCINT7 interrupt group
  PCMSK0 |= (1 << PCINT1);  // D9  - Vehicle button
  PCMSK0 |= (1 << PCINT2);  // D10 - Emergency button
  PCMSK0 |= (1 << PCINT3);  // D11 - Pedestrian button

  interrupts();
}

// =====================================================================
// Timer1 setup: CTC mode, 1 ms tick, prescaler 64
// 16,000,000 / 64 = 250,000 counts/sec -> 250 counts = 1 ms
// OCR1A = 250 - 1 = 249
// =====================================================================
void setupTimer1() {
  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 249;

  TCCR1B |= (1 << WGM12);              // CTC mode (Clear Timer on Compare Match)
  TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler = 64

  TIMSK1 |= (1 << OCIE1A);             // Enable Timer1 Compare A interrupt

  interrupts();
}

// =====================================================================
// Timer1 Compare Match ISR - our software time base (replaces millis())
// =====================================================================
ISR(TIMER1_COMPA_vect) {
  tickCounter++;
}

// =====================================================================
// Pin Change Interrupt ISR - handles Vehicle, Emergency, Pedestrian buttons
// =====================================================================
ISR(PCINT0_vect) {
  byte current = PINB;
  byte changed = current ^ lastPINB;

  // D9 = PB1 = PCINT1 -> Vehicle button
  if (changed & (1 << PB1)) {
    if (!(current & (1 << PB1))) { // falling edge = button pressed (active LOW)
      if ((tickCounter - lastVehicleInterrupt) > DEBOUNCE_MS) {
        vehicleRequest = true;
        lastVehicleInterrupt = tickCounter;
      }
    }
  }

  // D10 = PB2 = PCINT2 -> Emergency button
  if (changed & (1 << PB2)) {
    if (!(current & (1 << PB2))) {
      if ((tickCounter - lastEmergencyInterrupt) > DEBOUNCE_MS) {
        emergencyRequest = true;
        lastEmergencyInterrupt = tickCounter;
      }
    }
  }

  // D11 = PB3 = PCINT3 -> Pedestrian button
  if (changed & (1 << PB3)) {
    if (!(current & (1 << PB3))) {
      if ((tickCounter - lastPedestrianInterrupt) > DEBOUNCE_MS) {
        pedestrianRequest = true;
        lastPedestrianInterrupt = tickCounter;
      }
    }
  }

  lastPINB = current;
}

// =====================================================================
// Output helper - sets the six traffic LEDs in one call
// =====================================================================
void setLights(bool ng, bool ny, bool nr, bool eg, bool ey, bool er) {
  digitalWrite(PIN_NORTH_GREEN, ng ? HIGH : LOW);
  digitalWrite(PIN_NORTH_YELLOW, ny ? HIGH : LOW);
  digitalWrite(PIN_NORTH_RED, nr ? HIGH : LOW);
  digitalWrite(PIN_EAST_GREEN, eg ? HIGH : LOW);
  digitalWrite(PIN_EAST_YELLOW, ey ? HIGH : LOW);
  digitalWrite(PIN_EAST_RED, er ? HIGH : LOW);
}

// =====================================================================
// Sets outputs and duration for a given FSM state
// =====================================================================
void updateOutputsForState(byte state) {
  switch (state) {
    case STATE_NS_GREEN:
      setLights(true, false, false, false, false, true);
      digitalWrite(PIN_PEDESTRIAN_LED, LOW);
      digitalWrite(PIN_BUZZER, LOW);
      stateDuration = GREEN_DURATION;
      break;

    case STATE_NS_YELLOW:
      setLights(false, true, false, false, false, true);
      digitalWrite(PIN_PEDESTRIAN_LED, LOW);
      digitalWrite(PIN_BUZZER, LOW);
      stateDuration = YELLOW_DURATION;
      break;

    case STATE_EW_GREEN:
      setLights(false, false, true, true, false, false);
      digitalWrite(PIN_PEDESTRIAN_LED, LOW);
      digitalWrite(PIN_BUZZER, LOW);
      stateDuration = GREEN_DURATION;
      break;

    case STATE_EW_YELLOW:
      setLights(false, false, true, false, true, false);
      digitalWrite(PIN_PEDESTRIAN_LED, LOW);
      digitalWrite(PIN_BUZZER, LOW);
      stateDuration = YELLOW_DURATION;
      break;

    case STATE_PEDESTRIAN:
      setLights(false, false, true, false, false, true); // both roads red
      digitalWrite(PIN_PEDESTRIAN_LED, HIGH);
      digitalWrite(PIN_BUZZER, HIGH);
      stateDuration = PEDESTRIAN_DURATION;
      break;

    case STATE_EMERGENCY:
      // Emergency direction chosen as North; East held at red.
      setLights(true, false, false, false, false, true);
      digitalWrite(PIN_PEDESTRIAN_LED, LOW);
      digitalWrite(PIN_BUZZER, HIGH);
      stateDuration = EMERGENCY_DURATION;
      break;

    default:
      setLights(true, false, false, false, false, true);
      digitalWrite(PIN_PEDESTRIAN_LED, LOW);
      digitalWrite(PIN_BUZZER, LOW);
      stateDuration = GREEN_DURATION;
      break;
  }
}

// =====================================================================
// Serial status message per state
// =====================================================================
void printStateMessage(byte state) {
  switch (state) {
    case STATE_NS_GREEN:
      Serial.println(F("[FSM] North Green - South/North traffic flowing"));
      break;
    case STATE_NS_YELLOW:
      Serial.println(F("[FSM] North Yellow - prepare to stop"));
      break;
    case STATE_EW_GREEN:
      Serial.println(F("[FSM] East Green - East traffic flowing"));
      break;
    case STATE_EW_YELLOW:
      Serial.println(F("[FSM] East Yellow - prepare to stop"));
      break;
    case STATE_PEDESTRIAN:
      Serial.println(F("[FSM] Pedestrian crossing active - both roads RED"));
      break;
    case STATE_EMERGENCY:
      Serial.println(F("[FSM] EMERGENCY MODE - priority vehicle passage"));
      break;
    default:
      Serial.println(F("[FSM] Unknown state"));
      break;
  }
}

// =====================================================================
// Transitions the FSM into a new state
// =====================================================================
void enterState(byte newState) {
  currentState = newState;
  updateOutputsForState(newState);
  stateStartTick = getTicks();
  printStateMessage(newState);
}

// =====================================================================
// Advances the normal traffic-light sequence
// =====================================================================
void advanceTrafficState() {
  switch (currentState) {
    case STATE_NS_GREEN:
      enterState(STATE_NS_YELLOW);
      break;
    case STATE_NS_YELLOW:
      enterState(STATE_EW_GREEN);
      break;
    case STATE_EW_GREEN:
      enterState(STATE_EW_YELLOW);
      break;
    case STATE_EW_YELLOW:
      enterState(STATE_NS_GREEN);
      break;
    default:
      enterState(STATE_NS_GREEN);
      break;
  }
}

// =====================================================================
// Startup self-test: LED test, buzzer test, serial messages.
// delay() is ONLY used here, per requirements.
// =====================================================================
void runStartupSelfTest() {
  Serial.println(F("===================================="));
  Serial.println(F(" Traffic Light Controller - Booting  "));
  Serial.println(F("===================================="));

  Serial.println(F("[SELF-TEST] Testing North lights..."));
  digitalWrite(PIN_NORTH_GREEN, HIGH);  delay(300); digitalWrite(PIN_NORTH_GREEN, LOW);
  digitalWrite(PIN_NORTH_YELLOW, HIGH); delay(300); digitalWrite(PIN_NORTH_YELLOW, LOW);
  digitalWrite(PIN_NORTH_RED, HIGH);    delay(300); digitalWrite(PIN_NORTH_RED, LOW);

  Serial.println(F("[SELF-TEST] Testing East lights..."));
  digitalWrite(PIN_EAST_GREEN, HIGH);  delay(300); digitalWrite(PIN_EAST_GREEN, LOW);
  digitalWrite(PIN_EAST_YELLOW, HIGH); delay(300); digitalWrite(PIN_EAST_YELLOW, LOW);
  digitalWrite(PIN_EAST_RED, HIGH);    delay(300); digitalWrite(PIN_EAST_RED, LOW);

  Serial.println(F("[SELF-TEST] Testing Pedestrian LED..."));
  digitalWrite(PIN_PEDESTRIAN_LED, HIGH); delay(300); digitalWrite(PIN_PEDESTRIAN_LED, LOW);

  Serial.println(F("[SELF-TEST] Testing Buzzer..."));
  digitalWrite(PIN_BUZZER, HIGH); delay(300); digitalWrite(PIN_BUZZER, LOW);

  Serial.println(F("[SELF-TEST] Complete. Starting FSM..."));
}

// =====================================================================
// Handles the emergency mode (highest priority)
// Returns true if emergency mode is currently active (blocks other logic)
// =====================================================================
bool handleEmergency() {
  if (emergencyRequest) {
    emergencyRequest = false;
    if (currentState != STATE_EMERGENCY) {
      savedState = currentState; // remember where to resume
      enterState(STATE_EMERGENCY);
    }
  }

  if (currentState == STATE_EMERGENCY) {
    unsigned long now = getTicks();
    if (now - stateStartTick >= stateDuration) {
      Serial.println(F("[FSM] Emergency cleared - resuming normal operation"));
      enterState(savedState);
    }
    return true; // still (or was) in emergency this cycle
  }

  return false;
}

// =====================================================================
// Handles the pedestrian crossing mode (second priority)
// Returns true if pedestrian mode is currently active
// =====================================================================
bool handlePedestrian() {
  if (pedestrianRequest) {
    pedestrianRequest = false;
    if (currentState != STATE_PEDESTRIAN) {
      savedState = currentState; // remember where to resume
      enterState(STATE_PEDESTRIAN);
    }
  }

  if (currentState == STATE_PEDESTRIAN) {
    unsigned long now = getTicks();
    if (now - stateStartTick >= stateDuration) {
      Serial.println(F("[FSM] Pedestrian phase complete - resuming traffic"));
      enterState(savedState);
    }
    return true;
  }

  return false;
}

// =====================================================================
// Handles the vehicle button green-extension request
// =====================================================================
void handleVehicleRequest() {
  if (vehicleRequest) {
    vehicleRequest = false;
    if (currentState == STATE_NS_GREEN || currentState == STATE_EW_GREEN) {
      stateDuration += VEHICLE_EXTENSION;
      Serial.println(F("[FSM] Vehicle detected - extending green phase by 2s"));
    }
  }
}

// =====================================================================
// Handles the normal traffic sequence timing/advancement
// =====================================================================
void handleTrafficSequence() {
  unsigned long now = getTicks();
  if (now - stateStartTick >= stateDuration) {
    advanceTrafficState();
  }
}

// =====================================================================
// Arduino setup()
// =====================================================================
void setup() {
  Serial.begin(9600);

  setupPins();
  runStartupSelfTest();
  setupPCI();
  setupTimer1();

  enterState(STATE_NS_GREEN);
  Serial.println(F("[SYSTEM] Traffic system initialized."));
}

// =====================================================================
// Arduino loop() - Finite State Machine dispatcher
// Priority order: Emergency > Pedestrian > Vehicle extension > Sequence
// =====================================================================
void loop() {
  if (handleEmergency()) {
    return; // emergency has full priority, skip everything else
  }

  if (handlePedestrian()) {
    return; // pedestrian phase blocks normal sequence/vehicle logic
  }

  handleVehicleRequest();
  handleTrafficSequence();
}