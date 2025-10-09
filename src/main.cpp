#include <Arduino.h>

// -------- Config --------
static const uint8_t RX_PIN = 2; // INT0 on Nano

// Time thresholds (µs)
static const unsigned long SYNC_MIN_US   = 32000UL;
static const unsigned long SYNC_MAX_US   = 36000UL;
static const unsigned long BIT_0_MAX_US  = 400UL;   // HIGH < 350us => 0
static const unsigned long BIT_1_MIN_US  = 801UL;   // HIGH > 800us => 1
static const unsigned long FRAME_TIMEOUT = 36000UL; // from SYNC rising

// Optional: treat absurdly long HIGH during frame as noise
static const unsigned long NOISE_HIGH_REJECT_US = 1500UL;

// -------- State (ISR-touched) --------
volatile bool inSync = false;
volatile uint8_t  bitCount = 0;
volatile uint32_t frameData = 0;
volatile bool frameReady = false;

volatile unsigned long lastRiseUs = 0;
volatile unsigned long lastFallUs = 0;
volatile unsigned long syncStartUs = 0;
volatile uint8_t lastLevel = LOW;
volatile uint8_t level;
volatile unsigned long nowUs;
volatile unsigned long lowWidthUs;
volatile unsigned long highWidthUs;
// -------- Helpers --------
inline void resetSyncUnsafe_() {
  inSync = false;
  bitCount = 0;
  frameData = 0;
  syncStartUs = micros();
}

// CHANGE ISR: decides rising vs falling by reading current level
void ISR_CHANGE() {
  nowUs = micros();
  level = digitalRead(RX_PIN); // cheap enough at these rates

  // Rising edge: LOW -> HIGH
  if (level == HIGH && lastLevel == LOW) {
    lastRiseUs = nowUs;

    // LOW-width just ended: check for SYNC only when not inSync
    lowWidthUs = nowUs - lastFallUs;
    if (!inSync) {
      if (lowWidthUs >= SYNC_MIN_US && lowWidthUs <= SYNC_MAX_US) {
        // SYNC acquired at this rising; bits start in subsequent HIGH interval(s)
        inSync = true;
        bitCount = 0;
        frameData = 0;
        syncStartUs = nowUs;
      }
    } else {
      // While in frame, still enforce overall timeout
      if ((nowUs - syncStartUs) > FRAME_TIMEOUT) {
        resetSyncUnsafe_();
      }
    }
  }

  // Falling edge: HIGH -> LOW
  else if (level == LOW && lastLevel == HIGH) {
    lastFallUs = nowUs;

    // HIGH-width ended: decode a bit if we're in a frame
    if (inSync) {
      // Check timeout first
      if ((nowUs - syncStartUs) > FRAME_TIMEOUT) {
        resetSyncUnsafe_();
      } else {
        highWidthUs = nowUs - lastRiseUs;

        // Optional noise bail-out (protects against idle garbage)
        if (highWidthUs >= NOISE_HIGH_REJECT_US) {
          resetSyncUnsafe_();
        } else {
          // Classify bit
          if (highWidthUs > BIT_1_MIN_US) {
            frameData = (frameData << 1) | 1UL; // 1
          } else if (highWidthUs < BIT_0_MAX_US) {
            frameData = (frameData << 1);       // 0
          } else {
            // Invalid band => reset
            // resetSyncUnsafe_();
          }
          bitCount++;  
          // Completed 32 bits?
          if (inSync && bitCount >= 31) {
            frameData = (frameData << 1); // final 0 bit
            frameReady = true;
            inSync = false; // ready for next frame
          }
        }
      }
    }
  }

  lastLevel = level;
}

void setup() {
  Serial.begin(115200);
  pinMode(RX_PIN, INPUT);         // set pull-ups/downs per your RX module
  lastLevel = digitalRead(RX_PIN);
  unsigned long t = micros();
  lastRiseUs = t;
  lastFallUs = t;

  attachInterrupt(digitalPinToInterrupt(RX_PIN), ISR_CHANGE, CHANGE);
  Serial.println(F("Ready"));
}

void loop() {
  if (frameReady) {
    noInterrupts();
    uint32_t data = frameData;
    frameReady = false;
    interrupts();

    Serial.print(F("Frame: 0x"));
    Serial.print(data, HEX);
    Serial.print(F("  (bin: "));
    for (int i = 31; i >= 0; --i) Serial.print((data >> i) & 1U);
    Serial.println(F(")"));
  }
//   if(inSync) {
// Serial.print(F("."));
//   }
}
