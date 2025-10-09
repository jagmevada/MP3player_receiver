#include <Arduino.h>

/*
  PPM-like RX on Arduino Nano, pin 2 (INT0).

  Update per your spec:
  - SYNC detection: use LOW width (falling -> rising). Accept 32–36 ms.
  - Bit decoding after sync: use HIGH width (rising -> falling) only.
      * >800 us => bit 1
      * <350 us => bit 0
      * otherwise invalid -> reset to sync search
  - 32 bits must arrive within 35 ms from sync rising; otherwise reset.
*/

static const uint8_t RX_PIN = 2; // INT0 on Nano

// Thresholds (microseconds)
static const unsigned long SYNC_MIN_US   = 33000UL;
static const unsigned long SYNC_MAX_US   = 37000UL;
static const unsigned long BIT_1_MIN_US  = 801UL;   // HIGH >800us => 1
static const unsigned long BIT_0_MAX_US  = 500UL;   // HIGH <350us => 0
static const unsigned long FRAME_TIMEOUT = 35000UL; // from sync rising

// Optional noise reject for absurdly long highs during frame
static const unsigned long NOISE_HIGH_REJECT_MIN_US = 5000UL;

volatile unsigned long lastFallUs = 0;
volatile unsigned long lastRiseUs = 0;
volatile unsigned long syncStartUs = 0;

volatile bool inSync = false;
volatile uint8_t bitCount = 0;
volatile uint32_t frameData = 0;
volatile bool frameReady = false;

// Forward decls
void ISR_FALLING();
void ISR_RISING();

inline void resetSyncSearchUnsafe_() {
  inSync = false;
  bitCount = 0;
  frameData = 0;
  syncStartUs = 0;
}

void ISR_FALLING() {
  const unsigned long nowUs = micros();
  lastFallUs = nowUs;

  if (inSync) {
    // Decode bit from HIGH width (rising -> falling)
    // (Timeout first)
    if ((nowUs - syncStartUs) > FRAME_TIMEOUT) {
      resetSyncSearchUnsafe_();
    } else {
      const unsigned long highWidthUs = nowUs - lastRiseUs;

      // Optional: reject absurd highs during frame that aren't valid (helps with idle/noise)
      if (highWidthUs >= NOISE_HIGH_REJECT_MIN_US) {
        resetSyncSearchUnsafe_();
      } else {
        // Classify bit by HIGH width
        if (highWidthUs > BIT_1_MIN_US) {
          frameData = (frameData << 1) | 1UL; // bit = 1
          bitCount++;
        } else if (highWidthUs < BIT_0_MAX_US) {
          frameData = (frameData << 1);       // bit = 0
          bitCount++;
        } else {
          // Invalid width band
          // resetSyncSearchUnsafe_();
        }

        // Completed 32 bits?
        if (inSync && bitCount >= 32) {
          frameReady = true;
          inSync = false; // ready for next frame
        }
      }
    }
  }

  // After FALLING, we want to look for the next RISING
  attachInterrupt(digitalPinToInterrupt(RX_PIN), ISR_RISING, RISING);
}

void ISR_RISING() {
  const unsigned long nowUs = micros();
  lastRiseUs = nowUs;

  // Measure LOW width (falling -> rising) for SYNC detection
  const unsigned long lowWidthUs = nowUs - lastFallUs;

  if (!inSync) {
    if (lowWidthUs >= SYNC_MIN_US && lowWidthUs <= SYNC_MAX_US) {
      // SYNC acquired at this rising; bits start now (HIGH span after this rising)
      inSync = true;
      bitCount = 0;
      frameData = 0;
      syncStartUs = nowUs;
    }
    // else: keep searching
  } else {
    // Already in sync: rising simply starts the next HIGH; decoding happens on next FALLING
    // Still enforce overall timeout window
    if ((nowUs - syncStartUs) > FRAME_TIMEOUT) {
      resetSyncSearchUnsafe_();
    }
  }

  // After RISING, wait for next FALLING to end the HIGH and decode the bit
  attachInterrupt(digitalPinToInterrupt(RX_PIN), ISR_FALLING, FALLING);
}

void setup() {
  Serial.begin(115200);
  pinMode(RX_PIN, INPUT); // Use external pull resistor suitable for your receiver
  resetSyncSearchUnsafe_();
  attachInterrupt(digitalPinToInterrupt(RX_PIN), ISR_FALLING, FALLING); // start by catching a falling edge
  Serial.println(F("PPM-like RX started"));
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
    for (int i = 31; i >= 0; --i) {
      Serial.print((data >> i) & 1U);
    }
    Serial.println(F(")"));
  }
}
