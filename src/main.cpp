#include <Arduino.h>
#include <DNSServer.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <Audio.h>

static const uint8_t RX_PIN = 13; // INT0 on Nano

// Time thresholds (µs)
static const unsigned long SYNC_MIN_US   = 32000UL;
static const unsigned long SYNC_MAX_US   = 36000UL;
static const unsigned long BIT_0_MAX_US  = 400UL;   // HIGH < 350us => 0
static const unsigned long BIT_1_MIN_US  = 801UL;   // HIGH > 800us => 1
static const unsigned long FRAME_TIMEOUT = 36000UL; // from SYNC rising

// const uint32_t  KEY2   = 0xa0a0a0a0;     
// const uint32_t  KEY3   = 0xb0b0b0b0;    
// const uint32_t  KEY1   = 0xc0c0c0c0;   

// const uint32_t  KEY2   = 0xa5a5a5a0;     
// const uint32_t  KEY3   = 0xb5b5b5b0;    
// const uint32_t  KEY1   = 0xc5c5c5c0; 

// const uint32_t  KEY2   = 0x696969a0;     
// const uint32_t  KEY3   = 0x7a7a7a70;    
// const uint32_t  KEY1   = 0x8b8b8b80; 

const uint32_t  KEY2   = 0x30303030;     
const uint32_t  KEY3   = 0x41414140;    
const uint32_t  KEY1   = 0x52525250;   

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


void ISR_CHANGE();
inline void resetSyncUnsafe_();


#define DEBOUNCE_MS 200


#define BTN_MORNING 9
#define BTN_EVENING 8
#define BTN_PAUSE   7


volatile unsigned long lastInterruptTime[3] = {0, 0, 0};

// === Your Tested SPI SD Card GPIOs ===
#define SD_CS    5//4
#define SD_MOSI  3//2
#define SD_MISO  2//1
#define SD_SCK   4//3

// === Your Tested I2S DAC GPIOs ===
#define I2S_DOUT 11
#define I2S_BCLK 12//10
#define I2S_LRC  10//12

// === Wi-Fi AP Credentials ===
const char* ssid = "MP3Player3";
const char* password = "12345678";

// === Global Objects ===
File fsUploadFile;
Audio audio;
WebServer server(80);
DNSServer dnsServer;

String currentFilename = "";  // Track currently playing file name

volatile bool playMorningRequested = false;
volatile bool playEveningRequested = false;
volatile bool pauseToggleRequested = false;

void IRAM_ATTR handlePauseBtn();
void IRAM_ATTR handleMorningBtn();
void IRAM_ATTR handleEveningBtn();

void handleUpload() ;
// === HTTP HANDLERS ===

void handleWebUI() {
  File f = SD.open("/ui.html");
  if (!f) {
    server.send(500, "text/plain", "Missing ui.html on SD");
    return;
  }
  server.streamFile(f, "text/html");
  f.close();
}

void handleFilesJson() {
  File root = SD.open("/");
  if (!root) {
    server.send(500, "text/plain", "Failed to open SD root");
    return;
  }

  String json = "[";
  bool first = true;
  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory() && String(file.name()).endsWith(".mp3")) {
      if (!first) json += ",";
      json += "\"" + String(file.name()) + "\"";
      first = false;
    }
    file = root.openNextFile();
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handlePlay() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Missing 'file' parameter");
    return;
  }

  String filename = server.arg("file");
  currentFilename = filename;
  audio.stopSong();
  audio.connecttoFS(SD, filename.c_str());
  server.send(200, "text/plain", "Playing: " + filename);
}

void handlePause() {
  audio.pauseResume();
  server.send(200, "text/plain", "Toggled pause/resume.");
}

void handleStatus() {
  String json = "{";
  json += "\"position\":" + String(audio.getAudioCurrentTime());
  json += ",\"duration\":" + String(audio.getAudioFileDuration());
  json += ",\"running\":" + String(audio.isRunning() ? "true" : "false");
  json += ",\"filename\":\"" + currentFilename + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleSeek() {
  if (!server.hasArg("position")) {
    server.send(400, "text/plain", "Missing position");
    return;
  }
  int pos = server.arg("position").toInt();
  audio.setAudioPlayPosition(pos);
  server.send(200, "text/plain", "Seeked");
}

// === SETUP ===

void setup() {
  Serial.begin(115200);
  delay(1000);

  //
  pinMode(BTN_PAUSE, INPUT_PULLUP);
  pinMode(BTN_MORNING, INPUT_PULLUP);
  pinMode(BTN_EVENING, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BTN_PAUSE),   handlePauseBtn,   RISING);
  attachInterrupt(digitalPinToInterrupt(BTN_MORNING), handleMorningBtn, RISING);
  attachInterrupt(digitalPinToInterrupt(BTN_EVENING), handleEveningBtn, RISING);

  // upload feature
server.on("/upload", HTTP_POST, []() {
  server.send(200);
}, handleUpload);

// delete feature
server.on("/delete", HTTP_DELETE, []() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Missing file name");
    return;
  }

  String filename = "/" + server.arg("file");
  if (!SD.exists(filename)) {
    server.send(404, "text/plain", "File not found");
    return;
  }

  if (SD.remove(filename)) {
    Serial.println("Deleted: " + filename);
    server.send(200, "text/plain", "Deleted");
  } else {
    server.send(500, "text/plain", "Delete failed");
  }
});

// rename feature
 server.on("/rename", HTTP_POST, []() {
  if (!server.hasArg("from") || !server.hasArg("to")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }
  String from = "/" + server.arg("from");
  String to   = "/" + server.arg("to");

  if (!SD.exists(from)) {
    server.send(404, "text/plain", "Source file not found");
    return;
  }

  if (SD.rename(from, to)) {
    server.send(200, "text/plain", "Renamed");
  } else {
    server.send(500, "text/plain", "Rename failed");
  }
});


  // Start Wi-Fi AP
  Serial.println("Starting SoftAP...");
  bool ap_ok = WiFi.softAP(ssid, password, 1, 0, 4);
  if (!ap_ok) {
    Serial.println("SoftAP failed!");
    while (1);
  }
  // Start DNS server for captive portal redirection
  dnsServer.start(53, "*", WiFi.softAPIP());

  Serial.print("SoftAP IP: ");
  Serial.println(WiFi.softAPIP());

  // Start SPI SD
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("SD card init failed!");
    while (1);
  }
  Serial.println("SD card initialized.");

  // I2S Audio
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(21);  // Max volume

  // Register Routes
  server.on("/", handleWebUI);
  server.on("/files.json", handleFilesJson);
  server.on("/play", handlePlay);
  server.on("/pause", handlePause);
  server.on("/status.json", handleStatus);
  server.on("/seek", handleSeek);

  // Captive portal detection for Android/iOS/Windows
server.on("/generate_204", []() {
  server.sendHeader("Location", "/mp3launcher.html", true);
  server.send(302, "text/plain", "");
});
server.on("/hotspot-detect.html", []() {
  server.sendHeader("Location", "/mp3launcher.html", true);
  server.send(302, "text/plain", "");
});
server.on("/fwlink", []() {
  server.sendHeader("Location", "/mp3launcher.html", true);
  server.send(302, "text/plain", "");
});
server.on("/ncsi.txt", []() {
  server.send(200, "text/plain", "Microsoft NCSI");
});
server.on("/connecttest.txt", []() {
  server.send(200, "text/plain", "Microsoft Connect Test");
});
server.on("/library/test/success.html", []() {
  server.send(200, "text/html", "<html><body>Success</body></html>");
});
server.on("/redirect", []() {
  server.sendHeader("Location", "/mp3launcher.html", true);
  server.send(302, "text/plain", "");
});


// Catch-all for anything not found → redirect to launcher
server.onNotFound([]() {
  String uri = server.uri();
  if (uri.indexOf("favicon.ico") != -1) return;
  server.sendHeader("Location", "/mp3launcher.html", true);
  server.send(302, "text/plain", "");
});

  server.begin();
  Serial.println("Web server started");

  pinMode(RX_PIN, INPUT);         // set pull-ups/downs per your RX module
  lastLevel = digitalRead(RX_PIN);
  unsigned long t = micros();
  lastRiseUs = t;
  lastFallUs = t;

  attachInterrupt(digitalPinToInterrupt(RX_PIN), ISR_CHANGE, CHANGE);
  


}

// === LOOP ===

void loop() {
    if (frameReady) {
    noInterrupts();
    uint32_t data = frameData;
    frameReady = false;
    interrupts();
    if(data == KEY1)
    playMorningRequested = true;
    else if(data == KEY2)
    playEveningRequested = true;
    else if(data == KEY3)
          pauseToggleRequested = true;
          else;

    Serial.print(F("Frame: 0x"));
    Serial.print(data, HEX);
    Serial.print(F("  (bin: "));
    for (int i = 31; i >= 0; --i) Serial.print((data >> i) & 1U);
    Serial.println(F(")"));

    }

  audio.loop();
  server.handleClient();
  dnsServer.processNextRequest();


  if (pauseToggleRequested) {
    pauseToggleRequested = false;
    audio.pauseResume();
  }

  if (playMorningRequested) {
    playMorningRequested = false;
    audio.stopSong();
    currentFilename = "/morning.mp3";
    audio.connecttoFS(SD, currentFilename.c_str());
  }

  if (playEveningRequested) {
    playEveningRequested = false;
    audio.stopSong();
    currentFilename = "/evening.mp3";
    audio.connecttoFS(SD, currentFilename.c_str());
  }
}

// === DEBUG CALLBACKS ===

void audio_info(const char *info) {
  Serial.print("info        "); Serial.println(info);
}
void audio_id3data(const char *info) {
  Serial.print("id3data     "); Serial.println(info);
}
void audio_eof_mp3(const char *info) {
  Serial.print("eof_mp3     "); Serial.println(info);
}

void handleUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("Upload Start: %s\n", upload.filename.c_str());
    fsUploadFile = SD.open("/" + upload.filename, FILE_WRITE);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (fsUploadFile)
      fsUploadFile.close();
    Serial.printf("Upload End: %s (%u)\n", upload.filename.c_str(), upload.totalSize);
    server.send(200, "text/plain", "Upload success");
  }
}

void IRAM_ATTR handlePauseBtn() {
  unsigned long now = millis();
  // Only trigger if button is LOW for at least 30ms (strong debounce)
  if (now - lastInterruptTime[0] > DEBOUNCE_MS) {
    delayMicroseconds(5000); // 30ms debounce in ISR (safe for ESP32)
    if (digitalRead(BTN_PAUSE) == LOW) {
      lastInterruptTime[0] = millis();
      pauseToggleRequested = true;
    }
  }
}

void IRAM_ATTR handleMorningBtn() {
  unsigned long now = millis();
  // Only trigger if button is LOW for at least 30ms (strong debounce)
  if (now - lastInterruptTime[1] > DEBOUNCE_MS) {
    delayMicroseconds(5000); // 30ms debounce in ISR (safe for ESP32)
    if (digitalRead(BTN_MORNING) == LOW) {
      lastInterruptTime[1] = millis();
      playMorningRequested = true;
    }
  }
}

void IRAM_ATTR handleEveningBtn() {
  unsigned long now = millis();
  // Only trigger if button is LOW for at least 30ms (strong debounce)
  if (now - lastInterruptTime[2] > DEBOUNCE_MS) {
    delayMicroseconds(5000); // 30ms debounce in ISR (safe for ESP32)
    if (digitalRead(BTN_EVENING) == LOW) {
      lastInterruptTime[2] = millis();
      playEveningRequested = true;
    }
  }
}




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