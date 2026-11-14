#include <Arduino.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>
#include <LittleFS.h>
#include <driver/i2s.h>

#include "camaro.h"

// =====================================================
// HARDWARE
// =====================================================

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);

// Panel: a 128x128 sprite is pushed at (0, 0). The glass is mounted 90 degrees
// anticlockwise, which is TFT_eSPI rotation 5 on this display.
#define TFT_SPRITE_WIDTH    128
#define TFT_SPRITE_HEIGHT   128
#define TFT_SCREEN_ROTATION 5
#define TFT_SPRITE_ROTATION 0

// MPU6050
#define SDA_PIN 32
#define SCL_PIN 33
MPU6050 mpu(Wire);

// TTP223 touch sensor
#define TOUCH_PIN 27

// LEDs
#define LED_A_PIN 25   // LEDs 1 and 2
#define LED_B_PIN 26   // LEDs 3 and 4

// =====================================================
//  AUDIO - MAX98357A
// =====================================================

#define NOTIFICATION_AUDIO_BCLK 21
#define NOTIFICATION_AUDIO_LRC  22
#define NOTIFICATION_AUDIO_DOUT 2

#define NOTIFICATION_AUDIO_SAMPLE_RATE 44100
#define NOTIFICATION_AUDIO_FILE "/notification-click.wav"
#define LIGHTS_AUDIO_FILE "/lights.wav"

// Hold-to-talk walkie channel: this clip is played the moment the touch sensor
// is pressed, then the microphone records until the sensor is released.
#define WALKIE_SEND_AUDIO_FILE "/walkie-send.wav"

// Walkie audio over MQTT. Both directions carry raw 16 bit mono PCM at 16 kHz,
// so nothing has to be encoded or decoded on either side: the ESP32 only moves
// samples between I2S and the broker, and the browser can read and write the
// very same format through the Web Audio API.
//
// One packet is AUDIO_CHUNK_BYTES of audio (32 ms). Keeping the frames small
// matters twice: PubSubClient copies every payload into its own buffer, and the
// ESP32's TCP receive window is only a couple of kilobytes wide, so a burst of
// large packets would overflow it before loop() gets a chance to drain them.
#define AUDIO_STREAM_SAMPLE_RATE 16000
#define AUDIO_CHUNK_BYTES 1024                    // 512 samples, 32 ms at 16 kHz
#define AUDIO_CHUNK_SAMPLES (AUDIO_CHUNK_BYTES / 2)
#define AUDIO_WRITE_TIMEOUT_MS 500                // abandoned speaker write budget
#define SPEAKER_STREAM_TIMEOUT_MS 1500            // stop a stream that went quiet

TaskHandle_t notificationAudioTaskHandle = nullptr;

bool notificationAudioReady = false;
volatile bool notificationAudioPlaying = false;

// =====================================================
//  MICROPHONE - I2S (INMP441 style)
// =====================================================

// The microphone gets its own I2S peripheral (I2S_NUM_1) so it never fights
// with the MAX98357A speaker, which keeps I2S_NUM_0 for itself. The driver is
// installed once, but its bit clock is only started while a transmission is
// actually being recorded: a permanently running ~1 MHz BCLK next to the IMU
// wiring corrupts the MPU6050 readings enough to latch the "moving" state,
// which silently kills the car's auto-spin.
#define MIC_BCLK 13
#define MIC_WS   14
#define MIC_SD   34

#define MIC_SAMPLE_RATE 16000        // 16 kHz mono is plenty for voice
#define MIC_READ_SAMPLES 256         // 16 ms of audio per read at 16 kHz
#define MIC_DMA_BUF_COUNT 8
#define MIC_DMA_BUF_LEN 128          // 8 * 128 = 64 ms of buffering

// The MQTT walkie stream is raw PCM, so both ends have to agree on the format.
static_assert(
  MIC_SAMPLE_RATE == AUDIO_STREAM_SAMPLE_RATE,
  "The walkie stream carries the microphone rate, keep both at 16 kHz"
);

// INMP441: L/R tied to GND means the samples arrive on the left channel. If the
// level stays at 0 while talking, switch this to I2S_CHANNEL_FMT_ONLY_RIGHT.
#define MIC_CHANNEL_FORMAT I2S_CHANNEL_FMT_ONLY_LEFT

// Serial output while recording. A line every 100 ms is fast enough to see the
// level move, but slow enough to keep the monitor readable at 115200 baud.
#define MIC_PRINT_INTERVAL_MS 100
#define MIC_LEVEL_BAR_WIDTH 24
#define MIC_PRINT_RAW_SAMPLES 4      // raw sample values shown on every line

bool microphoneReady = false;

// Hold-to-talk state machine, driven by serviceWalkie() from loop().
bool walkieRecording = false;      // true while the microphone is being read
bool touchWasHeld = false;         // previous touch state, for edge detection
uint32_t walkieStartTime = 0;      // millis() when the recording started
uint32_t walkieSampleCount = 0;    // samples captured during this press
uint32_t lastMicPrint = 0;         // last time the level line was printed

// Outgoing audio packet being filled. The first two bytes are a little endian
// sequence number followed by the PCM samples: the browser uses the number to
// notice a packet that never arrived instead of playing the rest of the clip
// shifted in time.
static uint8_t walkieChunkFrame[AUDIO_CHUNK_BYTES + 2];
uint16_t walkieChunkIndex = 0;     // sequence number of the next packet
size_t walkieChunkFill = 0;        // PCM bytes already inside walkieChunkFrame
uint32_t walkieTransmissionId = 0; // id announced with the current transmission

// Published from the recorder below, but defined next to the MQTT helpers that
// need the topic definitions.
void publishMicStart();
void queueMicSamples(const int16_t* samples, size_t count);
void publishMicEnd(uint32_t durationMs);

// Also defined with the MQTT helpers: talking over the speaker stops whatever
// the dashboard was streaming into it.
void stopSpeakerStream();

// =====================================================
// 3D ENGINE
// =====================================================

#define DEG2RAD 0.0174532925f

float currentRoll = 0.0f;
float currentPitch = 0.0f;
float currentYaw = 0.0f;

float targetRoll = 0.0f;
float targetPitch = 0.0f;

// Previous raw targets, used to notice movement even when the gyro rate is low.
float previousRoll = 0.0f;
float previousPitch = 0.0f;

float px[VERTEX_COUNT];
float py[VERTEX_COUNT];
float pz[VERTEX_COUNT];

uint32_t lastFrame = 0;
float measuredFps = 0.0f;

// Latest sensor sample. Refreshed on every loop pass (not only once per rendered
// frame) so a quick flick of the board is never missed.
float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;
float gyroMagnitude = 0.0f;     // length of the gyro vector, degrees/second
float accX = 0.0f;
float accY = 0.0f;
float accZ = 0.0f;

// Gyro driven yaw. It is integrated only while the sensor is moving and unwinds
// while it is still, so the gyro bias can never keep the car turning for ever.
float yawAngle = 0.0f;          // yaw target, degrees
float yawRate = 0.0f;           // low passed gyro Z, degrees/second

// Motion detection
bool sensorMoving = false;
uint32_t lastMotionTime = 0;    // last time the sensor reported movement

// Auto-spin state
float spinOffset = 0.0f;        // extra turn angle from auto-spin (degrees)
float spinBlend = 0.0f;         // 0 = gyro drives the car, 1 = auto-spin
float spinSpeed = 0.0f;         // spin speed applied this frame, degrees/second

// =====================================================
// SETTINGS
// =====================================================

const float CAR_SIZE = 15.0f;
const float CAMERA_DISTANCE = 5.0f;
const float SMOOTHING = 0.18f;
const float VIEW_TURN_OFFSET = 90.0f;   // 90 = side view, try -90 to face the other way
const float YAW_VIEW_OFFSET = 0.0f;     // 180 = look at the other end of the car

// Projection centre inside the sprite
const float PROJECTION_CENTER_X = 64.0f;
const float PROJECTION_CENTER_Y = 80.0f;

// Motion detection. The hysteresis (enter > exit) stops the car from flickering
// between "follow the gyro" and "auto-spin" while the readings hover on the limit.
const float MOTION_ENTER_THRESHOLD = 8.0f;   // gyro vector length, degrees/second
const float MOTION_EXIT_THRESHOLD = 3.0f;    // must drop below this to count as still
const float MOTION_ANGLE_THRESHOLD = 1.5f;   // deg of tilt change per frame (fast moves only)
const uint32_t IDLE_DELAY_MS = 1500;         // stillness needed before auto-spin starts

// Auto-spin when the sensor is still
const float AUTO_SPIN_SPEED = 75.0f;     // degrees per second (negative = spin the other way)
const float SPIN_STOP_TIME = 0.12f;      // seconds for the spin to get out of the way
const float SPIN_START_TIME = 0.35f;     // seconds for the spin to ease back in

// Yaw (gyro Z) handling
const float YAW_RATE_SMOOTHING_TIME = 0.05f;  // seconds, low pass on the gyro rate
const float YAW_GYRO_DEADBAND = 0.8f;         // degrees/second ignored as noise
const float YAW_RETURN_TIME = 1.2f;           // seconds for the yaw to unwind when still

const uint32_t FRAME_INTERVAL_MS = 16;   // approximately 60 FPS

// Serial debug: set to 0 to silence the [dbg] output.
#define DEBUG_SERIAL 1

const uint32_t DEBUG_INTERVAL_MS = 250;      // how often the state line is printed
const uint32_t DEBUG_SENSOR_WARN_MS = 2000;  // how often to nag about a dead sensor

// =====================================================
// SERIAL DEBUG
// =====================================================

#if DEBUG_SERIAL
uint32_t lastDebugPrint = 0;
uint32_t lastSensorWarn = 0;
#endif

uint32_t stillTimeMs() {
  return millis() - lastMotionTime;
}

void debugPrintConfig() {
#if DEBUG_SERIAL
  Serial.println();
  Serial.println("----- DIECAST 64 DEBUG -----");
  Serial.printf(
    "motion: enter=%.1f deg/s exit=%.1f deg/s tilt=%.1f deg/frame idle=%lu ms\n",
    (double)MOTION_ENTER_THRESHOLD,
    (double)MOTION_EXIT_THRESHOLD,
    (double)MOTION_ANGLE_THRESHOLD,
    (unsigned long)IDLE_DELAY_MS
  );
  Serial.printf(
    "spin: %.1f deg/s, stop %.2f s, start %.2f s\n",
    (double)AUTO_SPIN_SPEED,
    (double)SPIN_STOP_TIME,
    (double)SPIN_START_TIME
  );
  Serial.printf(
    "yaw: deadband %.2f deg/s, return %.2f s\n",
    (double)YAW_GYRO_DEADBAND,
    (double)YAW_RETURN_TIME
  );
  Serial.printf(
    "mic: %d Hz, %d samples per read, send tone %s\n",
    MIC_SAMPLE_RATE,
    MIC_READ_SAMPLES,
    WALKIE_SEND_AUDIO_FILE
  );
  Serial.printf(
    "walkie: %d Hz mono pcm, %d byte packets over MQTT (mic -> web, speaker <- web)\n",
    AUDIO_STREAM_SAMPLE_RATE,
    AUDIO_CHUNK_BYTES
  );
  Serial.println("----------------------------");
#endif
}

// One compact line every DEBUG_INTERVAL_MS. "MOVE" means the gyro is driving
// the car, "IDLE" means it is auto-spinning. Paste this back when debugging.
void debugPrintState() {
#if DEBUG_SERIAL
  const uint32_t now = millis();
  if (now - lastDebugPrint < DEBUG_INTERVAL_MS) return;
  lastDebugPrint = now;

  // Serial.printf(
  //   "[dbg] t=%lu g=(%6.2f,%6.2f,%6.2f) |g|=%6.2f a=(%5.2f,%5.2f,%5.2f) "
  //   "raw=(%7.2f,%7.2f) rpy=(%7.2f,%7.2f,%7.2f) %s still=%5lums "
  //   "spin=%.2f spd=%6.1f off=%6.1f fps=%4.1f\n",
  //   (unsigned long)now,
  //   (double)gyroX, (double)gyroY, (double)gyroZ, (double)gyroMagnitude,
  //   (double)accX, (double)accY, (double)accZ,
  //   (double)targetRoll, (double)targetPitch,
  //   (double)currentRoll, (double)currentPitch, (double)currentYaw,
  //   sensorMoving ? "MOVE" : "IDLE",
  //   (unsigned long)stillTimeMs(),
  //   (double)spinBlend, (double)spinSpeed, (double)spinOffset,
  //   (double)measuredFps
  // );
#endif
}

void debugLogMotion(bool moving) {
#if DEBUG_SERIAL
  Serial.printf(
    "[motion] %s |g|=%.2f deg/s still=%lums yaw=%.2f\n",
    moving ? "START gyro follow" : "STOP  auto-spin resumes",
    (double)gyroMagnitude,
    (unsigned long)stillTimeMs(),
    (double)yawAngle
  );
#endif
}

void debugWarnSensor() {
#if DEBUG_SERIAL
  const uint32_t now = millis();
  if (now - lastSensorWarn < DEBUG_SENSOR_WARN_MS) return;
  lastSensorWarn = now;

  Serial.println("[mpu] NOT READY - no gyro data, car keeps auto-spinning");
#endif
}

// =====================================================
// DISPLAY
// =====================================================

void initDisplay() {
  tft.init();
  tft.setRotation(TFT_SCREEN_ROTATION);
  tft.fillScreen(TFT_BLACK);

  sprite.setColorDepth(16);

  if (!sprite.createSprite(TFT_SPRITE_WIDTH, TFT_SPRITE_HEIGHT)) {
    Serial.println("Sprite allocation failed!");

    while (true) {
      delay(1000);
    }
  }

  sprite.setRotation(TFT_SPRITE_ROTATION);
  sprite.setTextDatum(TL_DATUM);
}

void drawStatusScreen(const String& title, const String& detail, uint16_t titleColor) {
  sprite.fillSprite(TFT_BLACK);
  sprite.setTextDatum(MC_DATUM);
  sprite.setTextFont(2);
  sprite.setTextSize(1);

  sprite.setTextColor(titleColor, TFT_BLACK);
  sprite.drawString(title, TFT_SPRITE_WIDTH / 2, TFT_SPRITE_HEIGHT / 2 - 10);

  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.drawString(detail.substring(0, 20), TFT_SPRITE_WIDTH / 2, TFT_SPRITE_HEIGHT / 2 + 10);

  sprite.pushSprite(0, 0);
  sprite.setTextDatum(TL_DATUM);
}

void drawProvisioningScreen() {
  sprite.fillSprite(TFT_BLACK);
  sprite.setTextDatum(MC_DATUM);
  sprite.setTextFont(2);
  sprite.setTextSize(1);

  sprite.setTextColor(TFT_CYAN, TFT_BLACK);
  sprite.drawString("WIFI SETUP", TFT_SPRITE_WIDTH / 2, 32);

  sprite.drawFastHLine(20, 46, 88, TFT_MAGENTA);

  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.drawString("JOIN", TFT_SPRITE_WIDTH / 2, 62);

  sprite.setTextColor(TFT_GREEN, TFT_BLACK);
  sprite.drawString("ESP32-Setup-AP", TFT_SPRITE_WIDTH / 2, 80);

  sprite.setTextColor(TFT_YELLOW, TFT_BLACK);
  sprite.drawString("OPEN WEB APP", TFT_SPRITE_WIDTH / 2, 106);

  sprite.pushSprite(0, 0);
  sprite.setTextDatum(TL_DATUM);
}

// =====================================================
// BOOT ANIMATION
// =====================================================

void bootAnimation() {
  sprite.fillSprite(TFT_BLACK);

  sprite.setTextDatum(MC_DATUM);
  sprite.setTextColor(TFT_CYAN, TFT_BLACK);
  sprite.setTextSize(2);
  sprite.drawString("DIECAST 64", 64, 12);

  sprite.drawFastHLine(18, 47, 92, TFT_MAGENTA);

  sprite.setTextSize(1);
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.drawString("CYBER SYSTEM", 64, 62);

  const int barX = 15;
  const int barY = 79;
  const int barW = 98;
  const int barH = 10;

  sprite.drawRect(barX, barY, barW, barH, TFT_CYAN);

  for (int progress = 0; progress <= 100; progress += 4) {
    int fillW = (barW - 4) * progress / 100;

    sprite.fillRect(
      barX + 2, barY + 2,
      barW - 4, barH - 4,
      TFT_BLACK
    );

    if (fillW > 0) {
      sprite.fillRect(
        barX + 2, barY + 2,
        fillW, barH - 4,
        TFT_MAGENTA
      );
    }

    sprite.setTextColor(TFT_GREEN, TFT_BLACK);
    sprite.fillRect(40, 96, 48, 10, TFT_BLACK);
    sprite.drawCentreString(
      String(progress) + "%",
      64, 96, 1
    );

    sprite.pushSprite(0, 0);
    delay(35);
  }

  delay(400);

  sprite.fillSprite(TFT_BLACK);

  sprite.setTextDatum(MC_DATUM);

  sprite.setTextColor(TFT_GREEN, TFT_BLACK);
  sprite.drawString("SYSTEM ONLINE", 64, 51);

  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.drawString("ESP32 CORE", 64, 69);

  sprite.setTextColor(TFT_YELLOW, TFT_BLACK);
  sprite.drawString("BOOT SUCCESS", 64, 88);

  sprite.pushSprite(0, 0);
  delay(1500);

  sprite.setTextDatum(TL_DATUM);
}

// =====================================================
// FLOATING TEXT
// =====================================================

const uint8_t TEXT_BAND_HEIGHT = 24;
const uint8_t TEXT_FONT_HEIGHT = 16;       // TFT_eSPI font 2
const size_t FLOATING_TEXT_MAX_LENGTH = 64;
const float TEXT_SCROLL_SPEED = 34.0f;     // pixels per second

String floatingText;
int floatingTextWidth = 0;
float floatingTextScroll = 0.0f;

void setFloatingText(const String& text) {
  floatingText = text.substring(0, FLOATING_TEXT_MAX_LENGTH);
  floatingTextScroll = 0.0f;

  sprite.setTextFont(2);
  sprite.setTextSize(1);
  floatingTextWidth = floatingText.length() > 0 ? sprite.textWidth(floatingText) : 0;
}

void updateFloatingText(float dt) {
  if (floatingText.length() == 0) {
    floatingTextScroll = 0.0f;
    return;
  }

  // Always marquee horizontally, regardless of whether the text fits.
  floatingTextScroll += TEXT_SCROLL_SPEED * dt;

  if (floatingTextScroll > floatingTextWidth + TFT_SPRITE_WIDTH) {
    floatingTextScroll = 0.0f;
  }
}

int floatingTextX() {
  // Text enters from the right edge and exits past the left edge.
  return TFT_SPRITE_WIDTH - (int)floatingTextScroll;
}

void drawFloatingText() {
  if (floatingText.length() == 0) return;

  const int bandTop = TFT_SPRITE_HEIGHT - TEXT_BAND_HEIGHT;

  // Clear the band so the marquee does not leave a trail over the car.
  sprite.fillRect(0, bandTop, TFT_SPRITE_WIDTH, TEXT_BAND_HEIGHT, TFT_BLACK);
  sprite.drawFastHLine(0, bandTop, TFT_SPRITE_WIDTH, TFT_MAGENTA);

  sprite.setTextDatum(TL_DATUM);
  sprite.setTextFont(2);
  sprite.setTextSize(1);
  sprite.setTextColor(TFT_CYAN);

  // Horizontally scrolling only: the vertical position is fixed.
  const int y = bandTop + (TEXT_BAND_HEIGHT - TEXT_FONT_HEIGHT) / 2;

  sprite.drawString(floatingText, floatingTextX(), y);
}

// =====================================================
// TOUCH ANIMATION
// =====================================================

void drawTouchAnimation() {
  const uint32_t now = millis();

  const uint16_t cycle = now % 1200;
  const float phase = cycle / 1200.0f;

  sprite.fillSprite(TFT_BLACK);

  sprite.drawRect(2, 2, 124, 124, TFT_MAGENTA);
  sprite.drawRect(5, 5, 118, 118, TFT_BLUE);

  for (int ring = 0; ring < 3; ring++) {
    float ringPhase = fmodf(
      phase + ring * 0.3333f,
      1.0f
    );

    int radius = 9 + (int)(ringPhase * 43.0f);

    uint16_t ringColor =
      (ring == 1) ? TFT_MAGENTA : TFT_CYAN;

    sprite.drawCircle(64, 57, radius, ringColor);
  }

  int pulseRadius = 5 + (int)(
    3.0f * (0.5f + 0.5f * sinf(now * 0.012f))
  );

  sprite.fillCircle(64, 57, pulseRadius, TFT_GREEN);

  sprite.setTextDatum(MC_DATUM);

  sprite.setTextColor(TFT_CYAN, TFT_BLACK);
  sprite.drawString("TOUCH DETECTED", 64, 15, 1);

  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.drawString("CYBER INTERFACE", 64, 103, 1);

  sprite.setTextColor(TFT_GREEN, TFT_BLACK);
  sprite.drawString("HOLD TO ACTIVATE", 64, 115, 1);

  sprite.pushSprite(0, 0);

  sprite.setTextDatum(TL_DATUM);
}

// =====================================================
// 3D CAR RENDERER
// =====================================================

void drawCar(float roll, float pitch, float yaw) {
  float r = roll * DEG2RAD;
  float p = (pitch + VIEW_TURN_OFFSET + spinOffset) * DEG2RAD;
  float y = (yaw + YAW_VIEW_OFFSET) * DEG2RAD;

  float sr = sinf(r), cr = cosf(r);
  float sp = sinf(p), cp = cosf(p);
  float sy = sinf(y), cy = cosf(y);

  for (uint16_t i = 0; i < VERTEX_COUNT; i++) {
    float x = carVertices[i][0];
    float yy = carVertices[i][1];
    float z = carVertices[i][2];

    // X rotation
    float x1 = x;
    float y1 = yy * cr - z * sr;
    float z1 = yy * sr + z * cr;

    // Y rotation
    float x2 = x1 * cp + z1 * sp;
    float y2 = y1;
    float z2 = -x1 * sp + z1 * cp;

    // Z rotation
    float x3 = x2 * cy - y2 * sy;
    float y3 = x2 * sy + y2 * cy;
    float z3 = z2;

    float denominator = CAMERA_DISTANCE - z3;

    if (denominator < 0.5f) {
      denominator = 0.5f;
    }

    float perspective = CAMERA_DISTANCE / denominator;

    px[i] = PROJECTION_CENTER_X + x3 * CAR_SIZE * perspective;
    py[i] = PROJECTION_CENTER_Y - y3 * CAR_SIZE * perspective;
    pz[i] = z3;
  }

  for (uint16_t i = 0; i < EDGE_COUNT; i++) {
    uint16_t a = carEdges[i][0];
    uint16_t b = carEdges[i][1];

    uint16_t edgeColor =
      (pz[a] + pz[b] > 0.0f)
      ? TFT_CYAN
      : TFT_BLUE;

    sprite.drawLine(
      (int)px[a], (int)py[a],
      (int)px[b], (int)py[b],
      edgeColor
    );
  }
}

// =====================================================
// NORMAL CAR SCREEN
// =====================================================

void drawCarScreen() {
  sprite.fillSprite(TFT_BLACK);

  sprite.setTextDatum(TL_DATUM);
  sprite.setTextColor(TFT_CYAN, TFT_BLACK);
  sprite.setTextFont(2);
  sprite.setTextSize(1);
  sprite.setCursor(4, 4);
  sprite.print("DIECAST64 GARAGE");

  drawCar(
    currentRoll,
    currentPitch,
    currentYaw
  );

  drawFloatingText();

  sprite.pushSprite(0, 0);
}

void initNotificationAudio() {
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed!");
    return;
  }

  Serial.println("LittleFS mounted!");

  if (!LittleFS.exists(NOTIFICATION_AUDIO_FILE)) {
    Serial.println("Notification WAV file not found!");
    return;
  }

  i2s_config_t i2sConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = NOTIFICATION_AUDIO_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pinConfig = {
    .bck_io_num = NOTIFICATION_AUDIO_BCLK,
    .ws_io_num = NOTIFICATION_AUDIO_LRC,
    .data_out_num = NOTIFICATION_AUDIO_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  esp_err_t result = i2s_driver_install(I2S_NUM_0, &i2sConfig, 0, NULL);

  if (result != ESP_OK) {
    Serial.printf("I2S install failed: %d\n", result);
    return;
  }

  i2s_set_pin(I2S_NUM_0, &pinConfig);
  i2s_zero_dma_buffer(I2S_NUM_0);

  notificationAudioReady = true;
  Serial.println("Notification audio ready!");
}

// Streams a 16-bit PCM WAV straight to the MAX98357A. The sample rate is read
// from the file header, so a clip exported at 16 kHz still plays at the right
// pitch instead of being stretched over the 44.1 kHz clock.
bool playWavFile(const char* path) {
  if (!notificationAudioReady) return false;

  File file = LittleFS.open(path, "r");
  if (!file) {
    Serial.printf("[audio] missing %s\n", path);
    return false;
  }

  // The 44 byte canonical WAV header also carries the sample rate.
  uint8_t header[44];

  if (file.read(header, sizeof(header)) != sizeof(header)) {
    Serial.printf("[audio] %s is not a complete WAV file\n", path);
    file.close();
    return false;
  }

  const uint32_t sampleRate =
    (uint32_t)header[24] |
    ((uint32_t)header[25] << 8) |
    ((uint32_t)header[26] << 16) |
    ((uint32_t)header[27] << 24);

  if (sampleRate >= 8000 && sampleRate <= 96000) {
    i2s_set_sample_rates(I2S_NUM_0, sampleRate);
  } else {
    Serial.printf("[audio] %s reports %lu Hz, using %d Hz\n",
      path, (unsigned long)sampleRate, NOTIFICATION_AUDIO_SAMPLE_RATE);
    i2s_set_sample_rates(I2S_NUM_0, NOTIFICATION_AUDIO_SAMPLE_RATE);
  }

  uint8_t buffer[512];
  size_t bytesWritten;

  while (file.available()) {
    int bytesRead = file.read(buffer, sizeof(buffer));

    i2s_write(
      I2S_NUM_0,
      buffer,
      bytesRead,
      &bytesWritten,
      portMAX_DELAY
    );
  }

  file.close();
  return true;
}

void playNotificationAudio() {
  // Stay quiet while the walkie channel records, otherwise the speaker ends up
  // in the microphone signal and the loop stalls mid-recording.
  if (!notificationAudioReady || notificationAudioPlaying || walkieRecording) return;

  notificationAudioPlaying = true;
  playWavFile(NOTIFICATION_AUDIO_FILE);
  notificationAudioPlaying = false;
}

void playLightsAudio() {
  if (!notificationAudioReady || notificationAudioPlaying || walkieRecording) return;

  notificationAudioPlaying = true;
  playWavFile(LIGHTS_AUDIO_FILE);
  notificationAudioPlaying = false;
}

// "I am about to talk" tone. Blocks for the length of the clip, exactly like
// the other notification sounds.
void playWalkieSendAudio() {
  if (!notificationAudioReady || notificationAudioPlaying) return;

  notificationAudioPlaying = true;
  playWavFile(WALKIE_SEND_AUDIO_FILE);
  notificationAudioPlaying = false;
}

// =====================================================
// MICROPHONE / WALKIE TALKIE
// =====================================================

void initMicrophone() {
  if (!LittleFS.exists(WALKIE_SEND_AUDIO_FILE)) {
    Serial.printf("[mic] %s not found - send tone will be silent\n",
                  WALKIE_SEND_AUDIO_FILE);
  }

  i2s_config_t micConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = MIC_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = MIC_CHANNEL_FORMAT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = MIC_DMA_BUF_COUNT,
    .dma_buf_len = MIC_DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t micPins = {};
  micPins.bck_io_num = MIC_BCLK;
  micPins.ws_io_num = MIC_WS;
  micPins.data_out_num = I2S_PIN_NO_CHANGE;
  micPins.data_in_num = MIC_SD;
  micPins.mck_io_num = I2S_PIN_NO_CHANGE;

  esp_err_t result = i2s_driver_install(I2S_NUM_1, &micConfig, 0, NULL);

  if (result != ESP_OK) {
    Serial.printf("[mic] I2S install failed: %d\n", result);
    return;
  }

  result = i2s_set_pin(I2S_NUM_1, &micPins);

  if (result != ESP_OK) {
    Serial.printf("[mic] pin setup failed: %d\n", result);
    i2s_driver_uninstall(I2S_NUM_1);
    return;
  }

  i2s_zero_dma_buffer(I2S_NUM_1);
  i2s_stop(I2S_NUM_1);

  microphoneReady = true;
  Serial.println("[mic] ready");
}

// One readable line per MIC_PRINT_INTERVAL_MS: level in dBFS plus a small bar
// and a few raw sample values, so the numbers can be followed live.
void printMicLevel(
  const int16_t* samples,
  size_t count,
  int32_t peak,
  float rms,
  float dc,
  uint32_t elapsedMs
) {
#if DEBUG_SERIAL
  const uint32_t now = millis();
  if (now - lastMicPrint < MIC_PRINT_INTERVAL_MS) return;
  lastMicPrint = now;

  const float dbfs = (rms > 0.0f)
    ? 20.0f * log10f(rms / 32768.0f)
    : -96.0f;

  float level = (dbfs + 60.0f) / 60.0f;   // -60 dBFS .. 0 dBFS -> 0 .. 1
  if (level < 0.0f) level = 0.0f;
  if (level > 1.0f) level = 1.0f;

  const int filled = (int)(level * MIC_LEVEL_BAR_WIDTH + 0.5f);

  char bar[MIC_LEVEL_BAR_WIDTH + 1];

  for (int i = 0; i < MIC_LEVEL_BAR_WIDTH; i++) {
    bar[i] = (i < filled) ? '#' : '-';
  }
  bar[MIC_LEVEL_BAR_WIDTH] = '\0';

  Serial.printf(
    "[mic] %5.2fs n=%lu peak=%5ld rms=%6.1f dc=%6.1f %6.1f dBFS [%s] raw=(",
    (double)elapsedMs / 1000.0,
    (unsigned long)walkieSampleCount,
    (long)peak,
    (double)rms,
    (double)dc,
    (double)dbfs,
    bar
  );

  const size_t rawCount =
    (count < MIC_PRINT_RAW_SAMPLES) ? count : MIC_PRINT_RAW_SAMPLES;

  for (size_t i = 0; i < rawCount; i++) {
    Serial.print((int)samples[i]);
    if (i + 1 < rawCount) Serial.print(',');
  }

  Serial.println(")");
#endif
}

void readMicrophone() {
  if (!microphoneReady) return;

  int32_t rawSamples[MIC_READ_SAMPLES];
  int16_t samples[MIC_READ_SAMPLES];

  size_t bytesRead = 0;

  const esp_err_t result = i2s_read(
    I2S_NUM_1,
    rawSamples,
    sizeof(rawSamples),
    &bytesRead,
    0
  );

  (void)result;

  if (bytesRead == 0) return;

  const size_t count = bytesRead / sizeof(int32_t);

  int32_t peak = 0;
  int64_t sum = 0;
  int64_t sumSquares = 0;

  for (size_t i = 0; i < count; i++) {
    // Convert 32-bit INMP441 samples to 16-bit PCM.
    samples[i] = (int16_t)(rawSamples[i] >> 16);

    const int32_t sample = samples[i];
    const int32_t magnitude = (sample < 0) ? -sample : sample;

    if (magnitude > peak) peak = magnitude;

    sum += sample;
    sumSquares += (int64_t)sample * sample;
  }

  walkieSampleCount += count;

  // Send converted 16-bit PCM to MQTT.
  queueMicSamples(samples, count);

  const float rms = sqrtf((float)sumSquares / (float)count);
  const float dc = (float)sum / (float)count;

  printMicLevel(
    samples,
    count,
    peak,
    rms,
    dc,
    millis() - walkieStartTime
  );
}

void startWalkieRecording() {
  if (!microphoneReady) {
    Serial.println("[walkie] microphone not ready - nothing recorded");
    return;
  }

  // Half duplex: if the dashboard is currently talking through the speaker,
  // stop that stream before the microphone starts picking it up.
  stopSpeakerStream();

  // The I2S clock only runs while a transmission is in progress, so start it
  // first and then flush the DMA buffer: the microphone also hears the send
  // tone and none of that should end up in the transmission.
  i2s_start(I2S_NUM_1);
  i2s_zero_dma_buffer(I2S_NUM_1);

  walkieStartTime = millis();
  walkieSampleCount = 0;
  lastMicPrint = walkieStartTime;
  walkieRecording = true;

  // Tells every listener on the dashboard that a new transmission is starting,
  // then the packets below are streamed as they are captured.
  publishMicStart();

  Serial.printf(
    "[walkie] recording @ %d Hz - streaming to MQTT, release to stop\n",
    MIC_SAMPLE_RATE
  );
}

void stopWalkieRecording() {
  if (!walkieRecording) return;

  walkieRecording = false;

  // Clock the microphone back down: while it is running its bit clock sits next
  // to the MPU6050 I2C lines and the gyro readings get noisy, which latches
  // sensorMoving and freezes the car's auto-spin.
  i2s_stop(I2S_NUM_1);

  // Pushing the toy to reach the sensor, and the blocking send tone itself, both
  // read as movement. Clear the latch so the car picks its auto-spin back up
  // straight away instead of waiting for IDLE_DELAY_MS of detected stillness.
  sensorMoving = false;
  lastMotionTime = millis();

  const uint32_t durationMs = millis() - walkieStartTime;

  // Flushes the last partial packet and closes the transmission for the web
  // dashboard, which then turns the received packets into a playable clip.
  publishMicEnd(durationMs);

  Serial.printf(
    "[walkie] recording stopped: %.2f s, %lu samples @ %d Hz, %u packets sent\n",
    (double)durationMs / 1000.0,
    (unsigned long)walkieSampleCount,
    MIC_SAMPLE_RATE,
    (unsigned)walkieChunkIndex
  );
}

// Hold-to-talk. Called on every loop pass (not once per rendered frame) so the
// DMA buffers keep draining while the screen is being drawn.
void serviceWalkie() {
  const bool touchHeld = (digitalRead(TOUCH_PIN) == HIGH);

  if (walkieRecording) {
    if (touchHeld) {
      readMicrophone();
    } else {
      stopWalkieRecording();
      touchWasHeld = false;
    }
    return;
  }

  // Rising edge: announce the transmission, then record while it is held.
  if (touchHeld && !touchWasHeld) {
    touchWasHeld = true;

    playWalkieSendAudio();

    // The playback above blocks, so the finger may already be gone by now.
    if (digitalRead(TOUCH_PIN) != HIGH) {
      Serial.println("[walkie] released during send tone - nothing recorded");
      touchWasHeld = false;
      return;
    }

    startWalkieRecording();
    return;
  }

  if (!touchHeld) {
    touchWasHeld = false;
  }
}

// =====================================================
// REMOTE CONTROL (MQTT)
// =====================================================

const char* MQTT_HOST = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;
const char* TOPIC_PREFIX = "usman-esp32-9988";
const uint32_t MQTT_RETRY_MS = 5000;

WiFiClient netClient;
PubSubClient mqtt(netClient);

bool ledAOn = false;
bool ledBOn = false;

String topicLedASet;
String topicLedAState;
String topicLedBSet;
String topicLedBState;
String topicTextSet;
String topicTextState;

// Walkie audio. "mic" is the car's microphone (device -> dashboard) and
// "speaker" is the MAX98357A (dashboard -> device).
String topicMicStart;
String topicMicChunk;
String topicMicEnd;
String topicSpeakerStart;
String topicSpeakerChunk;
String topicSpeakerEnd;

void initTopics() {
  const String ledRoot = String(TOPIC_PREFIX) + "/led";
  const String textRoot = String(TOPIC_PREFIX) + "/text";
  const String audioRoot = String(TOPIC_PREFIX) + "/audio";
  const String micRoot = audioRoot + "/mic";
  const String speakerRoot = audioRoot + "/speaker";

  topicLedASet = ledRoot + "/a/set";
  topicLedAState = ledRoot + "/a/state";
  topicLedBSet = ledRoot + "/b/set";
  topicLedBState = ledRoot + "/b/state";

  topicTextSet = textRoot + "/set";
  topicTextState = textRoot + "/state";

  topicMicStart = micRoot + "/start";
  topicMicChunk = micRoot + "/chunk";
  topicMicEnd = micRoot + "/end";

  topicSpeakerStart = speakerRoot + "/start";
  topicSpeakerChunk = speakerRoot + "/chunk";
  topicSpeakerEnd = speakerRoot + "/end";
}

void publishLedStates() {
  mqtt.publish(topicLedAState.c_str(), ledAOn ? "ON" : "OFF", true);
  mqtt.publish(topicLedBState.c_str(), ledBOn ? "ON" : "OFF", true);
}

void publishFloatingTextState() {
  mqtt.publish(topicTextState.c_str(), floatingText.c_str(), true);
}

void applyFloatingText(const String& text) {
  setFloatingText(text);
  publishFloatingTextState();

  Serial.print("TFT text: ");
  Serial.println(floatingText);
}

// =====================================================
// WALKIE AUDIO STREAM (MQTT)
// =====================================================

// ---- car microphone -> dashboard ---------------------------------------

// Frames the buffered samples and hands them to the broker. The sequence number
// in front of the PCM lets the browser spot a lost packet, and the payload is a
// multiple of two bytes so every packet holds whole 16 bit samples.
void flushMicChunk() {
  if (walkieChunkFill == 0) return;

  walkieChunkFrame[0] = (uint8_t)(walkieChunkIndex & 0xFF);
  walkieChunkFrame[1] = (uint8_t)(walkieChunkIndex >> 8);

  if (mqtt.connected()) {
    mqtt.publish(
      topicMicChunk.c_str(),
      walkieChunkFrame,
      (unsigned int)(walkieChunkFill + 2),
      false
    );
  }

  walkieChunkIndex++;
  walkieChunkFill = 0;
}

// Appends freshly captured samples and publishes a packet every 32 ms of audio.
// Called from readMicrophone(), i.e. on every loop pass while the touch sensor
// is held down.
void queueMicSamples(const int16_t* samples, size_t count) {
  for (size_t i = 0; i < count; i++) {
    const uint16_t sample = (uint16_t)samples[i];

    // Little endian, exactly what an Int16Array in the browser expects.
    walkieChunkFrame[2 + walkieChunkFill] = (uint8_t)(sample & 0xFF);
    walkieChunkFrame[3 + walkieChunkFill] = (uint8_t)(sample >> 8);
    walkieChunkFill += 2;

    if (walkieChunkFill >= AUDIO_CHUNK_BYTES) {
      flushMicChunk();
    }
  }
}

// Announces the format of the transmission that is about to start. Not retained:
// a listener that connects later should not be told about a finished clip.
void publishMicStart() {
  walkieTransmissionId = millis();
  walkieChunkIndex = 0;
  walkieChunkFill = 0;

  if (!mqtt.connected()) {
    Serial.println("[walkie] MQTT offline - the audio cannot be published");
    return;
  }

  char payload[96];
  snprintf(
    payload, sizeof(payload),
    "{\"id\":%lu,\"sampleRate\":%d,\"channels\":1,\"bits\":16}",
    (unsigned long)walkieTransmissionId,
    AUDIO_STREAM_SAMPLE_RATE
  );

  mqtt.publish(topicMicStart.c_str(), payload, false);
}

void publishMicEnd(uint32_t durationMs) {
  flushMicChunk();

  if (!mqtt.connected()) return;

  char payload[128];
  snprintf(
    payload, sizeof(payload),
    "{\"id\":%lu,\"chunks\":%u,\"durationMs\":%lu}",
    (unsigned long)walkieTransmissionId,
    (unsigned)walkieChunkIndex,
    (unsigned long)durationMs
  );

  mqtt.publish(topicMicEnd.c_str(), payload, false);
}

// ---- dashboard -> car speaker ------------------------------------------

bool speakerStreamActive = false;   // true while dashboard audio is playing
bool speakerStreamMuted = false;    // stream dropped because the car was talking
uint32_t speakerStreamBytes = 0;
uint32_t speakerStreamStartMs = 0;
uint32_t speakerLastChunkMs = 0;    // last time a chunk arrived, for the timeout
uint32_t speakerSampleRate = AUDIO_STREAM_SAMPLE_RATE;

// Pulls a number out of a tiny JSON payload without pulling in a JSON library.
// Returns the fallback when the key is missing or not a number.
int32_t jsonIntValue(const String& json, const char* key, int32_t fallback) {
  const int keyIndex = json.indexOf(String("\"") + key + "\"");
  if (keyIndex < 0) return fallback;

  const int colon = json.indexOf(':', keyIndex + strlen(key));
  if (colon < 0) return fallback;

  // String::toInt() skips whitespace and stops at the first non digit, so the
  // closing quote and the fields after it are ignored.
  return (int32_t)json.substring(colon + 1).toInt();
}

void startSpeakerStream(uint32_t sampleRate) {
  if (!notificationAudioReady) {
    Serial.println("[speaker] audio output not ready - ignoring the stream");
    return;
  }

  // A new transmission replaces whatever was still playing.
  speakerStreamActive = false;
  speakerStreamMuted = false;
  notificationAudioPlaying = false;

  speakerStreamBytes = 0;
  speakerStreamStartMs = millis();
  speakerLastChunkMs = speakerStreamStartMs;

  // Half duplex, like a real walkie talkie: while the car is transmitting from
  // its own microphone, incoming audio is dropped instead of being played back
  // into the mic.
  if (walkieRecording) {
    speakerStreamMuted = true;
    Serial.println("[speaker] car is talking - incoming audio dropped");
    return;
  }

  speakerSampleRate =
    (sampleRate >= 8000 && sampleRate <= 48000)
    ? sampleRate
    : AUDIO_STREAM_SAMPLE_RATE;

  i2s_set_sample_rates(I2S_NUM_0, speakerSampleRate);
  i2s_zero_dma_buffer(I2S_NUM_0);

  // Notification clips stay quiet until the transmission ends.
  notificationAudioPlaying = true;
  speakerStreamActive = true;

  Serial.printf(
    "[speaker] streaming %lu Hz audio from the dashboard\n",
    (unsigned long)speakerSampleRate
  );
}

void stopSpeakerStream() {
  if (!speakerStreamActive && !speakerStreamMuted) return;

  const uint32_t durationMs = millis() - speakerStreamStartMs;

  speakerStreamActive = false;
  speakerStreamMuted = false;
  notificationAudioPlaying = false;

  Serial.printf(
    "[speaker] stream ended: %.2f s, %lu bytes\n",
    (double)durationMs / 1000.0,
    (unsigned long)speakerStreamBytes
  );

  speakerStreamBytes = 0;
}


// Safety net for a dashboard that vanishes mid-transmission (locked phone,
// dropped socket, closed tab): the "end" message never arrives, so the speaker
// would stay reserved and notification clips would never play again.
void serviceSpeakerStream() {
  if (!speakerStreamActive && !speakerStreamMuted) return;
  if (millis() - speakerLastChunkMs < SPEAKER_STREAM_TIMEOUT_MS) return;

  Serial.println("[speaker] stream timed out - releasing the output");
  stopSpeakerStream();
}

// The MAX98357A sits on a stereo I2S bus, so every mono sample is written to
// both channels. Handing the mono bytes straight to I2S would play a clip an
// octave low and at half speed. Returns false when the DMA stops accepting data.
bool writeSpeakerSamples(const uint8_t* data, size_t length) {
  static uint8_t stereo[AUDIO_CHUNK_BYTES * 2];

  const size_t samples = length / 2;

  for (size_t i = 0; i < samples; i++) {
    const uint8_t low = data[i * 2];
    const uint8_t high = data[i * 2 + 1];

    stereo[i * 4 + 0] = low;
    stereo[i * 4 + 1] = high;
    stereo[i * 4 + 2] = low;
    stereo[i * 4 + 3] = high;
  }

  const size_t total = samples * 4;
  const uint32_t start = millis();
  size_t written = 0;

  // i2s_write() queues the block for the DMA and only blocks while the buffer is
  // full, which is exactly the real time pacing the speaker needs. The budget
  // stops a stalled bus from freezing the main loop for ever.
  while (written < total) {
    if (millis() - start > AUDIO_WRITE_TIMEOUT_MS) return false;

    size_t accepted = 0;
    const esp_err_t result = i2s_write(
      I2S_NUM_0,
      stereo + written,
      total - written,
      &accepted,
      pdMS_TO_TICKS(50)
    );

    if (result != ESP_OK && result != ESP_ERR_TIMEOUT) return false;

    written += accepted;
  }

  return true;
}

void writeSpeakerChunk(const uint8_t* payload, unsigned int length) {
  if (!speakerStreamActive || length < 2) return;

  speakerLastChunkMs = millis();

  unsigned int offset = 0;

  // The dashboard sends ~1 kB packets; anything larger is written in slices so
  // the stereo conversion buffer never has to grow.
  while (offset + 1 < length) {
    unsigned int block = length - offset;
    if (block > AUDIO_CHUNK_BYTES) block = AUDIO_CHUNK_BYTES;
    if (block & 1) block--;   // whole samples only

    if (!writeSpeakerSamples(payload + offset, block)) {
      Serial.println("[speaker] write stalled - stopping the stream");
      stopSpeakerStream();
      return;
    }

    offset += block;
  }

  speakerStreamBytes += offset;
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  const String topicName(topic);

  // Audio is binary, so it is dispatched before the text branch below: building
  // a String and trimming it would destroy the PCM samples.
  if (topicName == topicSpeakerChunk) {
    writeSpeakerChunk(payload, length);
    return;
  }

  if (topicName == topicSpeakerStart) {
    String meta;
    meta.reserve(length + 1);

    for (unsigned int i = 0; i < length; i++) {
      meta += (char)payload[i];
    }

    startSpeakerStream(
      (uint32_t)jsonIntValue(meta, "sampleRate", AUDIO_STREAM_SAMPLE_RATE)
    );
    return;
  }

  if (topicName == topicSpeakerEnd) {
    stopSpeakerStream();
    return;
  }

  // Text must keep its original casing and spacing, so only trim the edges.
  String message;
  message.reserve(length);

  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();

  if (topicName == topicTextSet) {
    applyFloatingText(message);
    playNotificationAudio();
    return;
  }

  if (topicName == topicLedASet) {
    ledAOn = message.equalsIgnoreCase("ON");
    digitalWrite(LED_A_PIN, ledAOn ? HIGH : LOW);
    playLightsAudio();
    publishLedStates();
    return;
  }

  if (topicName == topicLedBSet) {
    ledBOn = message.equalsIgnoreCase("ON");
    digitalWrite(LED_B_PIN, ledBOn ? HIGH : LOW);
    playLightsAudio();
    publishLedStates();
    return;
  }
}

// =====================================================
// WIFI PROVISIONING
// =====================================================

WebServer server(80);
Preferences preferences;

bool provisioningMode = false;

const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
const uint32_t WIFI_RETRY_MS = 10000;

bool loadWifiCredentials(String& ssid, String& password) {
  preferences.begin("wifi", true);
  ssid = preferences.getString("ssid", "");
  password = preferences.getString("pass", "");
  preferences.end();

  return ssid.length() > 0;
}

bool connectToStoredWifi() {
  String ssid;
  String password;

  if (!loadWifiCredentials(ssid, password)) {
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());

  drawStatusScreen("CONNECTING", ssid, TFT_CYAN);

  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
  }

  return WiFi.status() == WL_CONNECTED;
}

void handleWifiSave() {
  if (server.hasArg("plain") == false) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }

  // Expected body: {"ssid":"...","password":"..."}
  String body = server.arg("plain");

  int sIdx = body.indexOf("\"ssid\":\"") + 8;
  int sEnd = body.indexOf("\"", sIdx);
  String ssid = body.substring(sIdx, sEnd);

  int pIdx = body.indexOf("\"password\":\"") + 12;
  int pEnd = body.indexOf("\"", pIdx);
  String pass = body.substring(pIdx, pEnd);

  preferences.begin("wifi", false);
  preferences.putString("ssid", ssid);
  preferences.putString("pass", pass);
  preferences.end();

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"status\":\"saved\"}");
  delay(1000);
  ESP.restart();
}

void startProvisioningAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32-Setup-AP", "12345678");

  server.on("/api/wifi", HTTP_POST, handleWifiSave);
  // Handle CORS preflight requests from the Next.js app
  server.on("/api/wifi", HTTP_OPTIONS, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "POST");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    server.send(204);
  });

  server.begin();

  provisioningMode = true;
  drawProvisioningScreen();

  Serial.println("Started provisioning AP: ESP32-Setup-AP");
}

// =====================================================
// SENSORS
// =====================================================

bool sensorReady = false;

uint32_t lastSensorRetry = 0;
const uint32_t SENSOR_RETRY_MS = 5000;

// Forget every motion related value. Called at boot and whenever the sensor is
// (re)initialised, so the animation always restarts from a known state.
void resetMotionState() {
  sensorMoving = false;
  lastMotionTime = millis();
  spinOffset = 0.0f;
  spinBlend = 0.0f;
  spinSpeed = 0.0f;
  yawAngle = 0.0f;
  yawRate = 0.0f;
}

void initSensors() {
  Wire.begin(SDA_PIN, SCL_PIN);

  const byte status = mpu.begin();

#if DEBUG_SERIAL
  Serial.printf("[mpu] begin() status=%u (0 = OK)\n", (unsigned)status);
#endif

  if (status != 0) {
    Serial.println("[mpu] connection failed - check SDA=32, SCL=33, 3V3 and GND");
    drawStatusScreen("MPU6050 ERROR", "CHECK WIRING", TFT_RED);
    sensorReady = false;
    lastSensorRetry = millis();
    return;
  }

  drawStatusScreen("CALIBRATING", "KEEP LEVEL", TFT_CYAN);
  delay(700);

  mpu.calcOffsets();
  mpu.update();

#if DEBUG_SERIAL
  Serial.printf(
    "[mpu] offsets gyro=(%.3f,%.3f,%.3f) deg/s acc=(%.4f,%.4f,%.4f) g\n",
    (double)mpu.getGyroXoffset(),
    (double)mpu.getGyroYoffset(),
    (double)mpu.getGyroZoffset(),
    (double)mpu.getAccXoffset(),
    (double)mpu.getAccYoffset(),
    (double)mpu.getAccZoffset()
  );
#endif

  currentRoll = targetRoll = previousRoll = mpu.getAngleX();
  currentPitch = targetPitch = previousPitch = mpu.getAngleY();

  // Yaw is integrated here (see updateOrientation). The library's angleZ is
  // never wrapped and keeps growing with the gyro bias, which is what made the
  // car creep round for ever even when the board was not moving.
  currentYaw = 0.0f;
  yawAngle = 0.0f;
  yawRate = 0.0f;

  resetMotionState();

  sensorReady = true;

  Serial.println("[mpu] ready - gyro follow armed");
}

void retrySensorIfNeeded() {
  if (millis() - lastSensorRetry < SENSOR_RETRY_MS) return;

  lastSensorRetry = millis();
  Serial.println("[mpu] retrying sensor init");
  initSensors();
}

void updateSensors() {
  if (!sensorReady) {
    debugWarnSensor();
    retrySensorIfNeeded();
    return;
  }

  mpu.update();

  gyroX = mpu.getGyroX();
  gyroY = mpu.getGyroY();
  gyroZ = mpu.getGyroZ();
  gyroMagnitude = sqrtf(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

  accX = mpu.getAccX();
  accY = mpu.getAccY();
  accZ = mpu.getAccZ();

  targetRoll = mpu.getAngleX();
  targetPitch = mpu.getAngleY();

  // Latch "moving" the moment the gyro wakes up. The hysteresis (enter > exit)
  // keeps the car in gyro-follow mode until the readings are really quiet, so
  // the animation cannot flicker between following and auto-spinning.
  const float motionThreshold =
    sensorMoving ? MOTION_EXIT_THRESHOLD : MOTION_ENTER_THRESHOLD;

  if (gyroMagnitude > motionThreshold) {
    lastMotionTime = millis();
    sensorMoving = true;
  }
}

void updateOrientation(float dt) {
  const uint32_t now = millis();

  // ---- movement detection ---------------------------------------------
  // The gyro is the primary signal, but a real tilt change counts too, so a
  // slow steady rotation still takes the car out of the auto-spin.
  const float tiltDelta =
    fabsf(targetRoll - previousRoll) + fabsf(targetPitch - previousPitch);

  previousRoll = targetRoll;
  previousPitch = targetPitch;

  if (tiltDelta > MOTION_ANGLE_THRESHOLD) {
    lastMotionTime = now;
    sensorMoving = true;
  }

  if (sensorMoving && (now - lastMotionTime) > IDLE_DELAY_MS) {
    sensorMoving = false;
  }

  // ---- yaw ------------------------------------------------------------
  // Integrate the gyro only while the board is moving. The residual Z bias
  // would otherwise keep the car turning for ever. As soon as it goes still the
  // yaw unwinds gently and the auto-spin takes over again.
  const float gyroZClean = (fabsf(gyroZ) < YAW_GYRO_DEADBAND) ? 0.0f : gyroZ;

  yawRate += (gyroZClean - yawRate) * (1.0f - expf(-dt / YAW_RATE_SMOOTHING_TIME));

  if (sensorMoving) {
    yawAngle += yawRate * dt;
  } else {
    yawAngle -= yawAngle * (1.0f - expf(-dt / YAW_RETURN_TIME));
    yawRate = 0.0f;
  }

  // Keep the angle bounded, but shift the smoothed value by the same whole
  // number of turns so wrapping never produces a visual jump.
  if (yawAngle > 180.0f || yawAngle < -180.0f) {
    const float turns = floorf((yawAngle + 180.0f) / 360.0f);
    yawAngle -= turns * 360.0f;
    currentYaw -= turns * 360.0f;
  }

  // ---- smoothing ------------------------------------------------------
  currentRoll += (targetRoll - currentRoll) * SMOOTHING;
  currentPitch += (targetPitch - currentPitch) * SMOOTHING;
  currentYaw += (yawAngle - currentYaw) * SMOOTHING;

  // ---- auto-spin ------------------------------------------------------
  // Fades out quickly when movement is detected and eases back in after
  // IDLE_DELAY_MS of stillness. exp() keeps it frame rate independent.
  const float targetBlend = sensorMoving ? 0.0f : 1.0f;
  const float spinTime = sensorMoving ? SPIN_STOP_TIME : SPIN_START_TIME;

  spinBlend += (targetBlend - spinBlend) * (1.0f - expf(-dt / spinTime));

  // Snap the last fraction so the car really sits still while it is moved.
  if (sensorMoving && spinBlend < 0.002f) spinBlend = 0.0f;
  if (!sensorMoving && spinBlend > 0.998f) spinBlend = 1.0f;

  spinSpeed = AUTO_SPIN_SPEED * spinBlend;
  spinOffset += spinSpeed * dt;

  if (spinOffset >= 360.0f) spinOffset -= 360.0f;
  if (spinOffset < 0.0f) spinOffset += 360.0f;

  // ---- state change log -----------------------------------------------
  static bool loggedMoving = false;

  if (sensorMoving != loggedMoving) {
    loggedMoving = sensorMoving;
    debugLogMotion(sensorMoving);
  }
}

void renderFrame() {
  if (digitalRead(TOUCH_PIN) == HIGH) {
    drawTouchAnimation();
  } else {
    drawCarScreen();
  }
}

// =====================================================
// CONNECTION MANAGEMENT
// =====================================================

void initMqtt() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);

  // Room for full-length floating-text payloads plus the walkie audio packets
  // (AUDIO_CHUNK_BYTES + 2). The audio frames are the largest thing either end
  // sends, so this sizing is what keeps them from being dropped.
  mqtt.setBufferSize(AUDIO_CHUNK_BYTES + 64);

  // connect() blocks. With the default 15 s socket timeout an unreachable
  // broker freezes the loop, which also starves the gyro sampling (the idle
  // timer then fires and the car starts auto-spinning again).
  mqtt.setSocketTimeout(2);
  mqtt.setKeepAlive(15);
}

void remoteLoop() {
  serviceSpeakerStream();

  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastWifiRetry = 0;
    if (millis() - lastWifiRetry > WIFI_RETRY_MS) {
      lastWifiRetry = millis();
      WiFi.reconnect();
    }
    return;
  }

  if (!mqtt.connected()) {
    static uint32_t lastMqttAttempt = 0;
    if (millis() - lastMqttAttempt < MQTT_RETRY_MS) return;
    lastMqttAttempt = millis();

    const String clientId = "esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);

    if (!mqtt.connect(clientId.c_str())) {
      Serial.print("MQTT failed, state ");
      Serial.println(mqtt.state());
      return;
    }

    Serial.println("MQTT connected");

    mqtt.subscribe(topicLedASet.c_str());
    mqtt.subscribe(topicLedBSet.c_str());
    mqtt.subscribe(topicTextSet.c_str());

    // Walkie audio can only be pushed to the speaker while we hold this
    // connection open, and the dashboard only talks while the user holds the
    // button, so the subscription is always on.
    mqtt.subscribe(topicSpeakerStart.c_str());
    mqtt.subscribe(topicSpeakerChunk.c_str());
    mqtt.subscribe(topicSpeakerEnd.c_str());

    publishLedStates();
    publishFloatingTextState();
  }

  mqtt.loop();
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  pinMode(TOUCH_PIN, INPUT);

  // LEDs off at boot
  pinMode(LED_A_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  digitalWrite(LED_A_PIN, LOW);
  digitalWrite(LED_B_PIN, LOW);

  initDisplay();
  bootAnimation();
  initNotificationAudio();
  initTopics();

  // The IMU has to be initialised and calibrated while the microphone is still
  // silent. initMicrophone() installs the I2S RX driver, and if that runs first
  // its bit clock is already ticking during calcOffsets(), which can skew the
  // gyro offsets enough that the car never leaves the "moving" state again.
  initSensors();
  initMicrophone();

  if (!connectToStoredWifi()) {
    startProvisioningAP();
    return;
  }

  initMqtt();

  debugPrintConfig();
  resetMotionState();

  lastFrame = millis();

  Serial.println("DIECAST 64 SYSTEM ONLINE");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  if (provisioningMode) {
    server.handleClient();
    delay(5);
    return;
  }

  remoteLoop();
  updateSensors();

  // Serviced every pass, before the frame gate: hold-to-talk must keep reading
  // the microphone even while no new frame is due.
  serviceWalkie();

  const uint32_t now = millis();

  if (now - lastFrame < FRAME_INTERVAL_MS) {
    return;
  }

  float dt = (now - lastFrame) / 1000.0f;
  if (dt > 0.1f) dt = 0.1f;   // guard against a long frame
  lastFrame = now;

  if (dt > 0.0f) {
    const float instantFps = 1.0f / dt;
    measuredFps = (measuredFps <= 0.0f)
      ? instantFps
      : (measuredFps * 0.9f + instantFps * 0.1f);
  }

  updateOrientation(dt);
  updateFloatingText(dt);
  renderFrame();

  debugPrintState();
}
