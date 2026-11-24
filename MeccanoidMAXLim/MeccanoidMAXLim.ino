/*
 * Experimental Meccano M.A.X. profile for BubblAI.
 * Two Smart Servos, head (ID 0), claw (ID 1), data GPIO25.
 * 5 V regulated external module supply, common GND, 3.3 V-safe bidirectional interface.
 * No automatic movement at boot. No wheel, IR or face driver in this first test.
 * Based on the G15 project's MeccanoidESP32 transport and BLE UART commands.
 * Hardware validation required: particularly LIM 0xF9 and claw mechanical travel.
 */
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <LittleFS.h>
#include "MeccanoidESP32.h"

#define UART_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define UART_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define UART_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

constexpr int PIN_SERVOS = 25;
// Set true only for the first boot of a dedicated blank MAX filesystem.
// Keep false on a board with recordings or other data to preserve.
constexpr bool FORMAT_NEW_MAX_FS = false;
constexpr uint32_t MAX_RECORD_MS = 60000;
constexpr uint32_t FEEDBACK_TIMEOUT = 1500;
constexpr uint16_t MAX_SAMPLES = 600;
constexpr uint32_t MAGIC = 0x4D415831; // MAX1, separate from G15 storage
constexpr int MAX_STEP = 3; // limit commanded angular step, including alignment

Chain bus(PIN_SERVOS);
MeccanoServo head = bus.getServo(0);
MeccanoServo claw = bus.getServo(1);

struct Sample {
  uint32_t ms;
  uint8_t headAngle;
  uint8_t clawAngle;
  uint8_t reserved[2];
};
struct Header {
  uint32_t magic;
  uint16_t count;
  uint8_t mask;
  uint8_t reserved;
};
static_assert(sizeof(Sample) == 8, "Sample format");
static_assert(sizeof(Header) == 8, "Header format");

enum Mode { IDLE, RECORD, ALIGN, PLAY };
Mode mode = IDLE;
Sample samples[MAX_SAMPLES];
uint16_t count = 0, cursor = 0;
uint8_t activeMask = 0;
uint32_t lastSeq[2] = {}, lastGood[2] = {}, capturedSeq[2] = {};
uint32_t recordStarted = 0, firstSampleAt = 0, playStarted = 0;
int commandedHead = 0, commandedClaw = 0;
bool holding = false, fsReady = false;
String recordName;

constexpr uint8_t QUEUE_LEN = 8;
constexpr size_t FRAME_LEN = 192;
char queueFrames[QUEUE_LEN][FRAME_LEN] = {};
uint8_t queueRead = 0, queueWrite = 0;
bool emergency = false;
portMUX_TYPE queueMux = portMUX_INITIALIZER_UNLOCKED;
BLEServer *bleServer = nullptr;
String serialFrame;

void queueFrame(const char *s) {
  const size_t len = strlen(s);
  portENTER_CRITICAL(&queueMux);
  const uint8_t next = (queueWrite + 1) % QUEUE_LEN;
  if (len >= FRAME_LEN || next == queueRead) emergency = true;
  else {
    memcpy(queueFrames[queueWrite], s, len + 1);
    queueWrite = next;
  }
  portEXIT_CRITICAL(&queueMux);
}

class Receiver : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String message = characteristic->getValue().c_str();
    message.trim();
    if (message.length()) queueFrame(message.c_str());
  }
};
class ServerEvents : public BLEServerCallbacks {
  void onDisconnect(BLEServer *) override {
    portENTER_CRITICAL(&queueMux);
    emergency = true;
    portEXIT_CRITICAL(&queueMux);
  }
};

void releaseAll() {
  mode = IDLE;
  holding = false;
  // Queue release even if a module is missing. Discovery also defaults to LIM.
  head.setLim(true);
  claw.setLim(true);
}

bool ready() {
  const uint32_t now = millis();
  return head.hasValidPosition() && claw.hasValidPosition() &&
    now - lastGood[0] <= FEEDBACK_TIMEOUT &&
    now - lastGood[1] <= FEEDBACK_TIMEOUT;
}

void refreshFeedback() {
  uint32_t seq[2] = {head.getInputSequence(), claw.getInputSequence()};
  for (int i = 0; i < 2; ++i) {
    if (seq[i] != lastSeq[i]) {
      lastSeq[i] = seq[i];
      lastGood[i] = millis();
    }
  }
}

bool validName(const String &name) {
  if (!name.length() || name.length() > 24) return false;
  for (size_t i = 0; i < name.length(); ++i) {
    char c = name[i];
    if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_')) return false;
  }
  const char *reserved[] = {"STOP","LIM","POSITIONS","DIAGNOSTIC","LISTE_MOUVEMENTS","FIN_ENREGISTREMENT"};
  for (auto s : reserved) if (name == s) return false;
  return !name.startsWith("ENREGISTRE_") && !name.startsWith("SUPPRIME_") &&
         !name.startsWith("YEUX_");
}
String pathFor(const String &name) {
  return String("/MAX_") + name + ".bin";
}

uint8_t movementMask() {
  if (count < 2) return 0;
  uint8_t hmin = 180, hmax = 0, cmin = 180, cmax = 0;
  for (uint16_t i = 0; i < count; ++i) {
    if (samples[i].headAngle < hmin) hmin = samples[i].headAngle;
    if (samples[i].headAngle > hmax) hmax = samples[i].headAngle;
    if (samples[i].clawAngle < cmin) cmin = samples[i].clawAngle;
    if (samples[i].clawAngle > cmax) cmax = samples[i].clawAngle;
  }
  return (hmax - hmin >= 4 ? 1 : 0) | (cmax - cmin >= 4 ? 2 : 0);
}

bool saveCapture() {
  if (!fsReady || count < 2) return false;
  const uint8_t mask = movementMask();
  if (!mask) return false;
  String path = pathFor(recordName);
  String tmp = path + ".tmp";
  if (LittleFS.exists(path.c_str())) return false; // no silent overwrite
  if (LittleFS.exists(tmp.c_str())) LittleFS.remove(tmp.c_str());
  File file = LittleFS.open(tmp.c_str(), "w");
  if (!file) return false;
  Header header = {MAGIC, count, mask, 0};
  bool ok = file.write(reinterpret_cast<const uint8_t *>(&header), sizeof(header)) == sizeof(header)
    && file.write(reinterpret_cast<const uint8_t *>(samples), count * sizeof(Sample)) == count * sizeof(Sample);
  file.close();
  if (ok) ok = LittleFS.rename(tmp.c_str(), path.c_str());
  if (!ok) LittleFS.remove(tmp.c_str());
  return ok;
}

void finishCapture() {
  if (mode != RECORD) { Serial.println("No recording"); return; }
  releaseAll();
  if (saveCapture()) {
    Serial.print("FIN_MOUVEMENT "); Serial.println(recordName);
  } else Serial.println("Recording not saved (too short, no movement, or filesystem error)");
}

void startCapture(const String &name) {
  if (mode != IDLE || !ready() || !fsReady || !validName(name)) {
    Serial.println("Recording refused: idle, two live servos, valid name and LittleFS required");
    return;
  }
  if (LittleFS.exists(pathFor(name).c_str())) {
    Serial.println("Name already exists; delete explicitly or choose another name");
    return;
  }
  releaseAll();
  count = 0;
  recordName = name;
  capturedSeq[0] = lastSeq[0];
  capturedSeq[1] = lastSeq[1];
  recordStarted = millis();
  mode = RECORD;
  Serial.print("DEBUT_MOUVEMENT "); Serial.println(recordName);
  Serial.println("Check that both servos are free before moving them gently by hand");
}

void captureTick() {
  uint32_t now = millis();
  if (now - recordStarted >= MAX_RECORD_MS || count >= MAX_SAMPLES) {
    finishCapture(); return;
  }
  // Only store a frame after both servos provided a new valid measurement.
  if (lastSeq[0] == capturedSeq[0] || lastSeq[1] == capturedSeq[1]) return;
  capturedSeq[0] = lastSeq[0];
  capturedSeq[1] = lastSeq[1];
  if (count == 0) firstSampleAt = now;
  Sample point = {now - firstSampleAt, uint8_t(head.getPosition()), uint8_t(claw.getPosition()), {0,0}};
  samples[count++] = point;
  Serial.printf("%u;%lu;%u;%u\n", unsigned(count), (unsigned long)point.ms,
      unsigned(point.headAngle), unsigned(point.clawAngle));
}

bool loadCapture(const String &name) {
  if (!fsReady || !validName(name)) return false;
  File file = LittleFS.open(pathFor(name).c_str(), "r");
  if (!file) return false;
  Header header = {};
  bool ok = file.read(reinterpret_cast<uint8_t *>(&header), sizeof(header)) == sizeof(header)
    && header.magic == MAGIC && header.count >= 2 && header.count <= MAX_SAMPLES
    && header.mask >= 1 && header.mask <= 3
    && file.size() == sizeof(Header) + header.count * sizeof(Sample);
  if (ok) ok = file.read(reinterpret_cast<uint8_t *>(samples), header.count * sizeof(Sample))
    == header.count * sizeof(Sample);
  file.close();
  if (!ok) return false;
  for (uint16_t i = 0; i < header.count; ++i) {
    if (samples[i].headAngle > 180 || samples[i].clawAngle > 180 ||
        samples[i].ms > MAX_RECORD_MS || (i == 0 && samples[i].ms != 0) ||
        (i && samples[i].ms <= samples[i-1].ms)) return false;
  }
  count = header.count;
  activeMask = header.mask;
  return movementMask() == activeMask;
}

void startPlayback(const String &name) {
  if (mode != IDLE || !ready() || !loadCapture(name)) {
    Serial.println("Playback refused: missing movement, invalid data, or servo feedback");
    return;
  }
  releaseAll();
  commandedHead = head.getPosition();
  commandedClaw = claw.getPosition();
  cursor = 0;
  mode = ALIGN;
  holding = true;
  Serial.print("LECTURE_MOUVEMENT "); Serial.println(name);
}

bool approach(int &value, int target) {
  int delta = target - value;
  value += constrain(delta, -MAX_STEP, MAX_STEP);
  return value == target;
}
bool commandTargets(int h, int c) {
  bool done = true;
  if (activeMask & 1) {
    if (!approach(commandedHead, h)) done = false;
    head.setPosition(commandedHead);
  }
  if (activeMask & 2) {
    if (!approach(commandedClaw, c)) done = false;
    claw.setPosition(commandedClaw);
  }
  return done;
}
int interpolateAngle(int a, int b, uint32_t elapsed, uint32_t duration) {
  if (!duration || elapsed >= duration) return b;
  return a + (int32_t(b-a) * int32_t(elapsed)) / int32_t(duration);
}

void playbackTick() {
  if (mode == ALIGN) {
    if (commandTargets(samples[0].headAngle, samples[0].clawAngle)) {
      playStarted = millis();
      mode = PLAY;
    }
    return;
  }
  uint32_t elapsed = millis() - playStarted;
  while (cursor + 1 < count && samples[cursor+1].ms <= elapsed) ++cursor;
  if (cursor + 1 == count) {
    if (commandTargets(samples[cursor].headAngle, samples[cursor].clawAngle)) {
      mode = IDLE;
      Serial.println("Playback complete; STOP or LIM releases the servos");
    }
    return;
  }
  const Sample &a = samples[cursor], &b = samples[cursor+1];
  uint32_t dt = elapsed - a.ms, span = b.ms - a.ms;
  commandTargets(interpolateAngle(a.headAngle,b.headAngle,dt,span),
                 interpolateAngle(a.clawAngle,b.clawAngle,dt,span));
}

void listMovements() {
  if (!fsReady) { Serial.println("LittleFS unavailable"); return; }
  File root = LittleFS.open("/");
  if (!root) return;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    String name = f.name();
    if (name.startsWith("/")) name.remove(0,1);
    if (name.startsWith("MAX_") && name.endsWith(".bin"))
      Serial.println(name.substring(4, name.length()-4));
    f.close();
  }
  root.close();
}
void diagnostic() {
  Serial.printf("HEAD ID0: %s angle=%d; CLAW ID1: %s angle=%d; ready=%s; LIM=0x%02X\n",
    head.hasValidPosition() ? "OK" : "MISSING",
    head.hasValidPosition() ? head.getPosition() : -1,
    claw.hasValidPosition() ? "OK" : "MISSING",
    claw.hasValidPosition() ? claw.getPosition() : -1,
    ready() ? "YES" : "NO", unsigned(LIM_BYTE));
}

void execute(String cmd) {
  cmd.trim(); cmd.toUpperCase();
  if (!cmd.length()) return;
  if (cmd == "STOP") {
    if (mode == RECORD) finishCapture();
    else releaseAll();
    return;
  }
  if (cmd == "LIM") { releaseAll(); return; }
  if (cmd == "DIAGNOSTIC" || cmd == "POSITIONS") { diagnostic(); return; }
  if (cmd == "FIN_ENREGISTREMENT") { finishCapture(); return; }
  if (mode != IDLE) { Serial.println("Busy; send STOP first"); return; }
  if (cmd == "LISTE_MOUVEMENTS") { listMovements(); return; }
  if (cmd.startsWith("ENREGISTRE_")) { startCapture(cmd.substring(11)); return; }
  if (cmd.startsWith("SUPPRIME_")) {
    String name = cmd.substring(9);
    if (fsReady && validName(name) && LittleFS.remove(pathFor(name).c_str())) Serial.println("Deleted");
    else Serial.println("Delete refused");
    return;
  }
  if (cmd.startsWith("YEUX_")) { Serial.println("MAX face driver not implemented"); return; }
  startPlayback(cmd);
}

void processFrame(String message) {
  // Accept existing BubblAI commands, with/without brackets.
  // STOP takes precedence over other commands in a single frame.
  message.replace("[",""); message.replace("]",""); message.toUpperCase();
  int from = 0;
  while (from < int(message.length())) {
    int comma = message.indexOf(',', from);
    if (comma < 0) comma = message.length();
    String cmd = message.substring(from, comma); cmd.trim();
    if (cmd == "STOP") { execute(cmd); return; }
    from = comma + 1;
  }
  // Send one movement command per message. For compound commands use the last,
  // as in G15 priority handling; face commands are not yet supported.
  String last;
  from = 0;
  while (from < int(message.length())) {
    int comma = message.indexOf(',', from);
    if (comma < 0) comma = message.length();
    String cmd = message.substring(from, comma); cmd.trim();
    if (cmd.startsWith("YEUX_")) Serial.println("MAX face driver not implemented");
    else if (cmd.length()) last = cmd;
    from = comma + 1;
  }
  if (last.length()) execute(last);
}

void processInputs() {
  bool restartBLE = false;
  char frame[FRAME_LEN] = {};
  portENTER_CRITICAL(&queueMux);
  if (emergency) {
    emergency = false;
    queueRead = queueWrite;
    restartBLE = true;
  } else if (queueRead != queueWrite) {
    memcpy(frame, queueFrames[queueRead], FRAME_LEN);
    queueRead = (queueRead + 1) % QUEUE_LEN;
  }
  portEXIT_CRITICAL(&queueMux);
  if (restartBLE) {
    releaseAll(); // do not save a partial capture on disconnect/overflow
    Serial.println("BLE disconnected or queue overflow: LIM");
    if (bleServer) bleServer->startAdvertising();
  }
  if (frame[0]) processFrame(String(frame));
  while (Serial.available()) {
    char c = char(Serial.read());
    if (c == '[') serialFrame = "";
    else if (c == ']' || c == '\n' || c == '\r') {
      if (serialFrame.length()) processFrame(serialFrame);
      serialFrame = "";
    } else if (serialFrame.length() < FRAME_LEN-1) serialFrame += c;
    else { serialFrame = ""; releaseAll(); }
  }
}

void setup() {
  Serial.begin(9600);
  head.setAutoUpdate(false);
  claw.setAutoUpdate(false);
  fsReady = LittleFS.begin(FORMAT_NEW_MAX_FS); // false by default: do not format existing data
  if (!fsReady) Serial.println("LittleFS mount failed: initialize a dedicated MAX partition before recording");
  releaseAll();
  BLEDevice::init("Mecanoid");
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerEvents());
  BLEService *service = bleServer->createService(UART_SERVICE);
  BLECharacteristic *rx = service->createCharacteristic(UART_RX, BLECharacteristic::PROPERTY_WRITE);
  rx->setCallbacks(new Receiver());
  BLECharacteristic *tx = service->createCharacteristic(UART_TX, BLECharacteristic::PROPERTY_NOTIFY);
  tx->addDescriptor(new BLE2902());
  service->start();
  bleServer->getAdvertising()->start();
  Serial.println("MAX experimental LIM; GPIO25 HEAD -> CLAW. No movement on boot.");
  Serial.println("[DIAGNOSTIC] [LIM] [ENREGISTRE_NOM] [FIN_ENREGISTREMENT] [NOM] [STOP]");
}

void loop() {
  processInputs();
  bus.update();
  refreshFeedback();
  if ((mode != IDLE || holding) && !ready()) {
    releaseAll();
    Serial.println("Servo feedback lost: LIM, recording discarded");
  }
  if (mode == RECORD) captureTick();
  else if (mode == ALIGN || mode == PLAY) playbackTick();
}
