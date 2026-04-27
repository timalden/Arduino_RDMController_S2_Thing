/*
  RDM Controller — S3HW companion
  ESP32-S2 Thing Plus: TX=34, RX=33, EN=3

  Serial commands mirror the S3HW dimmer interface but execute via RDM:
    discover            re-run RDM discovery, list found devices
    dev <n>             select active device by index (default 0)
    dev?                show active device UID and index

    addr <n>            SET DMX start address (1-512)
    addr?               GET DMX start address

    curve <n>           SET all channels to curve (sq/log/lin/tmp)
    curve <ch> <n>      SET single channel curve, ch=1-6 (sq/log/lin/tmp)
    curve?              GET per-channel curves

    label <str>         SET rig/circuit label (max 32 chars)
    label?              GET rig/circuit label

    mode <n>            SET operating mode (0=normal 1=test 2=safe)
    mode?               GET operating mode

    info                GET DEVICE_INFO + list manufacturer PIDs
    status              show active device + address + curves + label + mode
    help                show this list
*/

#include <Arduino.h>
#include <esp_dmx.h>
#include <rdm/controller.h>
#include "custom_controller.h"  // rdm_send_get/set_current_curve, RDM_PID_MFR_CURVE

// ============================================================
// Hardware
// ============================================================
const int transmitPin = 34;
const int receivePin  = 33;
const int enablePin   = 3;
const dmx_port_t dmxPort = 1;

// ============================================================
// RDM state
// ============================================================
#define MAX_DEVICES 32
#define NUM_CHANNELS 6

rdm_uid_t uids[MAX_DEVICES];
int       devicesFound  = 0;
int       activeDevice  = 0;   // index into uids[]

// ============================================================
// DMX output buffer — channel 1 at index 1, index 0 = start code
// ============================================================
uint8_t dmxBuffer[DMX_PACKET_SIZE] = {0};  // all channels start at zero

// ============================================================
// Curve helpers — match S3HW naming exactly
// ============================================================
const char *curveNames[4] = { "sq", "log", "lin", "tmp" };
const char *curveFullNames[4] = { "square law", "logarithmic", "linear", "Tempus analogue" };

int parseCurveName(const String &s) {
  if (s == "sq")  return 0;
  if (s == "log") return 1;
  if (s == "lin") return 2;
  if (s == "tmp") return 3;
  return -1;
}

const char *curveName(uint8_t id) {
  return (id < 4) ? curveFullNames[id] : "?";
}

// ============================================================
// Discovery
// ============================================================
void doDiscover() {
  Serial.println("Discovering RDM devices...");
  devicesFound = rdm_discover_devices_simple(dmxPort, uids, MAX_DEVICES);
  if (devicesFound == 0) {
    Serial.println("No RDM devices found.");
    return;
  }
  Serial.printf("%d device(s) found:\n", devicesFound);
  for (int i = 0; i < devicesFound; i++) {
    Serial.printf("[%d] " UIDSTR "%s\n", i, UID2STR(uids[i]),
                  i == activeDevice ? " <-- active" : "");
    printMfrPids(uids[i]);
  }
  if (activeDevice >= devicesFound) {
    activeDevice = 0;
    Serial.printf("Active device reset to [0].\n");
  }
}

// ============================================================
// Active device helpers
// ============================================================
bool checkDevice() {
  if (devicesFound == 0) {
    Serial.println("No devices found — run 'discover' first.");
    return false;
  }
  if (activeDevice >= devicesFound) {
    Serial.println("Invalid device index — run 'dev <n>'.");
    return false;
  }
  return true;
}

rdm_uid_t activeUID()      { return uids[activeDevice]; }
rdm_sub_device_t subDev()  { return RDM_SUB_DEVICE_ROOT; }

// ============================================================
// RDM command implementations
// ============================================================
void cmdAddrGet() {
  if (!checkDevice()) return;
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  uint16_t addr;
  if (rdm_send_get_dmx_start_address(dmxPort, &dest, subDev(), &addr, &ack)) {
    Serial.printf("DMX start address: %d\n", addr);
  } else {
    Serial.printf("GET DMX_START_ADDRESS failed (NACK 0x%02x)\n", ack.nack_reason);
  }
}

void cmdAddrSet(uint16_t addr) {
  if (!checkDevice()) return;
  if (addr < 1 || addr > 512) {
    Serial.println("Address out of range (1-512)");
    return;
  }
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  if (rdm_send_set_dmx_start_address(dmxPort, &dest, subDev(), addr, &ack)) {
    Serial.printf("DMX start address set to: %d\n", addr);
  } else {
    Serial.printf("SET DMX_START_ADDRESS failed (NACK 0x%02x)\n", ack.nack_reason);
  }
}

void cmdCurveGet() {
  if (!checkDevice()) return;
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  rdm_curve_id_t curves[NUM_CHANNELS];
  if (rdm_send_get_current_curve(dmxPort, &dest, subDev(), curves, &ack)) {
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
      Serial.printf("  CH%d: %s\n", ch + 1, curveName(curves[ch]));
    }
  } else {
    Serial.printf("GET MFR_CURVE failed (NACK 0x%02x)\n", ack.nack_reason);
  }
}

void cmdCurveSetAll(int curveId) {
  if (!checkDevice()) return;
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  rdm_curve_id_t curves[NUM_CHANNELS];
  for (int ch = 0; ch < NUM_CHANNELS; ch++) curves[ch] = (rdm_curve_id_t)curveId;
  if (rdm_send_set_current_curve(dmxPort, &dest, subDev(), curves, &ack)) {
    Serial.printf("All channels curve set to: %s\n", curveName(curveId));
  } else {
    Serial.printf("SET MFR_CURVE failed (NACK 0x%02x)\n", ack.nack_reason);
  }
}

void cmdCurveSetChannel(int ch, int curveId) {
  if (!checkDevice()) return;
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  // Use 0xFF mask for all other channels — responder preserves existing values
  rdm_curve_id_t curves[NUM_CHANNELS];
  for (int i = 0; i < NUM_CHANNELS; i++) curves[i] = 0xFF;
  curves[ch] = (rdm_curve_id_t)curveId;
  if (rdm_send_set_current_curve(dmxPort, &dest, subDev(), curves, &ack)) {
    Serial.printf("CH%d curve set to: %s\n", ch + 1, curveName(curveId));
  } else {
    Serial.printf("SET MFR_CURVE CH%d failed (NACK 0x%02x)\n", ch + 1, ack.nack_reason);
  }
}

void cmdLabelGet() {
  if (!checkDevice()) return;
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  char buf[RDM_MFR_LABEL_SIZE + 1];
  if (rdm_send_get_mfr_label(dmxPort, &dest, subDev(), buf, sizeof(buf), &ack)) {
    Serial.printf("Label: \"%s\"\n", buf);
  } else {
    Serial.printf("GET MFR_LABEL failed (NACK 0x%02x)\n", ack.nack_reason);
  }
}

void cmdLabelSet(const String &label) {
  if (!checkDevice()) return;
  if (label.length() > RDM_MFR_LABEL_SIZE) {
    Serial.printf("Label too long (max %d chars)\n", RDM_MFR_LABEL_SIZE);
    return;
  }
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  size_t len = strnlen(label.c_str(), RDM_MFR_LABEL_SIZE);
  if (rdm_send_set_mfr_label(dmxPort, &dest, subDev(), label.c_str(), &ack)) {
    Serial.printf("Label set to: \"%s\"\n", label.c_str());
  } else {
    Serial.printf("SET MFR_LABEL failed (NACK 0x%02x)\n", ack.nack_reason);
  }
}

void cmdModeGet() {
  if (!checkDevice()) return;
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  rdm_mfr_mode_t mode = 0;
  const char *modeNames[] = { "normal", "test", "safe" };
  if (rdm_send_get_mfr_mode(dmxPort, &dest, subDev(), &mode, &ack)) {
    const char *name = (mode < 3) ? modeNames[mode] : "?";
    Serial.printf("Mode: %d (%s)\n", mode, name);
  } else {
    Serial.printf("GET MFR_MODE failed (NACK 0x%02x)\n", ack.nack_reason);
  }
}

void cmdModeSet(uint8_t mode) {
  if (!checkDevice()) return;
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  if (rdm_send_set_mfr_mode(dmxPort, &dest, subDev(), mode, &ack)) {
    Serial.printf("Mode set to: %d\n", mode);
  } else {
    Serial.printf("SET MFR_MODE failed (NACK 0x%02x)\n", ack.nack_reason);
  }
}

void printMfrPids(rdm_uid_t dest) {
  rdm_ack_t ack;
  uint16_t supportedPids[64];
  size_t spBytes = rdm_send_get_supported_parameters(
      dmxPort, &dest, RDM_SUB_DEVICE_ROOT,
      supportedPids, sizeof(supportedPids), &ack);
  if (spBytes == 0) return;
  int nPids = spBytes / sizeof(uint16_t);
  bool anyMfr = false;
  for (int p = 0; p < nPids; p++) {
    rdm_pid_t pid = (rdm_pid_t)supportedPids[p];
    if (pid < RDM_PID_MANUFACTURER_SPECIFIC_BEGIN ||
        pid > RDM_PID_MANUFACTURER_SPECIFIC_END) continue;
    rdm_parameter_description_t paramDesc;
    if (rdm_send_get_parameter_description(dmxPort, &dest, RDM_SUB_DEVICE_ROOT,
                                           pid, &paramDesc, &ack)) {
      if (!anyMfr) { Serial.println("  Manufacturer PIDs:"); anyMfr = true; }
      Serial.printf("    0x%04x: \"%s\"  pdl=%u  range=%lu-%lu  default=%lu\n",
                    (unsigned)pid, paramDesc.description,
                    paramDesc.pdl_size,
                    (unsigned long)paramDesc.min_value,
                    (unsigned long)paramDesc.max_value,
                    (unsigned long)paramDesc.default_value);
    } else {
      if (!anyMfr) { Serial.println("  Manufacturer PIDs:"); anyMfr = true; }
      Serial.printf("    0x%04x: PARAMETER_DESCRIPTION NACK 0x%02x\n",
                    (unsigned)pid, ack.nack_reason);
    }
  }
  if (!anyMfr) Serial.println("  No manufacturer PIDs.");
}

void cmdInfo() {
  if (!checkDevice()) return;
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  rdm_device_info_t info;
  Serial.printf("[%d] " UIDSTR "\n", activeDevice, UID2STR(dest));
  if (rdm_send_get_device_info(dmxPort, &dest, subDev(), &info, &ack)) {
    Serial.printf("  Footprint: %d  Sub-devices: %d  Sensors: %d\n",
                  info.footprint, info.sub_device_count, info.sensor_count);
  } else {
    Serial.printf("  GET DEVICE_INFO failed\n");
  }
  printMfrPids(dest);
}

void cmdStatus() {
  if (!checkDevice()) return;
  Serial.printf("Active device [%d]: " UIDSTR "\n",
                activeDevice, UID2STR(activeUID()));
  cmdAddrGet();
  cmdPersonalityGet();
  cmdCurveGet();
  cmdLabelGet();
  cmdModeGet();
}

void cmdIdentifyGet() {
  if (!checkDevice()) return;
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  bool identify;
  if (rdm_send_get_identify_device(dmxPort, &dest, subDev(), &identify, &ack)) {
    Serial.printf("Identify: %s\n", identify ? "on" : "off");
  } else {
    Serial.printf("GET IDENTIFY failed (NACK 0x%02x)\n", ack.nack_reason);
  }
}

void cmdIdentifySet(bool on) {
  if (!checkDevice()) return;
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  if (rdm_send_set_identify_device(dmxPort, &dest, subDev(), on, &ack)) {
    Serial.printf("Identify: %s\n", on ? "on" : "off");
  } else {
    Serial.printf("SET IDENTIFY failed (NACK 0x%02x)\n", ack.nack_reason);
  }
}

void cmdPersonalityGet() {
  if (!checkDevice()) return;
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  uint8_t current = 0, count = 0;
  if (!rdm_send_get_dmx_personality(dmxPort, &dest, subDev(), &current, &count, &ack)) {
    Serial.printf("GET DMX_PERSONALITY failed (NACK 0x%02x)\n", ack.nack_reason);
    return;
  }
  Serial.printf("Personality: %d of %d\n", current, count);
  for (uint8_t p = 1; p <= count; p++) {
    rdm_dmx_personality_description_t desc;
    memset(&desc, 0, sizeof(desc));
    size_t pdl = rdm_send_get_dmx_personality_description(dmxPort, &dest, subDev(), p, &desc, &ack);
    if (pdl > 0) {
      // PDL = 1 (personality_num) + 2 (footprint) + strlen(description)
      // Null-terminate at the correct position in case responder pads to 32 bytes
      if (pdl >= 3) {
        size_t descLen = pdl - 3;
        if (descLen < RDM_ASCII_SIZE_MAX) desc.description[descLen] = '\0';
      }
      Serial.printf("  [%d] footprint=%-3d  \"%s\"%s\n",
                    p, desc.footprint, desc.description,
                    p == current ? "  <-- active" : "");
    } else {
      Serial.printf("  [%d] PERSONALITY_DESCRIPTION NACK 0x%02x\n", p, ack.nack_reason);
    }
  }
}

void cmdPersonalitySet(uint8_t n) {
  if (!checkDevice()) return;
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  if (rdm_send_set_dmx_personality(dmxPort, &dest, subDev(), n, &ack)) {
    Serial.printf("Personality set to %d (reboot recommended on dimmer)\n", n);
  } else {
    Serial.printf("SET DMX_PERSONALITY failed (NACK 0x%02x)\n", ack.nack_reason);
  }
}

// ============================================================
// Serial command parser
// ============================================================
void parseCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "discover") {
    doDiscover();

  } else if (cmd.startsWith("dev ")) {
    int n = cmd.substring(4).toInt();
    if (n >= 0 && n < devicesFound) {
      activeDevice = n;
      Serial.printf("Active device: [%d] " UIDSTR "\n",
                    activeDevice, UID2STR(activeUID()));
    } else {
      Serial.printf("Invalid device index (0-%d)\n", devicesFound - 1);
    }

  } else if (cmd == "dev?") {
    if (devicesFound > 0) {
      Serial.printf("Active device: [%d] " UIDSTR "\n",
                    activeDevice, UID2STR(activeUID()));
    } else {
      Serial.println("No devices — run 'discover'");
    }

  } else if (cmd.startsWith("level ")) {
    // level <ch 1-512> <val 0-255>
    String arg = cmd.substring(6);
    int sp = arg.indexOf(' ');
    if (sp > 0) {
      int ch  = arg.substring(0, sp).toInt();
      int val = arg.substring(sp + 1).toInt();
      if (ch >= 1 && ch <= 512 && val >= 0 && val <= 255) {
        dmxBuffer[ch] = (uint8_t)val;
        Serial.printf("CH%d level: %d\n", ch, val);
      } else {
        Serial.println("Usage: level <ch 1-512> <0-255>");
      }
    } else {
      Serial.println("Usage: level <ch 1-512> <0-255>");
    }

  } else if (cmd.startsWith("addr ")) {
    int a = cmd.substring(5).toInt();
    cmdAddrSet((uint16_t)a);

  } else if (cmd == "addr?") {
    cmdAddrGet();

  } else if (cmd.startsWith("curve ")) {
    String arg = cmd.substring(6);
    arg.trim();
    // Check if first token is a channel number
    int sp = arg.indexOf(' ');
    if (sp > 0) {
      int ch = arg.substring(0, sp).toInt() - 1;
      String curvePart = arg.substring(sp + 1);
      curvePart.trim();
      int cid = parseCurveName(curvePart);
      if (ch >= 0 && ch < NUM_CHANNELS && cid >= 0) {
        cmdCurveSetChannel(ch, cid);
      } else {
        Serial.println("Usage: curve <ch 1-6> sq/log/lin/tmp");
      }
    } else {
      int cid = parseCurveName(arg);
      if (cid >= 0) {
        cmdCurveSetAll(cid);
      } else {
        Serial.println("Curve: sq, log, lin, tmp");
      }
    }

  } else if (cmd == "curve?") {
    cmdCurveGet();

  } else if (cmd.startsWith("label ")) {
    cmdLabelSet(cmd.substring(6));

  } else if (cmd == "label?") {
    cmdLabelGet();

  } else if (cmd.startsWith("mode ")) {
    int n = cmd.substring(5).toInt();
    if (n < 0 || n > 2) {
      Serial.println("Mode: 0=normal 1=test 2=safe");
    } else {
      cmdModeSet((uint8_t)n);
    }

  } else if (cmd == "mode?") {
    cmdModeGet();

  } else if (cmd == "info") {
    cmdInfo();

  } else if (cmd == "identify on") {
    cmdIdentifySet(true);

  } else if (cmd == "identify off") {
    cmdIdentifySet(false);

  } else if (cmd == "identify?") {
    cmdIdentifyGet();

  } else if (cmd.startsWith("personality ")) {
    int n = cmd.substring(12).toInt();
    if (n < 1 || n > 255) {
      Serial.println("Usage: personality <n>  (1-based index)");
    } else {
      cmdPersonalitySet((uint8_t)n);
    }

  } else if (cmd == "personality?") {
    cmdPersonalityGet();

  } else if (cmd == "status") {
    cmdStatus();

  } else if (cmd == "help") {
    Serial.println("Commands:");
    Serial.println("  discover          re-run RDM discovery");
    Serial.println("  dev <n>           select active device by index");
    Serial.println("  dev?              show active device");
    Serial.println("  level <ch> <val>  Set DMX output level (ch 1-512, val 0-255)");
  Serial.println("  addr <n>          SET DMX start address (1-512)");
    Serial.println("  addr?             GET DMX start address");
    Serial.println("  curve <n>         SET all channels: sq/log/lin/tmp 0x8000");
    Serial.println("  curve <ch> <n>    SET single channel curve (ch 1-6)");
    Serial.println("  curve?            GET per-channel curves");
    Serial.println("  label <str>       SET rig/circuit label (max 32 chars) 0x8001");
    Serial.println("  label?            GET rig/circuit label");
    Serial.println("  mode <n>          SET operating mode (0=normal 1=test 2=safe) 0x8002");
    Serial.println("  mode?             GET operating mode");
    Serial.println("  info              GET device info");
    Serial.println("  status            show address + curves");
    Serial.println("  personality <n>   SET personality by 1-based index");
    Serial.println("  personality?      GET current personality + list all with footprints");
    Serial.println("  identify on/off   SET identify mode");
    Serial.println("  identify?         GET identify mode");

  } else {
    Serial.printf("Unknown: %s\n", cmd.c_str());
  }
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== S3HW RDM Controller ===");

  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {};
  dmx_driver_install(dmxPort, &config, personalities, 0);
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

  // Write the initial all-zero buffer and start continuous DMX output
  dmx_write(dmxPort, dmxBuffer, DMX_PACKET_SIZE);
  dmx_send(dmxPort);

  doDiscover();
  Serial.println("Ready. Type 'help' for commands.");
}

// ============================================================
// Loop — serial command parser
// ============================================================
static String serialBuf = "";

void loop() {
  // Continuous DMX output — write current buffer and send one frame.
  // dmx_send() is non-blocking; dmx_wait_sent() blocks until the frame
  // is complete before we loop. RDM exchanges preempt this automatically.
  dmx_write(dmxPort, dmxBuffer, DMX_PACKET_SIZE);
  dmx_send(dmxPort);

  // Service serial input while the DMX frame is transmitting (~23ms at 50Hz)
  unsigned long frameStart = millis();
  while (!dmx_wait_sent(dmxPort, 0)) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        if (serialBuf.length() > 0) {
          parseCommand(serialBuf);
          serialBuf = "";
        }
      } else if (c >= 32 && c < 127) {
        if (serialBuf.length() < 64) serialBuf += c;
      }
    }
    // Avoid spinning too hard
    if (millis() - frameStart > 100) break;
  }

  // Drain any remaining serial input after frame completes
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuf.length() > 0) {
        parseCommand(serialBuf);
        serialBuf = "";
      }
    } else if (c >= 32 && c < 127) {
      if (serialBuf.length() < 64) serialBuf += c;
    }
  }
}
