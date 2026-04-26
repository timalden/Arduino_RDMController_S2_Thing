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

    info                GET DEVICE_INFO
    status              show active device + curves + address
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
    Serial.printf("  [%d] " UIDSTR "%s\n", i, UID2STR(uids[i]),
                  i == activeDevice ? " <-- active" : "");
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

void cmdInfo() {
  if (!checkDevice()) return;
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  rdm_device_info_t info;
  if (rdm_send_get_device_info(dmxPort, &dest, subDev(), &info, &ack)) {
    Serial.printf("Footprint: %d  Sub-devices: %d  Sensors: %d\n",
                  info.footprint, info.sub_device_count, info.sensor_count);
  } else {
    Serial.printf("GET DEVICE_INFO failed\n");
  }
}

void cmdStatus() {
  if (!checkDevice()) return;
  Serial.printf("Active device [%d]: " UIDSTR "\n",
                activeDevice, UID2STR(activeUID()));
  cmdAddrGet();
  cmdCurveGet();
}

void cmdIdentifyGet() {
  if (!checkDevice()) return;
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  bool identify;
  if (rdm_send_get_identify_device(dmxPort, &dest, subDev(), &identify, &ack)) {
    Serial.printf("Identify: %s", identify ? "on" : "off");
  } else {
    Serial.printf("GET IDENTIFY failed (NACK 0x%02x)", ack.nack_reason);
  }
  Serial.println();
}

void cmdIdentifySet(bool on) {
  if (!checkDevice()) return;
  rdm_uid_t dest = activeUID();
  rdm_ack_t ack;
  if (rdm_send_set_identify_device(dmxPort, &dest, subDev(), on, &ack)) {
    Serial.printf("Identify: %s", on ? "on" : "off");
  } else {
    Serial.printf("SET IDENTIFY failed (NACK 0x%02x)", ack.nack_reason);
  }
  Serial.println();
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

  } else if (cmd == "info") {
    cmdInfo();

  } else if (cmd == "identify on") {
    cmdIdentifySet(true);

  } else if (cmd == "identify off") {
    cmdIdentifySet(false);

  } else if (cmd == "identify?") {
    cmdIdentifyGet();

  } else if (cmd == "status") {
    cmdStatus();

  } else if (cmd == "help") {
    Serial.println("Commands:");
    Serial.println("  discover          re-run RDM discovery");
    Serial.println("  dev <n>           select active device by index");
    Serial.println("  dev?              show active device");
    Serial.println("  addr <n>          SET DMX start address (1-512)");
    Serial.println("  addr?             GET DMX start address");
    Serial.println("  curve <n>         SET all channels: sq/log/lin/tmp");
    Serial.println("  curve <ch> <n>    SET single channel curve (ch 1-6)");
    Serial.println("  curve?            GET per-channel curves");
    Serial.println("  info              GET device info");
    Serial.println("  status            show address + curves");
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

  doDiscover();
  Serial.println("Ready. Type 'help' for commands.");
}

// ============================================================
// Loop — serial command parser
// ============================================================
static String serialBuf = "";

void loop() {
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
