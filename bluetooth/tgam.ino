#include "BluetoothSerial.h"
#include "arduinoFFT.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled.
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Bluetooth SPP is not available on this target.
#endif

BluetoothSerial SerialBT;

// ========================= From Ardunio =========================
bool ARDUINO_CONNECTED = true;
HardwareSerial ArduinoSerial(1);

static const int ARD_TX_PIN = 25;   // ESP32 TX1 -> Arduino RX
static const int ARD_RX_PIN = 26;   // ESP32 RX1 <- Arduino TX
static const uint32_t ARD_BAUD = 115200;

#pragma pack(push, 1)
struct ArduinoSensorData {
  uint16_t brightness;
  int16_t  temperature; // 25.34C -> 2534
  uint16_t water;
  uint16_t co2;
  uint8_t  valid;
};
#pragma pack(pop)

ArduinoSensorData latestArduinoData = {0, 0, 0, 0, 0};   // i will see e.g. 300,2512,420,780\n

static const size_t ARD_LINE_BUF_SIZE = 64;
char ardLineBuf[ARD_LINE_BUF_SIZE];
size_t ardLinePos = 0;

// ========================= To FPGA =========================
bool FPGA_CONNECTED = true;
//HardwareSerial FPGASerial(2);

//static const int FPGA_TX_PIN = 17;   // ESP32 TX2
//static const int FPGA_RX_PIN = 16;   // ESP32 RX2
//static const uint32_t FPGA_BAUD = 115200;

#pragma pack(push, 1)
struct BrainwavePacket {
  uint16_t header;
  uint16_t length;    // bytes of this pack
  uint32_t seq;

  uint8_t  environment_valid;
  uint16_t brightness;
  int16_t  temperature; // *100
  uint16_t water;
  uint16_t co2;

  uint8_t  heartrate;

  uint8_t  brainwave_valid;       // 1: valid, 0: invalid
  uint16_t raw_brainwave_count;
  uint16_t sample_rate_hz;  // 512
  uint16_t fft_size;        // 1024
  // to make brainwaves int: *100
  uint32_t delta;
  uint32_t theta;
  uint32_t lowAlpha;
  uint32_t highAlpha;
  uint32_t alpha;
  uint32_t lowBeta;
  uint32_t highBeta;
  uint32_t beta;
  uint32_t lowGamma;
  uint32_t midGamma;
  uint32_t gamma;

  int16_t raw_brainwaves[600];

  uint8_t crc;   // checksum
  uint16_t footer;
};
#pragma pack(pop)

BrainwavePacket txPacket;
uint32_t packetSeq = 0;

// ========================= Connection settings =========================
const char *TGAM_PIN = "0000";
String TGAM_NAME = "sichiray";
String ESP32_NAME = "ESP32-DevKitC-32E-MASTER";

// ========================= Sampling / FFT settings =========================
static const double SAMPLE_RATE_HZ = 512.0;      // TGAM raw rate
static const uint16_t FFT_SAMPLES = 1024;        // 2-second window, power of 2
static const uint16_t MAX_RAW_SAMPLES = 600;

// raw samples collected during the latest print interval
int16_t rawSamples[MAX_RAW_SAMPLES];
volatile uint16_t rawCount = 0;

// ring buffer for FFT
int16_t fftRing[FFT_SAMPLES];
uint16_t fftRingWriteIndex = 0;
uint32_t totalRawSamplesSeen = 0;

// FFT work buffers
double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_SAMPLES, SAMPLE_RATE_HZ);

// ========================= Computed brainwave powers =========================
bool brainwavesValid = false;

double deltaPower = 0.0;
double thetaPower = 0.0;
double lowAlphaPower = 0.0;
double highAlphaPower = 0.0;
double alphaPower = 0.0;
double lowBetaPower = 0.0;
double highBetaPower = 0.0;
double betaPower = 0.0;
double lowGammaPower = 0.0;
double midGammaPower = 0.0;
double gammaPower = 0.0;

// =========================
// ThinkGear parser
// Packet format:
// AA AA PLENGTH PAYLOAD... CHKSUM
// Raw wave code = 0x80, vlen = 2
// =========================
enum ParserState {
  WAIT_SYNC1,
  WAIT_SYNC2,
  WAIT_PLENGTH,
  WAIT_PAYLOAD,
  WAIT_CHKSUM
};
ParserState parserState = WAIT_SYNC1;

uint8_t payload[169];
uint8_t payloadLen = 0;
uint8_t payloadIndex = 0;
uint8_t checksumAcc = 0;

// ========================= Others / Timers =========================
uint32_t printInterval = 1000;   // ms

uint32_t lastPrintMs = 0;
uint32_t lastReconnectMs = 0;

// ========================= setup / loop / initial =========================
void tgamSetup() {
  Serial.begin(115200);
  delay(1500);

  println();
  println("# ESP32 TGAM raw -> FFT brainwaves");

  if (!SerialBT.begin(ESP32_NAME, true)) {
    println("[BT] SerialBT.begin() failed.");
    return;
  }
  if (FPGA_CONNECTED) {
    //FPGASerial.begin(FPGA_BAUD, SERIAL_8N1, FPGA_RX_PIN, FPGA_TX_PIN);
    println("[FPGA] Serial2 ready");
  }
  if (ARDUINO_CONNECTED) {
    ArduinoSerial.begin(ARD_BAUD, SERIAL_8N1, ARD_RX_PIN, ARD_TX_PIN);
    println("[ARD] Serial1 ready");
  }
  connectTGAM();

  lastPrintMs = millis();
  lastReconnectMs = millis();
}

void tgamLoop() {
  if (ARDUINO_CONNECTED) {
    pollArduinoSerial();
  }
  if (SerialBT.connected()) {
    while (SerialBT.available()) {
      uint8_t b = (uint8_t)SerialBT.read();
      feedThinkGearByte(b);
    }
  } else {
    if (millis() - lastReconnectMs >= 5000) {
      lastReconnectMs = millis();
      println("[BT] Not connected, retrying...");
      connectTGAM();
    }
  }
  if (millis() - lastPrintMs >= printInterval) {
    lastPrintMs += printInterval;
    printRawAndBrainwaves();
  }
  delay(5);
}

bool connectTGAM() {
  println("[BT] Starting master mode...");
  SerialBT.setPin(TGAM_PIN);

  print("[BT] Connecting to name: ");
  println(TGAM_NAME);
  bool connected = SerialBT.connect(TGAM_NAME);

  if (!connected) {
    println("[BT] connect() returned false, waiting up to 30s...");
    connected = SerialBT.connected(30000);
  }
  if (connected) {
    println("[BT] BT connected!");
  } else {
    println("[BT] BT connect failed.");
  }
  return connected;
}

// ========================= Data Functions =========================
void feedThinkGearByte(uint8_t b) {
  switch (parserState) {
    case WAIT_SYNC1:
      if (b == 0xAA) parserState = WAIT_SYNC2;
      break;

    case WAIT_SYNC2:
      if (b == 0xAA) parserState = WAIT_PLENGTH;
      else parserState = WAIT_SYNC1;
      break;

    case WAIT_PLENGTH:
      if (b == 0xAA) break;

      if (b > 169) {
        resetParser();
        break;
      }
      payloadLen = b;
      payloadIndex = 0;
      checksumAcc = 0;

      if (payloadLen == 0) parserState = WAIT_CHKSUM;
      else parserState = WAIT_PAYLOAD;
      break;

    case WAIT_PAYLOAD:
      payload[payloadIndex++] = b;
      checksumAcc += b;

      if (payloadIndex >= payloadLen) {
        parserState = WAIT_CHKSUM;
      }
      break;

    case WAIT_CHKSUM: {
      uint8_t calc = (uint8_t)(~checksumAcc);

      if (calc == b) {
        parseThinkGearPayload(payload, payloadLen);
      }
      resetParser();
      break;
    }
  }
}

void resetParser() {
  parserState = WAIT_SYNC1;
  payloadLen = 0;
  payloadIndex = 0;
  checksumAcc = 0;
}

void parseThinkGearPayload(const uint8_t *buf, uint8_t len) {
  uint8_t i = 0;

  while (i < len) {
    while (i < len && buf[i] == 0x55) {
      i++;
    }
    if (i >= len) break;

    uint8_t code = buf[i++];
    uint8_t vlen = 1;

    if (code >= 0x80) {
      if (i >= len) break;
      vlen = buf[i++];
    }

    if ((uint16_t)i + vlen > len) {
      break;
    }
    // only parse raw value from TGAM
    if (code == 0x80 && vlen == 2) {
      int16_t raw = (int16_t)(((uint16_t)buf[i] << 8) | buf[i + 1]);
      storeRawSample(raw);
    }

    i += vlen;
  }
}

void storeRawSample(int16_t value) {
  // store for n-second printout
  if (rawCount < MAX_RAW_SAMPLES) {
    rawSamples[rawCount++] = value;
  }
  // store into FFT ring buffer
  fftRing[fftRingWriteIndex] = value;
  fftRingWriteIndex = (fftRingWriteIndex + 1) % FFT_SAMPLES;
  totalRawSamplesSeen++;
}

// ========================= Print Functions =========================
void printRawAndBrainwaves() {
  //if (rawCount == 0) return;
  brainwavesValid = computeBrainwavesFromRaw();

  if (rawCount > 0 && brainwavesValid) {
    print("[BW] ");
    print("brainwaves: ");
    print("delta=");
    print(deltaPower, 2);
    print(", theta=");
    print(thetaPower, 2);
    print(", lowAlpha=");
    print(lowAlphaPower, 2);
    print(", highAlpha=");
    print(highAlphaPower, 2);
    print(", alpha=");
    print(alphaPower, 2);
    print(", lowBeta=");
    print(lowBetaPower, 2);
    print(", highBeta=");
    print(highBetaPower, 2);
    print(", beta=");
    print(betaPower, 2);
    print(", lowGamma=");
    print(lowGammaPower, 2);
    print(", midGamma=");
    print(midGammaPower, 2);
    print(", gamma=");
    println(gammaPower, 2);
  } else {
    println("brainwaves: not enough raw samples yet");
  }
  if (rawCount > 0) print("[RAW_BW] ");
  print("raw_count=");
  print(rawCount);
  print(" : ");

  for (uint16_t i = 0; i < rawCount; i++) {
    print(rawSamples[i]);
    if (i + 1 < rawCount) print(',');
  }
  println();

  print("[HR] BPM: ");
  println(getBpmInstant());

  println("--------------------------------------------------");

  if (FPGA_CONNECTED) sendPacketToFPGA();

  rawCount = 0;
}

bool computeBrainwavesFromRaw() {
  if (totalRawSamplesSeen < FFT_SAMPLES) {
    return false;   // not enough samples yet
  }
  // Copy oldest -> newest from ring buffer into FFT input
  // fftRingWriteIndex points to the oldest element (next to overwrite)
  double mean = 0.0;
  for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
    uint16_t idx = (fftRingWriteIndex + i) % FFT_SAMPLES;
    vReal[i] = (double)fftRing[idx];
    mean += vReal[i];
    vImag[i] = 0.0;
  }
  // Remove DC offset
  mean /= FFT_SAMPLES;
  for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
    vReal[i] -= mean;
  }
  // Window + FFT + magnitude
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // ASIC EEG band edges (same as NeuroSky)
  deltaPower     = sumBandPower(0.5,  2.75);
  thetaPower     = sumBandPower(3.5,  6.75);
  lowAlphaPower  = sumBandPower(7.5,  9.25);
  highAlphaPower = sumBandPower(10.0, 11.75);
  lowBetaPower   = sumBandPower(13.0, 16.75);
  highBetaPower  = sumBandPower(18.0, 29.75);
  lowGammaPower  = sumBandPower(31.0, 39.75);
  midGammaPower  = sumBandPower(41.0, 49.75);

  alphaPower = lowAlphaPower + highAlphaPower;
  betaPower  = lowBetaPower + highBetaPower;
  gammaPower = lowGammaPower + midGammaPower;

  return true;
}

double sumBandPower(double fLow, double fHigh) {
  double sum = 0.0;
  const double norm = (double)FFT_SAMPLES * (double)FFT_SAMPLES;

  // Only positive-frequency half is needed
  for (uint16_t k = 1; k < FFT_SAMPLES / 2; k++) {
    double freq = (k * SAMPLE_RATE_HZ) / FFT_SAMPLES;
    if (freq >= fLow && freq <= fHigh) {
      double mag = vReal[k];
      sum += (mag * mag) / norm;   // use squared magnitude as relative power, normalize
    }
  }
  return sum;
}

// ========================= To FPGA =========================
void sendPacketToFPGA() {
  memset(&txPacket, 0, sizeof(txPacket));

  txPacket.header = 0xAAAA;
  txPacket.length = sizeof(txPacket);
  txPacket.seq = packetSeq++;

  txPacket.environment_valid = latestArduinoData.valid ? 0x01 : 0x00;
  txPacket.brightness = latestArduinoData.brightness;
  txPacket.temperature = latestArduinoData.temperature;
  txPacket.water = latestArduinoData.water;
  txPacket.co2 = latestArduinoData.co2;

  txPacket.heartrate = getBpmInstant();

  txPacket.brainwave_valid = (rawCount > 0 && brainwavesValid) ? 0x01 : 0x00;
  txPacket.raw_brainwave_count = rawCount;
  txPacket.sample_rate_hz = (uint16_t)SAMPLE_RATE_HZ;
  txPacket.fft_size = FFT_SAMPLES;

  txPacket.delta     = toScaled(deltaPower);
  txPacket.theta     = toScaled(thetaPower);
  txPacket.lowAlpha  = toScaled(lowAlphaPower);
  txPacket.highAlpha = toScaled(highAlphaPower);
  txPacket.alpha     = toScaled(alphaPower);
  txPacket.lowBeta   = toScaled(lowBetaPower);
  txPacket.highBeta  = toScaled(highBetaPower);
  txPacket.beta      = toScaled(betaPower);
  txPacket.lowGamma  = toScaled(lowGammaPower);
  txPacket.midGamma  = toScaled(midGammaPower);
  txPacket.gamma     = toScaled(gammaPower);

  for (uint16_t i = 0; i < rawCount && i < 600; i++) {
    txPacket.raw_brainwaves[i] = rawSamples[i];
  }
  txPacket.crc = 0;
  txPacket.crc = crc8((uint8_t*)&txPacket, offsetof(BrainwavePacket, crc));
  //txPacket.crc = crc8((uint8_t*)&txPacket, sizeof(txPacket) - sizeof(txPacket.crc));
  txPacket.footer = 0xFFFF;

  Serial.write((const uint8_t*)&txPacket, sizeof(txPacket));
  Serial.flush();
  //FPGASerial.write((const uint8_t*)&txPacket, sizeof(txPacket));
  //FPGASerial.flush();
}

uint32_t toScaled(double x) {
  if (x <= 0.0) return 0;
  //double y = x * 100.0;
  double y = x;
  if (y > 4294967295.0) return 4294967295UL;    // biggest
  return (uint32_t)(y + 0.5);   // +0.5: round to int
}

uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
  }
  return crc;
}

// ========================= From Arduino =========================
void pollArduinoSerial() {
  while (ArduinoSerial.available()) {
    char c = (char)ArduinoSerial.read();

    if (c == '\r') continue;
    if (c == '\n') {
      ardLineBuf[ardLinePos] = '\0';

      if (ardLinePos > 0) {
        bool ok = parseArduinoLine(ardLineBuf);
        if (ok) {
          print("[ARD] brightness=");
          print(latestArduinoData.brightness);
          print(", temp_x100=");
          print(latestArduinoData.temperature);
          print(", water=");
          print(latestArduinoData.water);
          print(", co2=");
          println(latestArduinoData.co2);
        } else {
          print("[ARD] parse fail: ");
          println(ardLineBuf);
        }
      println("-------------------------");
      }
      ardLinePos = 0;
    } else {
      if (ardLinePos < ARD_LINE_BUF_SIZE - 1) {
        ardLineBuf[ardLinePos++] = c;
      } else {
        // overflow -> reset line buffer
        ardLinePos = 0;
      }
    }
  }
}

bool parseArduinoLine(const char* line) {
  unsigned int brightness = 0;
  int temperature = 0;
  unsigned int water = 0;
  unsigned int co2 = 0;

  int matched = sscanf(line, "%u,%d,%u,%u", &brightness, &temperature, &water, &co2);
  latestArduinoData.valid = 0;
  if (matched == 4) {
    latestArduinoData.brightness = (uint16_t)brightness;
    latestArduinoData.temperature = (int16_t)temperature;
    latestArduinoData.water = (uint16_t)water;
    latestArduinoData.co2 = (uint16_t)co2;
    latestArduinoData.valid = 1;
    return true;
  }
  return false;
}

