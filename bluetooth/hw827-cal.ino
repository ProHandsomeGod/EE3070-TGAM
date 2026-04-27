const int PULSE_PIN = 34;

// ===== Filter =====
float fastFilt = 0;       // fast filter
float slowBase = 0;       // slow filter
float pulseSig = 0;       // heartrate after filter
float prevPulseSig = 0;

// ===== Dynamic threshold =====
float peakEnv = 8.0;
float threshold = 4.0;

// ===== Beat detection =====
bool inBeat = false;
unsigned long lastSampleMs = 0;
unsigned long lastBeatMs = 0;

float bpmInstant = 0;

bool bpmChanged = false;

int getBpmInstant() {
  return (int)bpmInstant;
}

// ---------------------------- setup / loop ----------------------------
void hw827Setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  analogReadResolution(12);

  delay(1000);

  int x = analogRead(PULSE_PIN);
  fastFilt = x;
  slowBase = x;

  println("HW827 Heart Rate Start");
}

void hw827Loop() {
  unsigned long now = millis();
  // 100 Hz
  if (now - lastSampleMs >= 10) {
    lastSampleMs = now;

    int raw = analogRead(PULSE_PIN);
    // ===== Filter =====
    // fast: keep wave form
    fastFilt = 0.85 * fastFilt + 0.15 * raw;
    // slow: track base line
    slowBase = 0.98 * slowBase + 0.02 * raw;
    // remove base line: pulse signal
    pulseSig = fastFilt - slowBase;
    // ===== Dynamic threshold =====
    if (pulseSig > peakEnv) {
      peakEnv = pulseSig;
    } else {
      peakEnv *= 0.995;   // decrease slowly
    }
    if (peakEnv < 4.0) peakEnv = 4.0;

    threshold = peakEnv * 0.50;   // can be: 0.45 ~ 0.60
    if (threshold < 3.0) threshold = 3.0;

    // ===== Beat detection =====
    // condition：
    // 1. not in current beat
    // 2. signal over threshold
    // 3. signal increasing
    // 4. from last beat: at least 300ms (refractory period)
    if (!inBeat &&
        pulseSig > threshold &&
        pulseSig > prevPulseSig &&
        (now - lastBeatMs > 300)) {

      inBeat = true;

      if (lastBeatMs != 0) {
        unsigned long ibi = now - lastBeatMs;   // inter-beat interval
        // filter un-reasonable: 40~200 BPM
        if (ibi >= 300 && ibi <= 1500) {
          bpmInstant = 60000.0 / ibi;

          bpmChanged = true;
        }
      }
      lastBeatMs = now;
    }
    // <(threshold / 2): beat end
    if (inBeat && pulseSig < threshold * 0.5) {
      inBeat = false;
    }
    prevPulseSig = pulseSig;
  }
  
  bool withTgam = true;
  if (bpmChanged && !withTgam) {
    bpmChanged = false;
    
    print("InstantBPM=");
    print(bpmInstant, 1);
    print(", Threshold=");
    print(threshold, 2);
    print(", Pulse=");
    println(pulseSig, 2);
  }
}

