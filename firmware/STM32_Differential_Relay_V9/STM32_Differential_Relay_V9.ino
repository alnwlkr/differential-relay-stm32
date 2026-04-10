/* ************************************************** */
/*    ██████████ ██████████ █████ █████   ████████    */
/*   ░░███░░░░░█░░███░░░░░█░░███ ░░███   ███░░░░███   */
/*    ░███  █ ░  ░███  █ ░  ░███  ░███ █░███   ░███   */
/*    ░██████    ░██████    ░███████████░░█████████   */
/*    ░███░░█    ░███░░█    ░░░░░░░███░█ ░░░░░░░███   */
/*    ░███ ░   █ ░███ ░   █       ░███░  ███   ░███   */
/*    ██████████ ██████████       █████ ░░████████    */
/*   ░░░░░░░░░░ ░░░░░░░░░░       ░░░░░   ░░░░░░░░     */
/*                                                    */
/* ************************************************** */

// ==========================================
// differential_relay_v9.ino
// CHANGE LOG v8 → v9:
//   [1] Delete TURN_RATIO, DELTA_FACTOR, GAIN_PRI, GAIN_SEC_AUTO,
//       GAIN_SEC_HARDCODE
//   [2] Add EFFECTIVE_RATIO (float, runtime) Instead of 3 above.
//       EF can be calculate from I_pri_avg / I_sec_avg while Booting
//   [3] Delete runTurnRatioCalibration() + runGainCalibration()
//       Remains only runEffectiveRatioCalibration() 
//   [4] measureRMSBlocking() simplied by from removing gain_sec parameter
//   [5] Main loop: i_pri_comp = I_pri / EFFECTIVE_RATIO
//                 i_diff = |i_pri_comp - I_sec|
//   Result: No need to detect Yy/Yd/Dy, Work with all configuration
// ==========================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// ==========================================
// OLED 1.3" (SH1106) Configuration
// ==========================================
#define i2c_Address 0x3c
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ==========================================
// Pins Definition
// ==========================================
const int pin_ADC[6] = {PA0, PA1, PA2, PA3, PA4, PA5};
// [0]=Pri_A, [1]=Sec_A, [2]=Pri_B, [3]=Sec_B, [4]=Pri_C, [5]=Sec_C

#define BTN_PIN   PB12
#define BUZZER_PIN PB0

#define LED_A     PB1
#define LED_B     PB3
#define LED_C     PB4

// ==========================================
// CT Parameters
// CT_RATIO = (I_pri_rated / I_sec_rated) / R_burden  [A/V]
// MSQ-0.66: 30A/5A, R_burden=0.10Ω → CT_RATIO = 60.0 A/V
// ==========================================
const float R_BURDEN     = 0.10f;
const float CT_RATIO_PRI = (30.0f / 5.0f) / R_BURDEN;   // 60.0 A/V
const float CT_RATIO_SEC = (30.0f / 5.0f) / R_BURDEN;   // 60.0 A/V

// ==========================================
// Hardcode Gain Correction
//
// Use to compensate each CT Error for more accurate reading
// GAIN = I_clamp / I_stm (Calibrate at 60% - 80% of CT Rated Current)
//
// Select MODE with #define below:
//   GAIN_MODE 1 — All Pri + All Sec (2 Value)
//   GAIN_MODE 2 — Splitted 6 Channel (PA, SA, PB, SB, PC, SC)
// ==========================================
#define GAIN_MODE  2   

#if GAIN_MODE == 1
  const float GAIN_PRI_ALL = 1.000f;  
  const float GAIN_SEC_ALL = 1.000f;  

#elif GAIN_MODE == 2
  const float GAIN_CH[6] = {
    1.104f,   // [0] Pri_A
    1.100f,   // [1] Sec_A
    1.146f,   // [2] Pri_B
    1.137f,   // [3] Sec_B
    1.100f,   // [4] Pri_C
    1.141f,   // [5] Sec_C
  };
#else
  #error "GAIN_MODE must be 1 or 2"
#endif

// Helper inline — Return gain of Each Channel by GAIN_MODE 
inline float getGain(int i) {
#if GAIN_MODE == 1
  return (i % 2 == 0) ? GAIN_PRI_ALL : GAIN_SEC_ALL;
#else
  return GAIN_CH[i];
#endif
}

// ==========================================
// EFFECTIVE_RATIO (runtime)
//
// = I_pri_avg / I_sec_avg While Boot Healthy Condition
//
// loop: i_pri_comp = I_pri / EFFECTIVE_RATIO
//       i_diff = |i_pri_comp - I_sec|  ≈ 0 
//
// Fallback = 1.0f - If too less current
// ==========================================
float EFFECTIVE_RATIO = 1.0f;   // runtime — set in runEffectiveRatioCalibration()

// ==========================================
// Protection Mode
//   1 = Dual Slope (IEC): ch1→ 0.10*Ib+0.45 | ch2→ 0.50*Ib+1.55
//   2 = Single Slope:     0.60*Ib + 10.0
// ==========================================
#define PROTECTION_MODE  1

const float SLOPE1     = 0.10f;
const float PICKUP     = 0.45f;
const float KNEE_POINT = 5.0f;
const float SLOPE2     = 0.50f;
const float OFFSET2    = 1.55f;

const float SLOPE_S2   = 0.60f;
const float PICKUP_S2  = 10.0f;

// ==========================================
// Calibration Constants
// ==========================================
#define CALIB_SAMPLES          2000   // DC offset samples
#define CALIB_COUNTDOWN           3   // Seconds countdown DC offset
#define TR_COUNTDOWN             10   // Seconds countdown TR cal
#define TR_MEASURE_MS         10000   // Measuring Time TR (ms)
#define TR_SAMPLE_INTERVAL_US   500   // sampling interval (us)
#define TR_MIN_CURRENT          0.5f  // A — Minimum Current
#define DC_WARN_TOLERANCE       0.10f // V — warning threshold

float dc_offset[6];

// ==========================================
// Non-blocking Timers
// ==========================================
unsigned long prevSampleMicros = 0;
const unsigned long SAMPLE_INTERVAL_US = 1000;

unsigned long prevRMSMillis = 0;
const unsigned long RMS_INTERVAL_MS = 40;

unsigned long prevOLEDMillis = 0;
const unsigned long OLED_INTERVAL_MS = 250;

// ==========================================
// RMS Accumulators
// ==========================================
double sum_sq[6]    = {0,0,0,0,0,0};
int    sample_count = 0;

// ==========================================
// Results
// ==========================================
float rms_current[6] = {0,0,0,0,0,0};
float i_diff[3]      = {0,0,0};
float i_bias[3]      = {0,0,0};
bool  is_fault[3]    = {false,false,false};

// ==========================================
// Button & Display
// ==========================================
int  current_page   = 0;
bool last_btn_state = HIGH;
unsigned long last_btn_millis = 0;
const unsigned long DEBOUNCE_MS = 50;

// ==========================================
// shouldTrip()
// ==========================================
bool shouldTrip(float id, float ib) {
#if PROTECTION_MODE == 1
  float threshold = (ib < KNEE_POINT)
                  ? (SLOPE1 * ib + PICKUP)
                  : (SLOPE2 * ib - OFFSET2);
  return (id > threshold);
#elif PROTECTION_MODE == 2
  return (id > SLOPE_S2 * ib + PICKUP_S2);
#else
  #error "PROTECTION_MODE must be 1 or 2"
#endif
}

// ==========================================
// measureRMSBlocking()
//
// Outputed in Ampere Unit After applied CT_RATIO + gain
// Measure RMS with blocking — Use when calibration only
// No gain Multiplication Because of EFFECTIVE_RATIO
// Outputed in Ampere After applied CT_RATIO × Gain
// ==========================================

void measureRMSBlocking(float* out_rms, int nSamples, int intervalUs) {
  double acc[6] = {0,0,0,0,0,0};
  for (int s = 0; s < nSamples; s++) {
    for (int i = 0; i < 6; i++) {
      int   raw  = analogRead(pin_ADC[i]);
      float v_ac = ((raw / 4095.0f) * 3.3f) - dc_offset[i];
      acc[i]    += (double)(v_ac * v_ac);
    }
    delayMicroseconds(intervalUs);
  }
  for (int i = 0; i < 6; i++) {
    float v_rms = sqrt((float)(acc[i] / nSamples));
    float ratio  = (i % 2 == 0) ? CT_RATIO_PRI : CT_RATIO_SEC;
    out_rms[i]  = v_rms * ratio * getGain(i);
  }
}

// ==========================================
// STEP 1: DC Offset Calibration
// ==========================================
void runDCOffsetCalibration() {
  for (int sec = CALIB_COUNTDOWN; sec >= 1; sec--) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(5, 0);  display.println("== DC OFFSET CAL ==");
    display.setCursor(5, 14); display.println("No current required");
    display.setCursor(5, 24); display.println("CT can stay clamped");
    display.setCursor(5, 48); display.print("Starting in: ");
    display.print(sec); display.print("s");
    display.display();
    delay(1000);
  }

  display.clearDisplay();
  display.setCursor(5, 0);  display.println("== DC OFFSET CAL ==");
  display.setCursor(5, 18); display.println("Measuring...");
  display.display();

  double acc[6] = {0,0,0,0,0,0};
  for (int s = 0; s < CALIB_SAMPLES; s++) {
    for (int i = 0; i < 6; i++) acc[i] += analogRead(pin_ADC[i]);
    delayMicroseconds(500);
  }
  for (int i = 0; i < 6; i++) {
    dc_offset[i] = ((float)(acc[i] / CALIB_SAMPLES) / 4095.0f) * 3.3f;
  }

  display.clearDisplay();
  display.setCursor(0, 0); display.println("Offset(V)  status");
  display.drawFastHLine(0, 9, 128, SH110X_WHITE);

  const char* labels[6] = {"PA","SA","PB","SB","PC","SC"};
  for (int i = 0; i < 6; i++) {
    int y = 12 + i * 9;
    display.setCursor(0, y);
    display.print(labels[i]); display.print(":"); display.print(dc_offset[i], 3);
    display.setCursor(90, y);
    display.print(fabsf(dc_offset[i] - 1.65f) <= DC_WARN_TOLERANCE ? "[OK]" : "[!!]");
  }
  display.display();
  delay(5000);

  Serial.println("=== DC Offset Calibration ===");
  const char* sl[6] = {"Pri_A","Sec_A","Pri_B","Sec_B","Pri_C","Sec_C"};
  for (int i = 0; i < 6; i++) {
    Serial.print(sl[i]); Serial.print(" = "); Serial.print(dc_offset[i], 4);
    Serial.println(fabsf(dc_offset[i]-1.65f) <= DC_WARN_TOLERANCE ? "V [OK]" : "V [!!]");
  }
}

// ==========================================
// STEP 2: Effective Ratio Calibration  [v8]
//
// loop: i_pri_comp = I_pri / EFFECTIVE_RATIO ≈ I_sec While Healthy
//       i_diff = |i_pri_comp - I_sec| ≈ 0 While Healthy
//                                         Spike when Fault
//
// Fallback: EFFECTIVE_RATIO = 1.0f If current less than TR_MIN_CURRENT
// ==========================================
void runEffectiveRatioCalibration() {
  for (int sec = TR_COUNTDOWN; sec >= 1; sec--) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(5, 0);  display.println("== RATIO CAL (v8) ==");
    display.setCursor(5, 14); display.println("Apply normal load NOW");
    display.setCursor(5, 24); display.println("Need I > 0.5A");
    display.setCursor(5, 36); display.println("Keep load stable");
    display.setCursor(5, 50); display.print("Measuring in: ");
    display.print(sec); display.print("s");
    display.display();
    delay(1000);
  }

  display.clearDisplay();
  display.setCursor(5, 0);  display.println("== RATIO CAL (v8) ==");
  display.setCursor(5, 18); display.println("Measuring 10s...");
  display.setCursor(5, 30); display.println("Keep load stable!");
  display.display();

  const int trSamples = (TR_MEASURE_MS * 1000) / TR_SAMPLE_INTERVAL_US;
  float measured[6]   = {0,0,0,0,0,0};
  measureRMSBlocking(measured, trSamples, TR_SAMPLE_INTERVAL_US);

  // 3 Phase Average, in case of unbalance load
  float i_pri_avg = (measured[0] + measured[2] + measured[4]) / 3.0f;
  float i_sec_avg = (measured[1] + measured[3] + measured[5]) / 3.0f;

  Serial.println("=== Effective Ratio Calibration ===");
  Serial.print("I_pri avg = "); Serial.print(i_pri_avg, 3); Serial.println(" A");
  Serial.print("I_sec avg = "); Serial.print(i_sec_avg, 3); Serial.println(" A");

  // Check Minimum Current
  if (i_pri_avg < TR_MIN_CURRENT || i_sec_avg < TR_MIN_CURRENT) {
    display.clearDisplay();
    display.setCursor(5, 0);  display.println("== RATIO CAL (v8) ==");
    display.setCursor(5, 14); display.println("FAILED! Current<0.5A");
    display.setCursor(5, 26); display.print("Ipri="); display.print(i_pri_avg, 2);
    display.print(" Isec="); display.print(i_sec_avg, 2);
    display.setCursor(5, 40); display.println("Using ER=1.0 (fallback)");
    display.setCursor(5, 52); display.println("Restart to retry");
    display.display();

    EFFECTIVE_RATIO = 1.0f;
    Serial.println("[!!] FAILED — EFFECTIVE_RATIO=1.0 (fallback), please restart");
    delay(8000);
    return;
  }

  // Calculate EFFECTIVE_RATIO
  EFFECTIVE_RATIO = i_pri_avg / i_sec_avg;

  Serial.print("EFFECTIVE_RATIO = "); Serial.println(EFFECTIVE_RATIO, 4);
  Serial.println("[OK] Applied");

  display.clearDisplay();
  display.setCursor(5, 0);  display.println("== RATIO CAL (v8) ==");
  display.setCursor(5, 14); display.print("Ipri="); display.print(i_pri_avg, 3); display.print("A");
  display.setCursor(5, 26); display.print("Isec="); display.print(i_sec_avg, 3); display.print("A");
  display.setCursor(5, 40); display.print("ER = "); display.print(EFFECTIVE_RATIO, 4);
  display.setCursor(5, 52); display.println("[OK] Applied");
  display.display();
  delay(4000);
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(500000);

  analogReadResolution(12);
  for (int i = 0; i < 6; i++) pinMode(pin_ADC[i], INPUT_ANALOG);

  pinMode(BTN_PIN,   INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, HIGH);
  pinMode(LED_A, OUTPUT); digitalWrite(LED_A, LOW);
  pinMode(LED_B, OUTPUT); digitalWrite(LED_B, LOW);
  pinMode(LED_C, OUTPUT); digitalWrite(LED_C, LOW);

  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  if (!display.begin(i2c_Address, true)) {
    Serial.println("OLED Init Failed — all fallback values");
    for (int i = 0; i < 6; i++) dc_offset[i] = 1.65f;
    EFFECTIVE_RATIO = 1.0f;
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(10, 20); display.print("Relay Booting v9...");
    display.display();
    delay(500);

    runDCOffsetCalibration();          // Step 1: No Load — Measure DC offset
    runEffectiveRatioCalibration();    // Step 2: Normal Load  — Measure EFFECTIVE_RATIO
  }

  Serial.println("=== System Ready ===");
  Serial.print("EFFECTIVE_RATIO="); Serial.println(EFFECTIVE_RATIO, 4);
  Serial.println("RMS_Pri_A,RMS_Sec_A,RMS_Pri_B,RMS_Sec_B,RMS_Pri_C,RMS_Sec_C,"
                 "Idiff_A,Ibias_A,Fault_A,Idiff_B,Ibias_B,Fault_B,Idiff_C,Ibias_C,Fault_C");
}

// ==========================================
// LOOP
// ==========================================
void loop() {
  unsigned long currentMicros = micros();
  unsigned long currentMillis = millis();

  // ====================================================
  // 1. Sampling — Every 1 ms
  // ====================================================
  if (currentMicros - prevSampleMicros >= SAMPLE_INTERVAL_US) {
    prevSampleMicros += SAMPLE_INTERVAL_US;
    for (int i = 0; i < 6; i++) {
      int   raw  = analogRead(pin_ADC[i]);
      float v_ac = ((raw / 4095.0f) * 3.3f) - dc_offset[i];
      sum_sq[i] += (double)(v_ac * v_ac);
    }
    sample_count++;
  }

  // ====================================================
  // 2. RMS + Protection — Every 40 ms
  // ====================================================
  if (currentMillis - prevRMSMillis >= RMS_INTERVAL_MS) {
    prevRMSMillis += RMS_INTERVAL_MS;

    if (sample_count > 0) {
      bool any_fault = false;

      for (int i = 0; i < 6; i++) {
        float v_rms = sqrt((float)(sum_sq[i] / sample_count));
        sum_sq[i]   = 0;
        float ratio = (i % 2 == 0) ? CT_RATIO_PRI : CT_RATIO_SEC;
        rms_current[i] = v_rms * ratio * getGain(i);
      }
      sample_count = 0;

      for (int phase = 0; phase < 3; phase++) {
        int p_idx = phase * 2;
        int s_idx = p_idx + 1;

        float i_pri_comp = rms_current[p_idx] / EFFECTIVE_RATIO;
        float i_sec_comp = rms_current[s_idx];

        i_diff[phase] = fabsf(i_pri_comp - i_sec_comp);
        i_bias[phase] = (i_pri_comp + i_sec_comp) * 0.5f;

        is_fault[phase] = shouldTrip(i_diff[phase], i_bias[phase]);
        if (is_fault[phase]) any_fault = true;
      }

      digitalWrite(BUZZER_PIN, any_fault ? LOW : HIGH);
      digitalWrite(LED_A, is_fault[0] ? HIGH : LOW);
      digitalWrite(LED_B, is_fault[1] ? HIGH : LOW);
      digitalWrite(LED_C, is_fault[2] ? HIGH : LOW);

      for (int i = 0; i < 6; i++) {
        Serial.print(rms_current[i], 3); Serial.print(",");
      }
      for (int ph = 0; ph < 3; ph++) {
        Serial.print(i_diff[ph], 3); Serial.print(",");
        Serial.print(i_bias[ph], 3); Serial.print(",");
        Serial.print(is_fault[ph] ? "1" : "0");
        if (ph < 2) Serial.print(",");
      }
      Serial.println();
    }
  }

  // ====================================================
  // 3. Button — Non-blocking debounce
  // ====================================================
  bool current_btn_state = digitalRead(BTN_PIN);
  if (current_btn_state == LOW && last_btn_state == HIGH) {
    if (currentMillis - last_btn_millis >= DEBOUNCE_MS) {
      last_btn_millis = currentMillis;
      current_page++;
      if (current_page > 2) current_page = 0;
    }
  }
  last_btn_state = current_btn_state;

  // ====================================================
  // 4. OLED — Refresh Screen
  // ====================================================
  if (currentMillis - prevOLEDMillis >= OLED_INTERVAL_MS) {
    prevOLEDMillis += OLED_INTERVAL_MS;

    display.clearDisplay();
    display.drawRect(0, 0, 128, 64, SH110X_WHITE);

    display.setTextSize(1);
    display.setCursor(4, 4);
    display.print("PH-");
    const char phaseNames[] = {'A','B','C'};
    display.print(phaseNames[current_page]);
    display.setCursor(30, 4);  display.print("ER:");
    display.print(EFFECTIVE_RATIO, 2);
    display.setCursor(100, 4);
#if PROTECTION_MODE == 1
    display.print("M1");
#else
    display.print("M2");
#endif
    display.drawFastHLine(0, 14, 128, SH110X_WHITE);

    int p_idx = current_page * 2;
    int s_idx = p_idx + 1;

    display.setCursor(5, 18);
    display.print("P:"); display.print(rms_current[p_idx], 2); display.print("A");
    display.setCursor(67, 18);
    display.print("S:"); display.print(rms_current[s_idx], 2); display.print("A");

    display.setCursor(5, 30);
    display.print("Id:"); display.print(i_diff[current_page], 2);
    display.setCursor(67, 30);
    display.print("Ib:"); display.print(i_bias[current_page], 2);

    display.setCursor(5, 42);
    display.print("ER:"); display.print(EFFECTIVE_RATIO, 3);
    display.setCursor(67, 42);

    display.setCursor(5, 54);
    display.print("Sts: ");
    if (is_fault[current_page]) {
      display.setTextColor(SH110X_BLACK, SH110X_WHITE);
      display.print(" FAULT! ");
      display.setTextColor(SH110X_WHITE, SH110X_BLACK);
    } else {
      display.print("HEALTHY");
    }

    display.display();
  }
}

