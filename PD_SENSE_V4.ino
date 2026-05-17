/*
=============================================================
  PD-SENSE v4.0 — Competition Final
  Parkinson's Disease Monitoring & Therapeutic System
  MYOSA Mini Kit (ESP32-WROOM-32E)

  ALL FIXES APPLIED v3→v4:
  [A] Firebase_ESP_Client library (modern, secure auth)
  [B] Aggregate 10s window before upload (not raw stream)
  [C] Smart upload — only if significant change OR 10s passed
  [D] micros() based sampling for FFT accuracy
  [E] Spike filter on IMU (abs > physical limit = reject)
  [F] Gyro adaptive threshold constrained to safe range
  [G] WiFi full disconnect+reconnect (reliable reconnection)
  [H] Tremor classification (PD 3-6Hz vs Essential >6Hz)
  [I] setJSON single write per upload (not multiple set calls)
  [J] Fail-safe: sensor not found → continue gracefully

=============================================================
LIBRARIES — Arduino Library Manager:
  arduinoFFT              by Enrique Conde-Pardo
  SimpleKalmanFilter      by Denys Sene
  Firebase Arduino Client by Mobizt (Firebase_ESP_Client)
  Blynk                   search: BlynkSimpleEsp32

MYOSA LIBRARIES — ZIP from github.com/myosa-sensors/arduino-libraries:
  AccelAndGyro.h
  LightProximityAndGesture.h
  BarometricPressure.h
  TempAndHumidity.h
  AirQuality.h
  OLED.h
  Install: Sketch > Include Library > Add .ZIP Library

BOARD: ESP32 Dev Module | 240MHz | 115200 baud

CREATE config.h IN SAME FOLDER — ADD TO .gitignore:
  #define WIFI_SSID        "your_wifi"
  #define WIFI_PASS        "your_pass"
  #define BLYNK_TOKEN      "your_blynk_token"
  #define FB_API_KEY       "your_firebase_api_key"
  #define FB_DATABASE_URL  "https://your-project.firebaseio.com"
  #define FB_USER_EMAIL    "device@yourproject.com"
  #define FB_USER_PASS     "device_password"
  #define PATIENT_ID       "patient_001"

FIREBASE SETUP:
  1. console.firebase.google.com > Create project
  2. Authentication > Sign-in method > Enable Email/Password
  3. Create user: device@yourproject.com + password
  4. Realtime Database > Create database > Start in test mode
  5. Project Settings > General > Web API Key = FB_API_KEY
  6. Database URL = FB_DATABASE_URL

STAGE GUIDE:
  Stage 1: No WiFi. Serial Monitor only. Verify all sensors.
  Stage 2: Remove // from STAGE2 blocks. Test Blynk.
  Stage 3: Remove // from STAGE3 blocks. Test Firebase.
=============================================================
*/

#include "config.h"

// ── WATCHDOG ──────────────────────────────────────────────
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 10

// ── CORE ──────────────────────────────────────────────────
#include <Wire.h>
#include <WiFi.h>
#include <arduinoFFT.h>
#include <SimpleKalmanFilter.h>
#include <time.h>

// MYOSA sensor libraries
#include <AccelAndGyro.h>
#include <LightProximityAndGesture.h>
#include <BarometricPressure.h>
#include <TempAndHumidity.h>
#include <AirQuality.h>
#include <OLED.h>

// ── STAGE 2: Remove // from these lines ───────────────────
// #define BLYNK_PRINT Serial
// #include <BlynkSimpleEsp32.h>

// ── STAGE 3: Remove // from these lines ───────────────────
// #include <Firebase_ESP_Client.h>
// #include <addons/TokenHelper.h>
// #include <addons/RTDBHelper.h>

// ── SENSOR OBJECTS ────────────────────────────────────────
AccelAndGyro             imu;
LightProximityAndGesture apds;
BarometricPressure       bmp;
TempAndHumidity          si;
AirQuality               ccs;
OLED                     oled;
arduinoFFT               FFT;
SimpleKalmanFilter       kalmanGy(2, 2, 0.01);

// ── STAGE 3 OBJECTS: Remove // ────────────────────────────
// FirebaseData  fbData;
// FirebaseAuth  fbAuth;
// FirebaseConfig fbConfig;

// ── SENSOR HEALTH FLAGS ───────────────────────────────────
bool imu_ok  = false;
bool apds_ok = false;
bool bmp_ok  = false;
bool si_ok   = false;
bool ccs_ok  = false;
bool oled_ok = false;

// ── PINS ──────────────────────────────────────────────────
#define BUZZER_PIN 25

// ── FFT ───────────────────────────────────────────────────
#define SAMPLES      256
#define SAMPLE_RATE  100.0f
#define TREMOR_LO    3.0f   // Extended to catch essential tremor too
#define TREMOR_PD_LO 3.0f
#define TREMOR_PD_HI 6.0f
#define TREMOR_ET_LO 6.0f
#define TREMOR_ET_HI 12.0f

// ── SPIKE FILTER LIMITS ───────────────────────────────────
#define MAX_ACCEL_G  16.0f   // MPU6050 max range
#define MAX_GYRO_DPS 2000.0f // MPU6050 max range

// ── CONSTANTS ─────────────────────────────────────────────
#define TREMOR_CRITICAL 9.5f
#define FOG_CADENCE     0.5f
#define FOG_DURATION_MS 2000
#define NO_MOVE_MS      1800000UL
#define UPLOAD_INTERVAL 10000  // 10 second aggregate upload

// ── STATE MACHINE ─────────────────────────────────────────
typedef enum {
  STATE_RESTING,
  STATE_WALKING,
  STATE_TAP_TEST,
  STATE_FREEZE
} SystemState;

SystemState state = STATE_RESTING;
const char* SNAME[] = {"RESTING","WALKING","TAP_TEST","FREEZE"};

// ── FFT BUFFERS ───────────────────────────────────────────
double vReal[SAMPLES];
double vImag[SAMPLES];
float  ax_buf[SAMPLES];
float  ay_buf[SAMPLES];
float  az_buf[SAMPLES];
int    buf_idx    = 0;
bool   buf_filled = false;

// ── CALIBRATION ───────────────────────────────────────────
float cal_ax=0, cal_ay=0, cal_az=0, cal_gy=0;
int   tap_thresh = 120;
float max_gyro_window = 10.0f;

// ── TREMOR ────────────────────────────────────────────────
float  tremor_smooth  = 0;
bool   tremor_valid   = false;
int    tremor_conf    = 0;
String tremor_level   = "NONE";
String tremor_type    = "NONE"; // [H] "PD" "ESSENTIAL" "NONE"
float  dominant_freq  = 0;
float  prev_scores[3] = {0,0,0};
int    rising_count   = 0;
int    critical_count = 0;

// ── AGGREGATE WINDOW [B] ──────────────────────────────────
float  agg_tremor_sum  = 0;
float  agg_tremor_max  = 0;
int    agg_tremor_count= 0;
float  agg_tremor_avg  = 0;
float  last_uploaded_tremor = -1;
long   tm_aggregate    = 0;

// ── GAIT ──────────────────────────────────────────────────
float kal_gy        = 0;
float prev_gy       = 0;
long  step_ts[8]    = {0};
int   step_idx      = 0;
int   step_win      = 0;
long  cad_win_start = 0;
float cadence       = 0;
float step_cov      = 0;
float walk_rms      = 0;
bool  pd_shuffle    = false;

// ── FREEZE ────────────────────────────────────────────────
bool fog_active    = false;
long fog_start     = 0;
long fog_duration  = 0;
int  fog_exit_cnt  = 0;
int  fog_hr_count  = 0;
long fog_hr_start  = 0;
long last_buzz     = 0;
bool buzz_on       = false;
long low_cad_start = 0;

// ── TAP ───────────────────────────────────────────────────
long   tap_ts[20] = {0};
int    tap_count  = 0;
int    bk_grade   = -1;
String bk_label   = "---";
long   last_tap   = 0;
bool   tap_active = false;

// ── MEDICATION ────────────────────────────────────────────
long  med_ts    = 0;
bool  med_active= false;
int   mins_dose = 0;
float avg_dur   = 300;

// ── ENVIRONMENT ───────────────────────────────────────────
float env_t=0, env_h=0, pres=0;
int   eco2=400, tvoc=0;

// ── ALERTS ────────────────────────────────────────────────
long   no_move_start = 0;
bool   no_move_sent  = false;
String alert_str     = "NONE";

// ── SESSION ───────────────────────────────────────────────
long   session_start = 0;
String today_date    = "UNKNOWN";

// ── TIMERS ────────────────────────────────────────────────
long tm_state=0, tm_tap=0, tm_env=0;
long tm_oled=0,  tm_out=0, tm_debug=0, tm_wifi=0;

// ── SAMPLING [D] ──────────────────────────────────────────
#define SAMPLE_PERIOD_US 10000  // 100Hz = 10000 microseconds
int  sample_interval = 10;      // ms, for millis fallback

// ════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(300);

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  Serial.println("\n=== PD-SENSE v4.0 ===");

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(BUZZER_PIN, HIGH); delay(150);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("Buzzer: OK");

  Wire.begin();
  Wire.setClock(100000);
  Serial.println("I2C: 100kHz GPIO21=SDA GPIO22=SCL");

  initSensors();   // [J] graceful fail if sensor missing
  calibrate();

  if (bmp_ok) {
    pres = bmp.getPressure(false);
    Serial.println("Pressure: " + String(pres,1) + " hPa");
  }

  // Non-blocking CCS811 warmup
  Serial.println("CCS811 warmup 60s...");
  long ws = millis();
  int  lp = -1;
  while (millis() - ws < 60000) {
    esp_task_wdt_reset();
    int s = (millis()-ws)/1000;
    if (s != lp) {
      lp = s;
      if (oled_ok) {
        oled.clear();
        oled.print(0,0,"Warming CCS811");
        oled.print(0,1,String(60-s)+"s remaining");
        oled.display();
      }
      if (s%15==0)
        Serial.println("  "+String(60-s)+"s left");
    }
  }

  connectWiFi();

  if (WiFi.status() == WL_CONNECTED) {
    configTime(19800, 0, "pool.ntp.org");
    delay(2000);
    today_date = getDate();
    Serial.println("Date: " + today_date);
  }

  // STAGE 2: Remove //
  // Blynk.config(BLYNK_TOKEN);
  // Blynk.connect(3000);

  // STAGE 3: Remove //
  // fbConfig.api_key = FB_API_KEY;
  // fbConfig.database_url = FB_DATABASE_URL;
  // fbAuth.user.email = FB_USER_EMAIL;
  // fbAuth.user.password = FB_USER_PASS;
  // fbConfig.token_status_callback = tokenStatusCallback;
  // Firebase.begin(&fbConfig, &fbAuth);
  // Firebase.reconnectWiFi(true);

  session_start = millis();
  cad_win_start = millis();
  fog_hr_start  = millis();
  tm_aggregate  = millis();

  Serial.println("\n=== READY ===");
  Serial.println("Shake=tremor | Walk+stop=freeze+buzz");
  Serial.println("Tap 20x above APDS=BK | Swipe LEFT=medication");
}

// ════════════════════════════════════════════════════════
// WiFi CONNECT [G] — full disconnect+reconnect
// ════════════════════════════════════════════════════════
void connectWiFi() {
  WiFi.disconnect(true);
  delay(500);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi");
  int t = 0;
  while (WiFi.status() != WL_CONNECTED && t < 20) {
    delay(500); Serial.print("."); t++;
    esp_task_wdt_reset();
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("\nWiFi OK: " + WiFi.localIP().toString());
  else
    Serial.println("\nWiFi offline");
}

// ════════════════════════════════════════════════════════
// INIT SENSORS [J] — graceful fail
// ════════════════════════════════════════════════════════
void initSensors() {
  imu_ok = imu.begin();
  Serial.println(imu_ok ? "OK: MPU6050" : "WARN: MPU6050 missing");

  apds_ok = apds.begin();
  if (apds_ok) {
    apds.enableProximity();
    apds.enableGesture();
    Serial.println("OK: APDS9960");
  } else {
    Serial.println("WARN: APDS9960 missing");
  }

  bmp_ok = bmp.begin();
  Serial.println(bmp_ok ? "OK: BMP180" : "WARN: BMP180 missing");

  si_ok = si.begin();
  Serial.println(si_ok ? "OK: SI7021" : "WARN: SI7021 missing");

  ccs_ok = ccs.begin();
  if (ccs_ok) { ccs.setMode(2); Serial.println("OK: CCS811"); }
  else         { Serial.println("WARN: CCS811 missing"); }

  oled_ok = oled.begin();
  if (oled_ok) {
    oled.clear();
    oled.print(0,0,"PD-SENSE v4.0");
    oled.print(0,1,"Initializing...");
    oled.display();
    Serial.println("OK: OLED");
  } else {
    Serial.println("WARN: OLED missing");
  }
}

// ════════════════════════════════════════════════════════
// CALIBRATION
// ════════════════════════════════════════════════════════
void calibrate() {
  Serial.println("\n--- CALIBRATION ---");
  Serial.println("Keep STILL for 5 seconds");
  if (oled_ok) {
    oled.clear();
    oled.print(0,0,"Calibrating...");
    oled.print(0,1,"Keep STILL 5s");
    oled.display();
  }

  if (!imu_ok) {
    Serial.println("WARN: IMU missing, skip calibration");
    return;
  }

  float sx=0,sy=0,sz=0,sgy=0;
  int   ps=0;
  for (int i=0; i<500; i++) {
    esp_task_wdt_reset();
    sx  += imu.getAccelX(false);
    sy  += imu.getAccelY(false);
    sz  += imu.getAccelZ(false);
    sgy += imu.getGyroY(false);
    if (i<20 && apds_ok) ps += apds.readProximity();
    delay(10);
  }
  cal_ax = sx/500.0f; cal_ay = sy/500.0f;
  cal_az = sz/500.0f; cal_gy = sgy/500.0f;
  int base   = apds_ok ? (ps/20) : 60;
  tap_thresh = constrain(base+60, 80, 220);
  Serial.println("G X="+String(cal_ax,3)+" Y="+String(cal_ay,3)+" Z="+String(cal_az,3));
  Serial.println("GyroB="+String(cal_gy,4)+" TapThr="+String(tap_thresh));
  Serial.println("--- DONE ---\n");
}

// ════════════════════════════════════════════════════════
// MAIN LOOP
// ════════════════════════════════════════════════════════
void loop() {
  esp_task_wdt_reset();
  long t = millis();

  // STAGE 2: Remove //
  // Blynk.run();

  // WiFi check every 30s [G]
  if (t - tm_wifi >= 30000) {
    tm_wifi = t;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi lost — reconnecting...");
      connectWiFi();
    }
  }

  // [D] Micros-based precise IMU sampling
  taskIMU_precise();

  if (t-tm_state>=100) { tm_state=t; taskState(t); }

  if (buf_filled && state==STATE_RESTING) {
    buf_filled = false;
    taskFFT();
  }

  // Aggregate tremor data [B]
  if (tremor_valid) {
    agg_tremor_sum += tremor_smooth;
    agg_tremor_count++;
    if (tremor_smooth > agg_tremor_max)
      agg_tremor_max = tremor_smooth;
  }

  if (t-tm_tap>=50)    { tm_tap=t;   taskTap(t); taskGesture(); }
  if (t-tm_env>=10000) { tm_env=t;   taskEnv(); }
  taskBuzzer(t);
  if (t-tm_oled>=500)  { tm_oled=t;  taskOLED(); }

  // [B][C] Aggregate upload every 10s
  if (t-tm_aggregate>=UPLOAD_INTERVAL) {
    tm_aggregate=t;
    computeAggregate();
    if (shouldUpload()) {
      tm_out=t;
      taskOutput();
    }
  }

  if (t-tm_debug>=1000){ tm_debug=t; taskDebug(); }
  checkNoMove(t);
  powerManage(t);
}

// ════════════════════════════════════════════════════════
// IMU — PRECISE MICROS TIMING [D]
// ════════════════════════════════════════════════════════
void taskIMU_precise() {
  static unsigned long last_sample_us = 0;
  unsigned long now_us = micros();

  if (now_us - last_sample_us < SAMPLE_PERIOD_US) return;
  last_sample_us = now_us;

  if (!imu_ok) return;

  float ax_raw = imu.getAccelX(false) - cal_ax;
  float ay_raw = imu.getAccelY(false) - cal_ay;
  float az_raw = imu.getAccelZ(false) - cal_az;
  float gy_raw = imu.getGyroY(false)  - cal_gy;

  // [E] Spike filter — physical limits of MPU6050
  if (abs(ax_raw) > MAX_ACCEL_G  ||
      abs(ay_raw) > MAX_ACCEL_G  ||
      abs(az_raw) > MAX_ACCEL_G  ||
      abs(gy_raw) > MAX_GYRO_DPS) {
    Serial.println("SPIKE: rejected sample");
    return;
  }

  // [F] Constrain adaptive gyro threshold tracking
  max_gyro_window = constrain(
    max(max_gyro_window * 0.99f, abs(gy_raw)),
    1.0f, 500.0f
  );

  kal_gy = kalmanGy.updateEstimate(gy_raw);

  ax_buf[buf_idx] = ax_raw;
  ay_buf[buf_idx] = ay_raw;
  az_buf[buf_idx] = az_raw;
  buf_idx++;

  if (buf_idx >= SAMPLES) {
    buf_idx    = 0;
    buf_filled = true;
  }

  // Adaptive step threshold [F]
  float step_thr = constrain(0.6f*max_gyro_window, 0.5f, 200.0f);
  if (state==STATE_WALKING || state==STATE_FREEZE) {
    if (prev_gy > step_thr && kal_gy <= step_thr) {
      step_ts[step_idx%8] = millis();
      step_idx++;
      step_win++;
    }
  }
  prev_gy = kal_gy;
}

// ════════════════════════════════════════════════════════
// STATE ENGINE
// ════════════════════════════════════════════════════════
void taskState(long t) {
  float rms = computeRMS();

  if (t-cad_win_start>=4000) {
    cadence       = step_win/4.0f;
    step_cov      = computeCoV();
    pd_shuffle    = (step_cov>0.25f) && (walk_rms<0.4f);
    step_win      = 0;
    cad_win_start = t;
  }

  if (state==STATE_WALKING) walk_rms=rms;

  if (state!=STATE_FREEZE && state!=STATE_TAP_TEST) {
    if (rms>0.3f && cadence>=1.0f && cadence<=3.5f) {
      if (state!=STATE_WALKING) {
        state=STATE_WALKING;
        Serial.println("STATE > WALKING");
      }
    } else if (rms<0.15f) {
      if (state!=STATE_RESTING) {
        state=STATE_RESTING;
        Serial.println("STATE > RESTING");
      }
    }
  }

  // Freeze
  if (state==STATE_WALKING && cadence<FOG_CADENCE) {
    if (low_cad_start==0) low_cad_start=t;
    if (t-low_cad_start>FOG_DURATION_MS) {
      state=STATE_FREEZE; fog_active=true;
      fog_start=t; fog_exit_cnt=0; fog_hr_count++;
      Serial.println("\n!!! FREEZE !!! RAS 1Hz ON");
      if (fog_hr_count>=3) triggerAlert("REPEATED_FREEZE");
    }
  } else { low_cad_start=0; }

  if (state==STATE_FREEZE) {
    if (cadence>1.0f) {
      fog_exit_cnt++;
      if (fog_exit_cnt>=3) {
        fog_duration=t-fog_start;
        fog_active=false; state=STATE_WALKING;
        fog_exit_cnt=0;
        Serial.println("Freeze DONE "+String(fog_duration/1000)+"s");
        if (fog_duration>10000) triggerAlert("LONG_FREEZE");
      }
    } else { fog_exit_cnt=0; }
  }

  if (!apds_ok) return;
  uint8_t prox = apds.readProximity();
  if (prox>80 && state==STATE_RESTING && !tap_active) {
    state=STATE_TAP_TEST; tap_count=0; tap_active=true;
    Serial.println("\nSTATE > TAP TEST - tap 20x above sensor");
  }
  if (state==STATE_TAP_TEST && tap_count>=20) {
    scoreBK(); state=STATE_RESTING; tap_active=false;
    Serial.println("BK: "+String(bk_grade)+" "+bk_label);
  }
  if (t-fog_hr_start>=3600000) { fog_hr_count=0; fog_hr_start=t; }
}

// ════════════════════════════════════════════════════════
// FFT TREMOR — 5 layers + tremor classification [H]
// ════════════════════════════════════════════════════════
void taskFFT() {
  if (!imu_ok) return;

  // Layer 1: Stillness
  float rms=computeRMS();
  if (rms>0.15f) { tremor_conf=0; tremor_valid=false; return; }

  float mean=0;
  for (int i=0;i<SAMPLES;i++) {
    vReal[i]=sqrt(ax_buf[i]*ax_buf[i]+
                  ay_buf[i]*ay_buf[i]+
                  az_buf[i]*az_buf[i]);
    mean+=vReal[i];
  }
  mean/=SAMPLES;
  for (int i=0;i<SAMPLES;i++) { vReal[i]-=mean; vImag[i]=0; }

  FFT.Windowing(vReal,SAMPLES,FFT_WIN_TYP_HAMMING,FFT_FORWARD);
  FFT.Compute(vReal,vImag,SAMPLES,FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal,vImag,SAMPLES);

  // PSD
  float psd_total=0, psd_pd=0, psd_et=0;
  int   dominant_bin=0;
  float dominant_psd=0;

  for (int k=1;k<SAMPLES/2;k++) {
    float p   = (vReal[k]*vReal[k])/SAMPLES;
    float freq = k*(SAMPLE_RATE/SAMPLES);
    psd_total += p;
    if (freq>=TREMOR_PD_LO && freq<=TREMOR_PD_HI) psd_pd+=p;
    if (freq>=TREMOR_ET_LO && freq<=TREMOR_ET_HI) psd_et+=p;
    if (p>dominant_psd) { dominant_psd=p; dominant_bin=k; }
  }
  dominant_freq = dominant_bin*(SAMPLE_RATE/SAMPLES);

  // Guard [7 retained]
  if (psd_total==0) { return; }

  // Layer 4: Entropy
  float entropy=0;
  for (int k=1;k<SAMPLES/2;k++) {
    float p=(vReal[k]*vReal[k])/SAMPLES/psd_total;
    if (p>0) entropy-=p*log(p);
  }
  entropy/=log(SAMPLES/2-1);
  if (entropy>=0.4f) { tremor_conf=0; tremor_valid=false; return; }

  // Layer 3: Axis symmetry (lightweight)
  float sx=0,sy=0,sz=0;
  for (int i=0;i<SAMPLES;i++) {
    sx+=ax_buf[i]*ax_buf[i];
    sy+=ay_buf[i]*ay_buf[i];
    sz+=az_buf[i]*az_buf[i];
  }
  float mx=max(sx,max(sy,sz)), mn=min(sx,min(sy,sz));
  float sym=(mx>0)?(mn/mx):0;
  if (sym<0.35f) { tremor_conf=0; tremor_valid=false; return; }

  float raw_pd=min(10.0f,(psd_pd/psd_total)*80.0f);
  float raw_et=min(10.0f,(psd_et/psd_total)*80.0f);

  // Layer 2: Consecutive confirmation
  if (raw_pd>2.0f || raw_et>2.0f) tremor_conf++;
  else tremor_conf=0;

  if (tremor_conf>=3) {
    float raw = max(raw_pd, raw_et);
    tremor_smooth=0.3f*raw+0.7f*tremor_smooth;
    tremor_valid=true;
    tremor_level=getTremorLevel(tremor_smooth);

    // [H] Classify tremor type
    if (raw_pd >= raw_et && raw_pd > 2.0f) {
      tremor_type = "PD";
    } else if (raw_et > raw_pd && raw_et > 2.0f) {
      tremor_type = "ESSENTIAL";
    } else {
      tremor_type = "NONE";
    }

    prev_scores[2]=prev_scores[1];
    prev_scores[1]=prev_scores[0];
    prev_scores[0]=tremor_smooth;
    rising_count=(prev_scores[0]>prev_scores[1]&&
                  prev_scores[1]>prev_scores[2])?
                  rising_count+1:0;

    Serial.println("\nTREMOR: "+String(tremor_smooth,1)+
                   " "+tremor_level+
                   " TYPE:"+tremor_type+
                   " FREQ:"+String(dominant_freq,1)+"Hz"+
                   " ent:"+String(entropy,3)+
                   " sym:"+String(sym,3));

    if (tremor_smooth>=TREMOR_CRITICAL) {
      critical_count++;
      if (critical_count>=3) {
        triggerAlert("CRITICAL_TREMOR");
        critical_count=0;
      }
    } else { critical_count=0; }
  } else {
    tremor_valid=false;
  }
}

// ════════════════════════════════════════════════════════
// AGGREGATE [B]
// ════════════════════════════════════════════════════════
void computeAggregate() {
  if (agg_tremor_count>0) {
    agg_tremor_avg = agg_tremor_sum / agg_tremor_count;
  } else {
    agg_tremor_avg = 0;
  }
  Serial.println("AGG: avg="+String(agg_tremor_avg,1)+
                 " max="+String(agg_tremor_max,1)+
                 " freq="+String(dominant_freq,1)+
                 "Hz type="+tremor_type);
  // Reset window
  agg_tremor_sum   = 0;
  agg_tremor_max   = 0;
  agg_tremor_count = 0;
}

// [C] Smart upload logic
bool shouldUpload() {
  bool significant_change = abs(agg_tremor_avg - last_uploaded_tremor) > 0.5f;
  bool fog_changed = fog_active;  // Always upload when frozen
  bool alert_pending = (alert_str != "NONE");
  return (significant_change || fog_changed || alert_pending);
}

// ════════════════════════════════════════════════════════
// BUZZER RAS
// ════════════════════════════════════════════════════════
void taskBuzzer(long t) {
  if (state==STATE_FREEZE) {
    if (t-last_buzz>=500) {
      last_buzz=t; buzz_on=!buzz_on;
      digitalWrite(BUZZER_PIN, buzz_on?HIGH:LOW);
    }
  } else {
    if (buzz_on) { buzz_on=false; digitalWrite(BUZZER_PIN,LOW); }
  }
}

// ════════════════════════════════════════════════════════
// TAP TEST
// ════════════════════════════════════════════════════════
void taskTap(long t) {
  if (!apds_ok || state!=STATE_TAP_TEST || tap_count>=20) return;
  static uint8_t pp=0;
  uint8_t prox=apds.readProximity();
  if (pp<tap_thresh && prox>=tap_thresh && (t-last_tap)>200) {
    tap_ts[tap_count++]=t; last_tap=t;
    Serial.println("  TAP "+String(tap_count)+"/20");
  }
  pp=prox;
}

void scoreBK() {
  if (tap_count<20) { bk_grade=-1; return; }
  float iti[19],m=0;
  for (int i=0;i<19;i++) {
    iti[i]=(tap_ts[i+1]-tap_ts[i])/1000.0f; m+=iti[i];
  }
  m/=19.0f;
  float var=0;
  for (int i=0;i<19;i++) var+=(iti[i]-m)*(iti[i]-m);
  float cov=sqrt(var/19.0f)/m;
  if      (m<0.45f&&cov<0.15f) bk_grade=0;
  else if (m<0.55f||cov<0.25f) bk_grade=1;
  else if (m<0.70f||cov<0.35f) bk_grade=2;
  else if (m<0.90f)             bk_grade=3;
  else                          bk_grade=4;
  const char* L[]={"Normal","Slight","Mild","Moderate","Severe"};
  bk_label=L[bk_grade];
  Serial.println("  ITI="+String(m,3)+" CoV="+String(cov,3));
}

// ════════════════════════════════════════════════════════
// GESTURE — medication
// ════════════════════════════════════════════════════════
void taskGesture() {
  if (!apds_ok) return;
  uint8_t g=apds.readGesture();
  if (g==APDS9960_LEFT) {
    med_ts=millis(); med_active=true; mins_dose=0;
    Serial.println("\nMEDICATION LOGGED");
  }
}

// ════════════════════════════════════════════════════════
// ENVIRONMENT
// ════════════════════════════════════════════════════════
void taskEnv() {
  if (si_ok) {
    float tr=si.getTemperatureC(false);
    float hr=si.getHumidity(false);
    if (!isnan(tr)&&!isnan(hr)) { env_t=tr; env_h=hr; }
    else Serial.println("WARN: SI7021 NaN");
  }
  if (ccs_ok) {
    ccs.setEnvironmentalData(env_h, env_t);
    int e=ccs.getECO2(false), v=ccs.getTVOC(false);
    if (e>0&&e<10000) eco2=e;
    if (v>=0&&v<5000)  tvoc=v;
  }
  if (med_active&&med_ts>0) {
    mins_dose=(millis()-med_ts)/60000UL;
    if (mins_dose>avg_dur+30 && rising_count>=3)
      triggerAlert("MISSED_DOSE");
  }
  Serial.println("ENV T="+String(env_t,1)+
                 " H="+String(env_h,1)+
                 " CO2="+String(eco2)+
                 " TVOC="+String(tvoc));
}

// ════════════════════════════════════════════════════════
// NO MOVEMENT
// ════════════════════════════════════════════════════════
void checkNoMove(long t) {
  float rms=computeRMS();
  if (rms<0.05f) {
    if (no_move_start==0) no_move_start=t;
    if ((t-no_move_start)>NO_MOVE_MS&&!no_move_sent) {
      Serial.println("!!! NO MOVEMENT 30min !!!");
      triggerAlert("NO_MOVEMENT"); no_move_sent=true;
    }
  } else { no_move_start=0; no_move_sent=false; }
}

// ════════════════════════════════════════════════════════
// POWER MANAGE
// ════════════════════════════════════════════════════════
void powerManage(long t) {
  // Disable sleep during FFT-active period
  if (state == STATE_RESTING && buf_filled) return;
  float rms=computeRMS();
  static long idle_start=0;
  if (rms<0.08f) {
    if (idle_start==0) idle_start=t;
    if (t-idle_start>10000 && sample_interval!=50) {
      sample_interval=50;
      Serial.println("Power: 20Hz idle");
    }
  } else {
    if (idle_start!=0) {
      idle_start=0; sample_interval=10;
      buf_filled=false; buf_idx=0;
      Serial.println("Power: 100Hz active");
    }
  }
}

// ════════════════════════════════════════════════════════
// ALERT
// ════════════════════════════════════════════════════════
void triggerAlert(String type) {
  alert_str=type;
  Serial.println("!!! ALERT: "+type+
                 " TR="+String(tremor_smooth,1));
  // STAGE 2: Remove //
  // Blynk.logEvent("alert", type);

  // STAGE 3: Remove //
  // String path="/emergencies/"+String(PATIENT_ID)+
  //             "/"+String(millis());
  // FirebaseJson json;
  // json.set("type",   type);
  // json.set("tremor", tremor_smooth);
  // json.set("fog",    fog_active);
  // json.set("ts",     millis());
  // Firebase.RTDB.setJSON(&fbData, path.c_str(), &json);
}

// ════════════════════════════════════════════════════════
// OLED
// ════════════════════════════════════════════════════════
void taskOLED() {
  if (!oled_ok) return;
  long s=(millis()-session_start)/1000;
  char tmr[8];
  sprintf(tmr,"%02d:%02d",(int)(s/60),(int)(s%60));
  oled.clear();
  oled.print(0,0,"TR:"+String(tremor_smooth,1)+
                 "["+tremor_type+"] BK:"+
                 (bk_grade>=0?String(bk_grade):"--"));
  oled.print(0,1,fog_active?"!!FREEZE!! RAS":"GAIT:OK");
  oled.print(0,2,"CO2:"+String(eco2)+" "+String(env_t,1)+"C");
  oled.print(0,3,String(SNAME[state])+" "+String(tmr));
  oled.display();
}

// ════════════════════════════════════════════════════════
// OUTPUT — Blynk + Firebase [B][I]
// Single JSON write per 10s aggregate
// ════════════════════════════════════════════════════════
void taskOutput() {
  last_uploaded_tremor = agg_tremor_avg;

  // STAGE 2: Remove //
  // if (Blynk.connected()) {
  //   Blynk.virtualWrite(V1, agg_tremor_avg);
  //   Blynk.virtualWrite(V2, bk_grade>=0?bk_grade:0);
  //   Blynk.virtualWrite(V3, fog_active?255:0);
  //   Blynk.virtualWrite(V4, eco2);
  //   Blynk.virtualWrite(V5, tvoc);
  //   Blynk.virtualWrite(V6, env_t);
  //   Blynk.virtualWrite(V7, pres);
  //   Blynk.virtualWrite(V8, cadence);
  //   Blynk.virtualWrite(V9, mins_dose);
  // }

  // STAGE 3: Remove //  [I] Single setJSON call
  // if (Firebase.ready()) {
  //   String path = "/patients/" + String(PATIENT_ID) +
  //                 "/data/" + today_date + "/" + String(millis());
  //   FirebaseJson json;
  //   json.set("ts",              millis());
  //   json.set("tremor_avg",      agg_tremor_avg);
  //   json.set("tremor_max",      agg_tremor_max);
  //   json.set("tremor_level",    tremor_level);
  //   json.set("tremor_type",     tremor_type);     // PD or ESSENTIAL
  //   json.set("tremor_freq_hz",  dominant_freq);
  //   json.set("bk_grade",        bk_grade);
  //   json.set("bk_label",        bk_label);
  //   json.set("fog_active",      fog_active);
  //   json.set("fog_duration_ms", fog_duration);
  //   json.set("fog_count_hr",    fog_hr_count);
  //   json.set("cadence_hz",      cadence);
  //   json.set("step_cov",        step_cov);
  //   json.set("pd_shuffle",      pd_shuffle);
  //   json.set("state",           String(SNAME[state]));
  //   json.set("med_active",      med_active);
  //   json.set("mins_since_dose", mins_dose);
  //   json.set("eco2_ppm",        eco2);
  //   json.set("tvoc_ppb",        tvoc);
  //   json.set("pressure_hpa",    pres);
  //   json.set("temp_c",          env_t);
  //   json.set("humidity_pct",    env_h);
  //   json.set("alert",           alert_str);
  //   json.set("session_id",      String(session_start));
  //   Firebase.RTDB.setJSONAsync(&fbData, path.c_str(), &json);
  //   alert_str = "NONE";
  // }
}

// ════════════════════════════════════════════════════════
// DEBUG
// ════════════════════════════════════════════════════════
void taskDebug() {
  long s=(millis()-session_start)/1000;
  Serial.println(
    "t="+String(s)+
    " |"+String(SNAME[state])+
    " |TR="+String(tremor_smooth,1)+
    " TYPE="+tremor_type+
    " FREQ="+String(dominant_freq,1)+"Hz"+
    " |BK="+String(bk_grade)+
    " |FOG="+String(fog_active)+
    " |CAD="+String(cadence,2)+
    " |COV="+String(step_cov,2)+
    " |CO2="+String(eco2)+
    " |MED="+String(mins_dose)+"min"+
    " |AGG_AVG="+String(agg_tremor_avg,1)+
    " |ALT="+alert_str
  );
}

// ════════════════════════════════════════════════════════
// HELPERS
// ════════════════════════════════════════════════════════
float computeRMS() {
  float sum=0; int n=50;
  for (int i=0;i<n;i++) {
    int   idx=(buf_idx-1-i+SAMPLES)%SAMPLES;
    float m=sqrt(ax_buf[idx]*ax_buf[idx]+
                 ay_buf[idx]*ay_buf[idx]+
                 az_buf[idx]*az_buf[idx]);
    sum+=m*m;
  }
  return sqrt(sum/n);
}

float computeCoV() {
  int n=min(step_idx,8);
  if (n<3) return 0;
  float iv[7],mi=0; int c=0;
  for (int i=0;i<n-1&&c<7;i++) {
    float d=(float)(step_ts[(step_idx-1-i)%8]-
                    step_ts[(step_idx-2-i)%8]);
    if (d>100&&d<3000) { iv[c++]=d; mi+=d; }
  }
  if (c==0) return 0;
  mi/=c;
  float v=0;
  for (int i=0;i<c;i++) v+=(iv[i]-mi)*(iv[i]-mi);
  return sqrt(v/c)/mi;
}

String getTremorLevel(float s) {
  if (s<1.0f) return "NONE";
  if (s<3.0f) return "LOW";
  if (s<6.0f) return "MED";
  if (s<9.5f) return "HIGH";
  return "CRITICAL";
}

String getDate() {
  time_t now; struct tm ti;
  time(&now); localtime_r(&now,&ti);
  char buf[12];
  strftime(buf,sizeof(buf),"%Y-%m-%d",&ti);
  return String(buf);
}
