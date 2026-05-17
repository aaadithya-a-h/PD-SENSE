---
publishDate: 2026-05-17T11:59:00Z

title: PD-SENSE — Parkinson's Disease Monitoring and Therapeutic Wrist Device

excerpt: A wrist-worn device built on the MYOSA Mini Kit that detects Parkinson's motor symptoms in real time, delivers on-device gait freeze therapy, and streams patient data to a doctor dashboard — all from a ₹150 foam wristband.

image: PD-SENSE/pd-sense-cover.jpg

tags:
  - Healthcare
  - IoT
  - ESP32
  - Signal Processing
  - MYOSA
---

> Real-time motor symptom monitoring and therapeutic support for Parkinson's patients — built entirely on the MYOSA Mini IoT Kit.

---

## Acknowledgements

Built for **IEEE MYOSA Event 5.0**. Clinical methodology based on MDS-UPDRS Item 3.4 (finger tap test) and Rhythmic Auditory Stimulation research by Thaut et al. (1996) and Rochester et al. (2010). All hardware from the official MYOSA Mini IoT Kit. Libraries sourced from the MYOSA GitHub repository.

---

## Overview

**10 million people worldwide live with Parkinson's Disease. Most see a neurologist twice a year.** Between visits, tremors go untracked, gait freeze episodes cause falls, and medication wears off silently.

PD-SENSE fills that gap. It is a wrist-worn monitoring aid and therapeutic support tool that uses the full MYOSA sensor array to continuously measure motor symptoms, deliver on-device therapy, and stream patient data to a doctor dashboard — all from a low-cost foam-and-velcro assembly.

> This device does not diagnose Parkinson's Disease. It is a monitoring aid and therapeutic support tool for patients already under neurological care.

**Key features:**

* Real-time resting tremor detection using FFT analysis (4–6 Hz band)
* Automatic gait freeze detection with immediate Rhythmic Auditory Stimulation (RAS) therapy via on-board buzzer
* Daily automated bradykinesia scoring equivalent to the clinical MDS-UPDRS finger tap test
* Continuous Firebase logging accessible from a doctor web dashboard
* Medication wear-off prediction using gesture-logged dose timing
* Environmental trigger correlation — air quality and barometric pressure vs. tremor score
* Emergency alerts for critical tremor, no movement, and missed medication

---

## Demo / Examples

### **Images**

<p align="center">
  <img src="/assets/images/PD-SENSE/pd-sense-cover.jpg" width="800"><br/>
  <i>PD-SENSE assembled on foam base — all MYOSA sensor boards connected via JST daisy-chain</i>
</p>

<p align="center">
  <img src="/assets/images/PD-SENSE/pd-sense-oled-display.jpg" width="800"><br/>
  <i>OLED live readout — tremor score, bradykinesia grade, gait state, and session timer</i>
</p>

<p align="center">
  <img src="/assets/images/PD-SENSE/pd-sense-wiring.jpg" width="800"><br/>
  <i>JST daisy-chain wiring — six sensor boards connected in sequence with direct buzzer on GPIO25</i>
</p>

<p align="center">
  <img src="/assets/images/PD-SENSE/pd-sense-doctor-dashboard.jpg" width="800"><br/>
  <i>Doctor web dashboard — Firebase real-time tremor trend, freeze events, medication timeline</i>
</p>

<p align="center">
  <img src="/assets/images/PD-SENSE/pd-sense-blynk-app.jpg" width="800"><br/>
  <i>Blynk mobile app — live SuperChart, BK grade gauge, freeze LED during competition demo</i>
</p>

### **Videos**

<video controls width="100%">
  <source src="/pd-sense-demo.mp4" type="video/mp4">
</video>

---

## Features (Detailed)

### **1. Resting Tremor Detection — FFT Signal Processing**

The MPU6050 accelerometer samples at **100 Hz** using micros()-based uniform timing (critical for FFT accuracy). Every 256 samples (2.56 seconds), a Fast Fourier Transform runs on the combined magnitude of all three axes after gravity compensation and Hamming windowing.

Power spectral density in the **3–6 Hz band** (Parkinson's resting tremor) and **6–12 Hz band** (Essential tremor) are compared. The dominant band determines tremor type. A tremor score 0–10 is computed, smoothed via exponential moving average.

**Five validation layers prevent false positives:**

| Layer | Check | Purpose |
|-------|-------|---------|
| 1 | Motion RMS < 0.15g | Arm must be still |
| 2 | 3 consecutive FFT windows confirm | Prevents single-event spikes |
| 3 | Cross-axis symmetry ratio > 0.35 | Real tremor affects all 3 axes equally |
| 4 | Spectral entropy < 0.4 | Rhythmic, not random movement |
| 5 | Device state = RESTING | No tremor scoring during walking |

```plaintext
Serial output example:
TREMOR: 7.4  TYPE: PD  FREQ: 5.1Hz  entropy: 0.21  sym: 0.61
```

---

### **2. Gait Freeze Detection + RAS Therapy**

The MPU6050 gyroscope monitors angular velocity via a **Kalman filter** during walking. Step cadence is computed over a rolling 4-second window. When cadence drops below 0.5 Hz for more than 2 seconds — a gait freeze is confirmed.

The standalone buzzer on **GPIO25** immediately fires a **1 Hz rhythm** (500ms ON / 500ms OFF). This is **Rhythmic Auditory Stimulation (RAS)** — proven by Thaut et al. (1996) and Rochester et al. (2010) to re-engage the supplementary motor cortex and break freeze episodes.

**PD-SENSE does not just detect — it treats.**

```plaintext
!!! FREEZE DETECTED !!!
RAS buzzer ON — 1Hz therapeutic rhythm
Freeze resolved: 4.2 seconds
```

Three or more freezes in one hour triggers a REPEATED_FREEZE emergency alert. Freezes longer than 10 seconds trigger a LONG_FREEZE alert — because the buzzer alone could not resolve it.

---

### **3. Bradykinesia Scoring — MDS-UPDRS Finger Tap Test**

The APDS9960 proximity sensor faces upward on the wrist. The patient taps an index finger 2 cm above the sensor **20 times**. Each tap registers as a proximity spike above an adaptive threshold set during calibration.

From 20 taps, 19 inter-tap intervals (ITI) are computed. Mean ITI and coefficient of variation produce a bradykinesia grade matching **MDS-UPDRS Item 3.4** — the international clinical standard:

| Grade | Mean ITI | Clinical Meaning |
|-------|----------|-----------------|
| 0 | < 0.45 s | Normal |
| 1 | 0.45–0.55 s | Slight slowing |
| 2 | 0.55–0.70 s | Mild — some arrests |
| 3 | 0.70–0.90 s | Moderate — frequent hesitation |
| 4 | > 0.90 s | Severe — can barely tap |

This clinical assessment runs automatically, daily, without a doctor present.

---

### **4. Medication Wear-Off Prediction**

The patient swipes LEFT over the APDS9960 gesture sensor to log a medication dose. The system timestamps the event and tracks tremor score every 10 seconds. When the tremor score begins rising consistently over 3 consecutive readings after the predicted wear-off window — a **MISSED_DOSE** alert fires via Firebase Cloud Function.

The doctor dashboard displays a medication timeline with dose markers and the tremor score at the time of each dose — making the medication effect curve visible over the full day.

---

### **5. Environmental Trigger Correlation**

| Sensor | Measurement | Clinical Relevance |
|--------|-------------|-------------------|
| CCS811 | eCO2 (400–8192 ppm), TVOC (0–1187 ppb) | High VOC worsens oxidative stress in PD |
| BMP180 | Barometric pressure (300–1100 hPa) | Pressure drops correlate with worse stiffness |
| SI7021 | Temperature + Humidity | Autonomic dysfunction tracking |

SI7021 readings are fed into CCS811 as compensation values before each read — mandatory for accuracy. Over weeks of Firebase data, the doctor dashboard surfaces correlations between environmental changes and tremor score spikes.

---

### **6. Doctor Dashboard — Firebase Real-Time**

The website uses **Firebase Realtime Database `.on()` listeners** — data pushed from ESP32 appears in the browser within 1 second, no page refresh, no API polling.

Dashboard pages:
- **Patient List** — live tremor level, freeze count, BK grade per patient
- **Single Day View** — tremor line chart with freeze zones, medication markers, environment strip, alert log
- **Multi-Day Compare** — overlay chart for up to 4 dates, trend analysis, CSV export
- **Emergency Log** — all alert events, filterable by type, real-time updates

---

### **7. Emergency Alert System**

Three conditions trigger Firebase Cloud Function → email + WhatsApp to family and doctor:

| Alert | Condition | Why human needed |
|-------|-----------|-----------------|
| CRITICAL_TREMOR | Score ≥ 9.5 for 3 readings | Medication failed or neurological crisis |
| NO_MOVEMENT | RMS < 0.05g for 30 minutes daytime | Patient may have fallen or fainted |
| MISSED_DOSE | Predicted wear-off passed + tremor rising | Patient cannot reach medication |

Gait freeze is **not** in this list — the on-device buzzer handles it automatically.

---

## Usage Instructions

**Stage 1 — Verify sensors (no WiFi needed):**

```bash
# Upload PD_SENSE_V4.ino in Arduino IDE
# Open Serial Monitor → 115200 baud
# Expected startup:
OK: MPU6050 (0x69)
OK: APDS9960 (0x39)
OK: BMP180 (0x77)
OK: SI7021 (0x40)
OK: CCS811 (0x5A)
OK: OLED
Buzzer: OK
=== READY ===
```

**Testing each feature:**

```plaintext
Tremor test    : Shake wrist at ~5Hz — score rises to 7+ on Serial Monitor
Freeze test    : Walk, then stop suddenly — buzzer fires 1Hz
Tap test       : Tap finger 20× above APDS9960 — BK grade prints after 20 taps
Medication log : Swipe hand LEFT over APDS9960 — timestamp logged
```

**Stage 2 — Add Blynk:**

```plaintext
1. Create account at blynk.cloud
2. Add BLYNK_TOKEN to config.h
3. Remove // from Stage 2 comment blocks in .ino
4. Upload — verify live charts in Blynk app
```

**Stage 3 — Add Firebase:**

```plaintext
1. Create Firebase project — enable Realtime DB + Email/Password Auth
2. Create device user (device@yourproject.com) and doctor user
3. Add all credentials to config.h
4. Remove // from Stage 3 comment blocks in .ino
5. Deploy index.html to Firebase Hosting
6. Doctor logs in — charts update live as ESP32 writes data
```

---

## Tech Stack

* **ESP32-WROOM-32E** — Main MCU, 240MHz dual-core, built-in WiFi
* **MYOSA Arduino Libraries** — AccelAndGyro, LightProximityAndGesture, BarometricPressure, TempAndHumidity, AirQuality, OLED
* **arduinoFFT** — Tremor frequency analysis (256-point, 100Hz)
* **SimpleKalmanFilter** — Gyroscope noise reduction for gait detection
* **Firebase_ESP_Client** (Mobizt) — Secure async Realtime DB writes
* **BlynkSimpleEsp32** — Live mobile dashboard streaming
* **Firebase Realtime Database** — Cloud patient data storage
* **Firebase Authentication** — Secure doctor + device login
* **Firebase Hosting** — Doctor web dashboard deployment
* **Firebase Cloud Functions (Node.js)** — Emergency alert triggers
* **Nodemailer / Twilio** — Email and WhatsApp alert delivery
* **Chart.js** — Doctor dashboard data visualisation

---

## Requirements / Installation

**MYOSA Libraries — install as ZIP:**

```bash
# Download from: github.com/myosa-sensors/arduino-libraries
# Arduino IDE → Sketch → Include Library → Add .ZIP Library
```

**Additional libraries — Arduino Library Manager:**

```bash
arduinoFFT          # by Enrique Conde-Pardo
SimpleKalmanFilter  # by Denys Sene
BlynkSimpleEsp32    # by Blynk
Firebase_ESP_Client # by Mobizt
```

**Arduino IDE board settings:**

```plaintext
Board         : ESP32 Dev Module
CPU Frequency : 240 MHz
Upload Speed  : 115200
Flash Size    : 4 MB
```

**config.h — create in same folder as .ino, add to .gitignore:**

```cpp
#define WIFI_SSID        "your_wifi_name"
#define WIFI_PASS        "your_wifi_password"
#define BLYNK_TOKEN      "your_blynk_token"
#define FB_API_KEY       "your_firebase_api_key"
#define FB_DATABASE_URL  "https://your-project.firebaseio.com"
#define FB_USER_EMAIL    "device@yourproject.com"
#define FB_USER_PASS     "your_device_password"
#define PATIENT_ID       "patient_001"
```

**Firebase Realtime Database security rules:**

```json
{
  "rules": {
    "patients": {
      "$patient_id": {
        ".read":  "auth != null",
        ".write": "auth != null"
      }
    },
    "emergencies": {
      "$patient_id": {
        ".read":  "auth != null",
        ".write": "auth != null"
      }
    }
  }
}
```

---

## File Structure

```
/PD-SENSE
  ├── PD_SENSE_V4.ino              # Main firmware — staged: serial → Blynk → Firebase
  ├── config.h                     # Credentials — NEVER commit to GitHub
  ├── index.html                   # Doctor web dashboard — deploy to Firebase Hosting
  ├── Images/
  │   ├── pd-sense-cover.jpg
  │   ├── pd-sense-oled-display.jpg
  │   ├── pd-sense-wiring.jpg
  │   ├── pd-sense-doctor-dashboard.jpg
  │   └── pd-sense-blynk-app.jpg
  ├── Video/
  │   └── pd-sense-demo.mp4
  └── PD-SENSE_MYOSA_Submission.md
```

---

## License

MIT License. Open source for research and educational use.
Not intended for clinical diagnosis. Always refer to a licensed neurologist for medical decisions.

---

## Contribution Notes

Contributions welcome. Open a GitHub issue to suggest improvements or report bugs.

Priority areas:
- Firebase Cloud Function templates for email and WhatsApp alerts
- FFT threshold calibration profiles for different patient types
- Additional patient support in the doctor dashboard

Do not commit `config.h` — credentials must remain local. All pull requests must include a clear description of changes and clinical justification for any algorithm modifications.
