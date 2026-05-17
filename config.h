/*
=============================================================
  config.h — PD-SENSE Credentials File
=============================================================

  HOW TO USE THIS FILE:
  ─────────────────────
  1. This file goes in the SAME FOLDER as PD_SENSE_V4.ino
     Your folder should look like this:
       📁 PD_SENSE_V4
           📄 PD_SENSE_V4.ino   ← main code
           📄 config.h          ← this file (YOU EDIT THIS)

  2. Fill in EVERY value between the quotes below
     Replace only what is between "  "
     Do not delete the # or define words

  3. NEVER upload this file to GitHub
     Add config.h to your .gitignore file
     The .ino file is safe to share — it has no secrets

  HOW TO GET EACH VALUE:
  ──────────────────────
  WIFI_SSID     → Your home or phone hotspot WiFi name
  WIFI_PASS     → Your WiFi password

  BLYNK_TOKEN   → From Blynk app:
                  Create account at blynk.cloud
                  New Template → New Device → copy Auth Token

  FB_API_KEY    → From Firebase Console:
                  console.firebase.google.com
                  Project Settings → General → Web API Key

  FB_DATABASE_URL → From Firebase Console:
                  Realtime Database → copy the URL shown
                  Format: https://your-project.firebaseio.com

  FB_USER_EMAIL → The email you created in Firebase Auth
                  Authentication → Users → Add User
                  Example: device@yourproject.com

  FB_USER_PASS  → The password for that Firebase Auth user

  PATIENT_ID    → Any name you want for this patient
                  No spaces allowed. Use underscore.
                  Example: patient_001 or arun_patient

=============================================================
*/

// ── WiFi credentials ─────────────────────────────────────
#define WIFI_SSID        "your_wifi_name_here"
#define WIFI_PASS        "your_wifi_password_here"

// ── Blynk (add after Stage 1 works) ──────────────────────
#define BLYNK_TOKEN      "your_blynk_token_here"

// ── Firebase (add after Stage 2 works) ───────────────────
#define FB_API_KEY       "your_firebase_api_key_here"
#define FB_DATABASE_URL  "https://your-project.firebaseio.com"
#define FB_USER_EMAIL    "device@yourproject.com"
#define FB_USER_PASS     "your_device_password_here"

// ── Patient ID ────────────────────────────────────────────
#define PATIENT_ID       "patient_001"
