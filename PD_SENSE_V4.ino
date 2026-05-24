#undef ERROR
#include "config.h"
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 10
#include <Wire.h>
#include <WiFi.h>
#include <time.h>
#include <AccelAndGyro.h>
#include <LightProximityAndGesture.h>
#undef ERROR
#include <BarometricPressure.h>
#include <OLED.h>
#include <arduinoFFT.h>
#include <SimpleKalmanFilter.h>
#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp32.h>
AccelAndGyro             imu;
LightProximityAndGesture apds;
BarometricPressure       bmp;
oLed                     oled(128, 64, &Wire, -1);

#define SAMPLES          256
#define SAMPLE_RATE      100.0
#define G_CONV           980.0f
#define BUZZER_PIN       25
#define TREMOR_CRITICAL  9.5f
#define FOG_CADENCE      0.5f
#define FOG_DURATION_MS  2000
#define SAMPLE_PERIOD_US 10000

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLE_RATE);
SimpleKalmanFilter kalmanGy(2, 2, 0.01);

typedef enum { MODE_MONITOR, MODE_TAP_TEST } AppMode;
AppMode currentMode = MODE_MONITOR;
const char* MNAME[] = {"MONITOR","TAP TEST"};

typedef enum { STATE_RESTING, STATE_WALKING, STATE_FREEZE } SystemState;
SystemState state = STATE_RESTING;
const char* SNAME[] = {"REST","WALK","FREEZE"};

bool imu_ok=false, apds_ok=false, bmp_ok=false, oled_ok=false;

float ax_buf[SAMPLES], ay_buf[SAMPLES], az_buf[SAMPLES];
int   buf_idx=0;
bool  buf_filled=false;

float cal_ax=0, cal_ay=0, cal_az=0, cal_gy=0, cal_rms=1.0f;
float max_gyro_window=1.0f;
float kal_gy=0, prev_gy=0;

float tremor_smooth=0;
bool  tremor_valid=false;
int   tremor_conf=0;
String tremor_type="NONE";
float dominant_freq=0;
float prev_scores[3]={0,0,0};
int   rising_count=0, critical_count=0;
float agg_sum=0, agg_max=0, agg_avg=0;
int   agg_cnt=0;

long  step_ts[8]={0};
int   step_idx=0, step_win=0;
long  cad_win_start=0;
float cadence=0;
bool  fog_active=false;
long  fog_start=0, fog_duration=0;
int   fog_exit_cnt=0;
long  last_buzz=0, low_cad_start=0;
bool  buzz_on=false;

long  tap_ts[20]={0};
int   tap_count=0, bk_grade=-1;
String bk_label="---";
long  last_tap=0;

long  med_ts=0;
bool  med_active=false;
int   mins_dose=0;

float env_t=0, pres=0;
long  no_move_start=0;
bool  no_move_alert=false;
long  session_start=0;
long  tm_state=0,tm_tap=0,tm_env=0,tm_oled=0;
long  tm_debug=0,tm_wifi=0,tm_agg=0;

float computeRMS() {
  float sum=0;
  for (int i=0;i<50;i++) {
    int idx=(buf_idx-1-i+SAMPLES)%SAMPLES;
    sum+=ax_buf[idx]*ax_buf[idx]+ay_buf[idx]*ay_buf[idx]+az_buf[idx]*az_buf[idx];
  }
  return sqrt(sum/50.0f);
}

String getTremorLevel(float s) {
  if (s<1.0f) return "NONE";
  if (s<3.0f) return "LOW";
  if (s<6.0f) return "MED";
  if (s<9.5f) return "HIGH";
  return "CRIT";
}

void logMedication() {
  med_ts=millis(); med_active=true; mins_dose=0;
  Serial.println(">>> MEDICATION LOGGED");
  for (int i=0;i<3;i++) { digitalWrite(BUZZER_PIN,HIGH); delay(100); digitalWrite(BUZZER_PIN,LOW); delay(100); }
}

void startTapTest() {
  currentMode=MODE_TAP_TEST;
  tap_count=0; bk_grade=-1; bk_label="---"; last_tap=0;
  Serial.println(">>> TAP TEST - tap board 20x");
  digitalWrite(BUZZER_PIN,HIGH); delay(300); digitalWrite(BUZZER_PIN,LOW);
}

void handleSerial() {
  if (!Serial.available()) return;
  char c=Serial.read();
  if (c=='m'||c=='M') logMedication();
  if (c=='t'||c=='T') startTapTest();
  if (c=='r'||c=='R') { currentMode=MODE_MONITOR; Serial.println("MODE>MONITOR"); }
  if (c=='f'||c=='F') { state=STATE_FREEZE; fog_active=true; fog_start=millis(); Serial.println(">>> FREEZE sim"); }
  if (c=='s'||c=='S') {
    Serial.println("=== STATUS ===");
    Serial.println(imu_ok?"MPU6050:OK":"MPU6050:ERR");
    Serial.println(apds_ok?"APDS:OK":"APDS:ERR");
    Serial.println(bmp_ok?"BMP180:OK":"BMP180:ERR");
    Serial.println(oled_ok?"OLED:OK":"OLED:ERR");
    Serial.println("cal_rms="+String(cal_rms,4));
    Serial.println("RMS_now="+String(computeRMS(),4));
  }
}

void connectWiFi() {
  WiFi.disconnect(true); delay(300);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi");
  int t=0;
  while (WiFi.status()!=WL_CONNECTED&&t<20) { delay(500); Serial.print("."); t++; esp_task_wdt_reset(); }
  Serial.println(WiFi.status()==WL_CONNECTED?"\nWiFi OK":"\nWiFi offline");
}

void initSensors() {
  imu_ok=imu.begin(false); Serial.println(imu_ok?"OK:MPU6050":"ERR:MPU6050");
  apds_ok=apds.begin();    Serial.println(apds_ok?"OK:APDS9960":"ERR:APDS9960");
  bmp_ok=bmp.begin();      Serial.println(bmp_ok?"OK:BMP180":"ERR:BMP180");
  oled_ok=oled.begin();
  if (oled_ok) { oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(WHITE); oled.setCursor(0,0); oled.println("PD-SENSE v5.3"); oled.println("Starting..."); oled.display(); }
  Serial.println(oled_ok?"OK:OLED":"ERR:OLED");
}

void calibrate() {
  Serial.println("--- CAL: keep still 5s ---");
  if (oled_ok) { oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(WHITE); oled.setCursor(0,0); oled.println("Calibrating..."); oled.println("Keep STILL 5s"); oled.display(); }
  if (!imu_ok) return;
  float sx=0,sy=0,sz=0,sgy=0;
  for (int i=0;i<500;i++) { esp_task_wdt_reset(); sx+=imu.getAccelX(false); sy+=imu.getAccelY(false); sz+=imu.getAccelZ(false); sgy+=imu.getGyroY(false); delay(10); }
  cal_ax=sx/500.0f; cal_ay=sy/500.0f; cal_az=sz/500.0f; cal_gy=sgy/500.0f;
  float rsum=0;
  for (int i=0;i<200;i++) { float ax=(imu.getAccelX(false)-cal_ax)/G_CONV,ay=(imu.getAccelY(false)-cal_ay)/G_CONV,az=(imu.getAccelZ(false)-cal_az)/G_CONV; rsum+=ax*ax+ay*ay+az*az; delay(10); esp_task_wdt_reset(); }
  cal_rms=sqrt(rsum/200.0f);
  if (cal_rms<0.01f) cal_rms=0.01f;
  Serial.println("cal_rms="+String(cal_rms,4)+"g tap_thr="+String(cal_rms*1.8f,3)+"g");
}

void scoreBK() {
  if (tap_count<20) { bk_grade=-1; return; }
  float iti[19],m=0;
  for (int i=0;i<19;i++) { iti[i]=(tap_ts[i+1]-tap_ts[i])/1000.0f; m+=iti[i]; }
  m/=19.0f;
  float var=0;
  for (int i=0;i<19;i++) var+=(iti[i]-m)*(iti[i]-m);
  float cov=sqrt(var/19.0f)/m;
  if      (m<0.45f&&cov<0.15f) bk_grade=0;
  else if (m<0.55f||cov<0.25f) bk_grade=1;
  else if (m<0.70f||cov<0.35f) bk_grade=2;
  else if (m<0.90f)             bk_grade=3;
  else                           bk_grade=4;
  const char* L[]={"Normal","Slight","Mild","Moderate","Severe"};
  bk_label=L[bk_grade];
}

void setup() {
  Serial.begin(115200); delay(1000);
  const esp_task_wdt_config_t wdt_cfg={.timeout_ms=WDT_TIMEOUT*1000,.idle_core_mask=0,.trigger_panic=true};
  esp_task_wdt_deinit(); esp_task_wdt_init(&wdt_cfg); esp_task_wdt_add(NULL);
  Serial.println("\n=== PD-SENSE v5.3 ===");
  pinMode(BUZZER_PIN,OUTPUT); digitalWrite(BUZZER_PIN,HIGH); delay(150); digitalWrite(BUZZER_PIN,LOW);
  Wire.begin(); Wire.setClock(100000);
  initSensors();
  calibrate();
  if (bmp_ok) { pres=bmp.getPressure()/100.0f; env_t=bmp.getTempC(false); Serial.println("BMP:"+String(env_t,1)+"C "+String(pres,1)+"hPa"); }
  connectWiFi();
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(3000);
  session_start=millis(); cad_win_start=millis(); tm_agg=millis();
  Serial.println("\n=== READY === t/m/f/r/s");
}

void loop() {
  esp_task_wdt_reset();
  long t=millis();
  handleSerial();
  Blynk.run();
  if (t-tm_wifi>=30000) { tm_wifi=t; if(WiFi.status()!=WL_CONNECTED) connectWiFi(); }
  taskIMU(t);
  if (t-tm_state>=100)  { tm_state=t; taskState(t); }
  if (buf_filled&&state==STATE_RESTING) { buf_filled=false; taskFFT(); }
  if (tremor_valid) { agg_sum+=tremor_smooth; agg_cnt++; if(tremor_smooth>agg_max) agg_max=tremor_smooth; }
  if (t-tm_tap>=20)    { tm_tap=t; taskIMUTap(t); }
  if (t-tm_env>=10000) { tm_env=t; taskEnv(); }
  taskBuzzer(t);
  checkNoMove(t);
  if (t-tm_oled>=500)  { tm_oled=t; taskOLED(); }
if (t-tm_agg>=10000) {
  tm_agg=t;
  agg_avg=(agg_cnt>0)?(agg_sum/agg_cnt):0;
  Serial.println("AGG avg="+String(agg_avg,1)+" max="+String(agg_max,1));
  agg_sum=0; agg_max=0; agg_cnt=0;
  Blynk.virtualWrite(V1, tremor_smooth);
  Blynk.virtualWrite(V2, bk_grade>=0?bk_grade:0);
  Blynk.virtualWrite(V3, fog_active?1:0);
  Blynk.virtualWrite(V6, env_t);
  Blynk.virtualWrite(V7, pres);
  Blynk.virtualWrite(V8, cadence);
  Blynk.virtualWrite(V9, mins_dose);
}
  if (t-tm_debug>=1000) { tm_debug=t; taskDebug(); }
}

void taskIMU(long t) {
  static unsigned long last_us=0;
  if (micros()-last_us<SAMPLE_PERIOD_US) return;
  last_us=micros();
  if (!imu_ok) return;
  float ax=(imu.getAccelX(false)-cal_ax)/G_CONV;
  float ay=(imu.getAccelY(false)-cal_ay)/G_CONV;
  float az=(imu.getAccelZ(false)-cal_az)/G_CONV;
  float gy=imu.getGyroY(false)-cal_gy;
  if (abs(ax)>8||abs(ay)>8||abs(az)>8||abs(gy)>500) return;
  max_gyro_window=max(max_gyro_window*0.99f,abs(gy));
  if (max_gyro_window<1.0f) max_gyro_window=1.0f;
  kal_gy=kalmanGy.updateEstimate(gy);
  ax_buf[buf_idx]=ax; ay_buf[buf_idx]=ay; az_buf[buf_idx]=az;
  buf_idx++;
  if (buf_idx>=SAMPLES) { buf_idx=0; buf_filled=true; }
  float step_thr=constrain(0.6f*max_gyro_window,0.3f,100.0f);
  if (state==STATE_WALKING||state==STATE_FREEZE) {
    if (prev_gy>step_thr&&kal_gy<=step_thr) { step_ts[step_idx%8]=t; step_idx++; step_win++; }
  }
  prev_gy=kal_gy;
}

void taskIMUTap(long t) {
  if (currentMode!=MODE_TAP_TEST) return;
  if (tap_count>=20) {
    scoreBK();
    Serial.println("BK grade="+String(bk_grade)+" "+bk_label);
    if (oled_ok) {
      oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(WHITE);
      oled.setCursor(0,0); oled.println("BK RESULT:");
      oled.setCursor(0,16); oled.setTextSize(2); oled.println("Grade "+String(bk_grade));
      oled.setTextSize(1); oled.setCursor(0,40); oled.println(bk_label); oled.display();
    }
    delay(3000);
    currentMode=MODE_MONITOR; tap_count=0;
    return;
  }
  if (!imu_ok) return;
  float ax=(imu.getAccelX(false)-cal_ax)/G_CONV;
  float ay=(imu.getAccelY(false)-cal_ay)/G_CONV;
  float az=(imu.getAccelZ(false)-cal_az)/G_CONV;
  float instant=sqrt(ax*ax+ay*ay+az*az);
  static float prev_instant=0;
  float tap_thr=cal_rms*1.8f;
  if (prev_instant<tap_thr&&instant>=tap_thr&&(t-last_tap)>200) {
    tap_ts[tap_count++]=t; last_tap=t;
    Serial.println("TAP "+String(tap_count)+"/20 val="+String(instant,2));
    digitalWrite(BUZZER_PIN,HIGH); delay(30); digitalWrite(BUZZER_PIN,LOW);
  }
  prev_instant=instant;
}

void taskState(long t) {
  float rms=computeRMS();
  float walk_thr=cal_rms*2.0f;
  float rest_thr=cal_rms*1.5f;
  if (t-cad_win_start>=4000) { cadence=step_win/4.0f; step_win=0; cad_win_start=t; }
  if (currentMode==MODE_MONITOR) {
    if (state!=STATE_FREEZE) {
      if (rms>walk_thr) { if(state!=STATE_WALKING){ state=STATE_WALKING; Serial.println("STATE>WALKING"); } }
      else if (rms<rest_thr) { if(state!=STATE_RESTING){ state=STATE_RESTING; Serial.println("STATE>RESTING"); } }
    }
    if (state==STATE_WALKING&&cadence<FOG_CADENCE) {
      if (!low_cad_start) low_cad_start=t;
      if (t-low_cad_start>FOG_DURATION_MS) { state=STATE_FREEZE; fog_active=true; fog_start=t; fog_exit_cnt=0; Serial.println("!!! FREEZE !!! Buzzer ON"); }
    } else { low_cad_start=0; }
    if (state==STATE_FREEZE&&cadence>1.0f) {
      fog_exit_cnt++;
      if (fog_exit_cnt>=3) { fog_duration=t-fog_start; fog_active=false; state=STATE_WALKING; fog_exit_cnt=0; Serial.println("Freeze DONE "+String(fog_duration/1000)+"s"); }
    } else if (state==STATE_FREEZE) fog_exit_cnt=0;
  }
}

void taskFFT() {
  if (!imu_ok) return;
  if (computeRMS()>cal_rms*1.8f) { tremor_conf=0; tremor_valid=false; return; }
  float mean=0;
  for (int i=0;i<SAMPLES;i++) { vReal[i]=sqrt(ax_buf[i]*ax_buf[i]+ay_buf[i]*ay_buf[i]+az_buf[i]*az_buf[i]); mean+=vReal[i]; }
  mean/=SAMPLES;
  for (int i=0;i<SAMPLES;i++) { vReal[i]-=mean; vImag[i]=0; }
  FFT.windowing(FFTWindow::Hamming,FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
  float psd_total=0,psd_pd=0,psd_et=0; int dom_bin=0; float dom_psd=0;
  for (int k=1;k<SAMPLES/2;k++) {
    float p=(vReal[k]*vReal[k])/SAMPLES, freq=k*(SAMPLE_RATE/SAMPLES);
    psd_total+=p;
    if (freq>=3.0f&&freq<=6.0f) psd_pd+=p;
    if (freq>6.0f&&freq<=8.0f)  psd_et+=p;
    if (p>dom_psd) { dom_psd=p; dom_bin=k; }
  }
  dominant_freq=dom_bin*(SAMPLE_RATE/SAMPLES);
  if (psd_total<0.0001f) return;
  float raw_pd=min(10.0f,(psd_pd/psd_total)*80.0f);
  float raw_et=min(10.0f,(psd_et/psd_total)*80.0f);
  float raw=max(raw_pd,raw_et);
  if (raw>1.0f) tremor_conf++; else tremor_conf=0;
  if (tremor_conf>=3) {
    tremor_smooth=0.4f*raw+0.6f*tremor_smooth; tremor_valid=true;
    tremor_type=(raw_pd>=raw_et&&raw_pd>1.0f)?"PD":(raw_et>raw_pd&&raw_et>1.0f)?"ESSENTIAL":"NONE";
    prev_scores[2]=prev_scores[1]; prev_scores[1]=prev_scores[0]; prev_scores[0]=tremor_smooth;
    rising_count=(prev_scores[0]>prev_scores[1]&&prev_scores[1]>prev_scores[2])?rising_count+1:0;
    Serial.println("TREMOR="+String(tremor_smooth,1)+" "+getTremorLevel(tremor_smooth)+" "+tremor_type+" "+String(dominant_freq,1)+"Hz");
    if (tremor_smooth>=TREMOR_CRITICAL) { critical_count++; if(critical_count>=3){ Serial.println("!!! CRITICAL TREMOR !!!"); critical_count=0; } }
    else critical_count=0;
  } else { tremor_valid=false; tremor_smooth*=0.95f; }
}

void taskBuzzer(long t) {
  if (state==STATE_FREEZE&&currentMode==MODE_MONITOR) {
    if (t-last_buzz>=500) { last_buzz=t; buzz_on=!buzz_on; digitalWrite(BUZZER_PIN,buzz_on?HIGH:LOW); }
  } else { if(buzz_on){ buzz_on=false; digitalWrite(BUZZER_PIN,LOW); } }
}

void checkNoMove(long t) {
  if (computeRMS()<cal_rms*1.2f) {
    if (!no_move_start) no_move_start=t;
    if ((t-no_move_start)>1800000UL&&!no_move_alert) { Serial.println("!!! NO MOVEMENT 30min !!!"); no_move_alert=true; for(int i=0;i<5;i++){ digitalWrite(BUZZER_PIN,HIGH); delay(200); digitalWrite(BUZZER_PIN,LOW); delay(200); } }
  } else { no_move_start=0; no_move_alert=false; }
}

void taskEnv() {
  if (bmp_ok) { float nt=bmp.getTempC(false),np=bmp.getPressure()/100.0f; if(!isnan(nt)) env_t=nt; if(np>800&&np<1100) pres=np; }
  if (med_active&&med_ts>0) { mins_dose=(millis()-med_ts)/60000UL; if(mins_dose>240&&rising_count>=2) Serial.println("!!! MISSED DOSE !!!"); }
  Serial.println("ENV T="+String(env_t,1)+"C P="+String(pres,1)+"hPa");
}

void taskOLED() {
  if (!oled_ok) return;
  long s=(millis()-session_start)/1000;
  char tmr[8]; sprintf(tmr,"%02d:%02d",(int)(s/60),(int)(s%60));
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(WHITE);
  oled.setCursor(0,0); oled.println("["+String(MNAME[currentMode])+"] "+String(tmr));
  if (currentMode==MODE_MONITOR) {
    oled.setCursor(0,12); oled.println("TR:"+String(tremor_smooth,1)+" "+getTremorLevel(tremor_smooth)+" "+tremor_type);
    oled.setCursor(0,22); oled.println(fog_active?"!!FREEZE!! BUZZER":String(SNAME[state])+" C:"+String(cadence,1));
    oled.setCursor(0,32); oled.println("T:"+String(env_t,1)+"C P:"+String(pres,0));
    oled.setCursor(0,42); oled.println(med_active?"MED:"+String(mins_dose)+"min":"MED:none");
    oled.setCursor(0,52); oled.println("BK:"+(bk_grade>=0?String(bk_grade)+" "+bk_label:String("---")));
  } else {
    oled.setCursor(0,12); oled.println("Tap MPU6050 firmly");
    oled.setCursor(0,28); oled.setTextSize(2); oled.println(String(tap_count)+"/20");
    oled.setTextSize(1); oled.setCursor(0,50);
    oled.println(bk_grade>=0?"BK:"+String(bk_grade)+" "+bk_label:"Tapping...");
  }
  oled.display();
}

void taskDebug() {
  long s=(millis()-session_start)/1000;
  Serial.println("t="+String(s)+" ST="+String(SNAME[state])+" TR="+String(tremor_smooth,1)+" "+getTremorLevel(tremor_smooth)+" RMS="+String(computeRMS(),3)+" CAD="+String(cadence,2)+" FOG="+String(fog_active)+" BK="+String(bk_grade)+" MED="+String(mins_dose)+"m T="+String(env_t,1)+"C");
}