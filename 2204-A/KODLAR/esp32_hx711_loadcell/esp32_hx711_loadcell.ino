/*
  ESP32 DevKitC V4 + HX711 + 20 kg load cell
  Kutuphane: "HX711" (Bogdan Necula / bogde) - Arduino Kutuphane Yoneticisi'nden kurun.

  Baglanti:
    HX711 VCC -> ESP32 3V3
    HX711 GND -> ESP32 GND
    HX711 DT  -> ESP32 GPIO16
    HX711 SCK -> ESP32 GPIO4

  HD-38 toprak nem sensoru (dirençli, metal cubuklu) modulu:
    VCC -> ESP32 GPIO25 (sensor sadece olcum sirasinda beslenir; 5V KULLANMAYIN)
           (3V3'e baglamak isterseniz SOIL_POWER_PIN = -1 yapin)
    GND -> ESP32 GND
    AO  -> ESP32 GPIO34
    DO  -> bos birakin (dijital cikis kullanilmiyor)

  DHT11 (4 bacakli, mavi, cplak sensor), one bakinca soldan saga:
    pin1 VCC  -> ESP32 GPIO33 (kod HIGH yapar)
    pin2 DATA -> ESP32 GPIO32
    pin3 NC   -> bos
    pin4 GND  -> ESP32 "GND" yazan pin (DIKKAT: "CMD" pini GND degildir, flash hattidir!)
    10 kOhm direnc: pin1 (VCC) ile pin2 (DATA) arasina
    Ek kutuphane: "DHT sensor library" (Adafruit) + "Adafruit Unified Sensor"

  Load cell kablolari (genelde): Kirmizi=E+, Siyah=E-, Beyaz=A-, Yesil=A+
  (Renkler uretciye gore degisebilir; deger ters/negatif cikarsa A+ ve A- yer degistirin.)

  Seri Monitor: 115200 baud, "Newline" secili olmali.
  Komutlar:
    t        -> Tara (sifirla), yuk cell'in ustundeyken yapmayin
    c<gram>  -> Kalibrasyon. Bilinen agirligi koyup ornegin  c500  yazin (500 g)
    r        -> Kayitli kalibrasyonu sil
    d        -> Nem sensoru KURU (havada) referansini kaydet
    w        -> Nem sensoru ISLAK (suya batirilmis) referansini kaydet
    x        -> Nem kalibrasyonunu sil
    p<mL>    -> Sulama dozu (ornek p10). Guvenlik sinirlari asilirsa reddedilir
    pt<sn>   -> Pompayi N saniye calistir (debi kalibrasyonu icin), sonra olculen hacimden
    pf<mL/sn>-> debiyi kaydet (ornek pf0.85)
    q        -> Pompayi hemen durdur
    f1 / f0  -> Fan ac / kapat (acilista her zaman kapali)
    i        -> Sinirlari ve bugunku toplam sulamayi yazdir

  Pompa/fan (12 V, MOSFET modulu): PUMP_PIN=GPIO26, FAN_PIN=GPIO27. Pompa karttan beslenmez.
  Tara (t) komutuyla sifir noktasi kartta saklanir, acilista taralanmaz.
*/

#include <Arduino.h>
#include <HX711.h>
#include <Preferences.h>
#include <DHT.h>

const int HX_DT  = 16;
const int HX_SCK = 4;

// ---- DHT11 (hava sicaklik / nem) ----
const int  DHT_PIN       = 32;     // DATA
const int  DHT_POWER_PIN = 33;     // VCC bu GPIO'dan beslenir. -1 yaparsaniz VCC'yi 3V3'e baglayin
const unsigned long DHT_PERIOD_MS = 2000;   // DHT11 en fazla ~1 olcum/sn
DHT dht(DHT_PIN, DHT11);
unsigned long lastDhtMs = 0;

HX711 scale;
Preferences prefs;

float calFactor = 1.0f;      // ham deger / gram
bool  calibrated = false;

// ---- Filtre ayarlari ----
const int   MED_N      = 7;      // medyan penceresi (tek sayi). Buyuk = daha cok sicrama bastirma
const float EMA_ALPHA  = 0.25f;  // 0..1. Kucuk = daha yumusak ama yavas tepki
const float DEADBAND_G = 0.5f;   // bu kadardan az degisimde ekrandaki deger sabit kalir
const float ZERO_SNAP  = 1.0f;   // |deger| bunun altindaysa 0 yazilir (gram)

float medBuf[MED_N];
int   medCount = 0, medIdx = 0;
float ema = 0.0f;
bool  emaInit = false;
float shown = 0.0f;

void resetFilter() {
  medCount = 0; medIdx = 0; emaInit = false;
}

float median(float *src, int n) {
  float tmp[MED_N];
  for (int i = 0; i < n; i++) tmp[i] = src[i];
  for (int i = 1; i < n; i++) {            // basit insertion sort
    float k = tmp[i]; int j = i - 1;
    while (j >= 0 && tmp[j] > k) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = k;
  }
  return tmp[n / 2];
}

float filterSample(float x) {
  medBuf[medIdx] = x;
  medIdx = (medIdx + 1) % MED_N;
  if (medCount < MED_N) medCount++;
  float m = median(medBuf, medCount);
  if (!emaInit) { ema = m; emaInit = true; shown = m; }
  else          { ema += EMA_ALPHA * (m - ema); }
  if (fabsf(ema - shown) >= DEADBAND_G) shown = ema;
  return shown;
}

// ---- Pompa ve fan (MOSFET surucu ile) ----
// Pompa/fan karttan BESLENMEZ: harici 12 V adaptor + MOSFET modulu + her pompaya paralel ters diyot.
// Adaptorun eksisi ESP32 GND'sine baglanmali (ortak GND).
const int   PUMP_PIN   = 26;      // MOSFET modulu giris (SIG). Girise 10 kOhm pull-down koyun!
const int   FAN_PIN    = 27;      // fan MOSFET modulu girisi
const int   LEVEL_PIN  = -1;      // depo seviye anahtari girisi (-1 = devre disi)
const int   LEVEL_EMPTY_STATE = HIGH;   // depo bosken okunan seviye

// Guvenlik sinirlari. Pilot sonrasi yazili kilitlenecek; simdilik temkinli varsayilanlar.
const float MAX_DOSE_ML    = 50.0f;                   // tek seferde en fazla (mL)
const unsigned long MAX_PUMP_MS = 60UL * 1000UL;      // pompa en fazla bu kadar surekli calisir
const unsigned long MIN_WAIT_MS = 10UL * 60UL * 1000UL; // iki doz arasi asgari bekleme
const float DAILY_MAX_ML   = 200.0f;                  // 24 saatte toplam (RTC gelene kadar calisma suresine gore)

float pumpFlowMlPerS = 1.0f;      // KALIBRE EDILMELI: 'pt10' ile calistirin, olcuyu 'pf<mL/s>' ile girin
bool  pumpOn = false;
unsigned long pumpStartMs = 0, pumpDurMs = 0;
unsigned long lastDoseEndMs = 0;
bool  anyDose = false;
float dailyMl = 0.0f;
unsigned long dayStartMs = 0;
unsigned long lastHxOkMs = 0;     // HX711 son basarili okuma zamani
bool  fanOn = false;

bool weightValid() { return lastHxOkMs != 0 && millis() - lastHxOkMs < 3000; }

void pumpStop(const char* reason) {
  digitalWrite(PUMP_PIN, LOW);
  if (!pumpOn) return;
  pumpOn = false;
  float ml = (millis() - pumpStartMs) / 1000.0f * pumpFlowMlPerS;
  dailyMl += ml;
  lastDoseEndMs = millis();
  anyDose = true;
  Serial.print("Pompa: BITTI ("); Serial.print(reason); Serial.println(")");
  Serial.print("Sulama_mL: "); Serial.println(ml, 1);
}

void updatePump() {
  if (!pumpOn) return;
  unsigned long ran = millis() - pumpStartMs;
  if (ran >= pumpDurMs)                    pumpStop("sure doldu");
  else if (ran >= MAX_PUMP_MS)             pumpStop("azami sure siniri");
  else if (!weightValid())                 pumpStop("agirlik sensoru yanit vermiyor");
  else if (LEVEL_PIN >= 0 && digitalRead(LEVEL_PIN) == LEVEL_EMPTY_STATE) pumpStop("depo bos");
}

// Uzun bekleme sirasinda da pompa zamanlamasini kontrol eder
void delayWithPump(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { updatePump(); delay(2); }
}

#define PUMP_REJECT(msg) do { Serial.print("Pompa reddedildi: "); Serial.println(msg); return false; } while (0)

// seconds: calisma suresi, ml: tahmini hacim, isTest: kalibrasyon testi (doz ve bekleme sinirlari uygulanmaz)
bool startPump(float seconds, float ml, bool isTest) {
  if (millis() - dayStartMs >= 86400000UL) { dayStartMs = millis(); dailyMl = 0; }
  if (pumpOn)                               PUMP_REJECT("zaten calisiyor");
  if (seconds <= 0 || ml <= 0)              PUMP_REJECT("gecersiz miktar");
  if (!isTest && ml > MAX_DOSE_ML)          PUMP_REJECT("tek doz siniri asildi");
  if (seconds * 1000.0f > MAX_PUMP_MS)      PUMP_REJECT("azami pompa suresi asildi");
  if (!weightValid())                       PUMP_REJECT("agirlik sensoru yanit vermiyor");
  if (LEVEL_PIN >= 0 && digitalRead(LEVEL_PIN) == LEVEL_EMPTY_STATE) PUMP_REJECT("depo bos");
  if (!isTest && anyDose && millis() - lastDoseEndMs < MIN_WAIT_MS) {
    Serial.print("Pompa reddedildi: bekleme suresi dolmadi, kalan s=");
    Serial.println((MIN_WAIT_MS - (millis() - lastDoseEndMs)) / 1000);
    return false;
  }
  if (dailyMl + ml > DAILY_MAX_ML)          PUMP_REJECT("gunluk toplam siniri asildi");

  pumpStartMs = millis();
  pumpDurMs = (unsigned long)(seconds * 1000.0f);
  pumpOn = true;
  digitalWrite(PUMP_PIN, HIGH);
  Serial.print("Pompa: BASLADI ml="); Serial.print(ml, 1);
  Serial.print(" sure_s="); Serial.println(seconds, 1);
  return true;
}

// ---- Toprak nem sensoru (kapasitif, analog) ----
const int   SOIL_PIN        = 34;     // ADC1_CH6, sadece giris pini
const int   SOIL_SAMPLES    = 64;     // her olcumde ortalama alinacak ornek
const float SOIL_EMA_ALPHA  = 0.2f;
const unsigned long SOIL_PERIOD_MS = 1000;

int   soilDry = 0, soilWet = 0;       // kalibrasyon ham degerleri (kuru=havada, islak=suda)
bool  soilCalibrated = false;
float soilEma = 0.0f;
bool  soilEmaInit = false;
unsigned long lastSoilMs = 0;

// HD-38 dirençli sensor: cubuklar surekli akimda kalinca paslanir.
// VCC'yi bir GPIO'ya baglarsaniz (ornegin 25) ve asagidaki sabiti 25 yaparsaniz
// sensor sadece olcum sirasinda beslenir. -1 = VCC dogrudan 3V3'e bagli (guc kontrolu yok).
const int   SOIL_POWER_PIN  = 25;
const int   SOIL_SETTLE_MS  = 300;   // besleme acildiktan sonra AO'nun oturmasi icin bekleme

float readSoilRaw(int n) {
  if (SOIL_POWER_PIN >= 0) { digitalWrite(SOIL_POWER_PIN, HIGH); delayWithPump(SOIL_SETTLE_MS); }
  long sum = 0;
  for (int i = 0; i < n; i++) { sum += analogRead(SOIL_PIN); delayMicroseconds(200); }
  if (SOIL_POWER_PIN >= 0) digitalWrite(SOIL_POWER_PIN, LOW);
  return (float)sum / n;
}

void saveSoilCal() {
  prefs.putInt("sdry", soilDry);
  prefs.putInt("swet", soilWet);
}

void updateSoilCalFlag() {
  soilCalibrated = (soilDry > 0 && soilWet > 0 && abs(soilDry - soilWet) > 100);
}

void tareScale() {
  Serial.println("Taralaniyor, load cell bos olmali...");
  scale.tare(20);
  prefs.putLong("toff", scale.get_offset());   // elektrik kesilince sifir noktasi korunur
  resetFilter();
  Serial.println("Tara tamam (kaydedildi).");
}

void calibrate(float knownGrams) {
  if (knownGrams <= 0) {
    Serial.println("Gecersiz agirlik.");
    return;
  }
  long raw = scale.get_value(20);          // tara cikarilmis ham deger
  if (raw == 0) {
    Serial.println("Ham deger 0, agirlik koyduginizdan emin olun.");
    return;
  }
  calFactor = (float)raw / knownGrams;
  scale.set_scale(calFactor);
  calibrated = true;
  prefs.putFloat("cal", calFactor);
  resetFilter();
  Serial.print("Kalibrasyon kaydedildi, faktor = ");
  Serial.println(calFactor, 4);
}

void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;

  if (s == "t") {
    tareScale();
  } else if (s[0] == 'c') {
    calibrate(s.substring(1).toFloat());
  } else if (s == "r") {
    prefs.remove("cal");
    calibrated = false;
    scale.set_scale(1.0f);
    resetFilter();
    Serial.println("Kalibrasyon silindi.");
  } else if (s == "d") {
    soilDry = (int)readSoilRaw(256);
    updateSoilCalFlag(); saveSoilCal(); soilEmaInit = false;
    Serial.print("Nem KURU (havada) kaydedildi: "); Serial.println(soilDry);
  } else if (s == "w") {
    soilWet = (int)readSoilRaw(256);
    updateSoilCalFlag(); saveSoilCal(); soilEmaInit = false;
    Serial.print("Nem ISLAK (suda) kaydedildi: "); Serial.println(soilWet);
  } else if (s == "s") {
    // Tani: besleme acildiktan sonra AO'nun zamanla degisimini yazdirir
    if (SOIL_POWER_PIN >= 0) digitalWrite(SOIL_POWER_PIN, HIGH);
    for (int i = 0; i < 30; i++) {
      long sum = 0;
      for (int k = 0; k < 16; k++) { sum += analogRead(SOIL_PIN); delayMicroseconds(200); }
      Serial.print("SoilTrace t="); Serial.print(i * 20);
      Serial.print("ms v="); Serial.println(sum / 16);
      delay(17);
    }
    if (SOIL_POWER_PIN >= 0) digitalWrite(SOIL_POWER_PIN, LOW);
  } else if (s == "h") {
    // Tani: DHT hattinin bosta seviyesi ve baslat darbesine verilen yanit (kenar sayisi)
    pinMode(DHT_PIN, INPUT);        delay(5); int idleFloat = digitalRead(DHT_PIN);
    pinMode(DHT_PIN, INPUT_PULLDOWN); delay(5); int idleDown = digitalRead(DHT_PIN);
    pinMode(DHT_PIN, INPUT_PULLUP); delay(5); int idlePull  = digitalRead(DHT_PIN);
    Serial.print("DHTdiag bosta(pulldown)="); Serial.println(idleDown);   // 1 => hatta harici pull-up bagli
    pinMode(DHT_PIN, OUTPUT); digitalWrite(DHT_PIN, LOW); delay(20);
    digitalWrite(DHT_PIN, HIGH); delayMicroseconds(40);
    pinMode(DHT_PIN, INPUT_PULLUP);
    int edges = 0, last = digitalRead(DHT_PIN);
    unsigned long t0 = micros();
    while (micros() - t0 < 6000) {
      int v = digitalRead(DHT_PIN);
      if (v != last) { edges++; last = v; }
    }
    Serial.print("DHTdiag bosta(cekmesiz)="); Serial.print(idleFloat);
    Serial.print(" bosta(pullup)="); Serial.print(idlePull);
    Serial.print(" kenar_sayisi="); Serial.println(edges);   // saglikli DHT11: ~80+
  } else if (s == "x") {
    prefs.remove("sdry"); prefs.remove("swet");
    soilDry = soilWet = 0; soilCalibrated = false; soilEmaInit = false;
    Serial.println("Nem kalibrasyonu silindi.");
  } else if (s == "q") {
    if (pumpOn) pumpStop("elle durduruldu"); else { digitalWrite(PUMP_PIN, LOW); Serial.println("Pompa zaten kapali."); }
  } else if (s.startsWith("pt")) {            // kalibrasyon testi: pt10 = 10 sn calistir
    float sec = s.substring(2).toFloat();
    startPump(sec, sec * pumpFlowMlPerS, true);
  } else if (s.startsWith("pf")) {            // debi gir: pf0.85 (mL/sn)
    float f = s.substring(2).toFloat();
    if (f >= 0.05f && f <= 10.0f) {
      pumpFlowMlPerS = f; prefs.putFloat("flow", f);
      Serial.print("Pompa debisi kaydedildi (mL/sn): "); Serial.println(f, 3);
    } else Serial.println("Gecersiz debi (0.05-10 mL/sn).");
  } else if (s[0] == 'p') {                   // doz: p10 = 10 mL
    float ml = s.substring(1).toFloat();
    if (ml > 0) startPump(ml / pumpFlowMlPerS, ml, false);
    else Serial.println("Kullanim: p<mL>");
  } else if (s == "f1") {
    fanOn = true;  digitalWrite(FAN_PIN, HIGH); Serial.println("Fan: ACIK");
  } else if (s == "f0") {
    fanOn = false; digitalWrite(FAN_PIN, LOW);  Serial.println("Fan: KAPALI");
  } else if (s == "i") {
    Serial.print("Bilgi: debi_mL_s="); Serial.print(pumpFlowMlPerS, 3);
    Serial.print(" tek_doz_max="); Serial.print(MAX_DOSE_ML, 0);
    Serial.print(" azami_sure_s="); Serial.print(MAX_PUMP_MS / 1000);
    Serial.print(" bekleme_s="); Serial.print(MIN_WAIT_MS / 1000);
    Serial.print(" gunluk_max="); Serial.print(DAILY_MAX_ML, 0);
    Serial.print(" bugun_mL="); Serial.println(dailyMl, 1);
  } else {
    Serial.println("Komutlar: t | c<gram> | r | d | w | x | p<mL> | pt<sn> | pf<mL/sn> | q | f1 | f0 | i");
  }
}

void setup() {
  // Guvenlik: pompa ve fan her durumda KAPALI baslar (ilk satirlar)
  pinMode(PUMP_PIN, OUTPUT); digitalWrite(PUMP_PIN, LOW);
  pinMode(FAN_PIN,  OUTPUT); digitalWrite(FAN_PIN,  LOW);
  if (LEVEL_PIN >= 0) pinMode(LEVEL_PIN, INPUT_PULLUP);
  dayStartMs = millis();

  Serial.begin(115200);
  delay(500);
  Serial.println("\nESP32 HX711 agirlik olcer");

  scale.begin(HX_DT, HX_SCK);

  prefs.begin("loadcell", false);
  pumpFlowMlPerS = prefs.getFloat("flow", 1.0f);
  calFactor = prefs.getFloat("cal", 0.0f);
  if (calFactor != 0.0f) {
    calibrated = true;
    scale.set_scale(calFactor);
    Serial.print("Kayitli kalibrasyon yuklendi: ");
    Serial.println(calFactor, 4);
  } else {
    scale.set_scale(1.0f);
    Serial.println("Kalibrasyon yok. Bos iken 't', sonra agirlik koyup 'c<gram>' yazin.");
  }

  if (DHT_POWER_PIN >= 0) {
    pinMode(DHT_POWER_PIN, OUTPUT);
    digitalWrite(DHT_POWER_PIN, HIGH);   // DHT11 surekli beslenir
  }
  dht.begin();

  if (SOIL_POWER_PIN >= 0) { pinMode(SOIL_POWER_PIN, OUTPUT); digitalWrite(SOIL_POWER_PIN, LOW); }
  analogReadResolution(12);
  analogSetPinAttenuation(SOIL_PIN, ADC_11db);   // ~0-3.1 V
  soilDry = prefs.getInt("sdry", 0);
  soilWet = prefs.getInt("swet", 0);
  updateSoilCalFlag();
  if (soilCalibrated) {
    Serial.print("Nem kalibrasyonu yuklendi: kuru="); Serial.print(soilDry);
    Serial.print(" islak="); Serial.println(soilWet);
  } else {
    Serial.println("Nem kalibrasyonu yok. Sensor havada iken 'd', suya batirinca 'w' yazin.");
  }

  if (scale.wait_ready_timeout(2000)) {
    if (prefs.isKey("toff")) {
      scale.set_offset(prefs.getLong("toff"));
      Serial.println("Kayitli tara yuklendi (acilista taralanmadi).");
    } else {
      scale.tare(20);
      prefs.putLong("toff", scale.get_offset());
      Serial.println("Ilk tara yapildi ve kaydedildi.");
    }
  } else {
    Serial.println("HX711 bulunamadi! Baglantilari kontrol edin.");
  }
}

void printSoil() {
  float raw = readSoilRaw(SOIL_SAMPLES);
  if (!soilEmaInit) { soilEma = raw; soilEmaInit = true; }
  else              { soilEma += SOIL_EMA_ALPHA * (raw - soilEma); }

  Serial.print("NemHam: ");
  Serial.println(soilEma, 0);

  if (soilCalibrated) {
    // kuru ham deger genelde buyuk, islak kucuk; ters olsa da formul calisir
    float pct = 100.0f * (soilEma - soilDry) / (float)(soilWet - soilDry);
    pct = constrain(pct, 0.0f, 100.0f);
    Serial.print("Nem: ");
    Serial.print(pct, 1);
    Serial.println(" %");
  }
}

void printDht() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("DHT hata: okunamadi (baglanti/direnc kontrol edin)");
    return;
  }
  Serial.print("Sicaklik: ");
  Serial.print(t, 1);
  Serial.println(" C");
  Serial.print("Hava nemi: ");
  Serial.print(h, 0);
  Serial.println(" %");
}

void loop() {
  handleSerial();
  updatePump();

  if (millis() - lastDhtMs >= DHT_PERIOD_MS) {
    lastDhtMs = millis();
    printDht();
  }

  if (millis() - lastSoilMs >= SOIL_PERIOD_MS) {
    lastSoilMs = millis();
    printSoil();
    Serial.print("PompaDurum: "); Serial.println(pumpOn ? 1 : 0);
    Serial.print("FanDurum: ");   Serial.println(fanOn ? 1 : 0);
  }

  if (!scale.wait_ready_timeout(1000)) {
    Serial.println("HX711 yanit vermiyor.");
    return;
  }
  lastHxOkMs = millis();

  float rawG = scale.get_units(1);        // ham (filtresiz) deger: arastirma kaydi icin
  float val = filterSample(rawG);         // HX711 ~10 ornek/sn
  if (calibrated) {
    Serial.print("AgirlikHam: ");
    Serial.print(rawG, 2);
    Serial.println(" g");
    if (fabsf(val) < ZERO_SNAP) val = 0.0f;
    Serial.print("Agirlik: ");
    Serial.print(val, 1);
    Serial.println(" g");
  } else {
    Serial.print("Ham deger: ");
    Serial.println(val, 0);
  }
}
