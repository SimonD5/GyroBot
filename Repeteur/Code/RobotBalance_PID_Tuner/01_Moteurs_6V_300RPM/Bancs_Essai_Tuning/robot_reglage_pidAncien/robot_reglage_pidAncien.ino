// Réglage PID par WiFi - ESP32-C6 + MPU6050 + TB6612FNG
// Core ESP32 Arduino 3.x requis (ledcAttach / ledcWrite)
//
//   1. Allume le robot posé immobile (calibration du gyro ~2 s)
//   2. Connecte le téléphone au WiFi "RobotBalance" (ouvert)
//   3. Ouvre http://192.168.4.1
//   4. Règle Kp / Kd / Ki avec les boutons, "Armer", teste, recommence
//
// Série (115200) : p15 i0 d1 s7.5 m20 g(inverse gyro) v40(test roues) r ?

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>

// ---------------- Broches ----------------
#define SDA_PIN 2
#define SCL_PIN 3

#define PWMA 11
#define AIN1 7
#define AIN2 10
#define PWMB 23
#define BIN1 21
#define BIN2 20
#define STBY 22

// ---------------- WiFi ----------------
const char* AP_SSID = "RobotBalance";
const char* AP_PASS = "12345678";
#define AP_OUVERT 1                 // 1 = sans mot de passe
#define TELEMETRIE_SERIE 0          // 1 = affiche l'angle en série (USB branché uniquement)
WebServer server(80);

// ---------------- Réglages matériels ----------------
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
const int PWM_MAX  = 200;           // limite PWM (lipo 7,2V -> moteurs 6V)

// Moteur B câblé à l'envers (d'après test_moteurs.ino)
const int SIGN_MOTEUR_A = 1;
const int SIGN_MOTEUR_B = -1;

// ---------------- Paramètres PID (modifiables) ----------------
float Kp = 15.0;
float Ki = 0.0;
float Kd = 1.0;
float consigne = 7.5;               // angle d'équilibre mesuré (degrés)
int   pwmMin = 20;                  // PWM minimal pour vaincre la zone morte

// ---------------- Sécurité ----------------
const float ANGLE_CHUTE   = 18.0;   // angle ABSOLU (+/-) où le robot touche le sol : moteurs coupés
const float ANGLE_REPRISE = 6.0;    // écart sous lequel on repart
const float I_MAX         = 80.0;   // anti-windup (en unités de PWM)
const float ZONE_RAMPE    = 30.0;   // zone morte progressive : offset 0 -> pwmMin pour |sortie| 0 -> 30
const float ALPHA_D       = 0.4;    // filtre passe-bas sur le gyro du terme D (~20 Hz)

// ---------------- Filtre complémentaire ----------------
float alphaF = 0.98;                // poids du gyro dans le filtre (réglable par la page)
const unsigned long PERIODE_US = 5000;  // 200 Hz

// ---------------- Variables ----------------
const uint8_t MPU_ADDR = 0x68;
float gyroBias = 0;
float gyroSign = -1.0;              // -1 : réglé pour ton montage (IMU debout), 'g' pour inverser
float rateF = 0;                    // gyro filtré pour le terme D
unsigned long dtMaxUs = 0, dtMaxAff = 0, tDtReset = 0;
float angleAcc = 0;                 // angle venant de l'accéléromètre seul (diagnostic)
float angle = 0;
float rate = 0;
float integrale = 0;
float sortie = 0;
bool armed = false;                 // armé depuis la page web
bool actif = false;                 // le PID tourne
unsigned long tPrec = 0;
unsigned long tAffichage = 0;

// Enregistrement du dernier essai (200 Hz, figé à la chute)
const int LOG_N = 500;
int16_t logA[LOG_N], logC[LOG_N], logR[LOG_N], logS[LOG_N];
int  logIdx = 0, logCount = 0;
bool logFige = true;

// Suivi des essais
unsigned long tDebutActif = 0;
float tenue = 0;                    // durée (s) de l'équilibre en cours
float tenueMax = 0;                 // meilleure durée depuis le dernier réglage
int   derniereChute = 0;           // +1 = vers l'avant, -1 = vers l'arrière, 0 = aucune
float dureeChute = 0;               // durée (s) avant cette chute
int   testPwm = 0;                  // test des roues
unsigned long testFin = 0;

// ---------------- MPU6050 ----------------
void mpuEcrire(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// angle : 0° à la verticale du capteur, positif vers l'avant
// gyroRate : positif quand l'angle augmente
bool mpuLire(float &accAngle, float &gyroRate) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // température ignorée
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();
  (void)ax; (void)gy; (void)gz;

  accAngle = atan2((float)az, (float)ay) * RAD_TO_DEG;  // IMU montée "debout"
  gyroRate = gyroSign * gx / 65.5;                      // ±500 °/s => 65.5 LSB/(°/s)
  return true;
}

void mpuInit() {
  mpuEcrire(0x6B, 0x00);  // sortie de veille
  mpuEcrire(0x1A, 0x03);  // filtre passe-bas ~44 Hz
  mpuEcrire(0x1B, 0x08);  // gyro ±500 °/s
  mpuEcrire(0x1C, 0x00);  // accéléro ±2 g
}

void calibrationGyro() {
  Serial.println("Calibration gyro : ne pas bouger le robot...");
  float somme = 0;
  int n = 0;
  float a, g;
  for (int i = 0; i < 500; i++) {
    if (mpuLire(a, g)) {
      somme += g;
      n++;
    }
    delay(3);
  }
  gyroBias = (n > 0) ? somme / n : 0;
  Serial.print("Biais gyro = ");
  Serial.println(gyroBias);
}

// ---------------- Moteurs ----------------
// vitesse > 0 : in1 HIGH / in2 LOW
void moteur(int pwmPin, int in1, int in2, int vitesse) {
  vitesse = constrain(vitesse, -PWM_MAX, PWM_MAX);
  if (vitesse > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (vitesse < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  ledcWrite(pwmPin, abs(vitesse));
}

void commandeMoteurs(int v) {
  moteur(PWMA, AIN1, AIN2, SIGN_MOTEUR_A * v);
  moteur(PWMB, BIN1, BIN2, SIGN_MOTEUR_B * v);
}

// ---------------- Paramètres (série + web) ----------------
void razEssai() {            // nouveau réglage => nouvel essai
  integrale = 0;
  tenueMax = 0;
  derniereChute = 0;
  dureeChute = 0;
}

void appliquerParam(char c, float v) {
  switch (c) {
    case 'p': Kp = max(v, 0.0f); razEssai(); break;
    case 'i': Ki = max(v, 0.0f); razEssai(); break;
    case 'd': Kd = max(v, 0.0f); razEssai(); break;
    case 's': consigne = v;      razEssai(); break;
    case 'm': pwmMin = max((int)v, 0); razEssai(); break;
    case 'r':                    // valeurs de départ
      Kp = 15.0; Ki = 0.0; Kd = 1.0;
      razEssai();
      break;
    case 'g':                    // inverser le sens du gyro
      gyroSign = -gyroSign;
      gyroBias = -gyroBias;      // le biais mesuré change de signe aussi
      razEssai();
      break;
    case 'v':                    // test roues : PWM fixe 0,8 s (robot désarmé)
      testPwm = constrain((int)v, -PWM_MAX, PWM_MAX);
      testFin = millis() + 800;
      break;
    case 'z':                    // recalibrer le gyro (robot immobile)
      armed = false;
      commandeMoteurs(0);
      calibrationGyro();
      razEssai();
      break;
    case 'f': alphaF = constrain(v, 0.90f, 0.9995f); razEssai(); break;
    case 'a':                    // armer / désarmer
      armed = (v > 0.5);
      integrale = 0;
      break;
    case '?': break;
  }
}

void afficheParametres() {
  Serial.printf("Kp=%.2f Ki=%.3f Kd=%.3f consigne=%.2f pwmMin=%d armed=%d gyroSign=%.0f\n",
                Kp, Ki, Kd, consigne, pwmMin, armed, gyroSign);
}

void lireSerie() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;
  appliquerParam(s.charAt(0), s.substring(1).toFloat());
  afficheParametres();
}

// ---------------- Page web ----------------
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Reglage PID</title>
<style>
body{font-family:sans-serif;margin:12px;max-width:480px}
button{padding:14px 0;font-size:16px;margin:4px 0;width:100%}
.b{border:1px solid #bbb;border-radius:8px;padding:8px;margin:10px 0}
.t{display:flex;justify-content:space-between;align-items:center;font-size:18px;font-weight:bold}
.t input{width:110px;font-size:20px;padding:6px;text-align:right}
.g{display:flex;gap:6px;margin-top:6px}
.g button{margin:0;padding:14px 0}
#st{font-size:18px;font-weight:bold;margin:8px 0}
#ch{margin:6px 0;color:#b00}
</style></head><body>
<h3>Réglage PID</h3>
<div id="st">...</div>
<div>angle <b id="an">0</b>° &nbsp; accéléro <b id="aa">0</b>° &nbsp; gyro <b id="gr">0</b>°/s</div>
<div>sortie <b id="so">0</b> &nbsp; boucle max <b id="dm">0</b> ms (doit rester &lt; 10)</div>
<div>tenue <b id="te">0</b> s &nbsp; meilleure <b id="be">0</b> s</div>
<div id="ch">Dernière chute : -</div>
<button id="arm" onclick="toggleArm()">Armer</button>
<div id="ctl"></div>
<button onclick="send('r',0).then(()=>setTimeout(load,200))">Valeurs de départ (Kp 15, Ki 0, Kd 1)</button>
<h4>Diagnostic</h4>
<button onclick="window.open('/log')">Voir le dernier essai (enregistrement)</button>
<button onclick="if(confirm('Robot IMMOBILE et pose ? (2 s)'))send('z',0)">Recalibrer le gyro (robot immobile, 2 s)</button>
<button onclick="send('g',0)">Inverser le sens du gyro</button>
<div>Test roues (robot désarmé, 0,8 s, PWM fixe) :</div>
<div class="g" id="tw"></div>
<script>
const R=[['p','Kp',[-5,-1,1,5]],['d','Kd',[-0.2,-0.05,0.05,0.2]],['i','Ki',[-2,-0.5,0.5,2]],
['s','Consigne (°)',[-0.5,-0.1,0.1,0.5]],['m','PWM min',[-5,-1,1,5]],
['f','Filtre alpha',[-0.01,-0.002,0.002,0.01]]];
let armed=0;const V={};
const $=id=>document.getElementById(id);
function send(k,v){return fetch('/set?k='+k+'&v='+v)}
function chg(k,s){
 let v=Math.round((V[k]+s)*1000)/1000;
 if(k!='s'&&v<0)v=0;
 V[k]=v;$('v_'+k).value=v;send(k,v);
}
function build(){
 R.forEach(([k,n,st])=>{
  const d=document.createElement('div');d.className='b';
  const t=document.createElement('div');t.className='t';
  t.innerHTML='<span>'+n+'</span><input type="number" step="any" id="v_'+k+'">';
  const g=document.createElement('div');g.className='g';
  st.forEach(s=>{
   const b=document.createElement('button');
   b.textContent=(s>0?'+':'')+s;
   b.onclick=()=>chg(k,s);
   g.appendChild(b);
  });
  d.appendChild(t);d.appendChild(g);$('ctl').appendChild(d);
  $('v_'+k).onchange=()=>{V[k]=parseFloat($('v_'+k).value);send(k,V[k])};
 });
}
function load(){
 fetch('/params').then(r=>r.json()).then(j=>{
  R.forEach(([k])=>{V[k]=j[k];$('v_'+k).value=j[k]});
  armed=j.arm;upd();
 });
}
function upd(){$('arm').textContent=armed?'DESARMER (stop moteurs)':'Armer'}
function toggleArm(){armed=armed?0:1;send('a',armed).then(upd)}
build();load();
[20,30,40,50,60,80].forEach(v=>{
 const b=document.createElement('button');b.textContent=v;
 b.onclick=()=>send('v',v);$('tw').appendChild(b);
});
setInterval(()=>{
 fetch('/data').then(r=>r.json()).then(j=>{
  $('an').textContent=j.a;
  $('aa').textContent=j.aa;
  $('gr').textContent=j.r;
  $('dm').textContent=j.dm;
  $('so').textContent=j.o;
  $('te').textContent=j.t;
  $('be').textContent=j.b;
  $('st').textContent=j.act?"ACTIF":(j.arm?"ARME - tiens-le pres de l'equilibre":"DESARME");
  $('ch').textContent=j.f==0?"Dernière chute : -":"Dernière chute : vers l'"+(j.f>0?"AVANT":"ARRIERE")+" apres "+j.df+" s";
  if(j.arm!=armed){armed=j.arm;upd()}
 }).catch(()=>{$('st').textContent="Connexion perdue"});
},300);
</script></body></html>
)rawliteral";

void hRoot() { server.send_P(200, "text/html", PAGE); }

void hSet() {
  if (server.hasArg("k") && server.hasArg("v")) {
    String k = server.arg("k");
    if (k.length() > 0) appliquerParam(k.charAt(0), server.arg("v").toFloat());
  }
  server.send(200, "text/plain", "ok");
}

void hParams() {
  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"p\":%.2f,\"i\":%.3f,\"d\":%.3f,\"s\":%.2f,\"m\":%d,\"f\":%.4f,\"arm\":%d}",
           Kp, Ki, Kd, consigne, pwmMin, alphaF, armed ? 1 : 0);
  server.send(200, "application/json", buf);
}

void hLog() {
  if (actif) {
    server.send(200, "text/plain", "Robot actif : attends la chute ou desarme, puis recharge.");
    return;
  }
  if (logCount == 0) {
    server.send(200, "text/plain", "Pas d'essai enregistre.");
    return;
  }
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/plain", "");
  server.sendContent("angle,acc,gyro,sortie (200 Hz, ordre chronologique, la fin = la chute)\n");
  int start = (logCount < LOG_N) ? 0 : logIdx;
  char line[48];
  String chunk;
  chunk.reserve(1200);
  for (int k = 0; k < logCount; k++) {
    int i = (start + k) % LOG_N;
    snprintf(line, sizeof(line), "%.2f,%.2f,%.1f,%d\n", logA[i] / 100.0, logC[i] / 100.0, logR[i] / 10.0, logS[i]);
    chunk += line;
    if (chunk.length() > 1000) {
      server.sendContent(chunk);
      chunk = "";
    }
  }
  if (chunk.length()) server.sendContent(chunk);
  server.sendContent("");
}

void hData() {
  char buf[260];
  snprintf(buf, sizeof(buf),
           "{\"a\":%.1f,\"aa\":%.1f,\"r\":%.0f,\"o\":%.0f,\"act\":%d,\"arm\":%d,\"t\":%.1f,\"b\":%.1f,\"f\":%d,\"df\":%.1f,\"dm\":%.1f}",
           angle, angleAcc, rate, sortie, actif ? 1 : 0, armed ? 1 : 0, tenue, tenueMax, derniereChute, dureeChute, dtMaxAff / 1000.0);
  server.send(200, "application/json", buf);
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);
  commandeMoteurs(0);
  digitalWrite(STBY, HIGH);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpuInit();
  delay(100);
  calibrationGyro();

  // Angle initial depuis l'accéléromètre
  float a, g;
  if (mpuLire(a, g)) angle = a;

  // Point d'accès WiFi
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_AP);
#if AP_OUVERT
  WiFi.softAP(AP_SSID);
#else
  WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 4);
#endif
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  WiFi.setSleep(false);
  server.on("/", hRoot);
  server.on("/set", hSet);
  server.on("/params", hParams);
  server.on("/data", hData);
  server.on("/log", hLog);
  server.begin();
  Serial.print("WiFi : ");
  Serial.print(AP_SSID);
  Serial.print("  ->  http://");
  Serial.println(WiFi.softAPIP());

  afficheParametres();
  tPrec = micros();
}

// ---------------- Boucle ----------------
void loop() {
  server.handleClient();
  lireSerie();

  unsigned long now = micros();
  if (now - tPrec < PERIODE_US) return;
  unsigned long dus = now - tPrec;
  float dt = dus / 1e6;
  tPrec = now;
  if (dus > dtMaxUs) dtMaxUs = dus;
  if (millis() - tDtReset > 2000) {
    tDtReset = millis();
    dtMaxAff = dtMaxUs;
    dtMaxUs = 0;
  }

  float accAngle, gyroRate;
  if (!mpuLire(accAngle, gyroRate)) {
    commandeMoteurs(0);  // perte de l'IMU : on coupe
    return;
  }
  rate = gyroRate - gyroBias;
  angleAcc = accAngle;
  rateF += ALPHA_D * (rate - rateF);

  // Filtre complémentaire
  angle = alphaF * (angle + rate * dt) + (1.0 - alphaF) * accAngle;

  float erreur = angle - consigne;

  // Test des roues (robot désarmé) : PWM fixe pendant 0,8 s
  if (!armed && (long)(testFin - millis()) > 0) {
    commandeMoteurs(testPwm);
    actif = false;
    return;
  }

  // Mémorise la chute (pas le désarmement) : limites mécaniques à +/-18° absolus
  if (actif && armed && fabs(angle) > ANGLE_CHUTE) {
    derniereChute = (angle > 0) ? 1 : -1;
    dureeChute = tenue;
  }

  // Sécurité : désarmé, chute ou reprise (la reprise reste autour de la consigne)
  if (!armed || fabs(angle) > ANGLE_CHUTE) {
    actif = false;
  } else if (!actif && fabs(erreur) < ANGLE_REPRISE) {
    actif = true;
    integrale = 0;
    tDebutActif = millis();
    logIdx = 0; logCount = 0; logFige = false;
  }

  if (!actif) {
    commandeMoteurs(0);
    integrale = 0;
    sortie = 0;
    tenue = 0;
    logFige = true;
  } else {
    tenue = (millis() - tDebutActif) / 1000.0;
    if (tenue > tenueMax) tenueMax = tenue;

    // PID : P sur l'erreur, I avec anti-windup, D sur la vitesse gyro
    integrale += Ki * erreur * dt;
    integrale = constrain(integrale, -I_MAX, I_MAX);

    sortie = Kp * erreur + integrale + Kd * rateF;
    sortie = constrain(sortie, -PWM_MAX, PWM_MAX);

    if (!logFige) {
      logA[logIdx] = (int16_t)(angle * 100.0);
      logC[logIdx] = (int16_t)(accAngle * 100.0);
      logR[logIdx] = (int16_t)(rate * 10.0);
      logS[logIdx] = (int16_t)sortie;
      logIdx = (logIdx + 1) % LOG_N;
      if (logCount < LOG_N) logCount++;
    }

    // Compensation de zone morte progressive : l'offset monte de 0 à pwmMin
    // quand |sortie| passe de 0 à ZONE_RAMPE (pas d'à-coup autour de zéro)
    float absS = fabs(sortie);
    float offset = pwmMin * min(absS / ZONE_RAMPE, 1.0f);
    int cmd = (int)constrain(absS + offset, 0.0f, (float)PWM_MAX);
    if (sortie < 0) cmd = -cmd;
    commandeMoteurs(cmd);
  }

#if TELEMETRIE_SERIE
  // Télémétrie série à 20 Hz (Serial Plotter)
  if (millis() - tAffichage > 50) {
    tAffichage = millis();
    Serial.printf("angle:%.2f,consigne:%.2f,sortie:%.1f\n", angle, consigne, sortie);
  }
#endif
}
