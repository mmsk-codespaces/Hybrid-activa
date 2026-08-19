/**
 * esp32s3_dashboard.ino
 * ESP32-S3 — ReynLab Hybrid Activa WiFi Dashboard
 * ReynLab Technologies India Pvt. Ltd.
 *
 * Receives JSON from Arduino Mega 2560 via Serial2 (UART)
 * Hosts WiFi Access Point + HTTP dashboard on 192.168.4.1
 *
 * WIRING (3 wires to Mega):
 *   GPIO13 D7 (RX) ← [2.2kΩ to GND] ← [1kΩ] ← Mega TX3 Pin14   (5V→3.3V divider)
 *   GPIO14 D6 (TX) ──────────────────────────► Mega RX3 Pin15     (3.3V direct, no divider)
 *   GND            ──────────────────────────► Mega GND           (common ground)
 *   NOTE: GPIO16/17 are used by onboard PSRAM on ESP32-UE(N16R2) — never use them
 *
 * LIBRARIES REQUIRED (install via Arduino Library Manager):
 *   - ArduinoJson  by Benoit Blanchon  (v6.x)
 *
 * BOARD: ESP32S3 Dev Module (Arduino ESP32 core v2.x or v3.x)
 * Connect phone/laptop to WiFi: ReynLab-Hybrid / reynlab123
 * Open browser: http://192.168.4.1
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// ================================================================
// WiFi ACCESS POINT
// ================================================================
const char* AP_SSID     = "ReynLab-Hybrid";
const char* AP_PASSWORD = "reynlab123";

// ================================================================
// UART — Mega Serial3 connection
// ================================================================
// GPIO16/GPIO17 used by onboard PSRAM — avoid them
#define MEGA_RX_PIN   13      // D7 ← Mega TX3 Pin14 via voltage divider
#define MEGA_TX_PIN   14      // D6 → Mega RX3 Pin15 direct
#define MEGA_BAUD     115200

// ================================================================
// WEB SERVER
// ================================================================
WebServer server(80);

// ================================================================
// LIVE DATA from Mega
// ================================================================
struct {
  float  hubRPM    = 0.0f;
  float  icRPM     = 0.0f;
  float  throttle  = 0.0f;
  float  batV      = 0.0f;
  float  batI      = 0.0f;
  float  soc       = 0.0f;
  float  batW      = 0.0f;
  int    bmsValid  = 0;
  char          state[16]     = "EV_ONLY";
  float         evToIcRpm    = 2000.0f;
  unsigned long evToIcHoldMs = 5000UL;
  unsigned long icToEvHoldMs = 5000UL;
  float         icLowRpm     = 1000.0f;
  unsigned long lastUpdateMs = 0;
} live;

String serialBuf = "";

// ================================================================
// DASHBOARD HTML (served at /)
// ================================================================
const char HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ReynLab Hybrid</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#070B0F;color:#e0e0e0;font-family:'Segoe UI',sans-serif;padding:12px;min-height:100vh}
h1{text-align:center;color:#00D4FF;font-size:1rem;letter-spacing:2px;margin-bottom:10px;text-transform:uppercase}
.state{text-align:center;padding:12px;border-radius:10px;font-size:1.1rem;font-weight:700;letter-spacing:3px;margin-bottom:10px;transition:all .4s}
.EV_ONLY    {background:#0a2a1a;border:2px solid #00E87A;color:#00E87A}
.TRANS_TO_IC{background:#2a1f0a;border:2px solid #FFA040;color:#FFA040}
.IC_ONLY    {background:#2a1200;border:2px solid #FF6020;color:#FF6020}
.TRANS_TO_EV{background:#0a1a2a;border:2px solid #00D4FF;color:#00D4FF}
.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:8px;margin-bottom:8px}
.grid4{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-bottom:8px}
.card{background:#0D1520;border:1px solid #1a2a3a;border-radius:10px;padding:12px;text-align:center;transition:border .4s}
.card.active-ic{border:1px solid #FF6020;box-shadow:0 0 10px #FF602033}
.card.active-ev{border:1px solid #00E87A;box-shadow:0 0 10px #00E87A33}
.lbl{font-size:.68rem;color:#7a9ab0;text-transform:uppercase;letter-spacing:1px;margin-bottom:4px}
.val{font-size:2rem;font-weight:700;line-height:1}
.unit{font-size:.7rem;color:#7a9ab0;margin-top:3px}
.badge{display:inline-block;font-size:.6rem;font-weight:700;padding:2px 7px;border-radius:20px;margin-top:5px;letter-spacing:1px}
.badge-ic{background:#2a1200;color:#FF6020;border:1px solid #FF6020}
.badge-ev{background:#0a2a1a;color:#00E87A;border:1px solid #00E87A}
.ev{color:#00E87A}.ic{color:#FFA040}.bat{color:#00D4FF}.thr{color:#8B5CF6}.pwr{color:#FF4455}.tune{color:#F0C040}
.bar-bg{background:#1a2a3a;border-radius:4px;height:8px;margin-top:8px;overflow:hidden}
.bar{height:100%;border-radius:4px;transition:width .5s}
.bar-thr{background:#8B5CF6}
.bar-soc{background:#00D4FF}
.tune-row{display:flex;align-items:center;gap:6px;margin-top:10px}
.tune-row input{flex:1;background:#111a27;border:1px solid #F0C040;border-radius:6px;color:#F0C040;font-size:1.1rem;font-weight:700;text-align:center;padding:6px;width:0}
.tune-row button{background:#F0C040;border:none;border-radius:6px;color:#000;font-weight:700;font-size:.75rem;padding:7px 12px;cursor:pointer;letter-spacing:1px}
.tune-row button:active{background:#c09000}
.tune-fb{font-size:.65rem;margin-top:5px;min-height:14px}
.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:8px}
@media(max-width:500px){.grid3{grid-template-columns:1fr}}
.footer{text-align:center;font-size:.68rem;color:#3a5a7a;margin-top:10px}
.dot{display:inline-block;width:7px;height:7px;border-radius:50%;background:#00E87A;margin-right:5px;animation:blink 1s infinite}
.stale .dot{background:#FF4455;animation:none}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.3}}
@media(max-width:500px){.grid4{grid-template-columns:repeat(2,1fr)}}
</style>
</head>
<body>

<h1>ReynLab — Hybrid Activa 4G</h1>

<div id="state" class="state EV_ONLY">EV_ONLY</div>

<div class="grid4">
  <div class="card" id="hubCard">
    <div class="lbl">Hub Motor</div>
    <div class="val ev" id="hubRPM">0</div>
    <div class="unit">RPM</div>
    <div class="badge" id="hubBadge" style="display:none"></div>
  </div>
  <div class="card" id="icCard">
    <div class="lbl">IC Engine</div>
    <div class="val ic" id="icRPM">0</div>
    <div class="unit">RPM</div>
    <div class="badge" id="icBadge" style="display:none"></div>
  </div>
  <div class="card">
    <div class="lbl">Battery SoC</div>
    <div class="val bat" id="soc">--</div>
    <div class="unit">%</div>
    <div class="bar-bg"><div class="bar bar-soc" id="socbar" style="width:0%"></div></div>
  </div>
  <div class="card">
    <div class="lbl">Power</div>
    <div class="val pwr" id="batW">--</div>
    <div class="unit">W</div>
  </div>
</div>

<div class="grid">
  <div class="card">
    <div class="lbl">Battery Voltage</div>
    <div class="val bat" id="batV">--</div>
    <div class="unit">V</div>
  </div>
  <div class="card">
    <div class="lbl">Battery Current</div>
    <div class="val bat" id="batI">--</div>
    <div class="unit">A</div>
  </div>
</div>

<div class="card" style="margin-bottom:8px">
  <div class="lbl">EV → IC Activation RPM</div>
  <div class="val tune" id="evToIcDisp">2000</div>
  <div class="unit">RPM (current on Mega)</div>
  <div class="tune-row">
    <input type="number" id="rpmInput" min="500" max="5000" step="100" value="2000">
    <button onclick="setRPM()">SET</button>
  </div>
  <div class="tune-fb" id="rpmFb"></div>
</div>

<div class="grid3">
  <div class="card">
    <div class="lbl">EV&#8594;IC Hold</div>
    <div class="val tune" id="evHoldDisp">5</div>
    <div class="unit">sec (on Mega)</div>
    <div class="tune-row">
      <input type="number" id="evHoldInput" min="1" max="30" step="1" value="5">
      <button onclick="setEvHold()">SET</button>
    </div>
    <div class="tune-fb" id="evHoldFb"></div>
  </div>
  <div class="card">
    <div class="lbl">IC&#8594;EV Hold</div>
    <div class="val tune" id="icHoldDisp">5</div>
    <div class="unit">sec (on Mega)</div>
    <div class="tune-row">
      <input type="number" id="icHoldInput" min="1" max="30" step="1" value="5">
      <button onclick="setIcHold()">SET</button>
    </div>
    <div class="tune-fb" id="icHoldFb"></div>
  </div>
  <div class="card">
    <div class="lbl">IC Cutoff RPM</div>
    <div class="val tune" id="icThrDisp">1000</div>
    <div class="unit">RPM (on Mega)</div>
    <div class="tune-row">
      <input type="number" id="icThrInput" min="200" max="3000" step="100" value="1000">
      <button onclick="setIcThr()">SET</button>
    </div>
    <div class="tune-fb" id="icThrFb"></div>
  </div>
</div>

<div class="card" style="margin-bottom:8px">
  <div class="lbl">Throttle Position</div>
  <div class="val thr" id="thr">0</div>
  <div class="unit">%</div>
  <div class="bar-bg"><div class="bar bar-thr" id="thrbar" style="width:0%"></div></div>
</div>

<div class="footer" id="status">
  <span class="dot"></span>Connecting to Mega...
</div>

<script>
function set(id,v){document.getElementById(id).textContent=v}

async function sendCmd(url,fb,okMsg){
  try{
    const r=await fetch(url,{cache:'no-store'});
    if(r.ok){fb.style.color='#00E87A';fb.textContent=okMsg;}
    else{fb.style.color='#FF4455';fb.textContent='Error';}
  }catch(e){fb.style.color='#FF4455';fb.textContent='No response';}
}
async function setRPM(){
  const v=parseInt(document.getElementById('rpmInput').value);
  const fb=document.getElementById('rpmFb');
  if(isNaN(v)||v<500||v>5000){fb.style.color='#FF4455';fb.textContent='Enter 500-5000';return;}
  sendCmd('/setrpm?val='+v, fb, 'Set: '+v+' RPM');
}
async function setEvHold(){
  const v=parseInt(document.getElementById('evHoldInput').value);
  const fb=document.getElementById('evHoldFb');
  if(isNaN(v)||v<1||v>30){fb.style.color='#FF4455';fb.textContent='Enter 1-30';return;}
  sendCmd('/setevhold?val='+v, fb, 'Set: '+v+' s');
}
async function setIcHold(){
  const v=parseInt(document.getElementById('icHoldInput').value);
  const fb=document.getElementById('icHoldFb');
  if(isNaN(v)||v<1||v>30){fb.style.color='#FF4455';fb.textContent='Enter 1-30';return;}
  sendCmd('/setichold?val='+v, fb, 'Set: '+v+' s');
}
async function setIcThr(){
  const v=parseInt(document.getElementById('icThrInput').value);
  const fb=document.getElementById('icThrFb');
  if(isNaN(v)||v<200||v>3000){fb.style.color='#FF4455';fb.textContent='Enter 200-3000';return;}
  sendCmd('/seticthr?val='+v, fb, 'Set: '+v+' RPM');
}

async function poll(){
  try{
    const r=await fetch('/data',{cache:'no-store'});
    const d=await r.json();
    const icActive=(d.state==='IC_ONLY'||d.state==='TRANS_TO_IC');
    set('hubRPM', icActive ? '--' : Math.round(d.hubRPM));
    set('icRPM',  icActive ? Math.round(d.icRPM) : '--');
    set('thr', d.thr.toFixed(1));
    document.getElementById('thrbar').style.width=Math.min(d.thr,100)+'%';
    if(d.bms && !icActive){
      set('soc',  d.soc.toFixed(1));
      set('batV', d.batV.toFixed(1));
      set('batI', d.batI.toFixed(1));
      set('batW', Math.round(d.batW));
      document.getElementById('socbar').style.width=Math.min(d.soc,100)+'%';
    }else if(icActive){
      set('soc','--'); set('batV','--'); set('batI','--'); set('batW','--');
      document.getElementById('socbar').style.width='0%';
    }
    const sb=document.getElementById('state');
    sb.textContent=d.state; sb.className='state '+d.state;
    document.getElementById('hubCard').className='card'+(icActive?' active-ic':' active-ev');
    document.getElementById('icCard').className='card'+(icActive?' active-ic':'');
    const hubBadge=document.getElementById('hubBadge');
    const icBadge=document.getElementById('icBadge');
    if(icActive){
      hubBadge.textContent='IC MODE';hubBadge.className='badge badge-ic';hubBadge.style.display='inline-block';
      icBadge.textContent='RUNNING';icBadge.className='badge badge-ic';icBadge.style.display='inline-block';
    }else{
      hubBadge.textContent='EV MODE';hubBadge.className='badge badge-ev';hubBadge.style.display='inline-block';
      icBadge.style.display='none';
    }
    set('evToIcDisp',  Math.round(d.evToIcRpm));
    set('evHoldDisp',  Math.round(d.evToIcHoldMs/1000));
    set('icHoldDisp',  Math.round(d.icToEvHoldMs/1000));
    set('icThrDisp',   d.icLowRpm.toFixed(0));
    const st=document.getElementById('status');
    st.innerHTML='<span class="dot"></span>Live - '+new Date().toLocaleTimeString();
    st.className='footer';
  }catch(e){
    const st=document.getElementById('status');
    st.innerHTML='<span class="dot"></span>No data from Mega - check wiring';
    st.className='footer stale';
  }
}
poll();
setInterval(poll,1000);
</script>
</body>
</html>
)rawliteral";

// ================================================================
// JSON PARSER — line from Mega Serial3
// ================================================================
void parseLine(const String& line) {
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, line) != DeserializationError::Ok) return;

  live.hubRPM       = doc["hubRPM"]        | 0.0f;
  live.icRPM        = doc["icRPM"]         | 0.0f;
  live.throttle     = doc["thr"]           | 0.0f;
  live.batV         = doc["batV"]          | 0.0f;
  live.batI         = doc["batI"]          | 0.0f;
  live.soc          = doc["soc"]           | 0.0f;
  live.batW         = doc["batW"]          | 0.0f;
  live.bmsValid     = doc["bms"]           | 0;
  live.evToIcRpm    = doc["evToIcRpm"]     | live.evToIcRpm;
  live.evToIcHoldMs = doc["evToIcHoldMs"]  | live.evToIcHoldMs;
  live.icToEvHoldMs = doc["icToEvHoldMs"]  | live.icToEvHoldMs;
  live.icLowRpm     = doc["icLowRpm"]       | live.icLowRpm;
  strlcpy(live.state, doc["state"] | "EV_ONLY", sizeof(live.state));
  live.lastUpdateMs = millis();
}

// ================================================================
// HTTP HANDLERS
// ================================================================
void handleRoot() { server.send_P(200, "text/html", HTML); }

void handleData() {
  char json[420];
  snprintf(json, sizeof(json),
    "{\"hubRPM\":%.1f,\"icRPM\":%.1f,\"thr\":%.1f,"
    "\"batV\":%.1f,\"batI\":%.1f,\"soc\":%.1f,\"batW\":%.1f,"
    "\"bms\":%d,\"state\":\"%s\","
    "\"evToIcRpm\":%.0f,\"evToIcHoldMs\":%lu,\"icToEvHoldMs\":%lu,\"icLowRpm\":%.0f}",
    live.hubRPM, live.icRPM, live.throttle,
    live.batV, live.batI, live.soc, live.batW,
    live.bmsValid, live.state,
    live.evToIcRpm, live.evToIcHoldMs, live.icToEvHoldMs, live.icLowRpm
  );
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void handleSetRpm() {
  if (!server.hasArg("val")) { server.send(400, "text/plain", "missing val"); return; }
  float v = server.arg("val").toFloat();
  if (v < 500.0f || v > 5000.0f) { server.send(400, "text/plain", "out of range"); return; }
  live.evToIcRpm = v;
  char cmd[24]; snprintf(cmd, sizeof(cmd), "SET_RPM:%.0f\n", v);
  Serial2.print(cmd);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "ok");
}

void handleSetEvHold() {
  if (!server.hasArg("val")) { server.send(400, "text/plain", "missing val"); return; }
  int s = server.arg("val").toInt();
  if (s < 1 || s > 30) { server.send(400, "text/plain", "out of range"); return; }
  live.evToIcHoldMs = (unsigned long)s * 1000UL;
  char cmd[28]; snprintf(cmd, sizeof(cmd), "SET_EV_HOLD:%lu\n", live.evToIcHoldMs);
  Serial2.print(cmd);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "ok");
}

void handleSetIcHold() {
  if (!server.hasArg("val")) { server.send(400, "text/plain", "missing val"); return; }
  int s = server.arg("val").toInt();
  if (s < 1 || s > 30) { server.send(400, "text/plain", "out of range"); return; }
  live.icToEvHoldMs = (unsigned long)s * 1000UL;
  char cmd[28]; snprintf(cmd, sizeof(cmd), "SET_IC_HOLD:%lu\n", live.icToEvHoldMs);
  Serial2.print(cmd);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "ok");
}

void handleSetIcThr() {
  if (!server.hasArg("val")) { server.send(400, "text/plain", "missing val"); return; }
  float v = server.arg("val").toFloat();
  if (v < 200.0f || v > 3000.0f) { server.send(400, "text/plain", "out of range"); return; }
  live.icLowRpm = v;
  char cmd[24]; snprintf(cmd, sizeof(cmd), "SET_IC_RPM:%.0f\n", v);
  Serial2.print(cmd);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "ok");
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(MEGA_BAUD, SERIAL_8N1, MEGA_RX_PIN, MEGA_TX_PIN);

  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("WiFi AP: "); Serial.println(AP_SSID);
  Serial.print("IP:      "); Serial.println(WiFi.softAPIP());

  server.on("/",         handleRoot);
  server.on("/data",     handleData);
  server.on("/setrpm",   handleSetRpm);
  server.on("/setevhold", handleSetEvHold);
  server.on("/setichold", handleSetIcHold);
  server.on("/seticthr",  handleSetIcThr);
  server.begin();
  Serial.println("HTTP server started — open http://192.168.4.1");
}

// ================================================================
// LOOP
// ================================================================
void loop() {
  server.handleClient();

  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      serialBuf.trim();
      if (serialBuf.startsWith("{")) parseLine(serialBuf);
      serialBuf = "";
    } else if (c != '\r') {
      if (serialBuf.length() < 240) serialBuf += c;
    }
  }
}
