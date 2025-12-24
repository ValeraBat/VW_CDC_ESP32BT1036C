#include "bt_webui.h"
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ElegantOTA.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include "vw_cdc.h"      // Для cdc_pause
#include "bt1036_at.h"   // Для bt1036_pausePolling

// ========== Debug mode ==========
bool g_debugMode = false;

void btWebUI_setDebug(bool on) {
    g_debugMode = on;
    btWebUI_log(String("[SYS] Debug mode: ") + (on ? "ON" : "OFF"));
}

// ========== Параметры точки доступа (AP) ==========
static String apSsid = "VW-BT1036";
static String apPsk  = "12345678";
static const char* hostname = "vw-bt";

WebServer        webServer(80);
WebSocketsServer wsServer(81);
Preferences      prefs;

// ---------- Ring Buffer ----------
static const uint16_t LOG_CAPACITY = 128;
struct LogEntry { uint32_t id; String line; };
static LogEntry logBuf[LOG_CAPACITY];
static uint16_t logHead = 0; static uint16_t logCount = 0; static uint32_t logNextId = 1;

static void logAppend(const String &line) {
    LogEntry &e = logBuf[logHead]; e.id = logNextId++; e.line = line;
    logHead = (logHead + 1) % LOG_CAPACITY;
    if (logCount < LOG_CAPACITY) logCount++;
}

void btWebUI_log(const String &line, LogLevel level) {
    if ((level == LogLevel::DEBUG || level == LogLevel::VERBOSE) && !g_debugMode) {
        return;
    }
    // Serial.println(line); // Убрано для избежания помех
    if (level != LogLevel::VERBOSE) {
        logAppend(line);
    }
    wsServer.broadcastTXT(line.c_str());
}

void btWebUI_log(const String &line) {
    btWebUI_log(line, LogLevel::INFO);
}

void btWebUI_broadcastCdcRaw(const String &line) {
    wsServer.broadcastTXT(line.c_str());
}

static const char* stateToStr(BTConnState st) {
    switch (st) {
        case BTConnState::DISCONNECTED:   return "DISCONNECTED";
        case BTConnState::CONNECTING:     return "CONNECTING";
        case BTConnState::CONNECTED_IDLE: return "CONNECTED_IDLE";
        case BTConnState::PLAYING:        return "PLAYING";
        case BTConnState::PAUSED:         return "PAUSED";
    }
    return "UNKNOWN";
}

static void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_CONNECTED) {
        for (uint16_t i = 0; i < logCount; ++i) {
            uint16_t idx = (logHead + LOG_CAPACITY - logCount + i) % LOG_CAPACITY;
            wsServer.sendTXT(num, logBuf[idx].line.c_str());
        }
    }
}

// ======================= HTML PAGES =======================

// 1. MAIN PAGE
static const char MAIN_PAGE[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8"><title>VW BT1036</title>
<style>
body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:5px}
nav{margin-bottom:10px;padding:5px;background:#222;border-bottom:1px solid #444}
nav a{color:#8cf;margin-right:15px;text-decoration:none;font-weight:bold}
nav a:hover{text-decoration:underline}
nav a.active{color:#fff;border-bottom:2px solid #8cf}
section{margin-bottom:10px;padding:8px;border:1px solid #444;border-radius:4px;background:#1a1a1a}
button{margin:2px;padding:6px 12px;background:#333;color:#fff;border:1px solid #666;border-radius:3px;cursor:pointer}
button:active{background:#555}
.status-val{font-weight:bold;color:#fff}
label{display:inline-block;min-width:100px;color:#ccc;font-size:14px}
input[type=text],input[type=number]{width:120px;background:#222;color:#fff;border:1px solid #555;padding:2px}
summary{font-weight:bold;cursor:pointer;outline:none;color:#8cf;padding:5px 0}
summary:hover{color:#fff}
details{padding:5px}
small{color:#888}
</style></head>
<body>
<nav>
  <a href="/" class="active">Main</a>
  <a href="/bt">BT Debug</a>
  <a href="/cdc">CDC Debug</a>
  <a href="/logs">All Logs</a>
  <a href="/wifi">WiFi</a>
  <a href="/update" style="color:#fa0">OTA</a>
</nav>
<div id="ip_info" style="color:#aaa;font-size:0.8em;margin-bottom:5px;"></div>
<section>
  <h3 style="margin:0 0 10px 0">BT1036 Status</h3>
  <div style="display:flex;gap:20px;margin-bottom:10px;">
    <div>State: <span id="st_state" class="status-val">-</span></div>
    <div>Power: <span id="st_power" class="status-val">-</span></div>
  </div>
  <div>
    <button onclick="sendCmd('scan')">Scan</button>
    <button onclick="sendCmd('connect')">Connect Last</button>
    <button onclick="sendCmd('disconnect')">Disconnect</button>
  </div>
  <div style="margin-top:8px;">
    <button onclick="sendCmd('playpause')">Play/Pause</button>
    <button onclick="sendCmd('prev')">Prev</button>
    <button onclick="sendCmd('next')">Next</button>
  </div>
</section>
<section>
  <details><summary>Basic Config (Name/COD)</summary>
    <div style="padding-top:5px;">
      <div><label>NAME:</label><input id="name" type="text" value="VW_BT1036">
        <input id="nameSuffix" type="checkbox" checked><small>Suffix</small></div>
      <div><label>BLE NAME:</label><input id="lename" type="text" value="VW_BT1036">
        <input id="lenameSuffix" type="checkbox" checked><small>Suffix</small></div>
      <div><label>COD (hex):</label><input id="cod" type="text" value="240404"></div>
      <button onclick="sendBasic()" style="margin-top:5px;">Apply</button>
    </div>
  </details>
</section>
<section>
  <details><summary>Profiles & HFP</summary>
    <div style="padding-top:5px;">
      <div><label>PROFILE:</label><input id="profile" type="number" value="168"></div>
      <div><label>AUTOCONN:</label><input id="autoconn" type="number" value="168"></div>
      <div><label>HFPSR (Hz):</label><input id="hfpsr" type="number" value="16000"></div>
      <div><label>HFPCFG:</label>
        <input id="hfpBit0" type="checkbox" checked><small>Auto-reconn</small>
        <input id="hfpBit1" type="checkbox" checked><small>Echo cancel</small>
        <input id="hfpBit2" type="checkbox"><small>3-way</small>
      </div>
      <button onclick="sendProfile();sendHfp();" style="margin-top:5px;">Apply All</button>
    </div>
  </details>
</section>
<section>
  <details><summary>Audio Levels</summary>
    <div style="padding-top:5px;">
      <div><label>Mic Gain:</label><input id="micgain" type="number" value="8"></div>
      <div><label>A2DP Vol:</label><input id="a2dpvol" type="number" value="12"></div>
      <div><label>HFP Vol:</label><input id="hfpvol" type="number" value="12"></div>
      <button onclick="sendAudio()" style="margin-top:5px;">Apply</button>
    </div>
  </details>
</section>
<section>
  <details><summary>System</summary>
    <div style="padding-top:5px;">
      <button onclick="sendReboot('bt')">Reboot BT1036</button>
      <button onclick="sendReboot('esp')">Reboot ESP32</button>
      <button onclick="sendFactory()" style="color:#fa0;margin-left:10px;">Factory Setup</button>
    </div>
  </details>
</section>
<script>
function updateStatus(){fetch('/api/status').then(function(r){return r.json();}).then(function(st){
  document.getElementById('st_state').textContent=st.state;
  document.getElementById('st_power').textContent=st.devstat.powerOn?'ON':'OFF';
});}
setInterval(updateStatus,2000);updateStatus();
function sendCmd(a){fetch('/api/cmd?act='+a);}
function sendBasic(){
  var n=encodeURIComponent(document.getElementById('name').value);
  var ns=document.getElementById('nameSuffix').checked?1:0;
  var l=encodeURIComponent(document.getElementById('lename').value);
  var ls=document.getElementById('lenameSuffix').checked?1:0;
  var c=encodeURIComponent(document.getElementById('cod').value);
  fetch('/api/set_basic?name='+n+'&ns='+ns+'&lname='+l+'&ls='+ls+'&cod='+c);
}
function sendProfile(){
  var p=document.getElementById('profile').value,a=document.getElementById('autoconn').value;
  fetch('/api/set_profile?p='+p+'&a='+a);
}
function sendHfp(){
  var r=document.getElementById('hfpsr').value;
  var c=0;
  if(document.getElementById('hfpBit0').checked)c|=1;
  if(document.getElementById('hfpBit1').checked)c|=2;
  if(document.getElementById('hfpBit2').checked)c|=4;
  fetch('/api/set_hfp?rate='+r+'&cfg='+c);
}
function sendAudio(){
  var m=document.getElementById('micgain').value,a=document.getElementById('a2dpvol').value,h=document.getElementById('hfpvol').value;
  fetch('/api/audio?mg='+m+'&a2='+a+'&hf='+h+'&tx=10');
}
function sendReboot(t){fetch('/api/reboot?target='+t);}
function sendFactory(){fetch('/api/factory');}
</script>
</body></html>
)rawliteral";

// 2. WIFI PAGE
static const char WIFI_PAGE[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8"><title>WiFi Setup</title>
<style>
body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:5px}
nav{margin-bottom:10px;padding:5px;background:#222;border-bottom:1px solid #444}
nav a{color:#8cf;margin-right:15px;text-decoration:none;font-weight:bold}
nav a:hover{text-decoration:underline}
nav a.active{color:#fff;border-bottom:2px solid #8cf}
section{margin-bottom:10px;padding:8px;border:1px solid #444;border-radius:4px;background:#1a1a1a}
button{margin:2px;padding:6px 12px;background:#333;color:#fff;border:1px solid #666;border-radius:3px;cursor:pointer}
label{display:inline-block;min-width:100px;color:#ccc;font-size:14px}
input[type=text],input[type=password]{width:200px;background:#222;color:#fff;border:1px solid #555;padding:4px}
.net{padding:10px;border-bottom:1px solid #333;cursor:pointer}
.net:hover{background:#333}
</style></head>
<body>
<nav>
  <a href="/">Main</a>
  <a href="/bt">BT Debug</a>
  <a href="/cdc">CDC Debug</a>
  <a href="/logs">All Logs</a>
  <a href="/wifi" class="active">WiFi</a>
  <a href="/update" style="color:#fa0">OTA</a>
</nav>
<h2>WiFi Connection</h2>
<section>
  <label>SSID:</label><input id="ssid" type="text"><br>
  <label>Password:</label><input id="psk" type="password"><br>
  <button onclick="save()" style="background:#060;margin-top:10px;">Save & Connect</button>
  <div id="msg" style="color:#fa0;margin-top:5px;"></div>
</section>
<section>
  <h3>Scan Networks</h3>
  <button onclick="scan()">Scan</button>
  <div id="list" style="margin-top:10px;"></div>
</section>
<script>
function scan(){
  document.getElementById('list').innerHTML="Scanning...";
  fetch('/api/wifi/scan').then(function(r){return r.json();}).then(function(l){
    var d=document.getElementById('list');d.innerHTML="";
    if(!l.length)d.innerHTML="No networks.";
    l.forEach(function(n){
      var i=document.createElement('div');i.className='net';
      i.innerHTML='<b>'+n.ssid+'</b> <small>'+n.rssi+'dBm</small>';
      i.onclick=function(){document.getElementById('ssid').value=n.ssid;};
      d.appendChild(i);
    });
  });
}
function save(){
  var s=encodeURIComponent(document.getElementById('ssid').value);
  var p=encodeURIComponent(document.getElementById('psk').value);
  document.getElementById('msg').innerText="Saving...";
  fetch('/api/wifi/connect?ssid='+s+'&psk='+p).then(function(){
    document.getElementById('msg').innerText="Saved! ESP is connecting...";
  });
}
</script>
</body></html>
)rawliteral";

// 3. BT DEBUG PAGE
static const char BT_PAGE[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8"><title>BT Debug</title>
<style>
body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:5px}
nav{margin-bottom:10px;padding:5px;background:#222;border-bottom:1px solid #444}
nav a{color:#8cf;margin-right:15px;text-decoration:none;font-weight:bold}
nav a:hover{text-decoration:underline}
nav a.active{color:#fff;border-bottom:2px solid #8cf}
section{margin-bottom:10px;padding:8px;border:1px solid #444;border-radius:4px;background:#1a1a1a}
button{margin:2px;padding:6px 12px;background:#333;color:#fff;border:1px solid #666;border-radius:3px;cursor:pointer}
.status-val{font-weight:bold;color:#fff}
.log-box{background:#000;color:#0f0;font-family:monospace;overflow:auto;padding:4px;border:1px solid #333;font-size:12px}
.btn{font-size:12px;padding:2px 8px;background:#060;color:#fff;border:1px solid #666;cursor:pointer;margin-left:5px}
.btn:hover{background:#080}
.btn-dl{background:#036}
.btn-dl:hover{background:#048}
</style></head>
<body>
<nav>
  <a href="/">Main</a>
  <a href="/bt" class="active">BT Debug</a>
  <a href="/cdc">CDC Debug</a>
  <a href="/logs">All Logs</a>
  <a href="/wifi">WiFi</a>
  <a href="/update" style="color:#fa0">OTA</a>
</nav>
<h2>Bluetooth Debug</h2>
<section>
  <h3>Status</h3>
  <div style="display:flex;gap:30px;">
    <div>State: <span id="st_state" class="status-val">-</span></div>
    <div>Power: <span id="st_power" class="status-val">-</span></div>
  </div>
  <div style="margin-top:10px;">
    <div>Track: <span id="track_title" class="status-val">-</span></div>
    <div>Artist: <span id="track_artist" style="color:#aaa;">-</span></div>
    <div>Time: <span id="track_time" style="color:#aaa;">--:-- / --:--</span></div>
  </div>
</section>
<section>
  <h3>BT Log 
    <button class="btn" onclick="togglePause()" id="pauseBtn">Pause</button>
    <button class="btn btn-dl" onclick="downloadLog()">Download</button>
    <button class="btn" onclick="clr()">Clear</button>
  </h3>
  <div class="log-box" id="log_bt" style="height:50vh;"></div>
</section>
<section>
  <h3>Manual AT Command</h3>
  <input type="text" id="at_cmd" placeholder="e.g., AT+VER" style="width: 200px;">
  <button onclick="sendAt()" style="background:#036;">Send</button>
</section>
<section>
  <button onclick="toggleDebug()" id="debugBtn" style="background:#333;">Debug Mode: OFF</button>
</section>
<script>
var paused=false,debugMode=false;
var ws=new WebSocket('ws://'+location.hostname+':81/');
ws.onmessage=function(ev){
  var t=ev.data||"";
  if(t.indexOf("[BT]")==0||t.indexOf("[SYS]")==0){
    var d=document.createElement("div");d.textContent=t;
    var b=document.getElementById('log_bt');b.appendChild(d);
    if(!paused)b.scrollTop=99999;
  }
};
function sendAt(){
  var cmd=document.getElementById('at_cmd').value;
  if(cmd){fetch('/api/at_cmd?cmd='+encodeURIComponent(cmd));}
}
function clr(){document.getElementById('log_bt').innerHTML="";}
function togglePause(){
  paused=!paused;
  var btn=document.getElementById('pauseBtn');
  btn.textContent=paused?'Resume':'Pause';
  btn.style.background=paused?'#a00':'#060';
}
function toggleDebug(){
  fetch('/api/debug').then(function(r){return r.text();}).then(function(t){
    debugMode=(t==='ON');
    var btn=document.getElementById('debugBtn');
    btn.textContent='Debug Mode: '+t;
    btn.style.background=debugMode?'#060':'#333';
  });
}
function downloadLog(){
  var box=document.getElementById('log_bt');
  var lines=[];
  for(var i=0;i<box.children.length;i++)lines.push(box.children[i].textContent);
  var blob=new Blob([lines.join('\n')],{type:'text/plain'});
  var a=document.createElement('a');
  a.href=URL.createObjectURL(blob);
  a.download='bt_log.txt';
  a.click();
}
function updateStatus(){
  fetch('/api/status').then(function(r){return r.json();}).then(function(st){
    document.getElementById('st_state').textContent=st.state;
    document.getElementById('st_power').textContent=st.devstat.powerOn?'ON':'OFF';
  });
  fetch('/api/track').then(function(r){return r.json();}).then(function(t){
    document.getElementById('track_title').textContent=t.title||'-';
    document.getElementById('track_artist').textContent=t.artist||'-';
    var el=Math.floor(t.elapsed/60)+':'+String(t.elapsed%60).padStart(2,'0');
    var tot=Math.floor(t.total/60)+':'+String(t.total%60).padStart(2,'0');
    document.getElementById('track_time').textContent=el+' / '+tot;
  }).catch(function(){});
}
setInterval(updateStatus,2000);updateStatus();
fetch('/api/debug_status').then(function(r){return r.text();}).then(function(t){
  debugMode=(t==='ON');
  var btn=document.getElementById('debugBtn');
  btn.textContent='Debug Mode: '+t;
  btn.style.background=debugMode?'#060':'#333';
}).catch(function(){});
</script>
</body></html>
)rawliteral";

// 4. CDC DEBUG PAGE
static const char CDC_PAGE[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8"><title>CDC Debug</title>
<style>
body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:5px}
nav{margin-bottom:10px;padding:5px;background:#222;border-bottom:1px solid #444}
nav a{color:#8cf;margin-right:15px;text-decoration:none;font-weight:bold}
nav a:hover{text-decoration:underline}
nav a.active{color:#fff;border-bottom:2px solid #8cf}
section{margin-bottom:10px;padding:8px;border:1px solid #444;border-radius:4px;background:#1a1a1a}
button{margin:2px;padding:6px 12px;background:#333;color:#fff;border:1px solid #666;border-radius:3px;cursor:pointer}
.log-box{background:#000;color:#0f0;font-family:monospace;overflow:auto;padding:4px;border:1px solid #333;font-size:12px}
.btn{font-size:12px;padding:2px 8px;background:#060;color:#fff;border:1px solid #666;cursor:pointer;margin-left:5px}
.btn:hover{background:#080}
.btn-dl{background:#036}
.btn-dl:hover{background:#048}
.row{display:flex;gap:10px}.half{flex:1}
</style></head>
<body>
<nav>
  <a href="/">Main</a>
  <a href="/bt">BT Debug</a>
  <a href="/cdc" class="active">CDC Debug</a>
  <a href="/logs">All Logs</a>
  <a href="/wifi">WiFi</a>
  <a href="/update" style="color:#fa0">OTA</a>
</nav>
<h2>CDC Debug</h2>
<div class="row">
  <div class="half">
    <section>
      <h3>CDC Events
        <button class="btn" onclick="togglePauseEvt()" id="pauseEvt">Pause</button>
        <button class="btn btn-dl" onclick="downloadLog('log_evt','cdc_events')">Download</button>
      </h3>
      <div class="log-box" id="log_evt" style="height:45vh;"></div>
    </section>
  </div>
  <div class="half" id="raw_panel">
    <section>
      <h3>NEC Raw <small>(Debug Mode only)</small>
        <button class="btn" onclick="togglePauseNec()" id="pauseNec">Pause</button>
        <button class="btn btn-dl" onclick="downloadLog('log_nec','nec_raw')">Download</button>
      </h3>
      <div class="log-box" id="log_nec" style="height:45vh;"></div>
    </section>
  </div>
</div>
<section>
  <button onclick="clr()">Clear Both</button>
  <button onclick="toggleDebug()" id="debugBtn" style="background:#333;">Debug Mode: OFF</button>
</section>
<script>
var pausedEvt=false,pausedNec=false,debugMode=false;
var ws=new WebSocket('ws://'+location.hostname+':81/');
ws.onmessage=function(ev){
  var t=ev.data||"";
  if(t.indexOf("[CDC_NEC]")==0&&debugMode){
    var d=document.createElement("div");d.textContent=t;
    var b=document.getElementById('log_nec');b.appendChild(d);
    if(!pausedNec)b.scrollTop=99999;
  }else if(t.indexOf("[CDC]")==0||t.indexOf("[BTN]")==0){
    var d=document.createElement("div");d.textContent=t;
    var b=document.getElementById('log_evt');b.appendChild(d);
    if(!pausedEvt)b.scrollTop=99999;
  }
};
function clr(){document.getElementById('log_evt').innerHTML="";document.getElementById('log_nec').innerHTML="";}
function togglePauseEvt(){
  pausedEvt=!pausedEvt;
  var btn=document.getElementById('pauseEvt');
  btn.textContent=pausedEvt?'Resume':'Pause';
  btn.style.background=pausedEvt?'#a00':'#060';
}
function togglePauseNec(){
  pausedNec=!pausedNec;
  var btn=document.getElementById('pauseNec');
  btn.textContent=pausedNec?'Resume':'Pause';
  btn.style.background=pausedNec?'#a00':'#060';
}
function toggleDebug(){
  fetch('/api/debug').then(function(r){return r.text();}).then(function(t){
    debugMode=(t==='ON');
    updateDebugUI();
  });
}
function updateDebugUI(){
  var btn=document.getElementById('debugBtn');
  btn.textContent='Debug Mode: '+(debugMode?'ON':'OFF');
  btn.style.background=debugMode?'#060':'#333';
  document.getElementById('raw_panel').style.opacity=debugMode?'1':'0.4';
}
function downloadLog(id,name){
  var box=document.getElementById(id);
  var lines=[];
  for(var i=0;i<box.children.length;i++)lines.push(box.children[i].textContent);
  var blob=new Blob([lines.join('\n')],{type:'text/plain'});
  var a=document.createElement('a');
  a.href=URL.createObjectURL(blob);
  a.download=name+'.txt';
  a.click();
}
fetch('/api/debug_status').then(function(r){return r.text();}).then(function(t){
  debugMode=(t==='ON');
  updateDebugUI();
}).catch(function(){});
</script>
</body></html>
)rawliteral";

// 5. ALL LOGS PAGE
static const char LOGS_PAGE[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8"><title>All Logs</title>
<style>
body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:5px}
nav{margin-bottom:10px;padding:5px;background:#222;border-bottom:1px solid #444}
nav a{color:#8cf;margin-right:15px;text-decoration:none;font-weight:bold}
nav a:hover{text-decoration:underline}
nav a.active{color:#fff;border-bottom:2px solid #8cf}
section{margin-bottom:10px;padding:8px;border:1px solid #444;border-radius:4px;background:#1a1a1a}
button{margin:2px;padding:6px 12px;background:#333;color:#fff;border:1px solid #666;border-radius:3px;cursor:pointer}
.log-box{background:#000;color:#0f0;font-family:monospace;overflow:auto;padding:4px;border:1px solid #333;font-size:12px}
.btn{font-size:12px;padding:2px 8px;background:#060;color:#fff;border:1px solid #666;cursor:pointer;margin-left:5px}
.btn:hover{background:#080}
.btn-dl{background:#036}
.btn-dl:hover{background:#048}
</style></head>
<body>
<nav>
  <a href="/">Main</a>
  <a href="/bt">BT Debug</a>
  <a href="/cdc">CDC Debug</a>
  <a href="/logs" class="active">All Logs</a>
  <a href="/wifi">WiFi</a>
  <a href="/update" style="color:#fa0">OTA</a>
</nav>
<h2>All Logs</h2>
<section>
  <div style="margin-bottom:5px;">
    <button class="btn" onclick="togglePause()" id="pauseBtn">Pause</button>
    <button class="btn btn-dl" onclick="downloadLog()">Download</button>
    <button class="btn" onclick="clr()">Clear</button>
    <button onclick="toggleDebug()" id="debugBtn" style="background:#333;margin-left:20px;">Debug Mode: OFF</button>
  </div>
  <div class="log-box" id="log_all" style="height:70vh;"></div>
</section>
<script>
var paused=false,debugMode=false;
var ws=new WebSocket('ws://'+location.hostname+':81/');
ws.onmessage=function(ev){
  var t=ev.data||"";
  if(t.indexOf("SCOPE:")!=0){
    var d=document.createElement("div");
    d.textContent=t;
    if(t.indexOf("[BT]")==0)d.style.color='#0ff';
    else if(t.indexOf("[CDC]")==0||t.indexOf("[BTN]")==0)d.style.color='#0f0';
    else if(t.indexOf("[MAIN]")==0||t.indexOf("[SYS]")==0)d.style.color='#ff0';
    else if(t.indexOf("[CDC_NEC]")==0)d.style.color='#888';
    var b=document.getElementById('log_all');b.appendChild(d);
    if(!paused)b.scrollTop=99999;
  }
};
function clr(){document.getElementById('log_all').innerHTML="";}
function togglePause(){
  paused=!paused;
  var btn=document.getElementById('pauseBtn');
  btn.textContent=paused?'Resume':'Pause';
  btn.style.background=paused?'#a00':'#060';
}
function toggleDebug(){
  fetch('/api/debug').then(function(r){return r.text();}).then(function(t){
    debugMode=(t==='ON');
    var btn=document.getElementById('debugBtn');
    btn.textContent='Debug Mode: '+t;
    btn.style.background=debugMode?'#060':'#333';
  });
}
function downloadLog(){
  var box=document.getElementById('log_all');
  var lines=[];
  for(var i=0;i<box.children.length;i++)lines.push(box.children[i].textContent);
  var blob=new Blob([lines.join('\n')],{type:'text/plain'});
  var a=document.createElement('a');
  a.href=URL.createObjectURL(blob);
  a.download='all_logs.txt';
  a.click();
}
fetch('/api/debug_status').then(function(r){return r.text();}).then(function(t){
  debugMode=(t==='ON');
  var btn=document.getElementById('debugBtn');
  btn.textContent='Debug Mode: '+t;
  btn.style.background=debugMode?'#060':'#333';
}).catch(function(){});
</script>
</body></html>
)rawliteral";

// ======================= API HANDLERS =======================

static void handleRoot() { 
    String p = MAIN_PAGE;
    String ips = "AP: " + WiFi.softAPIP().toString();
    if (WiFi.status() == WL_CONNECTED) {
        ips += " | Home: " + WiFi.localIP().toString() + " (" + WiFi.SSID() + ")";
        ips += " | <a href='http://" + String(hostname) + ".local' style='color:#0f0'>http://" + String(hostname) + ".local</a>";
    }
    p.replace("<div id=\"ip_info\" style=\"color:#aaa;font-size:0.8em;margin-bottom:5px;\"></div>", 
              "<div id=\"ip_info\" style=\"color:#aaa;margin-bottom:5px;\">" + ips + "</div>");
    webServer.send(200, "text/html", p); 
}
static void handleWifiPage() { webServer.send_P(200, "text/html", WIFI_PAGE); }
static void handleBtPage() { webServer.send_P(200, "text/html", BT_PAGE); }
static void handleCdc() { webServer.send_P(200, "text/html", CDC_PAGE); }
static void handleLogs() { webServer.send_P(200, "text/html", LOGS_PAGE); }

static void handleStatus() {
    BTConnState st = bt1036_getState();
    BtDevStat   ds = bt1036_getDevStat();
    String json = "{";
    json += "\"state\":\"" + String(stateToStr(st)) + "\",";
    json += "\"devstat\":{\"powerOn\":" + String(ds.powerOn ? "true":"false") + "}";
    json += "}";
    webServer.send(200, "application/json", json);
}

static void handleSetBasic() {
    String name = webServer.arg("name");
    bool nsuf = webServer.arg("ns") == "1";
    String lname = webServer.arg("lname");
    bool lsuf = webServer.arg("ls") == "1";
    String cod = webServer.arg("cod");
    if (name.length()) bt1036_setName(name, nsuf);
    if (lname.length()) bt1036_setBLEName(lname, lsuf);
    if (cod.length()) bt1036_setCod(cod);
    webServer.send(200, "text/plain", "OK");
}

static void handleSetHfp() {
    uint32_t rate = webServer.arg("rate").toInt();
    uint8_t cfg = 0;
    if (webServer.arg("cfg") != "") cfg = (uint8_t)webServer.arg("cfg").toInt(); 
    bt1036_setHfpSampleRate(rate);
    bt1036_setHfpConfig(cfg);
    webServer.send(200, "text/plain", "OK");
}

static void handleProfile() {
    bt1036_setProfile(webServer.arg("p").toInt());
    bt1036_setAutoconn(webServer.arg("a").toInt());
    webServer.send(200, "text/plain", "OK");
}

static void handleAudio() {
    bt1036_setMicGain(webServer.arg("mg").toInt());
    bt1036_setSpkVol(webServer.arg("a2").toInt(), webServer.arg("hf").toInt());
    bt1036_setTxPower(webServer.arg("tx").toInt());
    webServer.send(200, "text/plain", "OK");
}

static void handleCmd() {
    String act = webServer.arg("act");
    if(act=="playpause") bt1036_playPause();
    else if(act=="next") bt1036_nextTrack();
    else if(act=="prev") bt1036_prevTrack();
    else if(act=="connect") bt1036_connectLast();
    else if(act=="disconnect") bt1036_disconnect();
    else if(act=="scan") bt1036_startScan();
    webServer.send(200, "text/plain", "OK");
}

static void handleReboot() {
    if(webServer.arg("target")=="bt") bt1036_softReboot();
    else { webServer.send(200, "text/plain", "Rebooting..."); delay(500); ESP.restart(); }
    webServer.send(200, "text/plain", "OK");
}
static void handleFactory() { bt1036_runFactorySetup(); webServer.send(200, "text/plain", "OK"); }

static void handleApiScan() {
    int n = WiFi.scanNetworks();
    String json = "[";
    for (int i=0; i<n; ++i) {
        if(i) json+=",";
        json += "{\"ssid\":\""+WiFi.SSID(i)+"\",\"rssi\":"+String(WiFi.RSSI(i))+"}";
    }
    json += "]";
    webServer.send(200, "application/json", json);
}

static void handleApiConnect() {
    String ssid = webServer.arg("ssid");
    String psk  = webServer.arg("psk");
    if(ssid.length()>0) {
        prefs.begin("wifi-config", false);
        prefs.putString("ssid", ssid);
        prefs.putString("psk", psk);
        prefs.end();
        WiFi.begin(ssid.c_str(), psk.c_str());
        webServer.send(200, "text/plain", "OK");
    } else webServer.send(400, "text/plain", "Bad SSID");
}

// ======================= INIT & LOOP =======================

void btWebUI_init() {
    prefs.begin("wifi-config", true);
    String s = prefs.getString("ssid", "");
    String p = prefs.getString("psk", "");
    prefs.end();

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(apSsid.c_str(), apPsk.c_str());
    if(s.length()) WiFi.begin(s.c_str(), p.c_str());

    if (MDNS.begin(hostname)) {
        MDNS.addService("http", "tcp", 80);
    }
    
    // Web Pages
    webServer.on("/", handleRoot);
    webServer.on("/wifi", handleWifiPage);
    webServer.on("/bt", handleBtPage);
    webServer.on("/cdc", handleCdc);
    webServer.on("/logs", handleLogs);
    
    // API Routes
    webServer.on("/api/status", handleStatus);
    webServer.on("/api/cmd", handleCmd);
    webServer.on("/api/audio", handleAudio);
    webServer.on("/api/set_basic", handleSetBasic);
    webServer.on("/api/set_profile", handleProfile);
    webServer.on("/api/set_hfp", handleSetHfp);
    webServer.on("/api/reboot", handleReboot);
    webServer.on("/api/factory", handleFactory);
    webServer.on("/api/wifi/scan", handleApiScan);
    webServer.on("/api/wifi/connect", handleApiConnect);
    
    webServer.on("/api/track", []() {
        TrackInfo ti = bt1036_getTrackInfo();
        // Escape quotes in strings for JSON
        String title = ti.title; title.replace("\"", "\\\"");
        String artist = ti.artist; artist.replace("\"", "\\\"");
        String album = ti.album; album.replace("\"", "\\\"");
        String json = "{";
        json += "\"title\":\"" + title + "\",";
        json += "\"artist\":\"" + artist + "\",";
        json += "\"album\":\"" + album + "\",";
        json += "\"elapsed\":" + String(ti.elapsedSec) + ",";
        json += "\"total\":" + String(ti.totalSec) + ",";
        json += "\"valid\":" + String(ti.valid ? "true" : "false");
        json += "}";
        webServer.send(200, "application/json", json);
    });
    
    webServer.on("/api/debug", []() {
        g_debugMode = !g_debugMode;
        btWebUI_log(String("[SYS] Debug mode: ") + (g_debugMode ? "ON" : "OFF"));
        webServer.send(200, "text/plain", g_debugMode ? "ON" : "OFF");
    });
    
    webServer.on("/api/debug_status", []() {
        webServer.send(200, "text/plain", g_debugMode ? "ON" : "OFF");
    });

    webServer.on("/api/at_cmd", []() {
        String cmd = webServer.arg("cmd");
        if (cmd.length() > 0) {
            btWebUI_log("[WEB] Manual command: " + cmd, LogLevel::INFO);
            bt1036_sendRawCommand(cmd);
            webServer.send(200, "text/plain", "OK");
        } else {
            webServer.send(400, "text/plain", "Bad Command");
        }
    });

    // Настраиваем OTA callbacks для безопасного обновления
    ElegantOTA.onStart([]() {
        btWebUI_log("[OTA] Update started. Pausing peripherals.");
        bt1036_pausePolling(true); // Останавливаем опросы BT-модуля
        cdc_pause(true);           // Отключаем прерывания от магнитолы
    });
    ElegantOTA.onEnd([](bool success) {
        btWebUI_log(String("[OTA] Update finished. Success: ") + success);
        bt1036_pausePolling(false); // Возобновляем опросы BT-модуля
        cdc_pause(false);           // Включаем прерывания обратно
        if (!success) {
            btWebUI_log("[OTA] Restarting ESP due to failed update.");
            delay(1000);
            ESP.restart();
        }
    });

    ElegantOTA.begin(&webServer);
    webServer.begin();
    wsServer.begin();
    wsServer.onEvent(onWsEvent);
}

void btWebUI_loop() {
    webServer.handleClient();
    ElegantOTA.loop();
    wsServer.loop();
}
