/* =============================================================
 *  FURA BALÃO – FIRMWARE  (rev-H3)             30 Jun 2025
 *  -----------------------------------------------------------
 *  • Compatível com Espressif core 2.x/3.x  (Arduino-ESP32)
 *  • Requer  Bibliotecas AsyncTCP  +  ESPAsyncWebServer
 *    https://github.com/esphome/AsyncTCP
 *    https://github.com/esphome/ESPAsyncWebServer
 *  • Patch MBEDTLS p/ núcleos antigos
 * ============================================================*/

/* ---------------- PINAGEM ---------------------------------- *
 *  ▶ Motor LEFT  (canal 0)   — lado esquerdo do robô
 *     ENA_L  PWM   → GPIO14
 *     IN1_L  dirA  → GPIO27
 *     IN2_L  dirB  → GPIO26
 *
 *  ▶ Motor RIGHT (canal 1)   — lado direito do robô
 *     ENA_R  PWM   → GPIO32
 *     IN1_R  dirA  → GPIO25
 *     IN2_R  dirB  → GPIO33
 *
 *  ▶ WEAPON        → GPIO12  - acionamento da arma (MTDI – manter LOW no boot)
 *  ▶ VSUP_SENSE    → GPIO34  - leitura de tensão da bateria - a desenvolver
 *  ▶ LED on-board  → GPIO2   - pisca led azul ao receber comandos
 * -----------------------------------------------------------
 *  ►  ALTERAÇÕES desta revisão
 *    · loop principal a cada 50 ms (↓CPU / sem stuttering)
 *    · watchdog movimento 5 s
 *    · telemetria: +uptime,+heap,+wifi.list  (MAC+RSSI)
 *    · temperatura e VBAT com 2 casas decimais
 *    · UI JS: throttle 80 ms / keep-alive 250 ms,
 *             até 3 clientes Wi-Fi na barra, user-select:none
/* ============================================================= */

/* ---------- INCLUDES BÁSICOS -------------------------------- */
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <esp_wifi.h>

/* ---------- MBEDTLS PATCH (núcleo 3.x) --------------------- */
extern "C" {
  #if !defined(mbedtls_md5_starts_ret)
    #define mbedtls_md5_starts_ret  mbedtls_md5_starts
    #define mbedtls_md5_update_ret  mbedtls_md5_update
    #define mbedtls_md5_finish_ret  mbedtls_md5_finish
  #endif
}
#include <ESPAsyncWebServer.h>

/* =================================================================
 *  LEDC compatibility layer – compila APENAS se a API não existir
 * ================================================================= */
#ifndef ledcSetup          // núcleos sem a API oficial
  #include <driver/ledc.h>
  static bool _ledcSetupCompat(uint8_t ch,uint32_t f,uint8_t res){
    ledc_timer_config_t t{}; t.speed_mode=LEDC_HIGH_SPEED_MODE;
    t.duty_resolution=static_cast<ledc_timer_bit_t>(res);
    t.timer_num=LEDC_TIMER_0; t.freq_hz=f; t.clk_cfg=LEDC_AUTO_CLK;
    return ledc_timer_config(&t)==ESP_OK;
  }
  static void _ledcAttachPinCompat(uint8_t pin,uint8_t ch){
    ledc_channel_config_t c{}; c.channel=static_cast<ledc_channel_t>(ch);
    c.gpio_num=pin; c.speed_mode=LEDC_HIGH_SPEED_MODE; c.timer_sel=LEDC_TIMER_0;
    ledc_channel_config(&c);
  }
  static void _ledcWriteCompat(uint8_t ch,uint32_t duty){
    ledc_set_duty(LEDC_HIGH_SPEED_MODE,static_cast<ledc_channel_t>(ch),duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE,static_cast<ledc_channel_t>(ch));
  }
  #define ledcSetup     _ledcSetupCompat
  #define ledcAttachPin _ledcAttachPinCompat
  #define ledcWrite     _ledcWriteCompat
#endif

/* ---------- SOFT-AP ---------------------------------------- */
const char AP_SSID[]="FURA-BALAO"; // Mudar para o nome do robô
const char AP_PASS[]="12345678";
/*  IPAddress declarado APÓS os #include (boa prática)  */
const IPAddress AP_IP(192,168,4,1), AP_NET(255,255,255,0);

/* ---------- PINOUT & PWM ---------------------------------- */
constexpr int ENA_L = 14, IN1 = 27, IN2 = 26;   // LEFT  motor pinos
constexpr int ENA_R = 32, IN3 = 25, IN4 = 33;   // RIGHT motor pinos

// constexpr int CH_L  = 0,  CH_R = 1, PWM_F = 20000, PWM_B = 8; // 20 kHz, 8 bits
// 10 kHz (mais silencioso) e resolução de 9 bits → passos ~0,2 %
constexpr int CH_L  = 0,  CH_R = 1, PWM_F = 10000, PWM_B = 9; 


constexpr int WEAPON_PIN = 12;   // MTDI – evitar HIGH durante boot
constexpr int VBAT_PIN   = 34;   // ADC1_CH6 (divisor 2:1 → VSUP)
constexpr int LED_PIN    = 2;    // LED on-board

/* ---------- CALIBRAÇÃO ------------------------------------ */
float leftTrim=1.0f, rightTrim=1.0f, maxPower=1.0f;
constexpr uint16_t KICK_MS=100, RAMP_MS=100;

/* ---------- GLOBAIS --------------------------------------- */
AsyncWebServer server(80); DNSServer dns;
volatile uint32_t kickEnd = 0;    // pulso de arrancada (volatile p/ uso no loop)
uint32_t lastCmdMs=0;             // timestamp último comando
char lastAck[13]={0};

float targetL=0,targetR=0,currentL=0,currentR=0;
bool  weaponActive=false,fault=false;
uint32_t ledFlashUntil = 0;

/* =========================================================
 *  UI  – HTML + CSS + JS  (estrutura minimamente alterada)
 * ======================================================= */
const char INDEX_HTML[] PROGMEM=R"rawliteral(
<!DOCTYPE html><html lang="pt-BR"><head><meta charset="utf-8">
<title>Mini-Tank</title><meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{
  --bg-main:#0d0f11;--bg-panel:#1a1d22;--accent:#12c2ff;--accent-2:#ff1fa6;
  --text-dim:#a0a6ad;--radius:10px;--shadow:0 4px 14px rgba(0,0,0,.45);
  --colors:#12c2ff,#ff8c00,#ff1fa6,#15e56b,#ffc400,#7f7fff,#ff5555,#00e0d5;
}
*{box-sizing:border-box;margin:0;padding:0}
html,body{background:var(--bg-main);color:#fff;font-family:"Segoe UI",Roboto,Arial,sans-serif}
.noSel,*:where(.noSel *){user-select:none;-webkit-user-select:none;-ms-user-select:none}
button{font-family:inherit}

.esp32-wrap{display:flex;gap:24px;padding:20px;touch-action:none}
@media(max-width:480px){.esp32-wrap{flex-direction:column;gap:18px}}
.group{display:flex;flex-direction:column;gap:10px;min-width:240px;align-items:center}
.center{flex:1 1 300px;max-width:360px;display:flex;flex-direction:column;align-items:center;gap:10px}

.btn{padding:10px 6px;border:0;border-radius:var(--radius);cursor:pointer;font-weight:600;
     text-transform:uppercase;color:#fff;box-shadow:var(--shadow);font-size:13px}
.power{width:100%;padding:10px 0;font-size:16px;border:0;border-radius:var(--radius);
       color:#fff;cursor:pointer;box-shadow:var(--shadow);text-transform:uppercase}
.power.off{background:#c41616}.power.on{background:#16a032}

.panel{display:grid;grid-template-columns:repeat(auto-fit,minmax(80px,1fr));gap:15px;width:100%}
.log,.tel{width:100%;height:90px;overflow:auto;padding:6px;background:var(--bg-panel);
          border:1px solid #292d33;border-radius:var(--radius);box-shadow:var(--shadow);
          font:12px "Roboto Mono",monospace;text-transform:uppercase}
.tel{color:var(--accent)}

.stick{width:200px;height:200px;border-radius:50%;position:relative;touch-action:none;
       background:radial-gradient(circle at center,#48505f 0%,#2b303a 70%,#171a20 100%);
       box-shadow:var(--shadow)}
.stick::after{content:"";position:absolute;inset:0;margin:auto;width:20px;height:20px;
              border-radius:50%;background:var(--accent);filter:drop-shadow(0 0 6px var(--accent))}

/* ---------- Cores cíclicas (se paint API disponível) ------- */
@supports(background:paint(solid-color)){
  .panel>.btn:nth-child(8n+1){background:var(--accent)}
  .panel>.btn:nth-child(8n+2){background:#ff8c00}
  .panel>.btn:nth-child(8n+3){background:#ff1fa6}
  .panel>.btn:nth-child(8n+4){background:#15e56b}
  .panel>.btn:nth-child(8n+5){background:#ffc400}
  .panel>.btn:nth-child(8n+6){background:#7f7fff}
  .panel>.btn:nth-child(8n+7){background:#ff5555}
  .panel>.btn:nth-child(8n+8){background:#00e0d5}
}
</style></head>
<body class="noSel">
<div class="esp32-wrap">
  <!-- Comandos -->
  <div class="group">
    <div id="statA"></div>
    <div class="panel" id="panel">
      <button class="btn" data-cmd="LED_RON">LED</button>
      <button class="btn" data-cmd="WEAPON_RON">ARMA</button>
      <button class="btn" data-cmd="AUX_RON">AUX</button>
    </div>
  </div>

  <!-- Centro -->
  <div class="center">
    <h2>Fura Balão</h2>
    <button id="power" class="power off">LIGAR</button>
    <div id="log" class="log"></div>
    <div id="tel" class="tel" title="Wi-Fi">–</div>
  </div>

  <!-- Joystick -->
  <div class="group">
    <div id="statM"></div>
    <div id="stick" class="stick"></div>
  </div>
</div>
<script>
/* =============== UI – JS =================================== */
const API="/cmd",TEL="/telemetry",
      COL=['#12c2ff','#ff8c00','#ff1fa6','#15e56b','#ffc400','#7f7fff','#ff5555','#00e0d5'];
const $=q=>document.querySelector(q);
const log=t=>{const d=document.createElement('div');d.textContent=t.toUpperCase();
               $('#log').prepend(d);while($('#log').children.length>5)$('#log').lastChild.remove();};

/* ---------- ESTADO GLOBAL --------------------------------- */
let powerOn=false,lastCmd='',inflight=null,joyActive=false;
const KEEPALIVE_MS=250;          // reenvio periódico
let pressT0=0;

/* ---------- POWER ----------------------------------------- */
$('#power').onpointerdown=()=>pressT0=Date.now();
$('#power').onpointerup=()=>{
  if(Date.now()-pressT0>1500){
    powerOn=!powerOn;
    $('#power').classList.toggle('on',powerOn);
    $('#power').classList.toggle('off',!powerOn);
    $('#power').textContent=powerOn?'LIGADO':'LIGAR';
    send(powerOn?'POWER_ON':'POWER_OFF');
  }
};

/* ---------- BOTÕES PAINEL --------------------------------- */
$('#panel').querySelectorAll('.btn').forEach((b,i)=>{
  b.style.background=COL[i%COL.length];
  b.onclick=()=>send(b.dataset.cmd);
});

/* ---------- JOYSTICK -------------------------------------- */
(()=>{const stick=$('#stick'),lbl=$('#statM'),R=100;
      let lastTs=0;
      const pos=e=>{const r=stick.getBoundingClientRect();
                    return{ x:e.clientX-r.left-R,y:e.clientY-r.top-R };};
      const ang=(x,y)=>((Math.atan2(-y,x)+2*Math.PI)%(2*Math.PI))*180/Math.PI|0;
      const pwr=(x,y)=>Math.round(Math.min(Math.hypot(x,y)/R,1)*100);
      const update=e=>{
        const p=pos(e),deg=ang(p.x,p.y),pw=pwr(p.x,p.y);
        lbl.textContent=`θ ${deg}° · ${pw}%`;
        lastCmd=`M${String(deg).padStart(4,'0')}P${String(pw).padStart(2,'0')}`;
        if(powerOn&&Date.now()-lastTs>80){send(lastCmd);lastTs=Date.now();}
      };
      stick.onpointerdown=e=>{joyActive=true;update(e);};
      stick.onpointermove=e=>{if(joyActive)update(e);e.preventDefault();};
      const release=()=>{joyActive=false;lbl.textContent='PARADO';
                         lastCmd='M0090P00';send(lastCmd);};
      ['pointerup','pointerleave','pointercancel'].forEach(ev=>stick.addEventListener(ev,release));
})();

/* ---------- KEEP-ALIVE ------------------------------------ */
setInterval(()=>{if(powerOn&&joyActive&&lastCmd)send(lastCmd);},KEEPALIVE_MS);

/* ---------- TELEMETRIA ------------------------------------ */
(async function poll(){
  const tel=$('#tel');
  try{
    const r=await fetch(TEL,{cache:'no-store'});
    if(r.ok){
      const t=await r.json();
      const v=t.vbat!==undefined?Number(t.vbat).toFixed(2):'--';
      const c=t.temp!==undefined?Number(t.temp).toFixed(2):'--';
      /* monta info de clientes Wi-Fi */
      let wifiTxt='–',title='';
      if(t.wifi?.list?.length){
        wifiTxt=t.wifi.list.slice(0,3).map(o=>`${o.mac.slice(-4)}(${o.rssi})`).join(' ');
        if(t.wifi.list.length>3)title=t.wifi.list.slice(3).map(o=>`${o.mac}(${o.rssi})`).join(' ');
      }
      tel.textContent=`V ${v} V | T ${c} °C | Wi-Fi ${wifiTxt}`;
      tel.title=title;
      if(t.ack)log(`ACK ${t.ack}`);
    }
  }catch{/* ignora */}
  setTimeout(poll,2000);
})();

/* ---------- ENVIO CMD (sem fila) --------------------------- */
async function send(cmd){
  try{
    if(inflight)inflight.abort();
    inflight=new AbortController();
    const resp=await fetch(API,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
                                body:`command=${encodeURIComponent(cmd)}`,signal:inflight.signal});
    inflight=null;if(!resp.ok)console.error(await resp.text());
  }catch(e){if(e.name!=='AbortError')console.error(e);}
}
</script></body></html>
)rawliteral";

/* ===========================================================
 *  MOTOR & UTIL – funções organizadas e comentadas
 * ========================================================= */

/* clamp(): limita valor ao intervalo [a,b] */
inline float clamp(float v,float a,float b){return v<a?a:v>b?b:v;}

/* drive(): aplica direção + PWM em pinos específicos
 *  – val ∈ [-1,+1]  */
inline void drive(float val,int en,int dirA,int dirB,uint8_t ch,uint8_t resBits){
  if(val>=0){digitalWrite(dirA,HIGH);digitalWrite(dirB,LOW);}
  else      {digitalWrite(dirA,LOW); digitalWrite(dirB,HIGH);val=-val;}
  const uint16_t duty=uint16_t(val*((1u<<resBits)-1));
  ledcWrite(ch,duty);
}

/* setMotorRaw(): trims + inversão física dos lados (L⇆R) */
void setMotorRaw(float l,float r){
  l=clamp(l*leftTrim ,-1,1);
  r=clamp(r*rightTrim,-1,1);
  /*  ►  L/R INVERTIDOS: valor “l” controla motor direito e vice-versa  */
  drive(l,ENA_R,IN3,IN4,CH_R,PWM_B);
  drive(r,ENA_L,IN1,IN2,CH_L,PWM_B);
}

/* -----------------------------------------------------------
 *  rampMotors() – rampa bidirecional c/ “zero-cross safe”
 *  – chamada a cada 50 ms
 * --------------------------------------------------------- */
void rampMotors() {
  /* Durante o kick: aplica força plena com o sinal do alvo */
  if (millis() < kickEnd) {
    setMotorRaw(copysignf(maxPower, targetL),
                copysignf(maxPower, targetR));
    return;
  }

  const float STEP = 50.0f / RAMP_MS;          // ≈0,25 por iteração

  auto accel = [&](float cur, float tgt) -> float {

    /* 1. Direções opostas?  -> freia até ZERO primeiro */
    if (cur * tgt < 0) {
      /* diminui magnitude até zero, sem jamais trocar o sinal */
      if (fabsf(cur) <= STEP) return 0.0f;            // já parou
      return cur + (cur > 0 ? -STEP : STEP);          // freando
    }

    /* 2. Mesmo sinal (ou um deles =0) -> acelera / desacelera normal */
    if (cur < tgt)  return fmin(cur + STEP, tgt);     // acelera
    if (cur > tgt)  return fmax(cur - STEP, tgt);     // freia
    return cur;                                       // já na meta
  };

  currentL = accel(currentL, targetL);
  currentR = accel(currentR, targetR);

  setMotorRaw(currentL * maxPower, currentR * maxPower);
}


/* polarToDiff(): ÂNGULO+POT → diferencial L/R (-1…+1)
 *  – 0° = direita | 90° = frente | 180° = trás | 270° = esquerda */
void polarToDiff(uint16_t deg,uint8_t pct,float &l,float &r){
  constexpr uint8_t SPIN=10;
  constexpr float SCALE=1.0f/100.0f;

  /* giros in-place */
  if(deg<=SPIN||deg>=360-SPIN){ l= pct*SCALE; r=-pct*SCALE;return;}
  if(abs((int)deg-180)<=SPIN){  l=-pct*SCALE; r= pct*SCALE;return;}

  /* movimento diferencial contínuo */
  const float k=0.70f;                 // curva mais suave (↑k → fecha)
  float rad=deg*PI/180.0f;
  float x=cosf(rad)*k,  y=sinf(rad);
  float L=y+x, R=y-x;
  float n=max(1.0f,max(fabsf(L),fabsf(R)));
  l=(L/n)*pct*SCALE;
  r=(R/n)*pct*SCALE;
}

/* ===========================================================
 *  HTTP HANDLERS
 * ========================================================= */
void handleCmd(const String &c){
  ledFlashUntil=millis()+300;
  if(c=="POWER_OFF"){targetL=targetR=0;return;}

 /* ---------- MOVIMENTO  (formato “MddddPpp”) ------------- */
  if(c.startsWith("M")){
    int idxP=c.indexOf('P'); if(idxP==-1)return;
    uint16_t ang=c.substring(1,idxP).toInt();         // 0-360°
    int raw=c.substring(idxP+1).toInt();              // 0-999
    uint8_t pwr=raw>100?100:raw;
    polarToDiff(ang,pwr,targetL,targetR);

/* -------- kick-start --------------------------- */
    kickEnd = pwr? millis()+KICK_MS : 0;
    strncpy(lastAck,c.c_str(),12);
    return;
  }

  if(c.endsWith("_RON"))  weaponActive=true;
  if(c.endsWith("_ROFF")) weaponActive=false;
}


/* ---------- /cmd ------------------------------------------ */
void apiCmd(AsyncWebServerRequest *r){
  const AsyncWebParameter *p=r->getParam("command",true);
  if(!p)p=r->getParam("command");
  if(!p){r->send(400,"application/json","{\"error\":\"Missing command\"}");return;}
  const String cmd=p->value(); Serial.printf("[CMD] %s\n",cmd.c_str());
  handleCmd(cmd); lastCmdMs=millis();
  r->send(200,"application/json","{\"ok\":true}");
}

/* ---------- /telemetry ------------------------------------ */
void apiTel(AsyncWebServerRequest* r){
  StaticJsonDocument<256> d;
  d["vbat"]=roundf((analogReadMilliVolts(VBAT_PIN)/1000.0f*2)*100)/100.0f;
  d["temp"]=roundf(temperatureRead()*100)/100.0f;
  d["uptime"]=millis()/1000;
  d["heap"]=ESP.getFreeHeap();

  JsonObject wifi=d.createNestedObject("wifi");
  wifi["qual"]=WiFi.RSSI()==0?0:map(WiFi.RSSI(),-90,-30,0,100);

  /* lista de clientes AP (até 8) */
  wifi_sta_list_t st; if(esp_wifi_ap_get_sta_list(&st)==ESP_OK){
    JsonArray lst=wifi.createNestedArray("list");
    for(int i=0;i<st.num;i++){
      const wifi_sta_info_t &s=st.sta[i];
      char mac[18]; sprintf(mac,"%02X:%02X:%02X:%02X:%02X:%02X",
                            s.mac[0],s.mac[1],s.mac[2],s.mac[3],s.mac[4],s.mac[5]);
      JsonObject o=lst.createNestedObject();
      o["mac"]=mac; o["rssi"]=s.rssi;
    }
  }
  d["ack"]=lastAck;
  String out; serializeJson(d,out);
  r->send(200,"application/json",out);
}

/* ---------- DEBUG ----------------------------------------- */
void debugTelemetry(){
  static uint32_t last=0;
  if(millis()-last<10000)return;
  last=millis();
  Serial.println(F("===== DEBUG ====="));
  Serial.printf("WiFi RSSI: %d dBm\n",WiFi.RSSI());
  Serial.printf("VBAT: %.2f V\n",analogReadMilliVolts(VBAT_PIN)/1000.0f*2);
  Serial.printf("Temp: %.2f ºC\n",temperatureRead());
  Serial.printf("Target L/R: %.2f %.2f | Current: %.2f %.2f\n",
                targetL,targetR,currentL,currentR);
  Serial.printf("Weapon: %s | Fault: %s\n",
                (weaponActive&&!fault)?"ON":"OFF",fault?"YES":"NO");
  Serial.printf("Free heap: %u B | Uptime: %lus\n",ESP.getFreeHeap(),millis()/1000);
  Serial.printf("Last ACK: %s\n",lastAck);
  Serial.println(F("==================\n"));
}

/* ===========================================================
 *  SETUP
 * ========================================================= */
void setup(){
  Serial.begin(115200); delay(500);
  Serial.println(F("==== FURA BALÃO – SETUP ===="));

  pinMode(LED_PIN,OUTPUT); pinMode(WEAPON_PIN,OUTPUT);
  pinMode(ENA_L,OUTPUT); pinMode(ENA_R,OUTPUT);
  pinMode(IN1,OUTPUT);   pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);   pinMode(IN4,OUTPUT);

  ledcSetup(CH_L,PWM_F,PWM_B); ledcSetup(CH_R,PWM_F,PWM_B);
  ledcAttachPin(ENA_L,CH_L);   ledcAttachPin(ENA_R,CH_R);
  Serial.println(F("[OK] PWM configurado"));

  WiFi.mode(WIFI_AP); WiFi.softAPConfig(AP_IP,AP_IP,AP_NET);
  WiFi.softAP(AP_SSID,AP_PASS); dns.start(53,"*",AP_IP);
  Serial.printf("[OK] AP: %s\n",AP_SSID);

  server.on("/",HTTP_GET,[](auto*r){r->send_P(200,"text/html",INDEX_HTML);});
  server.on("/cmd",HTTP_ANY,apiCmd);
  server.on("/telemetry",HTTP_GET,apiTel);
  server.begin(); Serial.println(F("[OK] HTTP server"));

  /* boot blink */
  for(uint8_t i=0;i<3;i++){digitalWrite(LED_PIN,HIGH);delay(150);
                           digitalWrite(LED_PIN,LOW); delay(150);}
  Serial.println(F("==== SETUP OK ====\n"));
}

/* ===========================================================
 *  LOOP
 * ========================================================= */
void loop(){
  dns.processNextRequest();

  /* atualização motores: 50 ms */
  static uint32_t t=0;
  if(millis()-t>=50){rampMotors();t=millis();}

  /* LED status */
  bool led;
  if(fault)              led=((millis()/250)&1);
  else if(ledFlashUntil>millis()) led=((millis()/50)&1);
  else                  led=HIGH;
  digitalWrite(LED_PIN,led);

  /* watchdog motores: para após 5 s sem comando */
  if(millis()-lastCmdMs>5000)targetL=targetR=0;

  digitalWrite(WEAPON_PIN,weaponActive&&!fault);
  debugTelemetry();
}
