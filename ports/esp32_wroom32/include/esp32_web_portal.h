#ifndef OPENNEAR_ESP32_WEB_PORTAL_H
#define OPENNEAR_ESP32_WEB_PORTAL_H

#include <pgmspace.h>

/*
 * OpenNear v6.3 Embedded Offline Web Provisioning & Cluster Management Portal
 * Stored in ESP32 Flash (PROGMEM) -> 0 bytes static RAM overhead.
 * Served at GET / on both SoftAP (http://192.168.4.1) and LAN STA IP.
 */
static const char kOpenNearPortalHtml[] PROGMEM = R"ONPORTAL(<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OpenNear 网关配网与管理台</title>
<style>
:root{--bg:#0b1120;--card:#111827;--bd:#1f2937;--tx:#e5e7eb;--mut:#9ca3af;--acc:#38bdf8;--ok:#22c55e;--warn:#f59e0b;--err:#ef4444}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--tx);font:14px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif}
.wrap{max-width:980px;margin:0 auto;padding:14px}
.top{display:flex;flex-wrap:wrap;gap:10px;align-items:center;justify-content:space-between;background:var(--card);border:1px solid var(--bd);border-radius:12px;padding:12px 16px;margin-bottom:12px}
.brand{display:flex;align-items:center;gap:10px}
.brand h1{margin:0;font-size:18px;letter-spacing:.3px}
.badge{display:inline-block;padding:2px 8px;border-radius:999px;font-size:12px;font-weight:600;background:#1e293b;color:var(--acc)}
.badge.ok{background:#052e16;color:var(--ok)}.badge.warn{background:#451a03;color:var(--warn)}.badge.err{background:#450a0a;color:var(--err)}
.pills{display:flex;flex-wrap:wrap;gap:6px;margin-bottom:12px}
.tabs{display:flex;flex-wrap:wrap;gap:6px;margin-bottom:12px}
.tab{background:var(--card);color:var(--mut);border:1px solid var(--bd);border-radius:8px;padding:8px 14px;cursor:pointer;font-weight:600}
.tab.active{background:#0c4a6e;color:#fff;border-color:var(--acc)}
.card{background:var(--card);border:1px solid var(--bd);border-radius:12px;padding:14px 16px;margin-bottom:12px}
.card h2{margin:0 0 10px;font-size:15px;color:#fff}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:10px}
label{display:block;font-size:12px;color:var(--mut);margin:6px 0 4px}
input,select{width:100%;padding:8px 10px;border-radius:8px;border:1px solid #374151;background:#030712;color:var(--tx);font-size:13px}
input:focus,select:focus{outline:none;border-color:var(--acc)}
.row{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
.btn{background:#0284c7;color:#fff;border:0;border-radius:8px;padding:8px 14px;font-weight:600;cursor:pointer;font-size:13px}
.btn:hover{background:#0369a1}.btn.sec{background:#374151}.btn.sec:hover{background:#4b5563}.btn.ok{background:#15803d}.btn.danger{background:#b91c1c}
.btn.sm{padding:4px 9px;font-size:12px}
table{width:100%;border-collapse:collapse;font-size:13px}
th,td{padding:8px 6px;border-bottom:1px solid var(--bd);text-align:left;white-space:nowrap}
th{color:var(--mut);font-weight:600;font-size:12px}
.tbl-wrap{overflow-x:auto}
.mono{font-family:ui-monospace,SFMono-Regular,Consolas,monospace}
.muted{color:var(--mut);font-size:12px}
.toast{position:fixed;right:16px;bottom:16px;max-width:340px;padding:10px 14px;border-radius:10px;background:#1e293b;border:1px solid var(--acc);color:#fff;font-size:13px;display:none;z-index:99}
.hide{display:none}
.ap-list{max-height:190px;overflow-y:auto;border:1px solid var(--bd);border-radius:8px;margin-top:8px}
.ap-item{display:flex;justify-content:space-between;align-items:center;padding:7px 10px;border-bottom:1px solid var(--bd);cursor:pointer}
.ap-item:hover{background:#1f2937}
.ap-item:last-child{border-bottom:0}
</style>
</head>
<body>
<div class="wrap">
  <div class="top">
    <div class="brand">
      <h1>OpenNear</h1>
      <span class="badge" id="b-role">加载中...</span>
      <span class="badge" id="b-fw">v6.3</span>
    </div>
    <div class="row">
      <input id="g-token" type="password" placeholder="Admin Token (如已设置)" style="width:185px" onchange="saveLocalToken()">
      <button class="btn sec sm" onclick="refreshAll()">刷新</button>
    </div>
  </div>

  <div class="pills" id="status-pills"></div>

  <div class="tabs">
    <button class="tab active" data-t="topo" onclick="switchTab('topo')">集群拓扑与数据</button>
    <button class="tab" data-t="wifi" onclick="switchTab('wifi')">Wi-Fi / 云端配网</button>
    <button class="tab" data-t="mqtt" onclick="switchTab('mqtt')">MQTT / Home Assistant</button>
    <button class="tab" data-t="ctrl" onclick="switchTab('ctrl')">节点控制 / OTA / 安全</button>
  </div>

  <section id="p-topo">
    <div class="card">
      <div class="row" style="justify-content:space-between;margin-bottom:8px">
        <h2 style="margin:0">全网在线节点（0~6 跳实时快照）</h2>
        <span class="muted mono" id="topo-meta">--</span>
      </div>
      <div class="tbl-wrap">
        <table>
          <thead><tr><th>Short-ID</th><th>角色</th><th>跳数</th><th>父节点</th><th>时隙</th><th>同步偏差</th><th>收包</th><th>OTA</th><th>实时载荷 (Hex / ASCII)</th><th>操作</th></tr></thead>
          <tbody id="node-rows"><tr><td colspan="10" class="muted">正在拉取集群快照...</td></tr></tbody>
        </table>
      </div>
    </div>
  </section>

  <section id="p-wifi" class="hide">
    <div class="card">
      <div class="row" style="justify-content:space-between">
        <h2 style="margin:0">扫描并连接公网 Wi-Fi</h2>
        <button class="btn sec sm" id="btn-scan" onclick="scanWifi()">扫描周边 Wi-Fi</button>
      </div>
      <div id="ap-list" class="ap-list hide"></div>
      <div class="grid" style="margin-top:8px">
        <div><label>Wi-Fi 名称 (SSID)</label><input id="w-ssid" placeholder="点击上方扫描或手动输入"></div>
        <div><label>Wi-Fi 密码</label><input id="w-pass" type="password" placeholder="开放网络可留空"></div>
      </div>
      <label>HTTP Webhook 云端上报地址（可选）</label>
      <input id="w-url" placeholder="http://your-server.local:8080/telemetry">
      <div class="row" style="margin-top:12px">
        <button class="btn ok" onclick="saveWifi()">保存并连接 Wi-Fi</button>
        <span class="muted">提交后将自动写入 NVS 并在连接路由器成功后自动关闭热点</span>
      </div>
    </div>
  </section>

  <section id="p-mqtt" class="hide">
    <div class="card">
      <h2>MQTT 3.1.1 与 Home Assistant 自动发现</h2>
      <div class="grid">
        <div><label>Broker 主机 / IP</label><input id="m-host" placeholder="192.168.1.100 或 broker.emqx.io"></div>
        <div><label>端口</label><input id="m-port" type="number" value="1883" min="1" max="65535"></div>
        <div><label>用户名（可选）</label><input id="m-user" placeholder="mqtt_user"></div>
        <div><label>密码（可选）</label><input id="m-pass" type="password" placeholder="mqtt_pass"></div>
      </div>
      <div class="row" style="margin-top:12px">
        <button class="btn ok" onclick="saveMqtt()">保存并连接 MQTT</button>
      </div>
      <div class="muted mono" style="margin-top:12px;line-height:1.7">
        HA 自动发现: homeassistant/sensor/opennear_&lt;ID&gt;/config<br>
        节点状态上报: opennear/node/0x&lt;ID&gt;/state<br>
        云端下行控制: opennear/cmd/0x&lt;ID&gt;<br>
        集群 OTA 触发: opennear/ota/trigger
      </div>
    </div>
  </section>

  <section id="p-ctrl" class="hide">
    <div class="card">
      <h2>节点下行指令控制（支持 1~6 跳自动选路）</h2>
      <div class="grid">
        <div><label>目标节点 Short-ID（16进制，FFF=全网广播）</label><input id="c-id" class="mono" value="FFF"></div>
        <div><label>控制载荷文本</label><input id="c-pay" placeholder="例如 MOTOR_ON 或 LED_1"></div>
      </div>
      <div class="row" style="margin-top:10px">
        <button class="btn" onclick="sendNodeCmd()">发送空口下行指令</button>
      </div>
    </div>

    <div class="card">
      <h2>多跳空中固件升级（Authenticated Cluster OTA）</h2>
      <div class="grid">
        <div><label>目标节点（FFF=全网，或指定 Short-ID）</label><input id="o-id" class="mono" value="FFF"></div>
        <div><label>目标版本号（如 0x00060300）</label><input id="o-ver" class="mono" value="0x00060300"></div>
      </div>
      <label>固件 HTTP 下载地址（.bin）</label>
      <input id="o-url" placeholder="http://192.168.1.10/firmware.bin">
      <div class="row" style="margin-top:10px">
        <button class="btn danger" onclick="triggerOta()">拉取并下发集群 OTA</button>
        <span class="muted">AES-128-CBC-MAC + CRC32 双重校验，全员就绪后自动同步切换</span>
      </div>
    </div>

    <div class="card">
      <h2>网关管理安全令牌（Admin Token）</h2>
      <div class="grid">
        <div><label>新 Admin Token（留空表示清除鉴权）</label><input id="t-new" type="password" placeholder="例如 OpenNear2026"></div>
      </div>
      <div class="row" style="margin-top:10px">
        <button class="btn sec" onclick="updateAdminToken()">保存 / 轮换 Token</button>
        <span class="muted">设置后所有写接口需在顶部输入 Token 或携带 X-OpenNear-Token 请求头</span>
      </div>
    </div>
  </section>
</div>
<div class="toast" id="toast"></div>
<script>
const $=id=>document.getElementById(id);
const esc=s=>String(s==null?'':s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
let toastTimer=null;
function toast(msg,isErr){
  const el=$('toast');
  el.textContent=msg;
  el.style.borderColor=isErr?'var(--err)':'var(--acc)';
  el.style.display='block';
  clearTimeout(toastTimer);
  toastTimer=setTimeout(()=>{el.style.display='none';},3200);
}
function saveLocalToken(){
  try{localStorage.setItem('on_admin_tok',$('g-token').value);}catch(e){}
}
function loadLocalToken(){
  try{const t=localStorage.getItem('on_admin_tok');if(t)$('g-token').value=t;}catch(e){}
}
function switchTab(name){
  document.querySelectorAll('.tab').forEach(b=>b.classList.toggle('active',b.dataset.t===name));
  ['topo','wifi','mqtt','ctrl'].forEach(k=>$('p-'+k).classList.toggle('hide',k!==name));
}
function hexToAscii(h){
  if(!h||h.length<2)return '';
  let out='';
  for(let i=0;i+1<h.length;i+=2){
    const v=parseInt(h.slice(i,i+2),16);
    out+=(v>=32&&v<=126)?String.fromCharCode(v):'.';
  }
  return out;
}
function quickCtrl(idHex){
  $('c-id').value=idHex.replace(/^0x/i,'');
  switchTab('ctrl');
  $('c-pay').focus();
}
async function postCmd(cmdText){
  const hdr={'Content-Type':'text/plain'};
  const tok=$('g-token').value.trim();
  if(tok)hdr['X-OpenNear-Token']=tok;
  const r=await fetch('/api/cmd',{method:'POST',headers:hdr,body:cmdText});
  const j=await r.json().catch(()=>({ok:false}));
  if(r.status===401){toast('鉴权失败：请在右上角填写正确的 Admin Token',true);return false;}
  if(!r.ok||!j.ok){toast('指令执行失败 ('+r.status+')',true);return false;}
  return true;
}
async function fetchStatus(){
  try{
    const r=await fetch('/api/status');
    const s=await r.json();
    $('b-role').textContent=(s.role||'NODE')+' '+s.id;
    $('b-fw').textContent=s.fw_ver||'v6.3';
    const pills=[];
    pills.push(s.wifi_ok
      ?'<span class="badge ok">Wi-Fi 已连: '+esc(s.wifi_ssid)+' ('+esc(s.wifi_ip)+' CH'+s.wifi_ch+' '+s.wifi_rssi+'dBm)</span>'
      :'<span class="badge warn">Wi-Fi 未连接</span>');
    if(s.ap_active)pills.push('<span class="badge">配网热点: '+esc(s.ap_ssid)+' (192.168.4.1)</span>');
    pills.push(s.mqtt_ok
      ?'<span class="badge ok">MQTT 在线: '+esc(s.mqtt_host)+':'+s.mqtt_port+'</span>'
      :'<span class="badge">MQTT: '+(s.mqtt_host?esc(s.mqtt_host)+' (重连中)':'未配置')+'</span>');
    pills.push(s.ble_ok?'<span class="badge ok">BLE 已连接</span>':'<span class="badge">BLE 待机</span>');
    pills.push(s.token_set?'<span class="badge ok">Token 保护已启用</span>':'<span class="badge warn">未设 Token</span>');
    pills.push('<span class="badge">运行 '+s.uptime_s+'s</span>');
    $('status-pills').innerHTML=pills.join('');
    if(!$('w-ssid').value&&s.wifi_ssid)$('w-ssid').value=s.wifi_ssid;
    if(!$('w-url').value&&s.cloud_url)$('w-url').value=s.cloud_url;
    if(!$('m-host').value&&s.mqtt_host)$('m-host').value=s.mqtt_host;
    if(s.mqtt_port&&$('m-port').value==='1883')$('m-port').value=String(s.mqtt_port);
  }catch(e){}
}
async function fetchData(){
  try{
    const r=await fetch('/api/data');
    const d=await r.json();
    $('topo-meta').textContent='SF='+d.sf+' | G='+d.g+' | Backup='+d.backup+' | Sub-G='+d.sub_g+' | 节点='+d.count;
    const tb=$('node-rows');
    if(!d.nodes||!d.nodes.length){
      tb.innerHTML='<tr><td colspan="10" class="muted">暂无在线节点</td></tr>';
      return;
    }
    const otaNames=['IDLE','RX','VERIFY','READY','COMMIT','APPLIED','FAIL'];
    tb.innerHTML=d.nodes.map(n=>{
      const asc=hexToAscii(n.data);
      const rCls=n.role==='G'?'ok':(n.role==='SUB_G'?'warn':'');
      return '<tr>'
        +'<td class="mono"><b>'+esc(n.id)+'</b></td>'
        +'<td><span class="badge '+rCls+'">'+esc(n.role)+'</span></td>'
        +'<td>Hop '+n.hop+'</td>'
        +'<td class="mono">'+esc(n.parent)+'</td>'
        +'<td>'+n.slot+'</td>'
        +'<td class="mono">'+n.skew_ns+' ns</td>'
        +'<td>'+n.rx+'</td>'
        +'<td>'+(otaNames[n.ota]||n.ota)+'</td>'
        +'<td class="mono">'+(n.data?esc(n.data)+' <span class="muted">('+esc(asc)+')</span>':'--')+'</td>'
        +'<td><button class="btn sec sm" onclick="quickCtrl(\''+esc(n.id)+'\')">控制</button></td>'
        +'</tr>';
    }).join('');
  }catch(e){}
}
let gScanAps=[];
async function scanWifi(){
  const btn=$('btn-scan');
  btn.disabled=true;btn.textContent='扫描中...';
  try{
    const r=await fetch('/api/scan');
    const list=await r.json();
    gScanAps=Array.isArray(list)?list:[];
    const box=$('ap-list');
    box.classList.remove('hide');
    if(!gScanAps.length){
      box.innerHTML='<div class="ap-item muted">未扫描到周边 2.4GHz Wi-Fi</div>';
    }else{
      box.innerHTML=gScanAps.map((a,i)=>
        '<div class="ap-item" onclick="pickApIdx('+i+')">'
        +'<span><b>'+esc(a.ssid)+'</b> <span class="muted">CH'+a.ch+(a.enc?' [加密]':' [开放]')+'</span></span>'
        +'<span class="mono muted">'+a.rssi+' dBm</span></div>'
      ).join('');
    }
  }catch(e){toast('扫描失败，请稍后重试',true);}
  btn.disabled=false;btn.textContent='扫描周边 Wi-Fi';
}
function pickApIdx(idx){
  const a=gScanAps[idx];
  if(!a)return;
  $('w-ssid').value=a.ssid;
  $('w-pass').focus();
  toast('已选中 Wi-Fi: '+a.ssid);
}
async function saveWifi(){
  const ssid=$('w-ssid').value.trim();
  const pass=$('w-pass').value;
  const url=$('w-url').value.trim();
  if(!ssid){toast('请填写 Wi-Fi SSID',true);return;}
  const cmd=url?('WIFI:'+ssid+'|'+pass+'|'+url):('WIFI:'+ssid+'|'+pass);
  if(await postCmd(cmd)){
    toast('Wi-Fi 已保存并开始连接，请稍候查看上方状态栏');
    setTimeout(fetchStatus,2500);
  }
}
async function saveMqtt(){
  const host=$('m-host').value.trim();
  const port=$('m-port').value.trim()||'1883';
  const user=$('m-user').value.trim();
  const pass=$('m-pass').value;
  if(!host){toast('请填写 MQTT Broker 地址',true);return;}
  let cmd='MQTT:'+host+'|'+port;
  if(user)cmd+='|'+user+'|'+pass;
  if(await postCmd(cmd)){
    toast('MQTT 配置已保存，正在连接 Broker...');
    setTimeout(fetchStatus,2500);
  }
}
async function sendNodeCmd(){
  const id=$('c-id').value.trim().replace(/^0x/i,'')||'FFF';
  const pay=$('c-pay').value;
  if(!pay){toast('请输入控制载荷',true);return;}
  if(await postCmd('CMD:'+id+'|'+pay)){
    toast('已下发空口指令至 0x'+id.toUpperCase());
  }
}
async function triggerOta(){
  const id=$('o-id').value.trim().replace(/^0x/i,'')||'FFF';
  const ver=$('o-ver').value.trim();
  const url=$('o-url').value.trim();
  if(!ver||!url){toast('请填写版本号与固件 HTTP URL',true);return;}
  if(await postCmd('OTA:'+id+'|'+ver+'|'+url)){
    toast('OTA 任务已触发，网关正在拉取固件...');
  }
}
async function updateAdminToken(){
  const nt=$('t-new').value.trim();
  if(await postCmd('TOKEN:'+nt)){
    $('g-token').value=nt;
    saveLocalToken();
    $('t-new').value='';
    toast(nt?'Admin Token 已更新并保存至 NVS':'Admin Token 已清除');
    fetchStatus();
  }
}
function refreshAll(){fetchStatus();fetchData();}
loadLocalToken();
refreshAll();
setInterval(refreshAll,3000);
</script>
</body>
</html>)ONPORTAL";

#endif /* OPENNEAR_ESP32_WEB_PORTAL_H */
