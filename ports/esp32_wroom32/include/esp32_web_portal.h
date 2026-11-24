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
.badge.ok{background:#052e16;color:var(--ok)}.badge.warn{background:#451a03;color:var(--warn)}.badge.err{background:#450a0a;color:var(--err)}.badge.purp{background:#3b0764;color:#d8b4fe}
.pills{display:flex;flex-wrap:wrap;gap:6px;margin-bottom:12px}
.tabs{display:flex;flex-wrap:wrap;gap:6px;margin-bottom:12px}
.tab{background:var(--card);color:var(--mut);border:1px solid var(--bd);border-radius:8px;padding:8px 14px;cursor:pointer;font-weight:600}
.tab.active{background:#0c4a6e;color:#fff;border-color:var(--acc)}
.card{background:var(--card);border:1px solid var(--bd);border-radius:12px;padding:14px 16px;margin-bottom:12px}
.card h2{margin:0 0 10px;font-size:15px;color:#fff}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:10px}
label{display:block;font-size:12px;color:var(--mut);margin:6px 0 4px}
input,select,textarea{width:100%;padding:8px 10px;border-radius:8px;border:1px solid #374151;background:#030712;color:var(--tx);font-size:13px}
input:focus,select:focus,textarea:focus{outline:none;border-color:var(--acc)}
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
      <span class="badge" id="b-fw">v7.2</span>
    </div>
    <div class="row">
      <input id="g-token" type="password" placeholder="Admin Token (如已设置)" style="width:185px" onchange="saveLocalToken()">
      <button class="btn sec sm" onclick="refreshAll()">刷新</button>
    </div>
  </div>

  <div class="pills" id="status-pills"></div>

  <div class="tabs">
    <button class="tab active" data-t="topo" onclick="switchTab('topo')">集群拓扑与数据</button>
    <button class="tab" data-t="loc" onclick="switchTab('loc')">室内定位 / 围栏 / CSI 雷达</button>
    <button class="tab" data-t="wifi" onclick="switchTab('wifi')">Wi-Fi / 云端配网</button>
    <button class="tab" data-t="mqtt" onclick="switchTab('mqtt')">MQTT / Home Assistant</button>
    <button class="tab" data-t="ctrl" onclick="switchTab('ctrl')">节点控制 / OTA / 安全</button>
  </div>

  <section id="p-topo">
    <div class="card">
      <div class="row" style="justify-content:space-between;margin-bottom:8px">
        <h2 style="margin:0">多跳无线集群实时拓扑图（点击节点可快速控制）</h2>
        <span class="muted mono" id="topo-meta">--</span>
      </div>
      <div style="overflow-x:auto;background:#030712;border:1px solid var(--bd);border-radius:10px;padding:6px;margin-bottom:12px">
        <svg id="topo-svg" width="100%" height="180" viewBox="0 0 880 180" style="display:block;min-width:520px"></svg>
      </div>
      <div class="tbl-wrap">
        <table>
          <thead><tr><th>Short-ID</th><th>节点名称 / MAC / 标签</th><th>角色</th><th>跳数</th><th>父节点</th><th>时隙</th><th>同步偏差</th><th>收包</th><th>OTA</th><th>实时载荷 (Hex / ASCII)</th><th>操作</th></tr></thead>
          <tbody id="node-rows"><tr><td colspan="11" class="muted">正在拉取集群快照...</td></tr></tbody>
        </table>
      </div>
    </div>
  </section>

  <section id="p-loc" class="hide">
    <div class="card">
      <div class="row" style="justify-content:space-between;margin-bottom:8px">
        <h2 style="margin:0">2D 室内定位 · 电子围栏 · Wi-Fi CSI 隔空感知雷达</h2>
        <div class="row">
          <button class="btn ok sm" id="btn-locdemo" onclick="toggleLocDemo()">🎬 一键运行围栏+CSI 联动演示</button>
          <button class="btn sec sm" onclick="stepLocDemo()">单步推进</button>
          <button class="btn sec sm" id="btn-blescan" onclick="toggleBleScan()">BLE 信标扫描: 开</button>
          <span class="muted mono" id="loc-meta">锚点=0 | 目标=0</span>
        </div>
      </div>
      <div id="loc-evt-banner" class="hide" style="background:#052e16;border:1px solid #22c55e;color:#bbf7d0;border-radius:8px;padding:7px 12px;margin-bottom:10px;font-size:12.5px;display:flex;justify-content:space-between;align-items:center">
        <span>⚡ <b>离线边缘自动化最新触发：</b><span class="mono" id="loc-evt-txt">--</span></span>
        <span class="badge ok">0 云端 · 本地毫秒级闭环</span>
      </div>
      <div style="overflow-x:auto;background:#030712;border:1px solid var(--bd);border-radius:10px;padding:6px;margin-bottom:12px">
        <svg id="loc-svg" width="100%" height="270" viewBox="0 0 880 270" style="display:block;min-width:520px"></svg>
      </div>
      <div class="tbl-wrap">
        <table>
          <thead><tr><th>目标 / 信标</th><th>来源</th><th>MAC / 标识</th><th>所在房间</th><th>估算坐标 (m)</th><th>RSSI / tx1m</th><th>锚点数</th><th>误差</th><th>操作</th></tr></thead>
          <tbody id="loc-rows"><tr><td colspan="9" class="muted">点击右上角「🎬 一键运行围栏+CSI 联动演示」可立即体验 4 房间 + 2 电子围栏 + Wi-Fi CSI 隔空人体雷达联动；或手动添加 1~4 个固定锚点</td></tr></tbody>
        </table>
      </div>
    </div>

    <div class="card">
      <div class="row" style="justify-content:space-between;margin-bottom:8px">
        <h2 style="margin:0">Wi-Fi CSI 隔空无源人体感知雷达（IEEE 802.11bf · 64 子载波 I/Q 多径扰动）</h2>
        <span class="muted">无需佩戴任何设备 · 穿墙感知房间内走动与静坐微动</span>
      </div>
      <div id="csi-cards" class="grid">
        <div class="muted">配置房间锚点或开启一键演示后，此处实时显示各房间 Wi-Fi CSI 16 频段子载波瀑布与人体活动评分</div>
      </div>
    </div>

    <div class="card">
      <h2>纯 ESP32 离线边缘电子围栏与空间+CSI 自动化规则（存 NVS · 断网/无 HA 照常毫秒级触发）</h2>
      <div class="grid">
        <div><label>电子围栏名称（留空半径=0 删除）</label><input id="f-name" placeholder="例如 StudyDesk 或 KitchenZone"></div>
        <div><label>围栏中心 X (m)</label><input id="f-x" type="number" step="0.1" value="4.2"></div>
        <div><label>围栏中心 Y (m)</label><input id="f-y" type="number" step="0.1" value="0.5"></div>
        <div><label>围栏半径 (m，0=删除)</label><input id="f-r" type="number" step="0.1" value="1.3"></div>
      </div>
      <div class="row" style="margin-top:8px;margin-bottom:10px">
        <button class="btn sec sm" onclick="saveFence()">保存 2D 圆形电子围栏</button>
        <div id="fence-pills" class="pills" style="margin:0"></div>
      </div>
      <div style="border-top:1px solid var(--bd);padding-top:10px;margin-top:6px">
        <div class="grid">
          <div><label>规则槽位 ID (1~8)</label><input id="r-id" type="number" min="1" max="8" value="1"></div>
          <div><label>触发条件</label>
            <select id="r-trig">
              <option value="3">进入 2D 电子围栏 (IN_FENCE)</option>
              <option value="4">离开 2D 电子围栏 (OUT_FENCE)</option>
              <option value="1">定位进入房间 (ENTER_ROOM)</option>
              <option value="2">定位离开房间 (LEAVE_ROOM)</option>
              <option value="5">Wi-Fi CSI 有人活动 ≥ 阈值 (CSI_MOTION)</option>
              <option value="6">Wi-Fi CSI 安静无人 ≤ 阈值 (CSI_QUIET)</option>
              <option value="0">删除该槽位规则</option>
            </select>
          </div>
          <div><label>房间名 或 围栏名</label><input id="r-zone" placeholder="例如 StudyDesk 或 Living"></div>
          <div><label>CSI 阈值 (0~100)</label><input id="r-th" type="number" min="0" max="100" value="45"></div>
          <div><label>联动目标 Tag (*=本机+全网)</label><input id="r-tag" value="*"></div>
          <div><label>联动空口指令</label><input id="r-cmd" placeholder="RELAY:1 或 MODE:STROBE"></div>
        </div>
        <div class="row" style="margin-top:10px">
          <button class="btn ok sm" onclick="saveRule()">保存离线自动化规则</button>
          <span class="muted">支持「蓝牙信标围栏 + Wi-Fi CSI 隔空雷达」双模融合触发，直接驱动板载继电器/同拍灯光或按 Tag 群发</span>
        </div>
        <div class="tbl-wrap" style="margin-top:10px">
          <table>
            <thead><tr><th>ID</th><th>触发类型</th><th>监控区域/围栏</th><th>CSI 阈值</th><th>目标 Tag</th><th>执行指令</th><th>状态</th><th>触发次数</th></tr></thead>
            <tbody id="rule-rows"><tr><td colspan="8" class="muted">暂未配置离线规则（点击上方「🎬 一键运行围栏+CSI 联动演示」可自动载入 4 条典型规则）</td></tr></tbody>
          </table>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>动态配置固定定位锚点（Anchor，无需改代码，存 NVS 永久生效）</h2>
      <div class="grid">
        <div><label>锚点设备 MAC（可直接点选上方在线节点）</label><input id="a-mac" class="mono" placeholder="AA:BB:CC:DD:EE:FF"></div>
        <div><label>所在房间名（留空并保存=移除锚点）</label><input id="a-room" placeholder="例如 客厅 / 卧室 / 书房"></div>
        <div><label>X 坐标（米）</label><input id="a-x" type="number" step="0.1" value="0.0"></div>
        <div><label>Y 坐标（米）</label><input id="a-y" type="number" step="0.1" value="0.0"></div>
      </div>
      <div class="row" style="margin-top:10px">
        <button class="btn ok" onclick="saveAnchor()">保存 / 更新锚点</button>
        <span class="muted">任意插电的 ESP32（网关或从机）都可作为锚点，自动通过私有时隙上报监听到的 RSSI</span>
      </div>
      <div id="anchor-pills" class="pills" style="margin-top:10px"></div>
    </div>

    <div class="card">
      <h2>按分组 Tag 批量群控（无需记 Short-ID）</h2>
      <div class="grid">
        <div><label>目标分组 Tag（例如 客厅 或 照明）</label><input id="tg-name" placeholder="客厅"></div>
        <div><label>下发指令载荷</label><input id="tg-pay" placeholder="例如 RELAY:1 或 MODE:STROBE"></div>
      </div>
      <div class="row" style="margin-top:10px">
        <button class="btn" onclick="sendTagCmd()">按 Tag 群发指令</button>
        <span class="muted">可在「集群拓扑」页点击任意节点的「标签」按钮随时动态贴标（逗号分隔多个 Tag）</span>
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
      <h2>MQTT 3.1.1 / MQTTS (TLS) 与 Home Assistant 自动发现</h2>
      <div class="grid">
        <div><label>Broker 主机 / IP</label><input id="m-host" placeholder="192.168.1.100 或 broker.emqx.io"></div>
        <div><label>端口（明文 1883 / TLS 8883）</label><input id="m-port" type="number" value="1883" min="1" max="65535"></div>
        <div><label>用户名（可选）</label><input id="m-user" placeholder="mqtt_user"></div>
        <div><label>密码（可选）</label><input id="m-pass" type="password" placeholder="mqtt_pass"></div>
      </div>
      <div class="row" style="margin-top:10px">
        <label style="display:flex;align-items:center;gap:6px;margin:0;color:var(--tx);cursor:pointer">
          <input id="m-tls" type="checkbox" style="width:auto" onchange="onTlsToggle()"> 启用 MQTTS / TLS 加密（需烧录 esp32_iot_mesh_gw_tls）
        </label>
        <span class="badge" id="tls-state-badge">TLS 检查中...</span>
      </div>
      <div class="row" style="margin-top:12px">
        <button class="btn ok" onclick="saveMqtt()">保存并连接 MQTT</button>
      </div>
      <div id="tls-ca-box" style="margin-top:14px;border-top:1px solid var(--bd);padding-top:10px">
        <label>可选 Root CA 证书 PEM（用于 EMQX Cloud / AWS IoT / 阿里云 IoT 严格验签；留空回退开发模式）</label>
        <textarea id="m-ca" rows="4" class="mono" placeholder="-----BEGIN CERTIFICATE-----&#10;...&#10;-----END CERTIFICATE-----"></textarea>
        <div class="row" style="margin-top:8px">
          <button class="btn sec sm" onclick="saveTlsCa(false)">保存 Root CA 至 NVS</button>
          <button class="btn sec sm" onclick="saveTlsCa(true)">清除 Root CA</button>
        </div>
      </div>
      <div class="muted mono" style="margin-top:12px;line-height:1.7">
        HA 自动发现: homeassistant/sensor/opennear_&lt;MACHEX&gt;/config（MAC 永久锚定 + 别名 + LWT）<br>
        网关在线遗嘱: <span id="mqtt-avty-hint">opennear/gw/&lt;MAC&gt;/status</span>（online / offline）<br>
        状态上报: opennear/node/&lt;MACHEX&gt;/state | 控制: opennear/cmd/&lt;MACHEX&gt; 或 0x&lt;ID&gt;
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
        <button class="btn sec sm" onclick="quickPreset('MODE:SYNC')">同拍 SYNC</button>
        <button class="btn sec sm" onclick="quickPreset('MODE:STROBE')">高频频闪 STROBE</button>
        <button class="btn sec sm" onclick="quickPreset('MODE:PULSE')">心跳同拍 PULSE</button>
        <button class="btn sec sm" onclick="quickPreset('RELAY:1')">继电器 ON</button>
        <button class="btn sec sm" onclick="quickPreset('RELAY:0')">继电器 OFF</button>
        <button class="btn sec sm" onclick="quickPreset('PING')">闪烁定位 PING</button>
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
      <h2>网关管理安全令牌（Admin Token）、集群空口密钥与维护</h2>
      <div class="grid">
        <div><label>新 Admin Token（留空表示清除鉴权）</label><input id="t-new" type="password" placeholder="例如 OpenNear2026"></div>
        <div><label>集群 AES-128 空口密钥（32位 Hex，留空恢复默认）</label><input id="k-net" class="mono" placeholder="例如 00112233445566778899AABBCCDDEEFF"></div>
      </div>
      <div class="row" style="margin-top:10px">
        <button class="btn sec" onclick="updateAdminToken()">保存 / 轮换 Token</button>
        <button class="btn sec" onclick="updateNetKey()">保存空口密钥</button>
        <button class="btn sec" onclick="resetWifi()">清除 Wi-Fi 并重开热点</button>
        <button class="btn danger" onclick="rebootGw()">重启网关</button>
      </div>
      <div class="muted" style="margin-top:8px">集群内所有节点需配置相同 NETKEY 方可解密通信；忘记 Token 可长按板载 BOOT 键 (GPIO0) 5 秒恢复出厂热点</div>
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
  ['topo','loc','wifi','mqtt','ctrl'].forEach(k=>$('p-'+k).classList.toggle('hide',k!==name));
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
    $('b-fw').textContent=s.fw_ver||'v6.6';
    const pills=[];
    if(s.mac)pills.push('<span class="badge mono">MAC: '+esc(s.mac)+(s.ble_mac?' (BLE '+esc(s.ble_mac)+')':'')+'</span>');
    pills.push(s.wifi_ok
      ?'<span class="badge ok">Wi-Fi 已连: '+esc(s.wifi_ssid)+' ('+esc(s.wifi_ip)+(s.mdns?' / '+esc(s.mdns):'')+' CH'+s.wifi_ch+' '+s.wifi_rssi+'dBm)</span>'
      :'<span class="badge warn">Wi-Fi 未连接</span>');
    if(s.ap_active)pills.push('<span class="badge">配网热点: '+esc(s.ap_ssid)+' (192.168.4.1)</span>');
    pills.push(s.mqtt_ok
      ?'<span class="badge ok">'+(s.mqtt_tls?'MQTTS':'MQTT')+' 在线: '+esc(s.mqtt_host)+':'+s.mqtt_port+'</span>'
      :'<span class="badge">'+(s.mqtt_tls?'MQTTS':'MQTT')+': '+(s.mqtt_host?esc(s.mqtt_host)+' (重连中)':'未配置')+'</span>');
    pills.push(s.tls_compiled
      ?'<span class="badge ok">TLS 模块已启用'+(s.tls_ca_set?' (CA验签)':' (开发模式)')+'</span>'
      :'<span class="badge">TLS 未编译(精简)</span>');
    pills.push(s.ble_ok?'<span class="badge ok">BLE 已连接</span>':'<span class="badge">BLE 待机</span>');
    gBleScan=!!s.ble_scan;
    if($('btn-blescan'))$('btn-blescan').textContent='BLE 信标扫描: '+(gBleScan?'开':'关');
    if(gBleScan)pills.push('<span class="badge purp">iBeacon/Eddystone 扫描中</span>');
    gLocDemo=!!s.loc_demo;
    if($('btn-locdemo')){
      $('btn-locdemo').textContent=gLocDemo?'⏹ 停止围栏+CSI 演示':'🎬 一键运行围栏+CSI 联动演示';
      $('btn-locdemo').className='btn sm '+(gLocDemo?'danger':'ok');
    }
    if(gLocDemo)pills.push('<span class="badge ok">🎬 围栏+CSI 联动演示运行中</span>');
    if(typeof s.free_heap==='number'){
      const fh=(s.free_heap/1024).toFixed(1),mh=((s.min_heap||0)/1024).toFixed(1);
      pills.push('<span class="badge mono '+(s.free_heap<28000?'warn':'')+'">Heap '+fh+'K (min '+mh+'K)</span>');
    }
    if(typeof s.tx_ok==='number'){
      pills.push('<span class="badge mono '+(s.rx_drops>0?'warn':'')+'">TX '+s.tx_ok+'/'+(s.tx_fail||0)+' · RX丢 '+((s.rx_drops)||0)+'</span>');
    }
    pills.push(s.token_set?'<span class="badge ok">Token 保护已启用</span>':'<span class="badge warn">未设 Token</span>');
    pills.push(s.netkey_custom?'<span class="badge ok">独立集群密钥</span>':'<span class="badge">默认演示密钥</span>');
    pills.push('<span class="badge">运行 '+s.uptime_s+'s</span>');
    $('status-pills').innerHTML=pills.join('');
    if(s.mqtt_avty&&$('mqtt-avty-hint'))$('mqtt-avty-hint').textContent=s.mqtt_avty;
    const tb=$('tls-state-badge');
    if(tb){
      tb.className='badge '+(s.tls_compiled?'ok':'warn');
      tb.textContent=s.tls_compiled?(s.tls_ca_set?'TLS已编译 · Root CA 严格验签':'TLS已编译 · 无CA开发模式'):'当前固件未编译 TLS 模块';
    }
    if(!$('w-ssid').value&&s.wifi_ssid)$('w-ssid').value=s.wifi_ssid;
    if(!$('w-url').value&&s.cloud_url)$('w-url').value=s.cloud_url;
    if(!$('m-host').value&&s.mqtt_host){
      $('m-host').value=s.mqtt_host;
      $('m-tls').checked=!!s.mqtt_tls;
      if(s.mqtt_port)$('m-port').value=String(s.mqtt_port);
    }
  }catch(e){}
}
let gAliases={};
let gTags={};
async function fetchAliases(){
  try{
    const [r1,r2]=await Promise.all([fetch('/api/aliases'),fetch('/api/tags')]);
    gAliases=await r1.json()||{};
    gTags=await r2.json()||{};
  }catch(e){}
}
async function renameNode(mac){
  if(!mac||mac==='--'||mac==='00:00:00:00:00:00'){toast('该节点暂未上报硬件 MAC',true);return;}
  const cur=gAliases[mac]||'';
  const name=prompt('为设备 '+mac+' 设置别名（留空清除）：',cur);
  if(name===null)return;
  if(await postCmd('ALIAS:'+mac+'|'+name.trim())){
    toast(name.trim()?'已保存别名并同步 Home Assistant':'已清除别名');
    await fetchAliases();
    fetchData();
  }
}
async function editTags(mac){
  if(!mac||mac==='--'||mac==='00:00:00:00:00:00'){toast('该节点暂未上报硬件 MAC',true);return;}
  const cur=gTags[mac]||'';
  const tags=prompt('为设备 '+mac+' 设置分组标签（多个用英文逗号分隔，如 客厅,照明；留空清除）：',cur);
  if(tags===null)return;
  if(await postCmd('TAG:'+mac+'|'+tags.trim())){
    toast(tags.trim()?'已保存节点标签':'已清除节点标签');
    await fetchAliases();
    fetchData();
  }
}
function fillAnchorForm(mac,room,xCm,yCm){
  $('a-mac').value=mac||'';
  if(room!==undefined)$('a-room').value=room;
  if(xCm!==undefined)$('a-x').value=(xCm/100).toFixed(2);
  if(yCm!==undefined)$('a-y').value=(yCm/100).toFixed(2);
  switchTab('loc');
  $('a-room').focus();
}
async function saveAnchor(){
  const mac=$('a-mac').value.trim();
  const room=$('a-room').value.trim();
  const xCm=Math.round((parseFloat($('a-x').value)||0)*100);
  const yCm=Math.round((parseFloat($('a-y').value)||0)*100);
  if(!mac){toast('请填写锚点 MAC 地址',true);return;}
  if(await postCmd('ANCHOR:'+mac+'|'+room+'|'+xCm+'|'+yCm)){
    toast(room?('已保存锚点 '+room+' ('+(xCm/100).toFixed(1)+'m, '+(yCm/100).toFixed(1)+'m)'):'已移除该锚点');
    setTimeout(fetchLocate,300);
  }
}
async function sendTagCmd(){
  const tg=$('tg-name').value.trim();
  const pay=$('tg-pay').value.trim();
  if(!tg||!pay){toast('请填写目标 Tag 与指令载荷',true);return;}
  if(await postCmd('CMD_TAG:'+tg+'|'+pay)){
    toast('已向标签 ['+tg+'] 下发群控指令: '+pay);
  }
}
let gBleScan=true;
let gLocDemo=false;
async function toggleBleScan(){
  const next=gBleScan?0:1;
  if(await postCmd('BLESCAN:'+next)){
    gBleScan=!!next;
    if($('btn-blescan'))$('btn-blescan').textContent='BLE 信标扫描: '+(gBleScan?'开':'关');
    toast(gBleScan?'已开启 BLE iBeacon/Eddystone 被动扫描':'已暂停 BLE 信标扫描');
    setTimeout(fetchStatus,300);
  }
}
async function toggleLocDemo(){
  const next=gLocDemo?0:1;
  if(await postCmd('LOCDEMO:'+next)){
    gLocDemo=!!next;
    if($('btn-locdemo')){
      $('btn-locdemo').textContent=gLocDemo?'⏹ 停止围栏+CSI 演示':'🎬 一键运行围栏+CSI 联动演示';
      $('btn-locdemo').className='btn sm '+(gLocDemo?'danger':'ok');
    }
    toast(gLocDemo?'🎬 已载入 4 房间 + 2 围栏 + Wi-Fi CSI 雷达自动联动演示！':'已停止演示并恢复 NVS 配置');
    setTimeout(()=>{fetchStatus();fetchLocate();},250);
  }
}
async function stepLocDemo(){
  if(await postCmd('LOCDEMO:2')){
    toast('已推进下一幕空间轨迹与 CSI 波形，并即时评估离线规则');
    setTimeout(fetchLocate,200);
  }
}
function fillFenceForm(nm,xCm,yCm,rCm){
  $('f-name').value=nm||'';
  if(xCm!==undefined)$('f-x').value=(xCm/100).toFixed(2);
  if(yCm!==undefined)$('f-y').value=(yCm/100).toFixed(2);
  if(rCm!==undefined)$('f-r').value=(rCm/100).toFixed(2);
}
async function saveFence(){
  const nm=$('f-name').value.trim();
  const xCm=Math.round((parseFloat($('f-x').value)||0)*100);
  const yCm=Math.round((parseFloat($('f-y').value)||0)*100);
  const rCm=Math.max(0,Math.round((parseFloat($('f-r').value)||0)*100));
  if(!nm){toast('请填写电子围栏名称',true);return;}
  if(await postCmd('FENCE:'+nm+'|'+xCm+'|'+yCm+'|'+rCm)){
    toast(rCm>0?('已保存 2D 电子围栏 '+nm+' (R='+(rCm/100).toFixed(1)+'m)'):('已移除电子围栏 '+nm));
    setTimeout(fetchLocate,250);
  }
}
async function saveRule(){
  const id=parseInt($('r-id').value,10)||1;
  const trig=parseInt($('r-trig').value,10)||0;
  const zone=$('r-zone').value.trim();
  const th=parseInt($('r-th').value,10)||40;
  const tag=$('r-tag').value.trim()||'*';
  const cmd=$('r-cmd').value.trim();
  if(trig>0&&(!zone||!cmd)){toast('请填写监控区域/围栏名称与执行指令',true);return;}
  if(await postCmd('RULE:'+id+'|'+trig+'|'+(zone||'None')+'|'+th+'|'+tag+'|'+(cmd||'NONE'))){
    toast(trig>0?('已保存离线自动化规则 R#'+id):('已删除规则 R#'+id));
    setTimeout(fetchLocate,250);
  }
}
function renderLocateSvg(d){
  const svg=$('loc-svg');
  if(!svg)return;
  const ancs=(d&&Array.isArray(d.anchors))?d.anchors:[];
  const tgts=(d&&Array.isArray(d.targets))?d.targets:[];
  const fns=(d&&Array.isArray(d.fences))?d.fences:[];
  const rls=(d&&Array.isArray(d.rules))?d.rules:[];
  const bleCnt=tgts.filter(t=>t.src&&t.src!=='opennear').length;
  $('loc-meta').textContent='锚点='+ancs.length+' | 目标='+tgts.length+' (BLE '+bleCnt+') | 围栏='+fns.length+' | 规则='+rls.length;
  const eb=$('loc-evt-banner');
  if(eb){
    if(d&&d.last_evt){
      eb.classList.remove('hide');
      $('loc-evt-txt').textContent=d.last_evt;
    }else{
      eb.classList.add('hide');
    }
  }
  const ap=$('anchor-pills');
  if(ap){
    ap.innerHTML=ancs.length?ancs.map(a=>
      '<span class="badge ok" style="cursor:pointer" onclick="fillAnchorForm(\''+esc(a.mac)+'\',\''+esc(a.room)+'\','+a.x_cm+','+a.y_cm+')">'
      +'⚓ '+esc(a.room)+' ('+(a.x_cm/100).toFixed(1)+'m,'+(a.y_cm/100).toFixed(1)+'m) · CSI '+(a.csi||0)+'</span>'
    ).join(''):'<span class="muted">暂未配置固定锚点（可在拓扑表中点击节点「设锚点」快速填入 MAC）</span>';
  }
  const fp=$('fence-pills');
  if(fp){
    fp.innerHTML=fns.length?fns.map(f=>
      '<span class="badge '+(f.in?'err':'')+'" style="cursor:pointer" onclick="fillFenceForm(\''+esc(f.name)+'\','+f.x_cm+','+f.y_cm+','+f.r_cm+')">'
      +'⭕ '+esc(f.name)+' R='+(f.r_cm/100).toFixed(1)+'m '+(f.in?'[有人在围栏内]':'[空闲]')+'</span>'
    ).join(''):'<span class="muted">暂无电子围栏</span>';
  }
  const cbox=$('csi-cards');
  if(cbox){
    const stNames=['安静无人','微动静坐','有人走动'];
    const stCls=['','warn','err'];
    cbox.innerHTML=ancs.length?ancs.map(a=>{
      const sc=Number(a.csi)||0,st=Number(a.csi_st)||0;
      const bins=Array.isArray(a.bins)?a.bins:[];
      const bars=bins.map(v=>{
        const h=Math.max(4,Math.min(34,Math.round((Number(v)||0)*0.34)));
        const bcol=st===2?'#f43f5e':(st===1?'#f59e0b':'#38bdf8');
        return '<span style="display:inline-block;width:6px;height:'+h+'px;background:'+bcol+';border-radius:2px"></span>';
      }).join('');
      return '<div style="background:#030712;border:1px solid var(--bd);border-radius:10px;padding:10px 12px">'
        +'<div class="row" style="justify-content:space-between;margin-bottom:6px">'
        +'<b>📡 '+esc(a.room)+'</b>'
        +'<span class="badge '+stCls[st]+'">'+(stNames[st]||'安静无人')+' · CSI '+sc+'</span></div>'
        +'<div style="height:36px;display:flex;align-items:flex-end;gap:3px;padding:2px 0">'+(bars||'<span class="muted">等待 CSI 采样...</span>')+'</div>'
        +'</div>';
    }).join(''):'<div class="muted">配置房间锚点或开启一键演示后，此处实时显示各房间 Wi-Fi CSI 16 频段子载波瀑布与人体活动评分</div>';
  }
  const rtb=$('rule-rows');
  if(rtb){
    const tNames=['--','进入房间','离开房间','进入围栏','离开围栏','CSI 有人活动 ≥','CSI 安静无人 ≤'];
    rtb.innerHTML=rls.length?rls.map(r=>
      '<tr><td class="mono"><b>R#'+r.id+'</b></td>'
      +'<td><span class="badge">'+(tNames[r.trig]||r.trig)+'</span></td>'
      +'<td><b>'+esc(r.zone)+'</b></td>'
      +'<td class="mono">'+(r.trig>=5?r.th:'--')+'</td>'
      +'<td><span class="badge purp">'+esc(r.tag)+'</span></td>'
      +'<td class="mono">'+esc(r.cmd)+'</td>'
      +'<td><span class="badge '+(r.on?'ok':'')+'">'+(r.on?'⚡ 触发中':'监听中')+'</span></td>'
      +'<td class="mono">'+r.fires+' 次</td></tr>'
    ).join(''):'<tr><td colspan="8" class="muted">暂未配置离线规则（点击上方「🎬 一键运行围栏+CSI 联动演示」可自动载入 4 条典型规则）</td></tr>';
  }
  const tb=$('loc-rows');
  if(tb){
    tb.innerHTML=tgts.length?tgts.map(t=>{
      const nm=(t.mac&&gAliases[t.mac])?gAliases[t.mac]:(t.label||t.id);
      const src=t.src||'opennear';
      const scls=src==='ibeacon'?'purp':(src==='eddystone'?'warn':'');
      const slbl=src==='ibeacon'?'iBeacon':(src==='eddystone'?'Eddystone':(src==='ble'?'BLE':'OpenNear'));
      const txTxt=(t.tx1m&&t.tx1m<0)?('/'+t.tx1m):'';
      return '<tr><td><b>'+esc(nm)+'</b></td>'
        +'<td><span class="badge '+scls+'">'+slbl+'</span></td>'
        +'<td class="mono muted">'+esc(t.mac)+'</td>'
        +'<td><span class="badge ok">'+esc(t.room)+'</span></td>'
        +'<td class="mono">('+(t.x_cm/100).toFixed(2)+', '+(t.y_cm/100).toFixed(2)+')</td>'
        +'<td class="mono">'+t.rssi+txTxt+' dBm</td><td>'+t.anchors+'</td>'
        +'<td class="mono">±'+(t.err_cm/100).toFixed(2)+' m</td>'
        +'<td><button class="btn sec sm" onclick="renameNode(\''+esc(t.mac)+'\')">改名</button></td></tr>';
    }).join(''):'<tr><td colspan="9" class="muted">暂无实时定位数据（点击右上角「🎬 一键运行围栏+CSI 联动演示」或添加至少 1 个在线锚点）</td></tr>';
  }
  if(!ancs.length&&!tgts.length&&!fns.length){
    svg.innerHTML='<text x="440" y="135" text-anchor="middle" fill="#64748b" font-size="13">点击右上角「🎬 一键运行围栏+CSI 联动演示」，或配置 1~4 个固定锚点后实时显示 2D 平面图</text>';
    return;
  }
  const pts=ancs.concat(tgts);
  let minX=-60,maxX=560,minY=-60,maxY=410;
  pts.forEach(p=>{
    if(p.x_cm-110<minX)minX=p.x_cm-110;
    if(p.x_cm+110>maxX)maxX=p.x_cm+110;
    if(p.y_cm-110<minY)minY=p.y_cm-110;
    if(p.y_cm+110>maxY)maxY=p.y_cm+110;
  });
  fns.forEach(f=>{
    if(f.x_cm-f.r_cm-40<minX)minX=f.x_cm-f.r_cm-40;
    if(f.x_cm+f.r_cm+40>maxX)maxX=f.x_cm+f.r_cm+40;
    if(f.y_cm-f.r_cm-40<minY)minY=f.y_cm-f.r_cm-40;
    if(f.y_cm+f.r_cm+40>maxY)maxY=f.y_cm+f.r_cm+40;
  });
  const spanX=Math.max(200,maxX-minX),spanY=Math.max(200,maxY-minY);
  const pad=42,W=880,H=270;
  const sx=x=>Math.round(pad+((x-minX)/spanX)*(W-pad*2));
  const sy=y=>Math.round(H-pad-((y-minY)/spanY)*(H-pad*2));
  const sr=r=>Math.max(14,Math.round((r/spanX)*(W-pad*2)));
  const out=[];
  for(let gx=0;gx<=4;++gx){
    const x=Math.round(pad+(gx/4)*(W-pad*2));
    out.push('<line x1="'+x+'" y1="16" x2="'+x+'" y2="'+(H-22)+'" stroke="#1e293b" stroke-width="1"/>');
  }
  for(let gy=0;gy<=3;++gy){
    const y=Math.round(16+(gy/3)*(H-38));
    out.push('<line x1="'+pad+'" y1="'+y+'" x2="'+(W-pad)+'" y2="'+y+'" stroke="#1e293b" stroke-width="1"/>');
  }
  fns.forEach(f=>{
    const x=sx(f.x_cm),y=sy(f.y_cm),r=sr(f.r_cm);
    const col=f.in?'#f43f5e':'#38bdf8';
    out.push('<circle cx="'+x+'" cy="'+y+'" r="'+r+'" fill="'+col+'" fill-opacity="'+(f.in?'0.18':'0.07')+'" stroke="'+col+'" stroke-width="'+(f.in?'2':'1.4')+'" stroke-dasharray="6,4"/>');
    out.push('<text x="'+x+'" y="'+(y-r+14)+'" text-anchor="middle" fill="'+col+'" font-size="10.5" font-weight="600">⭕ '+esc(f.name)+(f.in?' [触发]':'')+'</text>');
  });
  ancs.forEach(a=>{
    const x=sx(a.x_cm),y=sy(a.y_cm);
    const st=Number(a.csi_st)||0,sc=Number(a.csi)||0;
    if(st>0){
      const wcol=st===2?'#f43f5e':'#f59e0b';
      out.push('<circle cx="'+x+'" cy="'+y+'" r="'+(st===2?28:20)+'" fill="'+wcol+'" fill-opacity="0.16" stroke="'+wcol+'" stroke-opacity="0.7" stroke-dasharray="3,3"/>');
    }
    out.push('<polygon points="'+x+','+(y-10)+' '+(x+9)+','+(y+8)+' '+(x-9)+','+(y+8)+'" fill="#0284c7" stroke="#38bdf8" stroke-width="1.5"/>');
    out.push('<text x="'+x+'" y="'+(y+22)+'" text-anchor="middle" fill="#7dd3fc" font-size="11" font-weight="600">'+esc(a.room)+(sc>0?' (CSI '+sc+')':'')+'</text>');
  });
  tgts.forEach(t=>{
    const x=sx(t.x_cm),y=sy(t.y_cm);
    const nm=(t.mac&&gAliases[t.mac])?gAliases[t.mac]:(t.label||t.id);
    const src=t.src||'opennear';
    const col=src==='ibeacon'?'#a855f7':(src==='eddystone'?'#f59e0b':'#22c55e');
    out.push('<circle cx="'+x+'" cy="'+y+'" r="16" fill="'+col+'" fill-opacity="0.18" stroke="'+col+'" stroke-opacity="0.6" stroke-dasharray="3,2"/>');
    if(src==='opennear'){
      out.push('<circle cx="'+x+'" cy="'+y+'" r="6" fill="'+col+'" stroke="#fff" stroke-width="1.5"/>');
    }else{
      out.push('<polygon points="'+x+','+(y-7)+' '+(x+7)+','+y+' '+x+','+(y+7)+' '+(x-7)+','+y+'" fill="'+col+'" stroke="#fff" stroke-width="1.4"/>');
    }
    out.push('<text x="'+x+'" y="'+(y-11)+'" text-anchor="middle" fill="#f8fafc" font-size="11" font-weight="600">'+esc(nm)+' ['+esc(t.room)+']</text>');
  });
  svg.innerHTML=out.join('');
}
async function fetchLocate(){
  try{const r=await fetch('/api/locate');renderLocateSvg(await r.json());}catch(e){}
}
function renderTopoSvg(d){
  const svg=$('topo-svg');
  if(!svg)return;
  const nodes=(d&&Array.isArray(d.nodes))?d.nodes:[];
  if(!nodes.length){
    svg.setAttribute('viewBox','0 0 880 90');svg.setAttribute('height','90');
    svg.innerHTML='<text x="440" y="50" text-anchor="middle" fill="#64748b" font-size="13">等待节点入网...</text>';
    return;
  }
  const byHop={};
  let maxHop=0;
  nodes.forEach(n=>{
    const h=Math.max(0,Math.min(6,Number(n.hop)||0));
    if(h>maxHop)maxHop=h;
    (byHop[h]=byHop[h]||[]).push(n);
  });
  const W=880,rowH=86,H=Math.max(120,(maxHop+1)*rowH+32);
  svg.setAttribute('viewBox','0 0 '+W+' '+H);svg.setAttribute('height',String(H));
  const pos={};
  Object.keys(byHop).forEach(hk=>{
    const h=Number(hk),arr=byHop[h],step=W/(arr.length+1);
    arr.forEach((n,idx)=>{pos[n.id]={x:Math.round(step*(idx+1)),y:44+h*rowH,n};});
  });
  const parts=[];
  nodes.forEach(n=>{
    const c=pos[n.id],p=pos[n.parent];
    if(c&&p&&n.id!==n.parent){
      const col=n.hop>=3?'#a855f7':(n.hop===2?'#f59e0b':'#38bdf8');
      const dash=n.hop>=2?' stroke-dasharray="5,4"':'';
      const my=Math.round((p.y+c.y)/2);
      parts.push('<path d="M'+p.x+' '+(p.y+22)+' C'+p.x+' '+my+','+c.x+' '+my+','+c.x+' '+(c.y-22)+'" fill="none" stroke="'+col+'" stroke-width="2" stroke-opacity="0.7"'+dash+'/>');
    }
  });
  nodes.forEach(n=>{
    const p=pos[n.id];if(!p)return;
    const isBk=(d.backup&&n.id===d.backup);
    const fill=n.role==='G'?'#064e3b':(n.role==='SUB_G'?'#451a03':'#0f172a');
    const stroke=n.role==='G'?'#22c55e':(n.role==='SUB_G'?'#f59e0b':(isBk?'#38bdf8':'#475569'));
    const lbl=(n.mac&&gAliases[n.mac])?gAliases[n.mac]:(n.role+' '+n.id);
    const shortLbl=lbl.length>14?lbl.slice(0,13)+'…':lbl;
    const sub=n.id+(isBk?' [BK]':'')+' · '+(n.skew_ns||0)+'ns';
    parts.push('<g style="cursor:pointer" onclick="quickCtrl(\''+esc(n.id)+'\')">'
      +'<rect x="'+(p.x-58)+'" y="'+(p.y-22)+'" width="116" height="44" rx="9" fill="'+fill+'" stroke="'+stroke+'" stroke-width="1.8"/>'
      +'<text x="'+p.x+'" y="'+(p.y-4)+'" text-anchor="middle" fill="#f8fafc" font-size="11.5" font-weight="600">'+esc(shortLbl)+'</text>'
      +'<text x="'+p.x+'" y="'+(p.y+13)+'" text-anchor="middle" fill="#94a3b8" font-size="10" font-family="monospace">'+esc(sub)+'</text>'
      +'</g>');
  });
  svg.innerHTML=parts.join('');
}
async function fetchData(){
  try{
    const r=await fetch('/api/data');
    const d=await r.json();
    $('topo-meta').textContent='SF='+d.sf+' | G='+d.g+' | Backup='+d.backup+' | Sub-G='+d.sub_g+' | 节点='+d.count;
    renderTopoSvg(d);
    const tb=$('node-rows');
    if(!d.nodes||!d.nodes.length){
      tb.innerHTML='<tr><td colspan="11" class="muted">暂无在线节点</td></tr>';
      return;
    }
    const otaNames=['IDLE','RX','VERIFY','READY','COMMIT','APPLIED','FAIL'];
    tb.innerHTML=d.nodes.map(n=>{
      const asc=hexToAscii(n.data);
      const rCls=n.role==='G'?'ok':(n.role==='SUB_G'?'warn':'');
      const al=n.mac&&gAliases[n.mac]?('<b>'+esc(gAliases[n.mac])+'</b> '):'';
      const tg=n.mac&&gTags[n.mac]?('<br><span class="badge" style="font-size:10.5px">'+esc(gTags[n.mac])+'</span>'):'';
      return '<tr>'
        +'<td class="mono"><b>'+esc(n.id)+'</b></td>'
        +'<td class="mono">'+al+'<span class="muted">'+esc(n.mac||'--')+'</span>'+tg+'<br>'
          +'<button class="btn sec sm" style="padding:1px 6px;margin-top:3px" onclick="renameNode(\''+esc(n.mac||'')+'\')">改名</button> '
          +'<button class="btn sec sm" style="padding:1px 6px;margin-top:3px" onclick="editTags(\''+esc(n.mac||'')+'\')">标签</button> '
          +'<button class="btn sec sm" style="padding:1px 6px;margin-top:3px" onclick="fillAnchorForm(\''+esc(n.mac||'')+'\')">设锚点</button></td>'
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
  btn.disabled=true;btn.textContent='异步扫描中...';
  try{
    let list=[];
    for(let tries=0;tries<20;++tries){
      const r=await fetch('/api/scan');
      const d=await r.json();
      if(Array.isArray(d)){list=d;break;}
      if(!d.scanning){list=d.aps||[];break;}
      await new Promise(res=>setTimeout(res,400));
    }
    gScanAps=list;
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
function onTlsToggle(){
  const tls=$('m-tls').checked;
  const p=$('m-port').value.trim();
  if(tls&&(p===''||p==='1883'))$('m-port').value='8883';
  if(!tls&&p==='8883')$('m-port').value='1883';
}
async function saveMqtt(){
  const host=$('m-host').value.trim();
  const tls=$('m-tls').checked;
  const port=$('m-port').value.trim()||(tls?'8883':'1883');
  const user=$('m-user').value.trim();
  const pass=$('m-pass').value;
  if(!host){toast('请填写 MQTT Broker 地址',true);return;}
  let cmd=(tls?'MQTTS:':'MQTT:')+host+'|'+port;
  if(user||!tls)cmd+='|'+user+'|'+pass+'|'+(tls?'1':'0');
  if(await postCmd(cmd)){
    toast((tls?'MQTTS (TLS)':'MQTT')+' 配置已保存，正在连接 Broker...');
    setTimeout(fetchStatus,2500);
  }
}
async function saveTlsCa(clearOnly){
  const pem=clearOnly?'':$('m-ca').value.trim();
  const hdr={'Content-Type':'text/plain'};
  const tok=$('g-token').value.trim();
  if(tok)hdr['X-OpenNear-Token']=tok;
  const r=await fetch('/api/tls_ca',{method:'POST',headers:hdr,body:pem});
  const j=await r.json().catch(()=>({ok:false}));
  if(r.status===401){toast('鉴权失败：请在右上角填写 Admin Token',true);return;}
  if(r.status===501){toast('当前固件未启用 OPENNEAR_ENABLE_TLS=1，请烧录 esp32_iot_mesh_gw_tls',true);return;}
  if(!r.ok||!j.ok){toast('Root CA 格式无效（需包含 BEGIN CERTIFICATE）',true);return;}
  if(clearOnly)$('m-ca').value='';
  toast(clearOnly?'Root CA 已清除':'Root CA 证书已保存至 NVS，TLS 严格验签已启用');
  fetchStatus();
}
async function sendNodeCmd(){
  const id=$('c-id').value.trim().replace(/^0x/i,'')||'FFF';
  const pay=$('c-pay').value;
  if(!pay){toast('请输入控制载荷',true);return;}
  if(await postCmd('CMD:'+id+'|'+pay)){
    toast('已下发空口指令至 0x'+id.toUpperCase());
  }
}
async function quickPreset(pay){
  const id=$('c-id').value.trim().replace(/^0x/i,'')||'FFF';
  $('c-pay').value=pay;
  if(await postCmd('CMD:'+id+'|'+pay)){
    toast('已向 0x'+id.toUpperCase()+' 下发演示模式: '+pay);
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
async function updateNetKey(){
  const k=$('k-net').value.trim();
  if(k&&!/^[0-9a-fA-F]{32}$/.test(k)){toast('空口密钥需为 32 位十六进制字符（或留空恢复默认）',true);return;}
  if(await postCmd('NETKEY:'+k)){
    $('k-net').value='';
    toast(k?'独立集群密钥已保存并生效':'已恢复默认演示密钥');
    fetchStatus();
  }
}
async function resetWifi(){
  if(!confirm('确认清除已保存的 Wi-Fi 凭据并重新开启 OpenNear-Setup 配网热点？'))return;
  if(await postCmd('RESET_WIFI')){
    $('w-ssid').value='';$('w-pass').value='';
    toast('已清除 Wi-Fi，请重连 OpenNear-Setup 热点打开 http://192.168.4.1');
  }
}
async function rebootGw(){
  if(!confirm('确认重启 OpenNear 网关？'))return;
  if(await postCmd('REBOOT')){
    toast('网关正在重启，约 3 秒后自动恢复...');
  }
}
function refreshAll(){fetchStatus();fetchAliases().then(()=>{fetchData();fetchLocate();});}
loadLocalToken();
refreshAll();
setInterval(refreshAll,3000);
</script>
</body>
</html>)ONPORTAL";

#endif /* OPENNEAR_ESP32_WEB_PORTAL_H */
