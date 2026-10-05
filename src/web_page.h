// PID controller web dashboard, served by the ESP32
#pragma once
static const char WEB_PAGE[] = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>PID Controller</title>
<style>
:root{--bg:#f1f2ee;--panel:#fff;--line:#dadcd4;--ink:#1d2120;--mut:#646b68;--sp:#8a8f8c;--y:#1f7a8c;--u:#d17a22;--good:#2e7d4f;--bad:#b23a2e;--btn:#f6f7f3;color-scheme:light}
:root[data-theme="dark"]{--bg:#111416;--panel:#191d20;--line:#2a3034;--ink:#e5e9e7;--mut:#8d9693;--sp:#9aa19e;--y:#4cc0d6;--u:#f0a04b;--good:#5cc489;--bad:#ee6a5c;--btn:#20262a;color-scheme:dark}
@media(prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#111416;--panel:#191d20;--line:#2a3034;--ink:#e5e9e7;--mut:#8d9693;--sp:#9aa19e;--y:#4cc0d6;--u:#f0a04b;--good:#5cc489;--bad:#ee6a5c;--btn:#20262a;color-scheme:dark}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif;padding:16px}
main{max-width:1060px;margin:0 auto;display:grid;gap:14px}
header{display:flex;justify-content:space-between;align-items:center;gap:10px;flex-wrap:wrap}h1{font-size:20px;margin:0}
h2{font-size:12px;margin:0 0 10px;color:var(--mut);text-transform:uppercase;letter-spacing:.07em;font-weight:600}
.mono{font-family:ui-monospace,"SFMono-Regular",Consolas,monospace;font-variant-numeric:tabular-nums}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:14px;min-width:0}
.kpis{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px}@media(max-width:700px){.kpis{grid-template-columns:repeat(2,minmax(0,1fr))}}
.k b{display:block;font-size:26px;font-weight:650;white-space:nowrap}#perf{font-size:20px;line-height:1.5;white-space:normal}.k small{color:var(--mut)}
.cols{display:grid;grid-template-columns:minmax(0,2fr) minmax(0,1fr);gap:14px}@media(max-width:820px){.cols{grid-template-columns:1fr}}
canvas{width:100%;height:300px;display:block}
.legend{display:flex;gap:16px;flex-wrap:wrap;color:var(--mut);font-size:12px;margin-top:6px}.legend i{display:inline-block;width:16px;height:3px;vertical-align:middle;margin-right:5px}
.btns{display:flex;flex-wrap:wrap;gap:6px}
button{font:inherit;background:var(--btn);color:var(--ink);border:1px solid var(--line);border-radius:7px;padding:6px 11px;cursor:pointer}
button:hover{border-color:var(--y)}button.main{background:var(--y);color:var(--panel);border-color:var(--y)}button:focus-visible,input:focus-visible{outline:2px solid var(--y)}
label{display:block;color:var(--mut);font-size:12px;margin:12px 0 4px}input[type=range]{width:100%;accent-color:var(--y)}
.row{display:flex;gap:8px;align-items:center}.row input[type=number]{width:110px;font:inherit;padding:5px 7px;border:1px solid var(--line);border-radius:7px;background:var(--btn);color:var(--ink)}
.prog{height:8px;background:var(--line);border-radius:99px;margin-top:8px;overflow:hidden}.prog span{display:block;height:100%;background:var(--u);width:0}
.tag{font-size:12px;padding:2px 9px;border-radius:99px;background:var(--line);color:var(--mut)}.tag.on{background:var(--u);color:var(--panel)}
dl{display:grid;grid-template-columns:auto 1fr;gap:4px 12px;margin:0;font-size:13px}dt{color:var(--mut)}dd{margin:0}
.good{color:var(--good)}.bad{color:var(--bad)}
</style></head><body><main>
<header><h1>PID Motor Controller <span class="tag" id="mode">PID</span></h1>
<span class="row"><span class="mono" id="conn" style="color:var(--mut)">connecting…</span><button id="theme" aria-label="Toggle theme">◐</button></span></header>

<section class="kpis">
 <div class="panel k"><small>Setpoint</small><b class="mono" id="sp">–</b><small>rpm</small></div>
 <div class="panel k"><small>Measured speed</small><b class="mono" id="y" style="color:var(--y)">–</b><small>rpm</small></div>
 <div class="panel k"><small>PWM output</small><b class="mono" id="u" style="color:var(--u)">–</b><small>%</small></div>
 <div class="panel k"><small>Last step</small><b class="mono" id="perf">–</b><small id="perf2">overshoot · settling time</small></div>
</section>

<section class="cols">
 <div class="panel"><h2>Live response (last 10 seconds)</h2>
  <canvas id="chart" role="img" aria-label="Chart of setpoint, speed and PWM"></canvas>
  <div class="legend"><span><i style="background:var(--sp)"></i>setpoint</span><span><i style="background:var(--y)"></i>speed</span><span><i style="background:var(--u)"></i>PWM (right axis)</span></div>
 </div>
 <div style="display:grid;gap:14px;align-content:start">
  <div class="panel"><h2>Controls</h2>
   <label for="spr">Setpoint from this page <span class="mono" id="sprv">potentiometer</span></label>
   <input id="spr" type="range" min="0" max="3000" step="50" value="1500">
   <div class="btns" style="margin-top:8px"><button onclick="cmd('sp','-1')">Setpoint: potentiometer</button></div>
   <div class="btns" style="margin-top:12px">
    <button class="main" onclick="cmd('tune','')">Auto-tune</button>
    <button onclick="cmd('brake','')" id="brk">Brake</button>
    <button onclick="cmd('aw','')" id="aw">Anti-windup</button>
    <button onclick="cmd('reset','')">Default gains</button>
   </div>
   <div class="prog" id="progw" hidden><span id="prog"></span></div>
  </div>
  <div class="panel"><h2>Controller gains</h2>
   <div class="row"><span style="width:28px">Kp</span><input type="number" id="kp" step="0.0001"><button onclick="setG('kp')">OK</button></div>
   <div class="row" style="margin-top:6px"><span style="width:28px">Ki</span><input type="number" id="ki" step="0.0001"><button onclick="setG('ki')">OK</button></div>
   <h2 style="margin-top:14px">Identified model</h2>
   <dl class="mono" id="model"><dt>—</dt><dd>run auto-tune</dd></dl>
  </div>
 </div>
</section>
</main><script>
const $=id=>document.getElementById(id);
(function(){const r=document.documentElement;let t=null;try{t=localStorage.getItem('theme')}catch(e){}if(t)r.dataset.theme=t;
 const cur=()=>r.dataset.theme||(matchMedia('(prefers-color-scheme: dark)').matches?'dark':'light');
 const lab=()=>{$('theme').textContent=cur()=='dark'?'☀':'☾'};
 $('theme').onclick=()=>{r.dataset.theme=cur()=='dark'?'light':'dark';try{localStorage.setItem('theme',r.dataset.theme)}catch(e){}lab();draw()};lab()})();
let since=0,pts=[],editing={};
async function cmd(c,v){try{await fetch('/api/cmd?c='+c+'&v='+encodeURIComponent(v))}catch(e){}}
function setG(k){cmd(k,$(k).value);editing[k]=false}
['kp','ki'].forEach(k=>{$(k).onfocus=()=>editing[k]=true});
$('spr').oninput=e=>{$('sprv').textContent=e.target.value+' rpm';cmd('sp',e.target.value)};
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
function draw(){const c=$('chart'),dpr=devicePixelRatio||1,W=c.clientWidth,H=c.clientHeight;c.width=W*dpr;c.height=H*dpr;
 const x=c.getContext('2d');x.setTransform(dpr,0,0,dpr,0,0);x.clearRect(0,0,W,H);
 const L=46,R=40,T=8,B=22,pw=W-L-R,ph=H-T-B,tN=pts.length?pts[pts.length-1].t:0,t0=tN-10000;
 const X=t=>L+(t-t0)/10000*pw,Y=v=>T+ph-v/3300*ph,YU=v=>T+ph-v/100*ph;
 x.font='11px ui-monospace,monospace';x.fillStyle=css('--mut');x.strokeStyle=css('--line');x.lineWidth=1;
 for(let v=0;v<=3000;v+=500){x.beginPath();x.moveTo(L,Y(v));x.lineTo(L+pw,Y(v));x.stroke();x.textAlign='right';x.fillText(v,L-6,Y(v)+4)}
 x.textAlign='left';for(let v=0;v<=100;v+=25)x.fillText(v+'%',L+pw+6,YU(v)+4);
 x.textAlign='center';for(let s=0;s<=10;s+=2)x.fillText(s==10?'now':'-'+(10-s)+' s',L+s/10*pw,H-6);
 const line=(key,col,yf,w,dash)=>{x.strokeStyle=css(col);x.lineWidth=w;x.setLineDash(dash||[]);x.beginPath();let f=1;
  for(const p of pts){if(p.t<t0)continue;const px=X(p.t),py=yf(p[key]);f?x.moveTo(px,py):x.lineTo(px,py);f=0}x.stroke();x.setLineDash([])};
 line('u','--u',YU,1.5);line('sp','--sp',Y,1.5,[5,4]);line('y','--y',Y,2.2)}
async function refresh(){try{
 const s=await (await fetch('/api/state?since='+since)).json();$('conn').textContent='ESP32 online';
 for(const p of s.samples){pts.push(p);since=Math.max(since,p.seq)}const lim=pts.length?pts[pts.length-1].t-11000:0;pts=pts.filter(p=>p.t>=lim);
 $('sp').textContent=Math.round(s.sp);$('y').textContent=Math.round(s.y);$('u').textContent=Math.round(s.u*100);
 $('mode').textContent=s.mode=='tune'?'AUTO-TUNE':'PID';$('mode').className='tag'+(s.mode=='tune'?' on':'');
 $('progw').hidden=s.mode!='tune';$('prog').style.width=s.progress+'%';
 if(s.metrics.valid){$('perf').innerHTML='<span class="'+(s.metrics.os<5?'good':'bad')+'">'+s.metrics.os.toFixed(1)+'%</span> · '+s.metrics.ts.toFixed(2)+' s'}
 else $('perf').textContent=s.metrics.running?'measuring…':'–';
 $('brk').className=s.brake?'main':'';$('brk').textContent=s.brake?'Brake on':'Brake';
 $('aw').textContent='Anti-windup: '+(s.aw?'on':'off');
 if(!editing.kp)$('kp').value=s.kp.toFixed(6);if(!editing.ki)$('ki').value=s.ki.toFixed(6);
 $('sprv').textContent=s.spWeb<0?'potentiometer':s.spWeb+' rpm';
 if(s.model.K>0)$('model').innerHTML='<dt>gain K</dt><dd>'+s.model.K.toFixed(0)+' rpm per unit PWM</dd><dt>time constant τ</dt><dd>'+s.model.tau.toFixed(3)+' s</dd><dt>dead time θ</dt><dd>'+s.model.theta.toFixed(3)+' s</dd>';
 draw();
}catch(e){$('conn').textContent='ESP32 unreachable'}}
addEventListener('resize',draw);refresh();setInterval(refresh,250);
</script></body></html>)HTML";
