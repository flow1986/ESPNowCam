// Web UI of the rover tank (WiFi mode), served from "/". Speaks the binary packets of rover_proto.h over a WebSocket on port 81.
#pragma once
#include <pgmspace.h>

static const char WEB_UI[] PROGMEM = R"HTMLRAW(<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Rover</title>
<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:#111;color:#eee;font:14px sans-serif;user-select:none;-webkit-user-select:none;touch-action:manipulation}
#main{display:flex;flex-direction:column;height:100vh;height:100dvh}
#view{position:relative;flex:1;min-height:0;background:#000;display:flex;align-items:center;justify-content:center}
#cv{max-width:100%;max-height:100%}
#xh{position:absolute;left:50%;top:50%;width:44px;height:44px;margin:-22px 0 0 -22px;border:2px solid #f33;border-radius:50%;pointer-events:none}
#xh:before,#xh:after{content:"";position:absolute;background:#f33}
#xh:before{left:21px;top:-12px;width:2px;height:66px}
#xh:after{top:21px;left:-12px;height:2px;width:66px}
#st{position:absolute;left:4px;top:2px;font-size:12px;color:#8f8;text-shadow:0 0 3px #000}
#bar{position:absolute;right:4px;top:2px}
#bar button{background:#333c;color:#fff;border:0;border-radius:4px;padding:4px 8px;margin-left:4px}
#ctl{display:flex;justify-content:space-between;align-items:center;padding:6px;gap:6px}
.dp{display:grid;grid-template-columns:repeat(3,1fr);gap:4px;width:min(34vw,190px)}
.dp button,#mid button{touch-action:none;background:#2a3a4a;color:#fff;border:1px solid #567;border-radius:8px;font-size:18px;min-height:46px}
.dp button.on,#mid button.on{background:#3a7;border-color:#7fb}
.dp span{display:block}
#mid{display:flex;flex-direction:column;gap:4px;align-items:center;flex:1}
#mid div{display:flex;gap:4px}
#mid button{min-width:44px;font-size:14px;padding:0 6px}
#fire{background:#a22!important;font-weight:bold}
#set{display:none;position:absolute;inset:0;background:#000d;overflow:auto;padding:8px;z-index:5}
#set .row{display:flex;align-items:center;gap:6px;margin:3px 0}
#set label{flex:1}
#set input[type=range]{flex:1.2}
#set select{flex:1.2}
#set h3{margin:10px 0 4px;color:#7cf}
#set button{padding:6px 12px}
a{color:#7cf}
@media (orientation:landscape) and (max-height:520px){
 #main{flex-direction:row}
 #ctl{flex-direction:column;width:36vw;justify-content:space-around}
 .dp{width:min(30vw,180px)}
}
</style></head><body>
<div id="main">
 <div id="view"><canvas id="cv" width="320" height="240"></canvas><div id="xh"></div>
  <div id="st">connecting...</div>
  <div id="bar"><button id="bset">Settings</button></div>
  <div id="set"></div>
 </div>
 <div id="ctl">
  <div class="dp" id="dpd"><span></span><button data-k="f">&#9650;</button><span></span><button data-k="l">&#9664;</button><button data-k="b">&#9660;</button><button data-k="r">&#9654;</button></div>
  <div id="mid">
   <div><button data-s="25">25</button><button data-s="50">50</button><button data-s="75">75</button><button data-s="100">100</button></div>
   <div><button id="fire" data-k="fire">FIRE</button></div>
   <div id="spd">speed 50%</div>
  </div>
  <div class="dp" id="dpg"><span></span><button data-k="lu">lift &#9650;</button><span></span><button data-k="go">grip open</button><button data-k="ld">lift &#9660;</button><button data-k="gc">grip close</button></div>
 </div>
</div>
<script>
const $=i=>document.getElementById(i);
const CAMF=[['size',3,'u8'],['quality',4,'u8'],['brightness',5,'i8'],['contrast',6,'i8'],['saturation',7,'i8'],['hmirror',8,'u8'],['vflip',9,'u8'],['wb',10,'u8'],['effect',11,'u8'],['aeLevel',12,'i8']];
const SVF=[['centerL',3,'u16'],['centerR',5,'u16'],['gripMin',7,'u16'],['gripMax',9,'u16'],['liftMin',11,'u16'],['liftMax',13,'u16'],['gripRev',15,'u8'],['liftRev',16,'u8'],['revL',17,'u8'],['revR',18,'u8'],['trim',19,'i16'],['preview',21,'u8']];
const CAML=13,SVL=22,GRIP_RATE=60;
const G={u8:['getUint8','setUint8'],i8:['getInt8','setInt8'],u16:['getUint16','setUint16'],i16:['getInt16','setInt16']};
const rd=(dv,F,o)=>{for(const[k,p,t]of F)o[k]=dv[G[t][0]](p,true)};
const wr=(dv,F,o)=>{for(const[k,p,t]of F)dv[G[t][1]](p,o[k]|0,true)};
let ws,tank=0,speed=50,grip=50,lift=50,posInit=false,gotC=false,gotS=false,busy=false;
let rot=+localStorage.rot||0,xhOn=localStorage.xh!=='0';
const cam={},sv={preview:0},keys=new Set();
let frames=0,bytes=0,fps=0,kb=0,lastDrive='',lastDriveT=0,parts=[];
const mk=(t,l)=>{const b=new DataView(new ArrayBuffer(l));b.setUint8(0,165);b.setUint8(1,t);b.setUint8(2,tank);return b};
const tx=b=>{if(ws&&ws.readyState==1)ws.send(b.buffer)};
const ready=()=>gotC&&gotS;
const sendCam=()=>{if(!ready())return;const b=mk(2,CAML);wr(b,CAMF,cam);tx(b)};
const sendSv=()=>{if(!ready())return;const b=mk(5,SVL);wr(b,SVF,sv);tx(b)};
const getCfg=()=>{gotC=gotS=false;tx(mk(6,3))};
let tm;const later=f=>{clearTimeout(tm);tm=setTimeout(f,100)};

function connect(){
 ws=new WebSocket('ws://'+location.hostname+':81/');ws.binaryType='arraybuffer';
 ws.onopen=()=>{tank=0;tx(mk(3,3));tx(mk(6,3))};
 ws.onclose=()=>{$('st').textContent='disconnected';setTimeout(connect,1000)};
 ws.onmessage=e=>{
  const d=e.data,dv=new DataView(d),f=dv.getUint8(0);
  if(f==254){const fl=dv.getUint8(1);if(fl&1)parts=[];parts.push(d.slice(2));if(fl&2){const bl=new Blob(parts,{type:'image/jpeg'});parts=[];frames++;bytes+=bl.size;if(!busy){busy=true;createImageBitmap(bl).then(b=>{draw(b);busy=false}).catch(()=>busy=false)}}return}
  if(f!=165)return;
  const t=dv.getUint8(1);
  if(t==4){tank=dv.getUint8(2);if(!posInit&&d.byteLength>=5){grip=dv.getUint8(3);lift=dv.getUint8(4);posInit=true}}
  else if(t==2&&d.byteLength>=CAML){rd(dv,CAMF,cam);gotC=true;ui()}
  else if(t==5&&d.byteLength>=SVL){rd(dv,SVF,sv);sv.preview=0;gotS=true;ui()}
 };
}
function draw(b){
 const odd=rot&1,w=odd?b.height:b.width,h=odd?b.width:b.height,cv=$('cv');
 if(cv.width!=w||cv.height!=h){cv.width=w;cv.height=h}
 const c=cv.getContext('2d');c.save();c.translate(w/2,h/2);c.rotate(rot*Math.PI/2);c.drawImage(b,-b.width/2,-b.height/2);c.restore();b.close();
}

/* controls: buttons and keyboard share the same key names */
const KEYMAP={w:'f',arrowup:'f',s:'b',arrowdown:'b',a:'l',arrowleft:'l',d:'r',arrowright:'r',q:'go',e:'gc',r:'lu',f:'ld',' ':'fire'};
const press=(k,on)=>{on?keys.add(k):keys.delete(k);document.querySelectorAll('[data-k="'+k+'"]').forEach(b=>b.classList.toggle('on',on))};
document.querySelectorAll('[data-k]').forEach(b=>{
 const k=b.dataset.k;
 b.addEventListener('pointerdown',e=>{e.preventDefault();b.setPointerCapture(e.pointerId);press(k,true)});
 ['pointerup','pointercancel','lostpointercapture'].forEach(n=>b.addEventListener(n,()=>press(k,false)));
 b.addEventListener('contextmenu',e=>e.preventDefault());
});
function setSpeed(s){speed=s;$('spd').textContent='speed '+s+'%';document.querySelectorAll('[data-s]').forEach(b=>b.classList.toggle('on',+b.dataset.s==s))}
document.querySelectorAll('[data-s]').forEach(b=>b.addEventListener('click',()=>setSpeed(+b.dataset.s)));
setSpeed(50);
const typing=e=>/INPUT|SELECT|TEXTAREA/.test(e.target.tagName)&&e.target.type!=='range';
addEventListener('keydown',e=>{if(typing(e))return;const k=e.key.toLowerCase();
 if(k>='1'&&k<='4'){setSpeed(25*(+k));return}
 if(k=='+'||k=='=')setSpeed(Math.min(100,speed+25));if(k=='-')setSpeed(Math.max(25,speed-25));
 const m=KEYMAP[k];if(m){e.preventDefault();press(m,true)}});
addEventListener('keyup',e=>{const m=KEYMAP[e.key.toLowerCase()];if(m)press(m,false)});
addEventListener('blur',()=>[...keys].forEach(k=>press(k,false)));
document.addEventListener('visibilitychange',()=>[...keys].forEach(k=>press(k,false)));

const DT=25;
setInterval(()=>{
 const dt=DT/1000;
 if(keys.has('go'))grip-=GRIP_RATE*dt;if(keys.has('gc'))grip+=GRIP_RATE*dt;
 if(keys.has('lu'))lift+=GRIP_RATE*dt;if(keys.has('ld'))lift-=GRIP_RATE*dt;
 grip=Math.max(0,Math.min(100,grip));lift=Math.max(0,Math.min(100,lift));
 if(!tank)return;
 const fwd=(keys.has('f')?speed:0)-(keys.has('b')?speed:0),turn=(keys.has('r')?speed:0)-(keys.has('l')?speed:0),fire=keys.has('fire')?1:0;
 const s=[fwd,turn,fire,Math.round(grip),Math.round(lift)].join(),n=Date.now();
 if(s!=lastDrive||n-lastDriveT>100){lastDrive=s;lastDriveT=n;const b=mk(1,8);b.setInt8(3,fwd);b.setInt8(4,turn);b.setUint8(5,fire);b.setUint8(6,Math.round(grip));b.setUint8(7,Math.round(lift));tx(b)}
},DT);
setInterval(()=>{
 fps=frames;kb=frames?bytes/frames/1024:0;frames=bytes=0;
 $('st').textContent=(ws&&ws.readyState==1?'Tank '+(tank||'?'):'disconnected')+'  '+fps+' fps  '+kb.toFixed(1)+' KB'+(ready()?'':'  (loading settings)');
 if(ws&&ws.readyState==1)tx(mk(3,3));
},1000);

/* settings panel */
const FIELDS=[
 ['h','Camera'],
 ['cam','size','Resolution','sel',['160x120','240x176','320x240']],
 ['cam','quality','JPEG quality (higher = better, slower)','rng',5,50,5],
 ['cam','brightness','Brightness','rng',-2,2,1],['cam','contrast','Contrast','rng',-2,2,1],['cam','saturation','Saturation','rng',-2,2,1],
 ['cam','wb','White balance','sel',['Auto','Sunny','Cloudy','Office','Home']],
 ['cam','effect','Effect','sel',['None','Negative','Grayscale','Red','Green','Blue','Sepia']],
 ['cam','aeLevel','Exposure','rng',-2,2,1],
 ['cam','hmirror','Mirror','chk'],['cam','vflip','Flip','chk'],
 ['h','Drive servos'],
 ['sv','trim','Drive trim (1/10 %, + = steers right)','rng',-400,400,5],
 ['sv','centerL','Servo L center (us)','rng',1000,2000,5],['sv','centerR','Servo R center (us)','rng',1000,2000,5],
 ['sv','revL','Servo L reverse','chk'],['sv','revR','Servo R reverse','chk'],
 ['h','Gripper'],
 ['sv','gripMin','Grip min (us)','rng',500,2500,10],['sv','gripMax','Grip max (us)','rng',500,2500,10],['sv','gripRev','Grip reverse','chk'],
 ['sv','liftMin','Lift min (us)','rng',500,2500,10],['sv','liftMax','Lift max (us)','rng',500,2500,10],['sv','liftRev','Lift reverse','chk'],
 ['sv','preview','Preview on the tank','sel',['Off','Drive centers','Grip min','Grip max','Lift min','Lift max']],
];
const els=[];
function build(){
 const p=$('set');
 p.innerHTML='<div class="row"><b>Settings are stored on the tank.</b><span style="flex:1"></span><button id="bclose">Close</button></div>';
 const row=(l,i)=>{const r=document.createElement('div');r.className='row';const lb=document.createElement('label');lb.textContent=l;r.append(lb,i);p.append(r);return lb};
 /* view options live in the browser only */
 const vs=document.createElement('select');['0','90','180','270'].forEach((t,i)=>vs.add(new Option(t+' deg',i)));vs.value=rot;vs.onchange=()=>{rot=+vs.value;localStorage.rot=rot};row('Image rotation (browser)',vs);
 const xc=document.createElement('input');xc.type='checkbox';xc.checked=xhOn;xc.onchange=()=>{xhOn=xc.checked;localStorage.xh=xhOn?1:0;$('xh').style.display=xhOn?'block':'none'};row('Crosshair (browser)',xc);
 for(const f of FIELDS){
  if(f[0]=='h'){const h=document.createElement('h3');h.textContent=f[1];p.append(h);continue}
  const [grp,key,label,type]=f,o=grp=='cam'?cam:sv,send=grp=='cam'?sendCam:sendSv;let inp,val;
  if(type=='sel'){inp=document.createElement('select');f[4].forEach((t,i)=>inp.add(new Option(t,i)));inp.onchange=()=>{o[key]=+inp.value;key=='preview'?send():later(send)}}
  else if(type=='chk'){inp=document.createElement('input');inp.type='checkbox';inp.onchange=()=>{o[key]=inp.checked?1:0;later(send)}}
  else{inp=document.createElement('input');inp.type='range';inp.min=f[4];inp.max=f[5];inp.step=f[6];val=document.createElement('span');val.style.minWidth='46px';
   inp.oninput=()=>{o[key]=+inp.value;val.textContent=inp.value;later(send)}}
  const lb=row(label,inp);if(val)lb.parentNode.append(val);
  els.push({o,key,inp,val,type});
 }
 $('bclose').onclick=()=>{sv.preview=0;sendSv();p.style.display='none'};
}
function ui(){
 for(const e of els){const v=e.o[e.key];if(v===undefined)continue;if(e.type=='chk')e.inp.checked=!!v;else e.inp.value=v;if(e.val)e.val.textContent=v}
}
$('bset').onclick=()=>{$('set').style.display='block';getCfg()};
$('xh').style.display=xhOn?'block':'none';
build();connect();
</script></body></html>
)HTMLRAW";
