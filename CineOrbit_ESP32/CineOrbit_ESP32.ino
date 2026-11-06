
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>

// ESP32 DevKit V1 camera motor pins
#define ENA 25
#define IN1 26
#define IN2 27

const char* AP_NAME = "CineOrbit";
const char* AP_PASSWORD = "orbit123";

// ESP32 analogWrite range is 0 to 255
const int MOTOR_SPEED = 255;
const unsigned long COMMAND_TIMEOUT = 700;

WebServer server(80);
bool filesystemReady = false;
unsigned long lastCommandTime = 0;
bool motorRunning = false;

const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CineOrbit Studio V3</title>
<style>
*{box-sizing:border-box}
body{margin:0;padding:16px;background:#10141c;color:#f1f5f9;
font-family:Arial,sans-serif}
h1{margin-bottom:4px}
.sub,small{color:#aab8c9}
.panel{background:#1b2330;padding:15px;border-radius:14px;margin:14px 0}
button{border:0;border-radius:8px;padding:11px;font-weight:bold;cursor:pointer}
button:disabled{opacity:.4}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.row button{flex:1}
.primary{background:#3182ce;color:white}
.secondary{background:#394454;color:white}
.danger{background:#d94343;color:white}
.actions{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.action{background:#29384d;color:white;text-align:left}
input,select{background:#10141c;color:white;border:1px solid #536176;
padding:9px;border-radius:7px;max-width:100%}
input[type=number]{width:95px}
input[type=checkbox]{width:18px;height:18px;vertical-align:middle}
.timelineScroll{overflow-x:auto;border:1px solid #46546a;border-radius:10px}
.track{position:relative;min-width:100%;width:max-content;padding-bottom:12px}
.ruler{height:38px;position:relative;border-bottom:1px solid #536176;
background:#202a39}
.tick{position:absolute;top:0;height:100%;border-left:1px solid #536176;
padding:5px 0 0 4px;color:#b9c5d4;font-size:11px}
.lane{position:relative;min-height:90px;padding:10px;
background:#151c27}
.block{position:absolute;top:12px;height:64px;border-radius:8px;
padding:6px;color:white;cursor:grab;overflow:hidden;
border:2px solid #78a5dc;background:#2b5685;min-width:58px}
.block.stop{background:#66512b;border-color:#c8a65e}
.block.turn{background:#563d79;border-color:#aa8de0}
.block.active{outline:3px solid #f5d76e}
.block strong{font-size:12px;display:block;white-space:nowrap}
.block small{font-size:11px;white-space:nowrap;color:#fff}
#status{color:#81d8a5}
.roverBlock{display:flex;gap:8px;align-items:center;justify-content:space-between;
background:#254b43;border:1px solid #5bb69b;border-radius:8px;padding:9px;margin:7px 0}
.roverBlock button{padding:6px 9px}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;max-width:320px;margin:12px auto}
.pad button{min-height:52px;background:#294a67;color:white;touch-action:none}
.pad .stopPad{background:#8d3434}
.trackTitle{margin:10px 0 4px;color:#b9c5d4}

</style>
</head>
<body>
<h1>CineOrbit Studio</h1>
<p class="sub">Timeline V2   Motion sequencing</p>

<div class="panel">
<h3>Action library</h3>
<p><small>Drag an action onto the timeline, or tap to append it.</small></p>
<div class="actions">
<button class="action" draggable="true" data-type="left">Rotate Left</button>
<button class="action" draggable="true" data-type="right">Rotate Right</button>
<button class="action" draggable="true" data-type="stop">Stop Wait</button>
<button class="action" draggable="true" data-type="turn">360 Degree Rotation</button>
</div>
</div>

<div class="panel">
<h3>Timeline ruler</h3>
<p><small>Blocks are proportional to duration. Drag blocks to reorder them.
The ruler scrolls horizontally for longer sequences.</small></p>
<div class="timelineScroll" id="timelineScroll">
<div class="track" id="track">
<div class="ruler" id="ruler"></div>
<div class="lane" id="lane"></div>
</div>
</div>
<p>Total duration: <strong id="total">0.0 seconds</strong></p>
<p><small>Select a block to edit its duration or 360 degree direction below.</small></p>
<div id="editor"><small>No action selected.</small></div>
<div class="row" style="margin-top:12px">
<button class="secondary" onclick="duplicateSelected()">Duplicate</button>
<button class="secondary" onclick="deleteSelected()">Delete selected</button>
<button class="secondary" onclick="clearTimeline()">Clear timeline</button>
</div>
</div>

<div class="panel">
<h3>Rover timeline</h3>
<p><small>This track runs at the same time as the camera timeline. Add actions in the order you want the rover to perform them.</small></p>
<div class="row">
<button class="secondary" onclick="addRoverStep('forward')">Forward</button>
<button class="secondary" onclick="addRoverStep('backward')">Backward</button>
<button class="secondary" onclick="addRoverStep('left')">Turn Left</button>
<button class="secondary" onclick="addRoverStep('right')">Turn Right</button>
<button class="secondary" onclick="addRoverStep('stop')">Stop / Wait</button>
</div>
<p class="trackTitle">Rover sequence</p>
<div id="roverLane"><small>No rover actions added.</small></div>
<div id="roverEditor"><small>Select a rover action to edit its duration.</small></div>
<div class="row">
<button class="secondary" onclick="moveRoverSelected(-1)">Move earlier</button>
<button class="secondary" onclick="moveRoverSelected(1)">Move later</button>
<button class="secondary" onclick="deleteRoverSelected()">Delete selected</button>
<button class="secondary" onclick="clearRover()">Clear rover track</button>
</div>
</div>

<div class="panel">
<h3>Manual rover control</h3>
<p><small>Hold a direction to move. Release to stop. Manual controls are disabled during timeline playback.</small></p>
<label>Rover speed: <strong id="roverSpeedLabel">100</strong>
<input id="roverSpeed" type="range" min="70" max="255" value="100" style="width:100%">
</label>
<div class="pad">
<span></span><button onpointerdown="manualStart(event,'forward')" onpointerup="manualStop(event)" onpointerleave="manualStop(event)" onpointercancel="manualStop(event)">Forward</button><span></span>
<button onpointerdown="manualStart(event,'left')" onpointerup="manualStop(event)" onpointerleave="manualStop(event)" onpointercancel="manualStop(event)">Left</button>
<button class="stopPad" onclick="manualRoverStop()">Stop</button>
<button onpointerdown="manualStart(event,'right')" onpointerup="manualStop(event)" onpointerleave="manualStop(event)" onpointercancel="manualStop(event)">Right</button>
<span></span><button onpointerdown="manualStart(event,'backward')" onpointerup="manualStop(event)" onpointerleave="manualStop(event)" onpointercancel="manualStop(event)">Backward</button><span></span>
</div>
</div>

<div class="panel">
<h3>360 degree calibration</h3>
<label>Duration (seconds)
<input id="turnSeconds" type="number" min="1" max="120" step="0.1" value="17.1">
</label>
<p><small>17.1 seconds is a starting estimate for a 3.5 RPM motor. Calibrate with your platform installed.</small></p>
</div>

<div class="panel">
<h3>Playback</h3>
<label><input id="repeat" type="checkbox"> Repeat timeline continuously</label>
<div class="row" style="margin-top:12px">
<button class="primary" onclick="runTimeline()">Run</button>
<button class="secondary" onclick="pauseTimeline()">Pause Resume</button>
<button class="danger" onclick="stopTimeline()">Stop</button>
</div>
<p id="status">Ready</p>
<p id="progress">No sequence running.</p>
</div>

<div class="panel">
<h3>Save and exchange sequences</h3>
<input id="sequenceName" placeholder="Sequence name" style="width:100%">
<div class="row" style="margin-top:10px">
<button class="primary" onclick="saveSequence()">Save</button>
<button class="secondary" onclick="loadSequence()">Load saved</button>
<button class="danger" onclick="deleteSequence()">Delete saved</button>
</div>
<select id="savedList" style="width:100%;margin-top:10px">
<option value="">Choose saved sequence</option>
</select>
<hr style="border-color:#394454;margin:18px 0">
<div class="row">
<button class="primary" onclick="exportJSON()">Export JSON</button>
<button class="secondary" onclick="document.getElementById('importFile').click()">Import JSON</button>
<input id="importFile" type="file" accept=".json,application/json" hidden>
</div>
<p><small>Export downloads a file you can keep or share. Import replaces the current editor sequence after validation.</small></p>
</div>

<script>
const $ = id => document.getElementById(id);
const track = $('track'), ruler = $('ruler'), lane = $('lane');

let steps = []; // Camera track
let roverSteps = []; // Rover track
let selectedRoverId = null;
let nextId = 1;
let selectedId = null;
let running = false, paused = false, stopRequested = false;
let dragData = null, activeId = null;
const PX_PER_SECOND = 60;

function label(type) {
  return ({left:'Rotate Left',right:'Rotate Right',
    stop:'Stop / Wait',turn:'360 Degree Rotation'})[type];
}
function durationOf(s) {
  return s.type === 'turn'
    ? Math.max(1,Number($('turnSeconds').value)||17.1)
    : Math.max(.1,Number(s.duration)||.1);
}
function totalDuration(list=steps) {
  return list.reduce((sum,s)=>sum+durationOf(s),0);
}
function newStep(type) {
  const defaults = {left:3,right:3,stop:2,turn:17.1};
  return {id:nextId++,type,duration:defaults[type],direction:'right'};
}
function addStep(type,index=steps.length) {
  if(running)return;
  const s=newStep(type);
  steps.splice(index,0,s);
  selectedId=s.id;
  render();
}
function setStatus(s){$('status').textContent=s;}

function render() {
  const total=totalDuration();
  const width=Math.max(420,total*PX_PER_SECOND+20);
  track.style.width=width+'px';
  ruler.style.width=width+'px';
  lane.style.width=width+'px';
  ruler.innerHTML='';
  lane.innerHTML='';

  const tickEvery=total>100?10:(total>40?5:1);
  for(let t=0;t<=total+0.001;t+=tickEvery){
    const tick=document.createElement('div');
    tick.className='tick';
    tick.style.left=(t*PX_PER_SECOND)+'px';
    tick.textContent=t+'s';
    ruler.appendChild(tick);
  }

  let cursor=0;
  steps.forEach((s,index)=>{
    const dur=durationOf(s);
    const block=document.createElement('div');
    block.className='block '+s.type+
      (selectedId===s.id?' selected':'')+
      (activeId===s.id?' active':'');
    block.style.left=(cursor*PX_PER_SECOND+2)+'px';
    block.style.width=Math.max(54,dur*PX_PER_SECOND-4)+'px';
    block.draggable=!running;
    block.innerHTML='<strong></strong><small></small>';
    block.children[0].textContent=(index+1)+'. '+label(s.type);
    block.children[1].textContent=
      (s.type==='turn'?s.direction+'   ':'')+dur.toFixed(1)+' s';

    block.onclick=()=>{selectedId=s.id;render();};
    block.addEventListener('dragstart',e=>{
      if(running){e.preventDefault();return;}
      dragData={kind:'existing',id:s.id};
      e.dataTransfer.setData('text/plain','existing:'+s.id);
    });
    block.addEventListener('dragover',e=>e.preventDefault());
    block.addEventListener('drop',e=>{
      e.preventDefault();e.stopPropagation();
      dropOnStep(s.id);
    });
    lane.appendChild(block);
    cursor+=dur;
  });

  $('total').textContent=total.toFixed(1)+' seconds';
  renderEditor();
}

function renderEditor(){
  const editor=$('editor');
  editor.innerHTML='';
  const s=steps.find(x=>x.id===selectedId);
  if(!s){editor.innerHTML='<small>No action selected.</small>';return;}

  const title=document.createElement('p');
  title.textContent=label(s.type);
  editor.appendChild(title);

  if(s.type==='turn'){
    const select=document.createElement('select');
    [['right','Clockwise / Right'],['left','Counter clockwise / Left']]
      .forEach(([v,t])=>{
        const o=document.createElement('option');
        o.value=v;o.textContent=t;select.appendChild(o);
      });
    select.value=s.direction;
    select.disabled=running;
    select.onchange=()=>{s.direction=select.value;render();};
    editor.appendChild(select);
    const info=document.createElement('p');
    info.innerHTML='<small>Uses the 360 degree calibration duration above.</small>';
    editor.appendChild(info);
  }else{
    const input=document.createElement('input');
    input.type='number';input.min='.1';input.max='3600';input.step='.1';
    input.value=s.duration;input.disabled=running;
    input.onchange=()=>{
      s.duration=Math.max(.1,Math.min(3600,Number(input.value)||.1));
      render();
    };
    editor.appendChild(input);
    const info=document.createElement('small');
    info.textContent=' Duration in seconds';
    editor.appendChild(info);
  }
}

function dropOnStep(targetId){
  if(running||!dragData)return;
  const target=steps.findIndex(s=>s.id===targetId);
  if(target<0)return;

  if(dragData.kind==='new'){
    addStep(dragData.type,target);
  }else{
    const from=steps.findIndex(s=>s.id===dragData.id);
    if(from<0||from===target){dragData=null;return;}
    const [item]=steps.splice(from,1);
    const adjusted=from<target?target-1:target;
    steps.splice(adjusted,0,item);
    selectedId=item.id;
    render();
  }
  dragData=null;
}

document.querySelectorAll('.action').forEach(button=>{
  button.onclick=()=>addStep(button.dataset.type);
  button.addEventListener('dragstart',e=>{
    if(running){e.preventDefault();return;}
    dragData={kind:'new',type:button.dataset.type};
    e.dataTransfer.setData('text/plain','new:'+button.dataset.type);
  });
});

lane.addEventListener('dragover',e=>e.preventDefault());
lane.addEventListener('drop',e=>{
  e.preventDefault();
  if(running||!dragData)return;
  if(dragData.kind==='new')addStep(dragData.type);
  else{
    const from=steps.findIndex(s=>s.id===dragData.id);
    if(from>=0){
      const [item]=steps.splice(from,1);
      steps.push(item);selectedId=item.id;render();
    }
  }
  dragData=null;
});

function duplicateSelected(){
  if(running)return;
  const i=steps.findIndex(s=>s.id===selectedId);
  if(i<0)return;
  const copy={...steps[i],id:nextId++};
  steps.splice(i+1,0,copy);
  selectedId=copy.id;render();
}
function deleteSelected(){
  if(running)return;
  steps=steps.filter(s=>s.id!==selectedId);
  selectedId=null;render();
}
function clearTimeline(){
  if(running)return;
  steps=[];selectedId=null;render();
}
$('turnSeconds').addEventListener('change',()=>{
  const v=Math.max(1,Math.min(120,Number($('turnSeconds').value)||17.1));
  $('turnSeconds').value=v;render();
});

function roverLabel(type){
  return ({forward:'Forward',backward:'Backward',left:'Turn Left',right:'Turn Right',stop:'Stop / Wait'})[type];
}
function addRoverStep(type){
  if(running)return;
  const defaults={forward:3,backward:3,left:1.5,right:1.5,stop:2};
  const item={id:nextId++,type,duration:defaults[type]};
  roverSteps.push(item);selectedRoverId=item.id;renderRover();
}
function renderRover(){
  const lane=$('roverLane');lane.innerHTML='';
  if(!roverSteps.length){lane.innerHTML='<small>No rover actions added.</small>';return;}
  roverSteps.forEach((s,i)=>{
    const row=document.createElement('div');
    row.className='roverBlock';
    if(s.id===selectedRoverId)row.style.outline='2px solid #f5d76e';
    const name=document.createElement('span');
    name.textContent=(i+1)+'. '+roverLabel(s.type)+' - '+Number(s.duration).toFixed(1)+' s';
    row.appendChild(name);
    const select=document.createElement('button');select.className='secondary';
    select.textContent='Edit';select.onclick=()=>{selectedRoverId=s.id;renderRover();};
    row.appendChild(select);lane.appendChild(row);
  });
  const editor=$('roverEditor');editor.innerHTML='';
  const s=roverSteps.find(x=>x.id===selectedRoverId);
  if(!s){editor.innerHTML='<small>Select a rover action to edit its duration.</small>';return;}
  const labelEl=document.createElement('label');labelEl.textContent=roverLabel(s.type)+' duration (seconds) ';
  const input=document.createElement('input');input.type='number';input.min='.1';input.max='3600';input.step='.1';input.value=s.duration;input.disabled=running;
  input.onchange=()=>{s.duration=Math.max(.1,Math.min(3600,Number(input.value)||.1));renderRover();};
  labelEl.appendChild(input);editor.appendChild(labelEl);
}
function moveRoverSelected(delta){
  if(running)return;
  const i=roverSteps.findIndex(x=>x.id===selectedRoverId),j=i+delta;
  if(i<0||j<0||j>=roverSteps.length)return;
  [roverSteps[i],roverSteps[j]]=[roverSteps[j],roverSteps[i]];renderRover();
}
function deleteRoverSelected(){
  if(running)return;
  roverSteps=roverSteps.filter(x=>x.id!==selectedRoverId);selectedRoverId=null;renderRover();
}
function clearRover(){
  if(running)return;
  roverSteps=[];selectedRoverId=null;renderRover();
}
let manualHeld=false,manualTimer=null,manualDirection='';
async function manualStart(event,dir){
  if(running)return;
  if(event){event.preventDefault();if(event.currentTarget.setPointerCapture)event.currentTarget.setPointerCapture(event.pointerId);}
  manualHeld=true;manualDirection=dir;
  if(manualTimer)clearInterval(manualTimer);
  await request('/rover/move?dir='+dir);
  manualTimer=setInterval(()=>{if(manualHeld)request('/rover/move?dir='+manualDirection);},250);
}
async function manualStop(event){
  if(event)event.preventDefault();
  if(!manualHeld)return;
  manualHeld=false;if(manualTimer)clearInterval(manualTimer);manualTimer=null;
  await request('/rover/stop');
}
function manualRoverStop(){
  manualHeld=false;if(manualTimer)clearInterval(manualTimer);manualTimer=null;
  request('/rover/stop');
}
$('roverSpeed').addEventListener('input',()=>{
  $('roverSpeedLabel').textContent=$('roverSpeed').value;
  request('/rover/speed?value='+$('roverSpeed').value);
});

async function request(path){
  try{
    const response=await fetch(path,{cache:'no-store'});
    if(!response.ok)throw new Error('HTTP '+response.status);
    return true;
  }catch(e){
    setStatus('Connection lost. Motor timeout should stop movement.');
    return false;
  }
}
async function sendStop(){return request('/camera/stop');}
async function sendMove(dir){return request('/camera/move?dir='+dir);}
async function sendRoverStop(){return request('/rover/stop');}
async function sendRoverMove(dir){return request('/rover/move?dir='+dir);}

async function playTrack(sequence,kind,turnSeconds){
  const camera=kind==='camera';
  const move=camera?sendMove:sendRoverMove;
  const stop=camera?sendStop:sendRoverStop;
  for(let i=0;i<sequence.length&&!stopRequested;i++){
    const s=sequence[i];
    if(camera){activeId=s.id;render();}
    else {selectedRoverId=s.id;renderRover();}
    const duration=(camera&&s.type==='turn'?turnSeconds:Number(s.duration))*1000;
    const moving=camera?['left','right','turn'].includes(s.type):s.type!=='stop';
    const direction=camera?(s.type==='turn'?s.direction:s.type):s.type;
    let remaining=duration,last=Date.now(),heartbeat=0;
    if(!moving)await stop();
    while(remaining>0&&!stopRequested){
      if(paused){
        await stop();await sleep(80);last=Date.now();continue;
      }
      const now=Date.now();
      if(moving&&now-heartbeat>=250){
        const ok=await move(direction);
        if(!ok){stopRequested=true;break;}
        heartbeat=Date.now();
      }
      await sleep(40);
      const after=Date.now();remaining-=after-last;last=after;
      $('progress').textContent=(camera?'Camera':'Rover')+' action '+(i+1)+'/'+sequence.length+
        ' - '+Math.max(0,remaining/1000).toFixed(1)+'s left';
    }
    await stop();
  }
  await stop();
}

async function runTimeline(){
  if(running)return;
  if(!steps.length&&!roverSteps.length){setStatus('Add actions to one or both timelines first.');return;}
  running=true;paused=false;stopRequested=false;manualHeld=false;
  const cameraSequence=JSON.parse(JSON.stringify(steps));
  const roverSequence=JSON.parse(JSON.stringify(roverSteps));
  const turnSeconds=Math.max(1,Math.min(120,Number($('turnSeconds').value)||17.1));
  const repeat=$('repeat').checked;
  setStatus(repeat?'Repeating both timelines':'Running camera and rover timelines');
  let completed=false;
  try{
    do{
      const jobs=[];
      if(cameraSequence.length)jobs.push(playTrack(cameraSequence,'camera',turnSeconds));
      if(roverSequence.length)jobs.push(playTrack(roverSequence,'rover',turnSeconds));
      await Promise.all(jobs);
      if(stopRequested)break;
      completed=true;
      if(repeat){$('progress').textContent='Both tracks complete - restarting.';await sleep(100);}
    }while(repeat&&!stopRequested);
  }finally{
    await Promise.all([sendStop(),sendRoverStop()]);
    running=false;paused=false;activeId=null;render();renderRover();
    setStatus(stopRequested?'Stopped':(completed?'Both timelines complete':'Playback ended'));
    $('progress').textContent=stopRequested?'Playback cancelled.':'Playback finished.';
  }
}
function sleep(ms){return new Promise(resolve=>setTimeout(resolve,ms));}

function pauseTimeline(){
  if(!running)return;
  paused=!paused;
  if(paused){sendStop();sendRoverStop();setStatus('Paused - both motors stopped');}
  else setStatus('Resuming');
}
function stopTimeline(){
  stopRequested=true;paused=false;sendStop();sendRoverStop();setStatus('Stopping both motors...');
}

// Browser-local saved sequences
function savedSequences(){
  try{return JSON.parse(localStorage.getItem('cineorbit_sequences')||'{}');}
  catch(e){return {};}
}
function refreshSavedList(selected=''){
  const list=$('savedList');
  list.innerHTML='<option value="">Choose saved sequence</option>';
  Object.keys(savedSequences()).sort().forEach(name=>{
    const o=document.createElement('option');o.value=name;o.textContent=name;
    list.appendChild(o);
  });
  list.value=selected;
}
function saveSequence(){
  if(running)return;
  const name=$('sequenceName').value.trim();
  if(!name){setStatus('Enter a sequence name first.');return;}
  const all=savedSequences();
  all[name]={version:3,steps,roverSteps,turnSeconds:$('turnSeconds').value};
  localStorage.setItem('cineorbit_sequences',JSON.stringify(all));
  refreshSavedList(name);setStatus('Saved: '+name);
}
function loadSequence(){
  if(running)return;
  const name=$('savedList').value, data=savedSequences()[name];
  if(!data){setStatus('Choose a saved sequence first.');return;}
  if(!validateSequence(data)){setStatus('Saved sequence is invalid.');return;}
  steps=data.steps.map(s=>({...s,id:nextId++}));
  roverSteps=(data.roverSteps||[]).map(s=>({...s,id:nextId++}));
  selectedId=null;selectedRoverId=null;
  $('sequenceName').value=name;
  $('turnSeconds').value=data.turnSeconds||17.1;
  render();renderRover();setStatus('Loaded: '+name);
}
function deleteSequence(){
  if(running)return;
  const name=$('savedList').value;
  if(!name)return;
  const all=savedSequences();delete all[name];
  localStorage.setItem('cineorbit_sequences',JSON.stringify(all));
  refreshSavedList();setStatus('Deleted: '+name);
}

// JSON file export/import
function exportJSON(){
  const name=$('sequenceName').value.trim()||'CineOrbit-sequence';
  const data={
    app:'CineOrbit',
    version:3,
    name,
    turnSeconds:Number($('turnSeconds').value)||17.1,
    steps:steps.map(({type,duration,direction})=>({type,duration,direction})),
    roverSteps:roverSteps.map(({type,duration})=>({type,duration}))
  };
  const blob=new Blob([JSON.stringify(data,null,2)],{type:'application/json'});
  const url=URL.createObjectURL(blob);
  const a=document.createElement('a');
  a.href=url;a.download=name.replace(/[^a-z0-9_-]/gi,'_')+'.json';
  document.body.appendChild(a);a.click();a.remove();
  URL.revokeObjectURL(url);
  setStatus('JSON exported.');
}

function validateSequence(data){
  if(!data||!Array.isArray(data.steps)||data.steps.length>500)return false;
  if(data.roverSteps!==undefined&&(!Array.isArray(data.roverSteps)||data.roverSteps.length>500))return false;
  if(data.roverSteps!==undefined&&(!Array.isArray(data.roverSteps)||data.roverSteps.length>500))return false;
  if(data.steps.some(s=>{
    if(!s||!['left','right','stop','turn'].includes(s.type))return true;
    if(s.type==='turn')return !['left','right'].includes(s.direction);
    return !Number.isFinite(Number(s.duration))||
      Number(s.duration)<.1||Number(s.duration)>3600;
  }))return false;
  if((data.roverSteps||[]).some(s=>!s||!['forward','backward','left','right','stop'].includes(s.type)||
    !Number.isFinite(Number(s.duration))||Number(s.duration)<.1||Number(s.duration)>3600))return false;
  const t=Number(data.turnSeconds||17.1);
  return Number.isFinite(t)&&t>=1&&t<=120;
}

$('importFile').addEventListener('change',async e=>{
  const file=e.target.files[0];
  e.target.value='';
  if(!file)return;
  if(running){setStatus('Stop playback before importing.');return;}
  try{
    const data=JSON.parse(await file.text());
    if(!validateSequence(data))throw new Error('Invalid sequence');
    steps=data.steps.map(s=>({id:nextId++,type:s.type,duration:Number(s.duration)||3,direction:s.direction||'right'}));
    roverSteps=(data.roverSteps||[]).map(s=>({id:nextId++,type:s.type,duration:Number(s.duration)||3}));
    selectedRoverId=null;
    $('turnSeconds').value=Number(data.turnSeconds)||17.1;
    $('sequenceName').value=data.name||file.name.replace(/\.json$/i,'');
    selectedId=null;render();renderRover();setStatus('Imported: '+file.name);
  }catch(err){
    setStatus('Import failed: invalid CineOrbit JSON file.');
  }
});

refreshSavedList();
render();
renderRover();
</script>
</body>
</html>
)rawliteral";


// Camera motor: L298N #1
#define CAM_ENA 25
#define CAM_IN1 26
#define CAM_IN2 27

// Rover motors: L298N #2 (one dual-channel board)
#define ROVER_ENA 18
#define ROVER_IN1 19
#define ROVER_IN2 21
#define ROVER_ENB 22
#define ROVER_IN3 23
#define ROVER_IN4 32

#define IR_PIN 33
#define LED_PIN 13


const int CAMERA_SPEED = 255;
int roverSpeed = 100;

unsigned long cameraCommandTime=0, roverCommandTime=0;
bool cameraRunning=false, roverRunning=false;

void stopCamera(){
  analogWrite(CAM_ENA,0);
  digitalWrite(CAM_IN1,LOW);digitalWrite(CAM_IN2,LOW);
  cameraRunning=false;
}
void moveCamera(bool right){
  digitalWrite(CAM_IN1,right?HIGH:LOW);
  digitalWrite(CAM_IN2,right?LOW:HIGH);
  analogWrite(CAM_ENA,CAMERA_SPEED);
  cameraRunning=true;cameraCommandTime=millis();
}
void stopRover(){
  analogWrite(ROVER_ENA,0);analogWrite(ROVER_ENB,0);
  digitalWrite(ROVER_IN1,LOW);digitalWrite(ROVER_IN2,LOW);
  digitalWrite(ROVER_IN3,LOW);digitalWrite(ROVER_IN4,LOW);
  roverRunning=false;
}
void setRoverMotors(int leftDir,int rightDir){
  digitalWrite(ROVER_IN1,leftDir>0?HIGH:LOW);
  digitalWrite(ROVER_IN2,leftDir<0?HIGH:LOW);
  digitalWrite(ROVER_IN3,rightDir>0?HIGH:LOW);
  digitalWrite(ROVER_IN4,rightDir<0?HIGH:LOW);
  analogWrite(ROVER_ENA,roverSpeed);
  analogWrite(ROVER_ENB,roverSpeed);
  roverRunning=true;roverCommandTime=millis();
}
void moveRover(String dir){
  if(dir=="forward")setRoverMotors(1,1);
  else if(dir=="backward")setRoverMotors(-1,-1);
  else if(dir=="left")setRoverMotors(-1,1);
  else if(dir=="right")setRoverMotors(1,-1);
}

void setup(){
  Serial.begin(115200);

  // Mount LittleFS. Do not auto-format if mounting fails, so existing
  // filesystem contents are not erased unexpectedly.
  filesystemReady = LittleFS.begin(false);
  if(!filesystemReady){
    Serial.println("LittleFS mount failed. Camera test page will be unavailable.");
  }else{
    Serial.println("LittleFS mounted successfully.");
  }

  pinMode(CAM_ENA,OUTPUT);pinMode(CAM_IN1,OUTPUT);pinMode(CAM_IN2,OUTPUT);
  pinMode(ROVER_ENA,OUTPUT);pinMode(ROVER_IN1,OUTPUT);pinMode(ROVER_IN2,OUTPUT);
  pinMode(ROVER_ENB,OUTPUT);pinMode(ROVER_IN3,OUTPUT);pinMode(ROVER_IN4,OUTPUT);
  pinMode(IR_PIN,INPUT);pinMode(LED_PIN,OUTPUT);
  stopCamera();stopRover();

  WiFi.mode(WIFI_AP);WiFi.softAP(AP_NAME,AP_PASSWORD);
  Serial.print("CineOrbit IP: ");Serial.println(WiFi.softAPIP());

  server.on("/",[](){server.send_P(200,"text/html",PAGE);});

  // Serve the standalone Android camera-permission test page from LittleFS.
  server.on("/camera-test.html",HTTP_GET,[](){
    if(!filesystemReady){
      server.send(503,"text/plain","LittleFS is not available");
      return;
    }
    File file=LittleFS.open("/camera-test.html","r");
    if(!file){
      server.send(404,"text/plain","camera-test.html not found in LittleFS /data folder");
      return;
    }
    server.streamFile(file,"text/html");
    file.close();
  });

  // Camera motor endpoints
  server.on("/camera/move",[](){
    if(!server.hasArg("dir")){server.send(400,"text/plain","Missing direction");return;}
    String d=server.arg("dir");
    if(d=="left")moveCamera(false);else if(d=="right")moveCamera(true);
    else{server.send(400,"text/plain","Invalid direction");return;}
    server.send(200,"text/plain","OK");
  });
  server.on("/camera/stop",[](){stopCamera();server.send(200,"text/plain","STOPPED");});

  // Rover motor endpoints
  server.on("/rover/move",[](){
    if(!server.hasArg("dir")){server.send(400,"text/plain","Missing direction");return;}
    String d=server.arg("dir");
    if(d!="forward"&&d!="backward"&&d!="left"&&d!="right"){
      server.send(400,"text/plain","Invalid direction");return;
    }
    moveRover(d);server.send(200,"text/plain","OK");
  });
  server.on("/rover/stop",[](){stopRover();server.send(200,"text/plain","STOPPED");});
  server.on("/rover/speed",[](){
    if(server.hasArg("value"))roverSpeed=constrain(server.arg("value").toInt(),70,255);
    server.send(200,"text/plain",String(roverSpeed));
  });
  server.on("/status",[](){
    String json="{\"ir\":"+String(digitalRead(IR_PIN))+",\"roverSpeed\":"+String(roverSpeed)+"}";
    server.send(200,"application/json",json);
  });
  server.begin();
}

void loop(){
  server.handleClient();
  const unsigned long now=millis();
  if(cameraRunning&&now-cameraCommandTime>COMMAND_TIMEOUT)stopCamera();
  if(roverRunning&&now-roverCommandTime>COMMAND_TIMEOUT)stopRover();
}
