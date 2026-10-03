const DATA={observations:"training_view/observations.jsonl",view:"training_view/view.json",episode:"episode_source/episode.json",urdf:"episode_source/raw/interface/nero_description.urdf",source:"episode_source/"};
const LABELS=["接近","抓取","抬升","移动","放下","释放","停顿","失败动作"],ALIGN={hz:20};
// 界面显示中文，导出与记忆库存英文
const LABEL_EN={"接近":"approach","抓取":"grasp","抬升":"lift","移动":"move","放下":"place","释放":"release","停顿":"pause","失败动作":"failure"};
const LABEL_CN=Object.fromEntries(Object.entries(LABEL_EN).map(([cn,en])=>[en,cn]));
const TITLE_EN={"将黄色物块拾起并移动到P1区域":"Pick up the yellow cube and move it to the P1 area","将绿色物块拾起并移动到P2区域":"Pick up the green cube and move it to the P2 area","将蓝色物块拾起并移动到P3区域":"Pick up the blue cube and move it to the P3 area"};
// archive 模式：保留中文 title 并附 title_en（供记忆库/模板），否则 title 直接替换为英文（供下载文件）
const translateLabels=(obj,archive)=>{const out=JSON.parse(JSON.stringify(obj));out.processes=(out.processes||[]).map(p=>{const en=TITLE_EN[p.title]||p.title,tags=(p.tags||[]).map(t=>({...t,label:LABEL_EN[t.label]||t.label}));return archive?{...p,title_en:en,tags}:{...p,title:en,tags};});out.segments=(out.segments||[]).map(s=>({...s,label:LABEL_EN[s.label]||s.label,process_title:archive?s.process_title:(TITLE_EN[s.process_title]||s.process_title)}));return out;};
const PROCESS_TITLES=[["黄色物块 → P1","将黄色物块拾起并移动到P1区域"],["绿色物块 → P2","将绿色物块拾起并移动到P2区域"],["蓝色物块 → P3","将蓝色物块拾起并移动到P3区域"]];
let rows=[],view={},episode={},index=0,start=null,end=null,playing=false,timer=null,robotJoints=[],robotReady=false,imageUrls=new Map(),activeProcessId=null,boundarySuggestions=[],stageSegments=[];
const importedPaths=new WeakMap();
let batchMode=false,catalog=[],catalogIndex=-1,currentEpisodeName="",labelsStore=new Map(),draftWarnings=[],exportSeq=0,practiceMode=false,droppedSessions=new Map(),sessionPrompts=new Map();
let labels={schema_version:"nero.episode-process-labels.v2",source_view:"20hz_tcp_actions_v1",episode_outcome:"unreviewed",episode_note:"",processes:[]};
const $=id=>document.getElementById(id);
const showNumber=(value,digits=3)=>Number.isFinite(Number(value))?Number(value).toFixed(digits):"--";
const imagePath=path=>imageUrls.get(path)||DATA.source+path;
const currentRow=()=>rows[index];
const formatTime=()=> (index/ALIGN.hz).toFixed(2)+" s";
const xyz=pose=>pose&&pose.position_m?pose.position_m.map(value=>showNumber(value)).join(", "):"缺失";
function rpyDegrees(q){if(!Array.isArray(q)||q.length!==4)return "缺失";const [x,y,z,w]=q.map(Number),roll=Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),pitch=Math.asin(Math.max(-1,Math.min(1,2*(w*y-z*x)))),yaw=Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));return [roll,pitch,yaw].map(value=>showNumber(value*180/Math.PI,1)).join(", ");}
function gapBeforeCurrent(){if(index===0)return 0;return Math.max(0,Math.round((rows[index].target_monotonic_ns-rows[index-1].target_monotonic_ns)/(1e9/ALIGN.hz))-1);}
function identity(){return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];}
function multiply(a,b){return a.map(row=>b[0].map((_,j)=>row.reduce((sum,value,k)=>sum+value*b[k][j],0)));}
function originMatrix(position,rpy){const [r,p,y]=rpy,[sx,cx]=[Math.sin(r),Math.cos(r)],[sy,cy]=[Math.sin(p),Math.cos(p)],[sz,cz]=[Math.sin(y),Math.cos(y)];return [[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx,position[0]],[sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx,position[1]],[-sy,cy*sx,cy*cx,position[2]],[0,0,0,1]];}
function axisRotation(axis,angle){const [x,y,z]=axis,n=Math.hypot(x,y,z)||1,[u,v,w]=[x/n,y/n,z/n],c=Math.cos(angle),s=Math.sin(angle),d=1-c;return [[c+u*u*d,u*v*d-w*s,u*w*d+v*s,0],[v*u*d+w*s,c+v*v*d,v*w*d-u*s,0],[w*u*d-v*s,w*v*d+u*s,c+w*w*d,0],[0,0,0,1]];}
const vector=(text,fallback)=>String(text||fallback).trim().split(/\s+/).map(Number);
function parseUrdf(text){const xml=new DOMParser().parseFromString(text,"application/xml");if(xml.querySelector("parsererror"))throw new Error("URDF 格式无法解析");return [...xml.querySelectorAll("joint")].filter(joint=>/^joint[1-7]$/.test(joint.getAttribute("name"))||joint.getAttribute("name")==="end_effector_joint").map(joint=>{const origin=joint.querySelector("origin"),axis=joint.querySelector("axis");return {name:joint.getAttribute("name"),type:joint.getAttribute("type"),xyz:vector(origin?.getAttribute("xyz"),"0 0 0"),rpy:vector(origin?.getAttribute("rpy"),"0 0 0"),axis:vector(axis?.getAttribute("xyz"),"0 0 1")};});}
function jointPositions(angles){let transform=identity(),points=[[0,0,0]];robotJoints.forEach((joint,i)=>{transform=multiply(transform,originMatrix(joint.xyz,joint.rpy));points.push([transform[0][3],transform[1][3],transform[2][3]]);if(joint.type!=="fixed")transform=multiply(transform,axisRotation(joint.axis,Number(angles[i])||0));});return points;}
function drawRobot(){const canvas=$("robot-canvas");if(!canvas||!robotReady||!rows.length)return;const rect=canvas.getBoundingClientRect(),ratio=window.devicePixelRatio||1,width=Math.max(1,Math.round(rect.width*ratio)),height=Math.max(1,Math.round(rect.height*ratio));if(canvas.width!==width||canvas.height!==height){canvas.width=width;canvas.height=height;}const ctx=canvas.getContext("2d");ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,rect.width,rect.height);const obs=currentRow().observation||{},links=jointPositions(obs.joint_position_rad||[]),measured=obs.recorded_tcp_pose?.position_m||obs.tcp_pose?.position_m,target=obs.target_tcp_pose?.position_m,scale=Math.min(rect.width,rect.height)*.6;const project=point=>({x:rect.width*.53+(Number(point[0])-Number(point[1]))*scale*.72,y:rect.height*.84-Number(point[2])*scale*.88-(Number(point[0])+Number(point[1]))*scale*.28});const line=(a,b,color,widthValue=1,dash=[])=>{ctx.save();ctx.strokeStyle=color;ctx.lineWidth=widthValue;ctx.lineCap="round";ctx.setLineDash(dash);ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();ctx.restore();};const plane=[[-.6,-.6,0],[.6,-.6,0],[.6,.6,0],[-.6,.6,0]].map(project);ctx.save();ctx.beginPath();plane.forEach((p,i)=>i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y));ctx.closePath();ctx.fillStyle="rgba(111,150,158,.12)";ctx.fill();ctx.strokeStyle="rgba(126,171,177,.55)";ctx.stroke();ctx.restore();for(let i=1;i<5;i+=1){const x=-.6+1.2*i/5,y=-.6+1.2*i/5;line(project([x,-.6,0]),project([x,.6,0]),"rgba(126,171,177,.18)");line(project([-.6,y,0]),project([.6,y,0]),"rgba(126,171,177,.18)");}const base=project([0,0,0]);line(base,project([.08,0,0]),"#ff817a",2);line(base,project([0,.08,0]),"#59d9a2",2);line(base,project([0,0,.08]),"#78bdf0",2);for(let i=1;i<links.length;i+=1)line(project(links[i-1]),project(links[i]),i===links.length-1?"#e7eef3":"#71878b",i===links.length-1?6:5);if(Array.isArray(target)){const p=project(target);if(Array.isArray(measured))line(project(measured),p,"rgba(240,197,106,.85)",2,[7,5]);ctx.save();ctx.strokeStyle="#f0c56a";ctx.lineWidth=3;ctx.beginPath();ctx.arc(p.x,p.y,10,0,Math.PI*2);ctx.stroke();ctx.fillStyle="#f0c56a";ctx.font="bold 11px monospace";ctx.fillText("T_target",p.x+13,p.y+15);ctx.restore();}if(Array.isArray(measured)){const p=project(measured);ctx.save();ctx.fillStyle="#59d9a2";ctx.beginPath();ctx.arc(p.x,p.y,8,0,Math.PI*2);ctx.fill();ctx.fillStyle="#e7eef3";ctx.font="11px monospace";ctx.fillText("TCP",p.x+12,p.y-10);ctx.restore();}}
function renderQuality(){const raw=episode.raw_streams||{};$("aligned-frames").textContent=rows.length;$("aligned-coverage").textContent="保留完整对齐时间网格";$("rejected-frames").textContent="关闭";$("rejected-rate").textContent="不按时间差或动作幅度筛帧";$("raw-camera-drops").textContent=raw.camera_drops??"--";$("raw-robot-drops").textContent=raw.robot_state_drops??"--";}
// 候选边界与阶段色带：夹爪指令阶跃（抓取/释放，时间戳级精度）+ TCP 速度/垂直速度逐帧分类（接近/抬升/移动/放下/停顿）
const STAGE_COLORS={"接近":"#9aa7ff","抓取":"#f0c56a","抬升":"#c792ea","移动":"#59d9a2","放下":"#ff9e80","停顿":"#6b7f8e"};
function computeBoundarySuggestions(){
 boundarySuggestions=[];stageSegments=[];
 if(rows.length<2)return;
 const n=rows.length,pos=i=>{const p=rows[i].observation?.tcp_pose?.position_m;return Array.isArray(p)?p:null;};
 let open=Number(rows[0].observation?.target_gripper_opening_ratio)>0.5;
 const holding=new Array(n).fill(false);holding[0]=!open;
 rows.forEach((row,i)=>{const v=Number(row.observation?.target_gripper_opening_ratio);if(Number.isFinite(v)){if(open&&v<0.4){boundarySuggestions.push({frame:i,type:"close",color:"#f0c56a"});open=false;}else if(!open&&v>0.6){boundarySuggestions.push({frame:i,type:"open",color:"#59d9a2"});open=true;}}if(i>0)holding[i]=!open;});
 const speeds=[],vzs=[];
 for(let i=0;i<n;i+=1){
  const a=i>0?pos(i-1):null,b=i<n-1?pos(i+1):null;
  if(!a||!b){speeds.push(null);vzs.push(null);continue;}
  const dt=2/ALIGN.hz,vx=(b[0]-a[0])/dt*1000,vy=(b[1]-a[1])/dt*1000,vz=(b[2]-a[2])/dt*1000;
  speeds.push(Math.hypot(vx,vy,vz));vzs.push(vz);
 }
 const known=speeds.filter(v=>v!==null).sort((a,b)=>a-b),median=known.length?known[Math.floor(known.length/2)]:0;
 const pauseThr=Math.max(median*.3,2),moveThr=15;
 // 垂直/总速度做 ±3 帧平滑，抑制搬运时的自然上下起伏造成的抖动分类
 const smooth=(arr,k)=>arr.map((v,i)=>{if(v===null)return null;let s=0,c=0;for(let j=Math.max(0,i-k);j<=Math.min(n-1,i+k);j+=1){if(arr[j]!==null){s+=arr[j];c+=1;}}return s/c;});
 const spS=smooth(speeds,2),vzS=smooth(vzs,3);
 const labels=new Array(n).fill("接近");
 for(let i=1;i<n-1;i+=1){
  if(spS[i]===null){labels[i]=labels[i-1];continue;}
  if(spS[i]<pauseThr){labels[i]="停顿";continue;}
  if(vzS[i]>moveThr){labels[i]=holding[i]?"抬升":"接近";continue;}
  if(vzS[i]<-moveThr){labels[i]=holding[i]?"放下":"接近";continue;}
  labels[i]=holding[i]?"移动":"接近";
 }
 if(n>1)labels[n-1]=labels[n-2];
 const segs=[];labels.forEach((lab,i)=>{const last=segs[segs.length-1];if(last&&last.label===lab)last.end=i;else segs.push({start:i,end:i,label:lab});});
 const cleaned=[];
 segs.forEach(seg=>{const len=seg.end-seg.start+1;
  // 短于 0.4 s 的垂直运动视为搬运起伏，归入水平移动
  if((seg.label==="抬升"||seg.label==="放下")&&len<6)seg.label="移动";
  const minFrames=seg.label==="停顿"?ALIGN.hz:3;
  const prev=cleaned[cleaned.length-1];
  if(prev&&prev.label===seg.label){prev.end=seg.end;return;}
  if(prev&&len<minFrames){prev.end=seg.end;return;}
  cleaned.push({start:seg.start,end:seg.end,label:seg.label});});
 const merged=[];
 cleaned.forEach(seg=>{const prev=merged[merged.length-1];if(prev&&prev.label===seg.label){prev.end=seg.end;return;}merged.push(seg);});
 stageSegments=merged;
}
function renderBoundaryTicks(){const holder=$("boundary-ticks");if(!holder)return;holder.innerHTML="";if(!rows.length||!boundarySuggestions.length){holder.style.display="none";return;}holder.style.display="block";boundarySuggestions.forEach(sug=>{const tick=document.createElement("button");tick.type="button";tick.className="boundary-tick";tick.style.left=(rows.length>1?sug.frame/(rows.length-1)*100:0)+"%";tick.style.background=sug.color;tick.title=sug.type==="close"?"建议边界：抓取 · 帧 "+sug.frame+"（点击跳转）":"建议边界：释放 · 帧 "+sug.frame+"（点击跳转）";tick.addEventListener("click",()=>{index=sug.frame;render();});holder.append(tick);});}
function renderStageBand(){const holder=$("stage-band");if(!holder)return;holder.innerHTML="";if(!rows.length||!stageSegments.length){holder.style.display="none";return;}holder.style.display="block";const span=Math.max(rows.length-1,1);stageSegments.forEach(seg=>{const el=document.createElement("button");el.type="button";el.className="stage-seg";el.style.left=(seg.start/span*100)+"%";el.style.width=Math.max((seg.end-seg.start+1)/span*100,.35)+"%";el.style.background=STAGE_COLORS[seg.label]||"#71878b";el.title="建议阶段："+seg.label+" · 帧 "+seg.start+"–"+seg.end+"（"+(seg.start/ALIGN.hz).toFixed(2)+"–"+(seg.end/ALIGN.hz).toFixed(2)+" s）· 点击选中该区间";el.addEventListener("click",()=>{index=seg.start;start=seg.start;end=seg.end;render();});holder.append(el);});}
function render(){if(!rows.length)return;const row=currentRow(),obs=row.observation||{},align=row.alignment||{};$("external-image").src=imagePath(row.images.external);$("wrist-image").src=imagePath(row.images.wrist);$("timeline").value=index;$("frame-readout").textContent="帧 "+(index+1)+" / "+rows.length;$("time-readout").textContent=formatTime();$("range-start").textContent="区间开始："+(start===null?"--":start+" ("+(start/ALIGN.hz).toFixed(2)+" s)");$("range-end").textContent="区间结束："+(end===null?"--":end+" ("+(end/ALIGN.hz).toFixed(2)+" s)");$("gripper").textContent=showNumber(obs.gripper_opening_ratio*100,1)+"%";$("tcp-fk").textContent=xyz(obs.tcp_pose);$("tcp-next").textContent=row.tcp_pair?.valid?xyz(row.tcp_pair.next_tcp_pose):"无连续下一帧";$("tcp-measured").textContent=xyz(obs.recorded_tcp_pose);$("tcp-target").textContent=xyz(obs.target_tcp_pose);$("tcp-rpy").textContent=rpyDegrees(obs.tcp_pose?.orientation_xyzw);const err=align.camera_error_s;$("camera-error").textContent=err?"外 "+showNumber(err.external,4)+"s / 腕 "+showNumber(err.wrist,4)+"s":"--";$("joint-values").innerHTML=(obs.joint_position_rad||[]).map((joint,i)=>"<div><span>J"+(i+1)+"</span><b>"+showNumber(Number(joint)*180/Math.PI,1)+"°</b></div>").join("");const gap=gapBeforeCurrent();$("current-gap").textContent=gap?"缺 "+gap:"正常";$("current-gap-detail").textContent=index===0?"起始帧":"与前帧间隔 "+((row.target_monotonic_ns-rows[index-1].target_monotonic_ns)/1e9).toFixed(3)+" s";$("target-state").textContent=obs.target_tcp_pose?"目标 TCP 已记录":"目标 TCP 缺失";$("range-status").textContent=start===null||end===null?"未选择完整区间":Math.abs(end-start)+1+" 帧";$("action-state").textContent=row.action?"7 维动作已生成；H16 待第二阶段":"该帧无有效下一步动作";drawRobot();renderProcesses();}
const processById=id=>labels.processes.find(item=>item.id===id);
const makeId=()=>globalThis.crypto?.randomUUID?.()||"process-"+Date.now()+"-"+Math.random().toString(16).slice(2);
const selectedRange=()=>start===null||end===null?null:[Math.min(start,end),Math.max(start,end)];
const PROCESS_BAND_COLORS=["#59d9a2","#f0c56a","#78bdf0","#c792ea","#ff9e80","#9aa7ff"];let processBandSignature="";
// 一键草案：用记忆库模板 + 本条夹爪锚点 + 阶段色带，自动生成全部过程与阶段标签
async function generateDraft(){
 if(!rows.length)return alert("请先插入 Episode。");
 if(!currentEpisodeName)return alert("请先插入 Episode。");
 const res=await fetch("/api/get-template",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({source_episode:currentEpisodeName})}),data=await res.json();
 if(!data.found)return alert("记忆库中没有匹配该任务（prompt）的模板。\n先认真标注并导出 2~3 条同类 episode，模板成型后再来生成草案。");
 const t=data.template,closes=boundarySuggestions.filter(s=>s.type==="close").map(s=>s.frame),opens=boundarySuggestions.filter(s=>s.type==="open").map(s=>s.frame);
 if(!closes.length)return alert("本条未检测到夹爪闭合锚点，无法生成草案。");
 if(labels.processes.length&&!confirm("生成草案将替换当前 "+labels.processes.length+" 个过程，是否继续？"))return;
 draftWarnings=[];
 const span=rows.length,n=Math.min(closes.length,t.process_slots.length),bounds=[...new Set(stageSegments.flatMap(seg=>[seg.start,seg.end+1]))];
 const snap=v=>{let best=v,bd=21;for(const b of bounds){const d=Math.abs(b-v);if(d<bd){bd=d;best=b;}}return best;};
 const processes=[];
 for(let i=0;i<n;i+=1){
  const slot=t.process_slots[i];
  let s=Math.round(closes[i]+slot.start_offset_from_close_ms.median_ms*ALIGN.hz/1000);
  let e=opens[i]!==undefined?Math.round(opens[i]+slot.end_offset_from_open_ms.median_ms*ALIGN.hz/1000):(closes[i+1]!==undefined?closes[i+1]-1:span-1);
  s=Math.max(0,Math.min(s,span-2));e=Math.max(s+1,Math.min(e,span-1));
  s=Math.min(snap(s),e-1);e=Math.max(snap(e),s+1);
  const tags=[];
  stageSegments.forEach(seg=>{const a=Math.max(seg.start,s),b=Math.min(seg.end,e);if(b<a)return;
   const last=tags[tags.length-1];
   if(last&&last.label===seg.label){last.end_frame=b;last.end_time_s=b/ALIGN.hz;}
   else tags.push({label:seg.label,start_frame:a,end_frame:b,start_time_s:a/ALIGN.hz,end_time_s:b/ALIGN.hz});});
  tags.forEach(tag=>{const dur=(tag.end_frame-tag.start_frame+1)*1000/ALIGN.hz;
   const st=slot.phase_durations[LABEL_EN[tag.label]||tag.label];
   // 单样本无波动参考、小于 1s 的碎片不参与标红
   if(st&&st.n>=2&&dur>=1000){const iqr=st.p75_ms-st.p25_ms;if(dur<st.p25_ms-1.5*iqr||dur>st.p75_ms+1.5*iqr)draftWarnings.push({slot:i,label:tag.label,start_frame:tag.start_frame,note:"时长 "+(dur/1000).toFixed(1)+"s 偏离历史 "+(st.p25_ms/1000).toFixed(1)+"–"+(st.p75_ms/1000).toFixed(1)+"s（"+st.n+" 条历史）"});}});
  processes.push({id:makeId(),title:slot.title||("过程 "+(i+1)),start_frame:s,end_frame:e,tags});
 }
 labels.processes=processes.sort((a,b)=>a.start_frame-b.start_frame);activeProcessId=processes[0]?.id||null;
 render();
 $("import-status").textContent="草案已生成："+processes.length+" 个过程（按锚点对齐、边界吸附到阶段信号）"+(draftWarnings.length?"；⚠ "+draftWarnings.length+" 处时长偏离历史区间，见标签卡片。":"；各阶段时长均在历史区间内。")+" 请逐段确认后导出。";
}
function renderProcessBand(){const holder=$("process-band");if(!holder)return;const sig=rows.length+"#"+labels.processes.map(p=>p.id+":"+p.start_frame+"-"+p.end_frame+":"+p.title+":"+(p.id===activeProcessId?1:0)).join("|");if(sig===processBandSignature)return;processBandSignature=sig;holder.innerHTML="";if(!rows.length||!labels.processes.length){holder.style.display="none";return;}holder.style.display="block";const span=Math.max(rows.length-1,1);labels.processes.forEach((p,i)=>{const el=document.createElement("button");el.type="button";el.className="process-seg"+(p.id===activeProcessId?" active":"");el.style.left=(p.start_frame/span*100)+"%";el.style.width=Math.max((p.end_frame-p.start_frame+1)/span*100,1.2)+"%";el.style.background=PROCESS_BAND_COLORS[i%PROCESS_BAND_COLORS.length];el.title=p.title+" · 帧 "+p.start_frame+"–"+p.end_frame+"（"+(p.start_frame/ALIGN.hz).toFixed(2)+"–"+(p.end_frame/ALIGN.hz).toFixed(2)+" s）· 点击设为当前过程并跳转";el.textContent=p.title;el.addEventListener("click",()=>{activeProcessId=p.id;index=p.start_frame;render();});holder.append(el);});}
function renderProcesses(){renderProcessBand();const holder=$("segments");holder.innerHTML="";const active=processById(activeProcessId);$("active-process-state").textContent=active?"当前过程："+active.title+"。选择标签片段的开始和结束帧后，点击过程标签即可加入此过程。":"尚未创建过程。先选择完整过程的开始和结束帧，填写过程标题，再点击“创建过程”。";if(!labels.processes.length){holder.className="segments empty";holder.textContent="尚未创建标注过程。";return;}holder.className="segments";labels.processes.forEach(process=>{const card=document.createElement("article");card.className="process"+(process.id===activeProcessId?" active":"");const head=document.createElement("div");head.className="process-head";const text=document.createElement("div"),title=document.createElement("strong"),range=document.createElement("small");title.textContent=process.title;range.textContent="帧 "+process.start_frame+" – "+process.end_frame+" · "+(process.start_frame/ALIGN.hz).toFixed(2)+"s – "+(process.end_frame/ALIGN.hz).toFixed(2)+"s";text.append(title,range);const actions=document.createElement("div");actions.className="process-actions";const choose=document.createElement("button");choose.textContent=process.id===activeProcessId?"当前过程":"设为当前";choose.className="ghost";choose.addEventListener("click",()=>{activeProcessId=process.id;renderProcesses();});const remove=document.createElement("button");remove.textContent="删除";remove.className="delete";remove.addEventListener("click",()=>{labels.processes=labels.processes.filter(item=>item.id!==process.id);if(activeProcessId===process.id)activeProcessId=null;renderProcesses();});actions.append(choose,remove);head.append(text,actions);card.append(head);const tags=document.createElement("div");tags.className="process-tags";if(!process.tags.length){const empty=document.createElement("small");empty.textContent="尚未添加标签片段。";tags.append(empty);}process.tags.forEach((tag,tagIndex)=>{const tagRow=document.createElement("div");tagRow.className="process-tag";const badge=document.createElement("span");badge.className="badge";badge.textContent=tag.label+(draftWarnings.some(w=>w.label===tag.label&&w.start_frame===tag.start_frame)?" ⚠":"");if(draftWarnings.some(w=>w.label===tag.label&&w.start_frame===tag.start_frame)){badge.title=draftWarnings.find(w=>w.label===tag.label&&w.start_frame===tag.start_frame).note;}const info=document.createElement("span");info.textContent="帧 "+tag.start_frame+" – "+tag.end_frame;const del=document.createElement("button");del.className="delete";del.textContent="删除";del.addEventListener("click",()=>{process.tags.splice(tagIndex,1);renderProcesses();});tagRow.append(badge,info,del);tags.append(tagRow);});card.append(tags);holder.append(card);});}
function createProcess(){const range=selectedRange(),title=$("process-title").value.trim();if(!range)return alert("请先设定完整过程的开始帧和结束帧。");if(!title)return alert("请为完整过程填写描述性标题。");const process={id:makeId(),title,start_frame:range[0],end_frame:range[1],tags:[]};labels.processes.push(process);labels.processes.sort((a,b)=>a.start_frame-b.start_frame);activeProcessId=process.id;$("process-title").value="";start=end=null;render();}
function addSegment(label){const process=processById(activeProcessId),range=selectedRange();if(!process)return alert("请先创建过程，或在“已标注阶段”中设为当前过程。");if(!range)return alert("请先用“设为开始”和“设为结束”选择标签片段。");if(range[0]<process.start_frame||range[1]>process.end_frame)return alert("标签片段必须位于当前过程的完整时间范围内。");process.tags.push({label,start_frame:range[0],end_frame:range[1],start_time_s:range[0]/ALIGN.hz,end_time_s:range[1]/ALIGN.hz});process.tags.sort((a,b)=>a.start_frame-b.start_frame);start=end=null;render();}
function exportLabels(quiet){labels.episode_outcome=document.querySelector('input[name="outcome"]:checked').value;labels.episode_note=$("episode-note").value.trim();labels.updated_at=new Date().toISOString();labels.total_sensor_frames=rows.length;labels.segments=labels.processes.flatMap(process=>process.tags.map(tag=>Object.assign({},tag,{process_id:process.id,process_title:process.title})));const exportData=translateLabels(labels);const blob=new Blob([JSON.stringify(exportData,null,2)],{type:"application/json"}),link=document.createElement("a");link.href=URL.createObjectURL(blob);link.download="labels.json";link.click();URL.revokeObjectURL(link.href);if(currentEpisodeName&&!practiceMode){const seq=++exportSeq;fetch("/api/save-labels",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({source_episode:currentEpisodeName,labels:translateLabels(labels,true)})}).then(r=>r.json()).then(res=>{if(res&&res.saved&&res.template_episodes&&!quiet&&exportSeq===seq)$("import-status").textContent="labels.json 已导出（标签与标题为英文），并已写入记忆库（该任务模板现有 "+res.template_episodes+" 条标注）。";}).catch(()=>{});}
 else if(currentEpisodeName&&practiceMode&&!quiet)$("import-status").textContent="labels.json 已导出（练习模式：只下载，未写入记忆库）。";}
function normalizeLabels(value){const source=value&&typeof value==="object"?value:{},processes=Array.isArray(source.processes)?source.processes.map(item=>Object.assign({},item,{id:item.id||makeId(),title:item.title||"未命名过程",tags:Array.isArray(item.tags)?item.tags.map(tag=>Object.assign({},tag,{label:LABEL_CN[tag.label]||tag.label})):[]})):[];if(!processes.length&&Array.isArray(source.segments))source.segments.forEach(segment=>processes.push({id:makeId(),title:segment.note||segment.label||"已导入过程",start_frame:segment.start_frame,end_frame:segment.end_frame,tags:[{label:LABEL_CN[segment.label]||segment.label,start_frame:segment.start_frame,end_frame:segment.end_frame,start_time_s:segment.start_time_s,end_time_s:segment.end_time_s}]}));return {schema_version:"nero.episode-process-labels.v2",source_view:source.source_view||"20hz_tcp_actions_v1",episode_outcome:source.episode_outcome||"unreviewed",episode_note:source.episode_note||"",processes};}
function nearest(items,times,target){let after=times.findIndex(value=>value>=target),choices=[after-1,after].filter(item=>item>=0&&item<times.length),selected=choices.reduce((best,item)=>best===null||Math.abs(times[item]-target)<Math.abs(times[best]-target)?item:best,null);return [items[selected],Math.abs(times[selected]-target)];}
function interpolateRobot(items,times,target){const after=times.findIndex(value=>value>=target);if(after<=0||after>=times.length)return null;const left=items[after-1],right=items[after],span=times[after]-times[after-1];if(span<=0)return null;const alpha=(target-times[after-1])/span,blend=key=>(left[key]||[]).map((value,i)=>(1-alpha)*Number(value)+alpha*Number(right[key]?.[i]??value));return {joint_position_rad:blend("joint_position_rad"),joint_velocity_rad_s:blend("joint_velocity_rad_s"),gripper_opening_ratio:(1-alpha)*Number(left.gripper_opening_ratio)+alpha*Number(right.gripper_opening_ratio),target_gripper_opening_ratio:(1-alpha)*Number(left.target_gripper_opening_ratio??left.gripper_opening_ratio)+alpha*Number(right.target_gripper_opening_ratio??right.gripper_opening_ratio),left,right,leftTime:times[after-1],rightTime:times[after]};}
function buildRowsFromRaw(metadata,cameraRows,robotRows){
  const streams={external:[],wrist:[]};
  cameraRows.forEach(row=>{if(streams[row.source])streams[row.source].push(row);});
  Object.values(streams).forEach(list=>list.sort((a,b)=>a.capture_monotonic_ns-b.capture_monotonic_ns));
  robotRows.sort((a,b)=>a.feedback_monotonic_ns-b.feedback_monotonic_ns);
  if(!streams.external.length||!streams.wrist.length||!robotRows.length)throw new Error("缺少外部相机、腕部相机或机器人状态流");
  const bounds=metadata.collection_monotonic_ns||{},lower=bounds.start||-Infinity,upper=bounds.end||Infinity;
  Object.keys(streams).forEach(source=>streams[source]=streams[source].filter(row=>row.capture_monotonic_ns>=lower&&row.capture_monotonic_ns<=upper));
  robotRows=robotRows.filter(row=>row.feedback_monotonic_ns>=lower&&row.feedback_monotonic_ns<=upper);
  const cameraTimes=Object.fromEntries(Object.entries(streams).map(([key,list])=>[key,list.map(row=>row.capture_monotonic_ns)]));
  const robotTimes=robotRows.map(row=>row.feedback_monotonic_ns);
  const begin=Math.max(cameraTimes.external[0],cameraTimes.wrist[0],robotTimes[0]);
  const finish=Math.min(cameraTimes.external.at(-1),cameraTimes.wrist.at(-1),robotTimes.at(-1));
  const period=Math.round(1e9/ALIGN.hz),aligned=[];let rejected=0;
  for(let target=begin;target<=finish;target+=period){
    const cameras={external:nearest(streams.external,cameraTimes.external,target),wrist:nearest(streams.wrist,cameraTimes.wrist,target)};
    const robot=interpolateRobot(robotRows,robotTimes,target);
    if(!robot){rejected+=1;continue;}
    const raw=Math.abs(target-robot.leftTime)<Math.abs(robot.rightTime-target)?robot.left:robot.right,tcp=raw.measured_tcp_pose||null;
    aligned.push({sensor_frame_index:aligned.length,target_monotonic_ns:target,prompt:metadata.prompt||"",images:{external:cameras.external[0].image,wrist:cameras.wrist[0].image},observation:{joint_position_rad:robot.joint_position_rad,joint_velocity_rad_s:robot.joint_velocity_rad_s,gripper_opening_ratio:robot.gripper_opening_ratio,target_gripper_opening_ratio:robot.target_gripper_opening_ratio,tcp_pose:tcp,recorded_tcp_pose:tcp,target_tcp_pose:raw.target_tcp_pose||null},alignment:{raw_camera_frame_index:{external:cameras.external[0].source_frame_index,wrist:cameras.wrist[0].source_frame_index},camera_error_s:{external:cameras.external[1]/1e9,wrist:cameras.wrist[1]/1e9},robot_left_sample_index:robot.left.sample_index,robot_right_sample_index:robot.right.sample_index}});
  }
  return {rows:aligned,view:{schema_version:"nero.tcp-vla.training-view.v1",rate_hz:ALIGN.hz,sensor_frames:aligned.length,rejected_grid_points:rejected,threshold_filtering:"disabled"}};
}
function makeFileMap(files){const map=new Map();[...files].forEach(file=>{const path=(importedPaths.get(file)||file.webkitRelativePath||file.name).replace(/\\/g,"/"),parts=path.split("/"),episodeIndex=parts.findIndex(part=>/^episode_\d+$/.test(part));map.set(episodeIndex>=0?parts.slice(episodeIndex).join("/"):parts.slice(1).join("/")||path,file);});return map;}
function selectedEpisodeName(metadataFile){const parts=(importedPaths.get(metadataFile)||metadataFile.webkitRelativePath||"").replace(/\\/g,"/").split("/");const name=parts.length>=2?parts[parts.length-2]:"";if(!/^episode_\d+$/.test(name))throw new Error("所选文件夹必须是包含 episode.json 的 episode_编号 目录");return name;}
async function loadImportedEpisode(files){const map=makeFileMap(files),metadataEntry=[...map.entries()].find(([path])=>path==="episode.json"||path.endsWith("/episode.json")),metadataPath=metadataEntry?.[0],metadataFile=metadataEntry?.[1];if(!metadataFile)throw new Error("请选择一个包含 episode.json 的 episode 文件夹");const prefix=metadataPath.slice(0,-"episode.json".length),lookup=path=>map.get(prefix+path),metadata=JSON.parse(await metadataFile.text()),cameraFile=lookup(metadata.files?.raw_camera),robotFile=lookup(metadata.files?.raw_robot_state),urdfFile=lookup(metadata.processing_interface?.urdf||"raw/interface/nero_description.urdf");if(!cameraFile||!robotFile||!urdfFile)throw new Error("所选文件夹缺少相机清单、机器人状态或 URDF");const cameraRows=(await cameraFile.text()).trim().split(/\r?\n/).filter(Boolean).map(JSON.parse),robotRows=(await robotFile.text()).trim().split(/\r?\n/).filter(Boolean).map(JSON.parse);imageUrls.forEach(url=>URL.revokeObjectURL(url));imageUrls=new Map();cameraRows.forEach(row=>{const file=lookup(row.image);if(file)imageUrls.set(row.image,URL.createObjectURL(file));});const result=buildRowsFromRaw(metadata,cameraRows,robotRows);rows=result.rows;view=result.view;episode=metadata;robotJoints=parseUrdf(await urdfFile.text());robotReady=robotJoints.length===8;if(!robotReady)throw new Error("URDF 中的关节链不完整");if(!rows.length)throw new Error("原始数据无法生成有效的 20 Hz 帧");index=0;start=end=null;$("timeline").max=rows.length-1;$("load-state").textContent="已插入 "+prefix.split("/").filter(Boolean).at(-1)+"："+rows.length+" 个 20 Hz 对齐观测帧";$("import-status").textContent="已在浏览器本地完成 20 Hz 对齐；原始数据未被修改。";$("robot-state").textContent="已加载导入数据的 URDF · J1–J7 与 TCP 同步";renderQuality();render();}
// 选中父文件夹（含多个 episode_编号 子目录）时进入批量模式；只读 episode.json 建目录，切换到哪组才加载哪组
function groupEpisodeFiles(map){
  if([...map.keys()].some(path=>path==="episode.json"))return null;
  const groups=new Map();
  [...map.entries()].forEach(([path,file])=>{const slash=path.indexOf("/");if(slash<0)return;const name=path.slice(0,slash);if(/^episode_\d+$/.test(name)){let group=groups.get(name);if(!group){group=new Map();groups.set(name,group);}group.set(path.slice(slash+1),file);}});
  return groups.size>1?groups:null;
}
// 会话式导入：单集/多集/父文件夹都累积进同一会话；同名 episode 覆盖需确认；旧格式（无 episode.json）跳过
async function mergeGroupsIntoSession(groups){
  const added=[];let skipped=0;
  for(const [name,g] of groups.entries()){
    if(!g.get("episode.json")){skipped+=1;continue;}
    if(droppedSessions.has(name)){
      if(!confirm("会话中已有同名 "+name+"，覆盖其文件与已标注内容？"))continue;
      droppedSessions.set(name,g);labelsStore.delete(name);sessionPrompts.delete(name);
    }else{
      droppedSessions.set(name,g);
      let prompt="";
      try{prompt=String(JSON.parse(await g.get("episode.json").text()).prompt||"");}catch{}
      sessionPrompts.set(name,prompt);
    }
    added.push(name);
  }
  return {added,skipped};
}
function refreshSessionCatalog(){
  catalog=[...droppedSessions.keys()].sort().map(name=>({name,prompt:sessionPrompts.get(name)||""}));
  const sel=$("batch-select");sel.innerHTML="";
  catalog.forEach((item,i)=>{const opt=document.createElement("option");opt.value=String(i);sel.append(opt);});
  $("batch-nav").hidden=catalog.length<2;
}
async function loadImportedEpisode(files){
  const map=makeFileMap(files),groups=groupEpisodeFiles(map);
  let sessionGroups=groups;
  if(!groups){
    const entry=[...map.entries()].find(([path])=>path==="episode.json"||path.endsWith("/episode.json"));
    if(!entry)throw new Error("请选择包含 episode.json 的单个 Episode 文件夹");
    const name=selectedEpisodeName(entry[1]);
    sessionGroups=new Map([[name,new Map([...map.entries()].map(([p,f])=>[p.startsWith(name+"/")?p.slice(name.length+1):p,f]))]]);
  }
  const {added,skipped}=await mergeGroupsIntoSession(sessionGroups);
  if(!added.length){$("import-status").textContent="未新增 episode"+(skipped?"（"+skipped+" 组旧格式已跳过）":"（同名导入已被取消）")+"。";return;}
  batchMode=true;
  refreshSessionCatalog();
  $("load-state").textContent="批量模式：会话内共 "+catalog.length+" 组"+(skipped?"（另有 "+skipped+" 组旧格式已跳过）":"");
  const target=catalog.findIndex(c=>c.name===added[added.length-1]);
  await loadCatalogEpisode(target>=0?target:0);
}
async function loadCatalogEpisode(i){
  if(i<0||i>=catalog.length)return;
  if(catalogIndex>=0&&currentEpisodeName&&currentEpisodeName!==catalog[i].name)labelsStore.set(currentEpisodeName,labels);
  catalogIndex=i;const name=catalog[i].name;
  $("import-status").textContent="正在加载 "+name+"…";
  try{await loadSingleEpisode(droppedSessions.get(name),name);}
  catch(error){$("import-status").textContent=name+" 加载失败："+error.message;}
  refreshBatchNav();
}
function refreshBatchNav(){
  const sel=$("batch-select");
  sel.value=String(Math.max(catalogIndex,0));
  [...sel.options].forEach((opt,i)=>{const lb=labelsStore.get(catalog[i].name);opt.textContent=(lb&&lb.processes.length?"● ":"○ ")+catalog[i].name+(catalog[i].prompt?" · "+catalog[i].prompt:"");});
  const labeled=[...labelsStore.values()].filter(lb=>lb.processes.length).length;
  $("batch-progress").textContent="第 "+(catalogIndex+1)+" / "+catalog.length+" 组 · 已标 "+labeled+" 组";
  $("batch-nav").hidden=catalog.length<2;
}
// 任意文件夹支持：拖入的 episode 若不在数据根目录，自动分批上传入库，之后走统一处理管道
async function postImportJson(path,body,stage){
  let response;
  try{response=await fetch(path,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body)});}
  catch{throw new Error(stage+"：工作台服务未响应，请确认 http://127.0.0.1:8790/ 可以打开。");}
  const data=await response.json().catch(()=>({}));
  if(!response.ok)throw new Error(stage+"："+(data.error||"服务返回 "+response.status));
  return data;
}
async function ensureEpisodeOnServer(name,episodeMap){
  const entries=[...episodeMap.entries()].map(([path,file])=>[path.startsWith(name+"/")?path.slice(name.length+1):path,file]);
  const metadataFile=entries.find(([path])=>path==="episode.json")?.[1];
  if(!metadataFile)throw new Error(name+" 缺少 episode.json");
  const metadata=JSON.parse(await metadataFile.text()),files=entries.map(([path,file])=>({path,size:file.size})),request={name,metadata,files};
  const probe=await postImportJson("/api/episode-exists",request,"检查 "+name);
  if(probe.complete)return;
  const overwrite=Boolean(probe.conflict);
  if(overwrite&&!confirm("数据根目录已有同名 "+name+"，但 episode.json 内容不同。用拖入的版本覆盖？"))throw new Error("已取消导入（同名 episode 内容不同）");
  await postImportJson("/api/import-begin",{name,metadata,overwrite},"准备导入 "+name);
  const needed=overwrite?entries:entries.filter(([path])=>probe.missing.includes(path)),BATCH=32;
  for(let i=0;i<needed.length;i+=BATCH){
    const fd=new FormData();
    needed.slice(i,i+BATCH).forEach(([path,file])=>fd.append("path:"+path,file,path));
    let uploaded=false;
    for(let attempt=0;attempt<3&&!uploaded;attempt++){
      try{
        const r=await fetch("/api/import-files",{method:"POST",headers:{"X-Episode-Name":name,"X-Overwrite":overwrite?"1":"0"},body:fd});
        const data=await r.json().catch(()=>({}));
        if(!r.ok)throw new Error(data.error||"服务返回 "+r.status);
        uploaded=true;
      }catch(error){
        if(attempt===2)throw new Error("上传 "+name+" 第 "+(i+1)+"–"+Math.min(i+BATCH,needed.length)+" 个文件失败："+error.message+"。已上传文件可在服务恢复后继续导入。");
        await new Promise(resolve=>setTimeout(resolve,500*(attempt+1)));
      }
    }
    $("import-status").textContent="正在把 "+name+" 入库到数据根目录（"+Math.min(i+BATCH,needed.length)+"/"+needed.length+" 个待上传文件）…";
  }
  await postImportJson("/api/import-end",request,"核验 "+name);
}
async function loadSingleEpisode(episodeMap,name){
  const entry=[...episodeMap.entries()].find(([path])=>path==="episode.json"||path.endsWith("/episode.json"));
  if(!entry)throw new Error(name+" 缺少 episode.json");
  const prefix=entry[0].slice(0,-"episode.json".length),lookup=path=>episodeMap.get(prefix+path)||episodeMap.get(path);
  await ensureEpisodeOnServer(name,episodeMap);
  const metadata=JSON.parse(await entry[1].text()),cameraFile=lookup(metadata.files?.raw_camera),urdfFile=lookup(metadata.processing_interface?.urdf||"raw/interface/nero_description.urdf");
  sessionPrompts.set(name,String(metadata.prompt||""));const catEntry=catalog.find(x=>x.name===name);if(catEntry)catEntry.prompt=String(metadata.prompt||"");
  if(!cameraFile||!urdfFile)throw new Error(name+" 缺少相机清单或 URDF");
  const cameraRows=(await cameraFile.text()).trim().split(/\r?\n/).filter(Boolean).map(JSON.parse);
  imageUrls.forEach(url=>URL.revokeObjectURL(url));imageUrls=new Map();
  cameraRows.forEach(item=>{const file=lookup(item.image);if(file)imageUrls.set(item.image,URL.createObjectURL(file));});
  const result=await postImportJson("/api/build-tcp-view",{source_episode:name},"生成 "+name+" 的观测视图");
  rows=result.observations;view=result.view;episode=metadata;robotJoints=parseUrdf(await urdfFile.text());robotReady=robotJoints.length===8;
  if(!robotReady)throw new Error("URDF 中的关节链不完整");
  if(!rows.length)throw new Error("原始数据无法生成有效的 20 Hz 观测");
  index=0;start=end=null;currentEpisodeName=name;draftWarnings=[];
  labels=labelsStore.get(name)||{schema_version:"nero.episode-process-labels.v2",source_view:"20hz_tcp_actions_v1",episode_outcome:"unreviewed",episode_note:"",processes:[]};
  labelsStore.set(name,labels);
  $("timeline").max=rows.length-1;$("load-state").textContent="已插入 "+name+"："+rows.length+" 个观测帧";
  computeBoundarySuggestions();renderBoundaryTicks();renderStageBand();
  $("import-status").textContent="原始反馈已先执行 FK，再对齐到 20 Hz；已生成 "+view.control_steps+" 步 7 维动作，不按相机/反馈时间差或动作幅度筛帧。时间轴下方：刻度=夹爪事件（黄=抓取，绿=释放），色带=建议阶段（点击色块选中该区间，再点标签按钮即可入段）。H16 留待第二阶段。";
  $("robot-state").textContent="FK 先于 20 Hz 对齐 · URDF 已加载";renderQuality();render();
  if(batchMode)refreshBatchNav();
}
function refreshBatchNav(){
  const sel=$("batch-select");
  sel.value=String(Math.max(catalogIndex,0));
  [...sel.options].forEach((opt,i)=>{const lb=labelsStore.get(catalog[i].name);opt.textContent=(lb&&lb.processes.length?"● ":"○ ")+catalog[i].name+(catalog[i].prompt?" · "+catalog[i].prompt:"");});
  const labeled=[...labelsStore.values()].filter(lb=>lb.processes.length).length;
  $("batch-progress").textContent="第 "+(catalogIndex+1)+" / "+catalog.length+" 组 · 已标 "+labeled+" 组";
}
function exportAllLabels(){
  const all=[...labelsStore.entries()].filter(([,lb])=>lb.processes.length);
  if(!all.length)return alert("还没有已标注的过程。");
  all.forEach(([name,lb],i)=>{setTimeout(()=>{const blob=new Blob([JSON.stringify(translateLabels(lb),null,2)],{type:"application/json"}),link=document.createElement("a");link.href=URL.createObjectURL(blob);link.download="labels_"+name+".json";link.click();URL.revokeObjectURL(link.href);},i*400);});
}
async function load(){try{throw new Error("请点击‘插入数据’选择一个原始 Episode");}catch(error){$("load-state").textContent=error.message;}}
$("timeline").addEventListener("input",event=>{index=Number(event.target.value);render();});$("play").addEventListener("click",()=>setPlaying(!playing));$("speed").addEventListener("change",()=>playing&&setPlaying(true));function setPlaying(value){playing=value;$("play").textContent=value?"❚❚":"▶";clearInterval(timer);if(value)timer=setInterval(()=>{index=index>=rows.length-1?0:index+1;render();},1000/(ALIGN.hz*Number($("speed").value)));}
$("set-start").addEventListener("click",()=>{start=index;render();});$("set-end").addEventListener("click",()=>{end=index;render();});$("clear-range").addEventListener("click",()=>{start=end=null;render();});$("create-process").addEventListener("click",createProcess);$("generate-draft").addEventListener("click",generateDraft);$("import-labels").addEventListener("click",()=>$("import-file").click());$("insert-data").addEventListener("click",()=>$("dataset-import").click());
$("dataset-import").addEventListener("change",async event=>{const files=event.target.files;if(!files.length)return;$("import-status").textContent="正在读取并对齐原始 Episode…";try{await loadImportedEpisode(files);}catch(error){$("import-status").textContent="插入失败："+error.message;}finally{event.target.value="";}});
// 拖拽插入：支持同时拖入一个或多个 episode 文件夹（或其父文件夹），走与"插入数据"相同的分组管道
function addDroppedFile(file,path,out){importedPaths.set(file,path);out.push(file);}
async function traverseHandle(handle,pathPrefix,out){
  if(handle.kind==="file"){addDroppedFile(await handle.getFile(),pathPrefix+handle.name,out);return;}
  for await(const child of handle.values())await traverseHandle(child,pathPrefix+handle.name+"/",out);
}
async function traverseEntry(entry,pathPrefix,out){
  if(entry.isFile){
    const file=await new Promise((resolve,reject)=>entry.file(resolve,reject));
    addDroppedFile(file,pathPrefix+entry.name,out);return;
  }
  const reader=entry.createReader();
  for(;;){
    const batch=await new Promise((resolve,reject)=>reader.readEntries(resolve,reject));
    if(!batch.length)break;
    for(const child of batch)await traverseEntry(child,pathPrefix+entry.name+"/",out);
  }
}
async function handleDrop(event){
  const items=[...(event.dataTransfer?.items||[])].filter(item=>item.kind==="file");
  const candidates=items.map(item=>{
    let handlePromise=null,entry=null;
    try{if(typeof item.getAsFileSystemHandle==="function")handlePromise=item.getAsFileSystemHandle();}catch{}
    try{if(typeof item.webkitGetAsEntry==="function")entry=item.webkitGetAsEntry();}catch{}
    return {handlePromise,entry};
  });
  if(!candidates.length)return;
  $("import-status").textContent="正在读取拖入的 "+candidates.length+" 个文件夹…";
  try{
    const handles=await Promise.all(candidates.map(async item=>{try{return item.handlePromise?await item.handlePromise:null;}catch{return null;}}));
    const files=[];
    for(let i=0;i<candidates.length;i++){
      if(handles[i])await traverseHandle(handles[i],"upload/",files);
      else if(candidates[i].entry)await traverseEntry(candidates[i].entry,"upload/",files);
      else throw new Error("浏览器无法读取所拖文件夹，请用“插入数据”选择文件夹，并确认页面从 http://127.0.0.1:8790/ 打开。");
    }
    if(!files.length)throw new Error("拖入的内容里没有文件");
    await loadImportedEpisode(files);
  }catch(error){$("import-status").textContent="插入失败："+(error.name==="EncodingError"?"浏览器旧式拖拽接口无法读取此文件夹；请用“插入数据”选择文件夹，并确认页面从 http://127.0.0.1:8790/ 打开。":error.message);}
}
document.addEventListener("dragover",event=>{event.preventDefault();document.body.classList.add("drop-hover");$("drop-zone").classList.add("drop-hot");});
document.addEventListener("dragleave",event=>{if(!event.relatedTarget){document.body.classList.remove("drop-hover");$("drop-zone").classList.remove("drop-hot");}});
document.addEventListener("drop",event=>{event.preventDefault();document.body.classList.remove("drop-hover");$("drop-zone").classList.remove("drop-hot");handleDrop(event);});
$("drop-zone").addEventListener("click",()=>$("dataset-import").click());
$("import-file").addEventListener("change",async event=>{const file=event.target.files[0];if(!file)return;try{const imported=JSON.parse(await file.text());if(/^15hz(?:_|$)/i.test(String(imported.source_view||"")))throw new Error("15 Hz 标签帧号不能直接用于 20 Hz 视图，请按原始时间重新确认边界。");labels=normalizeLabels(imported);if(batchMode&&currentEpisodeName)labelsStore.set(currentEpisodeName,labels);activeProcessId=labels.processes[0]?.id||null;$("episode-note").value=labels.episode_note||"";const outcome=document.querySelector('input[name="outcome"][value="'+labels.episode_outcome+'"]');if(outcome)outcome.checked=true;renderProcesses();}catch(error){alert(error.message||"无法读取 labels.json");}});
$("batch-prev").addEventListener("click",()=>loadCatalogEpisode(catalogIndex-1));$("batch-next").addEventListener("click",()=>loadCatalogEpisode(catalogIndex+1));$("batch-select").addEventListener("change",event=>loadCatalogEpisode(Number(event.target.value)));$("batch-export-all").addEventListener("click",exportAllLabels);
$("practice-mode").addEventListener("click",()=>{practiceMode=!practiceMode;$("practice-mode").textContent="练习模式："+(practiceMode?"开":"关");$("practice-mode").classList.toggle("practice-on",practiceMode);$("import-status").textContent=practiceMode?"练习模式已开启：导出只下载 labels.json，不写入记忆库——随便测，不会污染模板。":"练习模式已关闭：导出将正常写入记忆库。";});
$("clear-memory").addEventListener("click",async()=>{if(!confirm("确定清空标注记忆库？所有已存档标注和任务模板将被删除（不影响原始数据和已下载的 labels.json）。"))return;$("clear-memory").disabled=true;try{const r=await fetch("/api/clear-memory",{method:"POST",headers:{"Content-Type":"application/json"},body:"{}"}),d=await r.json();$("import-status").textContent="记忆库已清空（删除 "+d.removed+" 个文件）。下次导出将从零开始重建模板。";}catch(e){$("import-status").textContent="清空失败："+e.message;}$("clear-memory").disabled=false;});
LABELS.forEach(label=>{const button=document.createElement("button");button.textContent=label;button.addEventListener("click",()=>addSegment(label));$("label-buttons").append(button);});
PROCESS_TITLES.forEach(([shortLabel,title])=>{const button=document.createElement("button");button.textContent=shortLabel;button.title=title;button.addEventListener("click",()=>{$("process-title").value=title;});$("title-presets").append(button);});window.addEventListener("resize",drawRobot);document.addEventListener("keydown",event=>{if(event.target.matches("textarea,input,select"))return;if(event.code==="Space"){event.preventDefault();setPlaying(!playing);}if(event.key==="ArrowLeft"){index=Math.max(0,index-1);render();}if(event.key==="ArrowRight"){index=Math.min(rows.length-1,index+1);render();}if(batchMode&&event.key==="["){loadCatalogEpisode(catalogIndex-1);}if(batchMode&&event.key==="]"){loadCatalogEpisode(catalogIndex+1);}});load();
// TCP FK preprocessing is performed by the local server.
// Import now performs FK on raw feedback rows first, then server-side time alignment.
