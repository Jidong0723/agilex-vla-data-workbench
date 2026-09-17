const DATA = {
  observations: "training_view/observations.jsonl",
  view: "training_view/view.json",
  episode: "episode_source/episode.json",
  urdf: "episode_source/raw/interface/nero_description.urdf",
  source: "episode_source/",
};
const LABELS = ["接近", "抓取", "抬升", "移动", "放下", "释放", "停顿", "失败动作"];
let rows = [], view = {}, episode = {}, index = 0, start = null, end = null, playing = false, timer = null, robotJoints = [], robotReady = false;
const camera = { yaw: -0.76, pitch: 0.54, zoom: 1 };
let labels = { schema_version: "nero.episode-labels.v1", source_view: "15hz_v1", episode_outcome: "unreviewed", episode_note: "", segments: [] };
const $ = (id) => document.getElementById(id);

function showNumber(value, digits = 3) { return Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : "--"; }
function imagePath(relative) { return `${DATA.source}${relative}`; }
function currentRow() { return rows[index]; }
function formatTime(row) { return `${(index / 15).toFixed(2)} s`; }
function xyz(pose) { const p = pose?.position_m; return p ? p.map(v => showNumber(v)).join(", ") : "缺失"; }
function rpyDegrees(q) {
  if (!Array.isArray(q) || q.length !== 4) return "缺失";
  const [x, y, z, w] = q.map(Number);
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return [roll, pitch, yaw].map(v => showNumber(v * 180 / Math.PI, 1)).join(", ");
}
function gapBeforeCurrent() {
  if (index === 0) return 0;
  const period = 1e9 / 15;
  const delta = rows[index].target_monotonic_ns - rows[index - 1].target_monotonic_ns;
  return Math.max(0, Math.round(delta / period) - 1);
}
function matrixIdentity() { return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]; }
function matrixMultiply(a, b) { return a.map((row, i) => b[0].map((_, j) => row.reduce((sum, value, k) => sum + value * b[k][j], 0))); }
function originMatrix(xyz, rpy) {
  const [r,p,y] = rpy, [sx,cx] = [Math.sin(r),Math.cos(r)], [sy,cy] = [Math.sin(p),Math.cos(p)], [sz,cz] = [Math.sin(y),Math.cos(y)];
  return [[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx,xyz[0]],[sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx,xyz[1]],[-sy,cy*sx,cy*cx,xyz[2]],[0,0,0,1]];
}
function axisRotation(axis, angle) {
  const [x,y,z] = axis, n = Math.hypot(x,y,z) || 1, [u,v,w] = [x/n,y/n,z/n], c = Math.cos(angle), s = Math.sin(angle), d = 1-c;
  return [[c+u*u*d,u*v*d-w*s,u*w*d+v*s,0],[v*u*d+w*s,c+v*v*d,v*w*d-u*s,0],[w*u*d-v*s,w*v*d+u*s,c+w*w*d,0],[0,0,0,1]];
}
function vector(text, fallback) { return String(text || fallback).trim().split(/\s+/).map(Number); }
function parseUrdf(text) {
  const xml = new DOMParser().parseFromString(text, "application/xml");
  if (xml.querySelector("parsererror")) throw new Error("URDF 格式无法解析");
  return [...xml.querySelectorAll("joint")].filter(joint => /^joint[1-7]$/.test(joint.getAttribute("name")) || joint.getAttribute("name") === "end_effector_joint").map(joint => {
    const origin = joint.querySelector("origin"), axis = joint.querySelector("axis");
    return { name: joint.getAttribute("name"), type: joint.getAttribute("type"), xyz: vector(origin?.getAttribute("xyz"), "0 0 0"), rpy: vector(origin?.getAttribute("rpy"), "0 0 0"), axis: vector(axis?.getAttribute("xyz"), "0 0 1") };
  });
}
function jointPositions(angles) {
  let transform = matrixIdentity(), positions = [[0,0,0]];
  robotJoints.forEach((joint, i) => {
    transform = matrixMultiply(transform, originMatrix(joint.xyz, joint.rpy));
    positions.push([transform[0][3], transform[1][3], transform[2][3]]);
    if (joint.type !== "fixed") transform = matrixMultiply(transform, axisRotation(joint.axis, Number(angles[i]) || 0));
  });
  return positions;
}
function projectPoint(point, center, scale) {
  let [x,y,z] = point.map((v,i) => v-center[i]);
  const cy=Math.cos(camera.yaw), sy=Math.sin(camera.yaw), cp=Math.cos(camera.pitch), sp=Math.sin(camera.pitch);
  const rx=cy*x-sy*y, ry=sy*x+cy*y, rz=z, py=cp*ry-sp*rz, pz=sp*ry+cp*rz;
  const factor = 1 / Math.max(.25, 1 + pz * .45);
  return [rx*scale*factor, -py*scale*factor, pz];
}
function drawRobot() {
  const canvas=$("robot-canvas"); if (!canvas || !robotReady || !rows.length) return;
  const rect=canvas.getBoundingClientRect(), ratio=window.devicePixelRatio || 1, width=Math.max(1,Math.round(rect.width*ratio)), height=Math.max(1,Math.round(rect.height*ratio));
  if(canvas.width!==width || canvas.height!==height){canvas.width=width;canvas.height=height;} const ctx=canvas.getContext("2d"); ctx.setTransform(ratio,0,0,ratio,0,0); ctx.clearRect(0,0,rect.width,rect.height);
  const points=jointPositions(currentRow().observation?.joint_position_rad || []); const mins=[0,1,2].map(i=>Math.min(...points.map(p=>p[i]))), maxs=[0,1,2].map(i=>Math.max(...points.map(p=>p[i]))), center=mins.map((v,i)=>(v+maxs[i])/2); center[2]=Math.min(0,center[2]); const extent=Math.max(.5,...maxs.map((v,i)=>v-mins[i])); const scale=Math.min(rect.width,rect.height)*.64/extent*camera.zoom;
  const origin=[rect.width/2,rect.height*.58], projected=points.map(p=>projectPoint(p,center,scale));
  ctx.lineWidth=1; ctx.strokeStyle="rgba(102,174,252,.18)"; for(let i=-4;i<=4;i++){const a=projectPoint([i*.1,-.4,0],center,scale),b=projectPoint([i*.1,.4,0],center,scale),c=projectPoint([-.4,i*.1,0],center,scale),d=projectPoint([.4,i*.1,0],center,scale);ctx.beginPath();ctx.moveTo(origin[0]+a[0],origin[1]+a[1]);ctx.lineTo(origin[0]+b[0],origin[1]+b[1]);ctx.moveTo(origin[0]+c[0],origin[1]+c[1]);ctx.lineTo(origin[0]+d[0],origin[1]+d[1]);ctx.stroke();}
  ctx.lineWidth=5; ctx.lineCap="round"; ctx.strokeStyle="#66aefc"; ctx.beginPath(); projected.forEach((p,i)=>i?ctx.lineTo(origin[0]+p[0],origin[1]+p[1]):ctx.moveTo(origin[0]+p[0],origin[1]+p[1])); ctx.stroke();
  projected.forEach((p,i)=>{const x=origin[0]+p[0],y=origin[1]+p[1],tip=i===projected.length-1;ctx.beginPath();ctx.fillStyle=tip?"#f4bb63":"#48d3b2";ctx.arc(x,y,tip?8:6,0,Math.PI*2);ctx.fill();ctx.fillStyle="#e9f1f7";ctx.font="12px system-ui";ctx.fillText(i===0?"BASE":tip?"TCP":`J${i}`,x+9,y-8);});
}
function installRobotControls() {
  const canvas=$("robot-canvas"); let last=null;
  canvas.addEventListener("pointerdown", e=>{last=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);}); canvas.addEventListener("pointermove", e=>{if(!last)return;camera.yaw+=(e.clientX-last[0])*.012;camera.pitch=Math.max(-1.25,Math.min(1.25,camera.pitch+(e.clientY-last[1])*.012));last=[e.clientX,e.clientY];drawRobot();}); canvas.addEventListener("pointerup",()=>last=null); canvas.addEventListener("pointercancel",()=>last=null); canvas.addEventListener("wheel",e=>{e.preventDefault();camera.zoom=Math.max(.45,Math.min(2.4,camera.zoom*(e.deltaY>0?.9:1.1)));drawRobot();},{passive:false});
  $("reset-view").addEventListener("click",()=>{camera.yaw=-.76;camera.pitch=.54;camera.zoom=1;drawRobot();}); window.addEventListener("resize",drawRobot);
}
function renderQuality() {
  const rejected = Number(view.rejected_grid_points || 0), total = rows.length + rejected;
  const raw = episode.raw_streams || {};
  $("aligned-frames").textContent = rows.length;
  $("aligned-coverage").textContent = total ? `${(rows.length / total * 100).toFixed(2)}% 覆盖率` : "--";
  $("rejected-frames").textContent = rejected;
  $("rejected-rate").textContent = total ? `${(rejected / total * 100).toFixed(2)}% 被跳过` : "--";
  $("raw-camera-drops").textContent = raw.camera_drops ?? "--";
  $("raw-robot-drops").textContent = raw.robot_state_drops ?? "--";
}

function render() {
  if (!rows.length) return;
  const row = currentRow(), obs = row.observation || {}, align = row.alignment || {};
  $("external-image").src = imagePath(row.images.external);
  $("wrist-image").src = imagePath(row.images.wrist);
  $("timeline").value = index;
  $("frame-readout").textContent = `帧 ${index + 1} / ${rows.length}`;
  $("time-readout").textContent = formatTime(row);
  $("range-start").textContent = `区间开始：${start === null ? "--" : `${start} (${(start / 15).toFixed(2)} s)`}`;
  $("range-end").textContent = `区间结束：${end === null ? "--" : `${end} (${(end / 15).toFixed(2)} s)`}`;
  $("gripper").textContent = `${showNumber(obs.gripper_opening_ratio * 100, 1)}%`;
  $("tcp-fk").textContent = xyz(obs.tcp_pose);
  $("tcp-measured").textContent = xyz(obs.recorded_tcp_pose);
  $("tcp-target").textContent = xyz(obs.target_tcp_pose);
  $("tcp-rpy").textContent = rpyDegrees(obs.recorded_tcp_pose?.orientation_xyzw || obs.tcp_pose?.orientation_xyzw);
  const err = align.camera_error_s; $("camera-error").textContent = err ? `外 ${showNumber(err.external, 4)}s / 腕 ${showNumber(err.wrist, 4)}s` : "--";
  const joints = obs.joint_position_rad || [];
  $("joint-values").innerHTML = joints.map((joint, i) => `<div><span>J${i + 1}</span><b>${showNumber(Number(joint) * 180 / Math.PI, 1)}°</b></div>`).join("");
  const missing = gapBeforeCurrent();
  $("current-gap").textContent = missing ? `缺 ${missing}` : "正常";
  $("current-gap-detail").textContent = index === 0 ? "起始帧" : `与前帧间隔 ${((row.target_monotonic_ns - rows[index - 1].target_monotonic_ns) / 1e9).toFixed(3)} s`;
  $("target-state").textContent = obs.target_tcp_pose ? "目标 TCP 已记录" : "目标 TCP 缺失";
  $("range-status").textContent = start === null || end === null ? "未选择完整区间" : `${Math.abs(end - start) + 1} 帧`;
  $("action-state").textContent = index < rows.length - 1 ? "可生成下一步 TCP 动作" : "末帧无下一步动作";
  drawRobot();
  renderSegments();
}
function renderSegments() {
  const holder = $("segments"); holder.innerHTML = "";
  if (!labels.segments.length) { holder.className = "segments empty"; holder.textContent = "尚未添加过程标签。"; return; }
  holder.className = "segments";
  labels.segments.forEach((segment, i) => {
    const el = document.createElement("article"); el.className = "segment";
    el.innerHTML = `<span class="badge">${segment.label}</span><div><strong>帧 ${segment.start_frame} – ${segment.end_frame}</strong><small>${(segment.start_frame / 15).toFixed(2)}s – ${(segment.end_frame / 15).toFixed(2)}s${segment.note ? ` · ${segment.note}` : ""}</small></div><button class="delete" data-index="${i}">删除</button>`;
    holder.append(el);
  });
  holder.querySelectorAll(".delete").forEach(button => button.addEventListener("click", () => { labels.segments.splice(Number(button.dataset.index), 1); renderSegments(); }));
}
function setPlaying(value) { playing = value; $("play").textContent = value ? "❚❚" : "▶"; clearInterval(timer); if (value) timer = setInterval(() => { index = index >= rows.length - 1 ? 0 : index + 1; render(); }, 1000 / (15 * Number($("speed").value))); }
function addSegment(label) {
  if (start === null || end === null) return alert("请先用“设为开始”和“设为结束”选择一个区间。");
  const left = Math.min(start, end), right = Math.max(start, end);
  labels.segments.push({ label, start_frame: left, end_frame: right, start_time_s: left / 15, end_time_s: right / 15, note: $("segment-note").value.trim() });
  labels.segments.sort((a,b) => a.start_frame - b.start_frame); $("segment-note").value = ""; start = end = null; render();
}
function exportLabels() {
  labels.episode_outcome = document.querySelector('input[name="outcome"]:checked').value;
  labels.episode_note = $("episode-note").value.trim(); labels.updated_at = new Date().toISOString(); labels.total_sensor_frames = rows.length;
  const blob = new Blob([JSON.stringify(labels, null, 2)], { type: "application/json" }); const link = document.createElement("a"); link.href = URL.createObjectURL(blob); link.download = "labels.json"; link.click(); URL.revokeObjectURL(link.href);
}
async function load() {
  try {
    const [response, viewResponse, episodeResponse, urdfResponse] = await Promise.all([fetch(DATA.observations), fetch(DATA.view), fetch(DATA.episode), fetch(DATA.urdf)]);
    if (!response.ok || !viewResponse.ok || !episodeResponse.ok || !urdfResponse.ok) throw new Error(`数据读取失败`);
    rows = (await response.text()).trim().split("\n").filter(Boolean).map(JSON.parse); view = await viewResponse.json(); episode = await episodeResponse.json(); robotJoints=parseUrdf(await urdfResponse.text()); robotReady=robotJoints.length===8; if (!robotReady) throw new Error("URDF 中的关节链不完整"); if (!rows.length) throw new Error("没有观测帧");
    $("timeline").max = rows.length - 1; $("load-state").textContent = `已加载 ${rows.length} 个 15 Hz 对齐观测帧`; render();
    renderQuality();
    $("robot-state").textContent = "URDF 已加载 · J1–J7 与 TCP 同步";
  } catch (error) { $("load-state").textContent = `加载失败：${error.message}。请通过本地服务打开页面。`; }
}
$("timeline").addEventListener("input", (e) => { index = Number(e.target.value); render(); });
$("play").addEventListener("click", () => setPlaying(!playing)); $("speed").addEventListener("change", () => playing && setPlaying(true));
$("set-start").addEventListener("click", () => { start = index; render(); }); $("set-end").addEventListener("click", () => { end = index; render(); }); $("clear-range").addEventListener("click", () => { start = end = null; render(); });
$("export-labels").addEventListener("click", exportLabels); $("import-labels").addEventListener("click", () => $("import-file").click());
$("import-file").addEventListener("change", async (event) => { const f = event.target.files[0]; if (!f) return; try { labels = JSON.parse(await f.text()); $("episode-note").value = labels.episode_note || ""; const r = document.querySelector(`input[name="outcome"][value="${labels.episode_outcome || "unreviewed"}"]`); if (r) r.checked = true; render(); } catch { alert("无法读取 labels.json"); } });
LABELS.forEach(label => { const button = document.createElement("button"); button.textContent = label; button.addEventListener("click", () => addSegment(label)); $("label-buttons").append(button); });
installRobotControls();
document.addEventListener("keydown", (event) => { if (event.target.matches("textarea, input, select")) return; if (event.code === "Space") { event.preventDefault(); setPlaying(!playing); } if (event.key === "ArrowLeft") { index = Math.max(0, index - 1); render(); } if (event.key === "ArrowRight") { index = Math.min(rows.length - 1, index + 1); render(); } });
load();
