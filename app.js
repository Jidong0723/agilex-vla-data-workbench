const DATA = {
  observations: "training_view/observations.jsonl",
  view: "training_view/view.json",
  episode: "episode_source/episode.json",
  source: "episode_source/",
};
const LABELS = ["接近", "抓取", "抬升", "移动", "放下", "释放", "停顿", "失败动作"];
let rows = [], view = {}, episode = {}, index = 0, start = null, end = null, playing = false, timer = null;
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
    const [response, viewResponse, episodeResponse] = await Promise.all([fetch(DATA.observations), fetch(DATA.view), fetch(DATA.episode)]);
    if (!response.ok || !viewResponse.ok || !episodeResponse.ok) throw new Error(`数据读取失败`);
    rows = (await response.text()).trim().split("\n").filter(Boolean).map(JSON.parse); view = await viewResponse.json(); episode = await episodeResponse.json(); if (!rows.length) throw new Error("没有观测帧");
    $("timeline").max = rows.length - 1; $("load-state").textContent = `已加载 ${rows.length} 个 15 Hz 对齐观测帧`; render();
    renderQuality();
  } catch (error) { $("load-state").textContent = `加载失败：${error.message}。请通过本地服务打开页面。`; }
}
$("timeline").addEventListener("input", (e) => { index = Number(e.target.value); render(); });
$("play").addEventListener("click", () => setPlaying(!playing)); $("speed").addEventListener("change", () => playing && setPlaying(true));
$("set-start").addEventListener("click", () => { start = index; render(); }); $("set-end").addEventListener("click", () => { end = index; render(); }); $("clear-range").addEventListener("click", () => { start = end = null; render(); });
$("export-labels").addEventListener("click", exportLabels); $("import-labels").addEventListener("click", () => $("import-file").click());
$("import-file").addEventListener("change", async (event) => { const f = event.target.files[0]; if (!f) return; try { labels = JSON.parse(await f.text()); $("episode-note").value = labels.episode_note || ""; const r = document.querySelector(`input[name="outcome"][value="${labels.episode_outcome || "unreviewed"}"]`); if (r) r.checked = true; render(); } catch { alert("无法读取 labels.json"); } });
LABELS.forEach(label => { const button = document.createElement("button"); button.textContent = label; button.addEventListener("click", () => addSegment(label)); $("label-buttons").append(button); });
document.addEventListener("keydown", (event) => { if (event.target.matches("textarea, input, select")) return; if (event.code === "Space") { event.preventDefault(); setPlaying(!playing); } if (event.key === "ArrowLeft") { index = Math.max(0, index - 1); render(); } if (event.key === "ArrowRight") { index = Math.min(rows.length - 1, index + 1); render(); } });
load();
