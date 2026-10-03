// 一键导出：labels.json 下载 + 记忆库存档 + LeRobot 动作中间数据，三者共用同一份标注
async function exportAllData() {
  if (!labels.processes.length) {
    alert("请先创建至少一个带任务标题的完整过程（或先点“生成草案”），再导出。");
    return;
  }
  const episodeName = currentEpisodeName || $("load-state").textContent.match(/episode_\d+/)?.[0];
  if (!episodeName) {
    alert("请先插入一个原始 Episode。");
    return;
  }
  if (batchMode && catalogIndex >= 0) labelsStore.set(episodeName, labels);
  $("import-status").textContent = "正在导出：labels.json 下载 + 记忆库存档 + LeRobot 中间数据（视频编码可能需要一些时间）…";
  window.__lerobotBusy = true;
  exportLabels(true);
  try {
    const exportedLabels = translateLabels(labels);
    const response = await fetch("/api/export-lerobot", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        source_episode: episodeName,
        processes: exportedLabels.processes,
        labels: exportedLabels,
        export_directory: $("export-directory").value.trim(),
        episode_outcome: document.querySelector('input[name="outcome"]:checked')?.value || "unreviewed",
        episode_note: $("episode-note").value.trim(),
      }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || "导出失败");
    const item = result.created;
    $("import-status").textContent = "导出完成：LeRobot 中间数据 " + item.name + "（" + item.episodes + " 段，" + item.frames + " 帧；未按时间差或动作幅度筛帧）及 labels.json 已保存到 " + item.path + "；浏览器也已下载标签。此目录尚不能直接训练；第二阶段需完成 H16、审核、划分和归一化。";
  } catch (error) {
    $("import-status").textContent = "导出失败：" + error.message + "（labels.json 部分可能已完成：文件已下载、记忆库已存档）";
  } finally {
    window.__lerobotBusy = false;
  }
}

$("export-all").addEventListener("click", exportAllData);
