async function exportEpisodes() {
  if (!labels.processes.length) {
    alert("请先创建至少一个完整过程，再导出 Episode。");
    return;
  }
  const importedName = $("load-state").textContent.match(/episode_\d+/)?.[0];
  const sourceEpisode = importedName || "episode_000000";
  $("import-status").textContent = "正在生成切割后的 Episode…";
  try {
    const response = await fetch("/api/export-episodes", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ source_episode: sourceEpisode, processes: labels.processes }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || "导出失败");
    $("import-status").textContent = "已生成：" + result.created.map(item => item.name).join("、") + "。保存位置：G:\\codex-yufan\\after data processing";
  } catch (error) {
    $("import-status").textContent = "导出失败：" + error.message;
  }
}

$("export-episodes").addEventListener("click", exportEpisodes);
