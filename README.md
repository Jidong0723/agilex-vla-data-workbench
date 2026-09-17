# AgileX VLA Data Workbench

一个本地优先的数据审阅与人工标注工具，用于将 NERO 机械臂采集的原始 Episode 转换为可检查的 15 Hz 训练视图。

它将双相机画面、机械臂关节状态、FK TCP 位姿、时间对齐质量与过程标签放在同一个页面中。原始数据始终只读，标签作为独立 `labels.json` 导出。

## 功能

- 双视角视频逐帧浏览、15 Hz 时间轴与播放控制。
- 接近、抓取、抬升、移动、放下、释放等过程标签。
- 双相机与机械臂反馈的时间对齐质量统计。
- 基于 NERO URDF 和关节反馈的三维投影，以及 FK、实测与目标 TCP 位姿读数。
- 独立导入、导出标签，不修改原始 Episode 或派生训练视图。

## 本地运行

1. 为本机 Episode 建立以下本地链接（它们不应提交到 Git）：
   - `training_view/`：15 Hz 派生视图，含 `observations.jsonl` 与 `view.json`。
   - `episode_source/`：原始 Episode，含图像、`episode.json` 和 URDF。
2. 在仓库根目录启动任意静态文件服务，例如：

   ```powershell
   python -m http.server 8790 --bind 127.0.0.1
   ```

3. 在浏览器打开 `http://127.0.0.1:8790/`。

详细的数据约定与 15 Hz 对齐规则见 [docs/data-contract.md](docs/data-contract.md)。

## 仓库结构

```text
.
├── index.html              # 本地标注工作台入口
├── app.js                  # 数据加载、时间轴、标签与三维投影
├── styles.css              # 页面样式
├── docs/                   # 数据格式和工作流说明
├── .github/                # GitHub Issue 模板
├── CONTRIBUTING.md         # 提交规范
├── CHANGELOG.md            # 用户可见版本记录
└── .gitignore              # 排除 Episode、派生视图和导出标签
```

## 数据与隐私

Episode 原始图像、原始机器人状态、派生训练视图和导出的 `labels.json` 均被 `.gitignore` 排除。公开仓库只应包含应用代码、文档和不含真实采集数据的示例。

## 许可证

项目尚未选择开源许可证。在添加 `LICENSE` 前，请勿将本仓库视为向第三方授予使用、修改或分发权限。
