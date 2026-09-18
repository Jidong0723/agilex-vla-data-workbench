# 数据约定

## 输入

本工具不直接写入采集数据，而是读取两类本地只读数据：

- `episode_source/`：原始 Episode，提供图像路径、采集元数据和 NERO URDF。
- `training_view/`：从原始 Episode 生成的 15 Hz 派生视图，提供 `observations.jsonl` 和 `view.json`。

## 15 Hz 有效帧

每个候选时刻必须同时满足以下条件，才会出现在 `observations.jsonl`：

1. 外部相机和腕部相机各有一张最近图像，且各自的时间误差不超过 30 ms。
2. 机械臂在该时刻前后各有一条反馈，可插值得到 7 轴关节角、关节速度和夹爪开合值；两侧反馈距离该时刻均不超过 35 ms。
3. 基于插值关节状态的 FK 能生成有效 TCP 位姿。

因此，“对齐跳过帧”表示候选时刻未能同时满足相机与机械臂的时间对齐要求，不等于原始采集线程一定丢帧。

## 标签文件

导出的 `labels.json` 使用 `nero.episode-process-labels.v2`：每个完整过程有一个可读标题和时间范围，过程下可包含多个细粒度标签片段。文件同时保留平铺的 `segments` 字段，方便与早期工具兼容。

它以 15 Hz 帧索引标记区间，不会修改原始 Episode 或训练视图。

## 插入原始 Episode

“插入数据”接受一个原始 `episode_xxxxxx` 文件夹，读取 `episode.json`、相机清单、机器人状态、图像和 URDF。在浏览器内存中按上述对齐规则生成审阅视图；刷新页面或选择另一条 Episode 后，内存视图会被替换，原始文件不会被写入。

## 切割导出

“导出 episode”按照完整过程的数量切割源 Episode。每个过程对应一个输出目录，名称为原始 Episode 名称加序号后缀，例如 `episode_000001.1`、`episode_000001.2`。每个目录保留原始 Episode 的文件布局：`episode.json`、相机清单、机器人状态、引用图像与接口文件。其元数据会记录来源 Episode、过程标题、过程标签及新的时间边界。

导出仅能从 `G:\\codex-yufan\\dataset\\episodes` 读取，并且仅能写入 `G:\\codex-yufan\\after data processing`。若目标名称已存在，导出会停止且不会覆盖已有数据。
