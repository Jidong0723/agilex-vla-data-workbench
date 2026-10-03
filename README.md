# AgileX VLA Data Workbench

本地只读审阅与标注工具。第一阶段从原始 NERO Episode 生成双相机、机械臂状态、TCP FK 和 7 维动作的 20 Hz 中间视图，并可按人工标注范围导出 LeRobot 格式的中间数据。**动作在人工标注前生成；H16 窗口和归一化统计留到第二阶段，导出结果仍不是可直接训练的数据集。**

## 第一阶段工作流

1. 在本机 `server.py` 配置的原始数据目录中放置采集 Episode，并确认其 `episode.json`、双相机清单/图像、机器人状态与 URDF/Pinocchio 接口文件完整。
2. 安装 `requirements.txt` 中的本地导出依赖，启动 `python server.py`，打开 `http://127.0.0.1:8790/`。
   拖入文件夹时优先使用 Chrome/Edge 的文件夹句柄接口；若浏览器拒绝拖拽读取，可改用页面的“插入数据”选择同一文件夹。不要从 `file:///` 打开页面。
3. 通过“插入数据”选择 Episode（支持任意文件夹，自动入库）。外部目录中的文件会复制到本地数据根目录；已完整入库的文件跳过，中断后只补传缺失或大小不符的文件，并在全部文件核验后开始后处理。服务器先对每条原始关节反馈执行 Pinocchio FK 得到 TCP，再按时间戳将相机、关节状态对齐到 20 Hz：机器人状态在有效反馈间插值，相机选最近帧，随后计算相邻 TCP 对与 7 维动作。相机/反馈时间差与单步动作幅度仅作为数值留存，不进行阈值判定、筛除或裁剪。原始 Episode 不会被修改。
4. 在审阅页人工创建过程标题与阶段标签，可独立导出 `labels.json`。
5. 导出“LeRobot 动作中间数据”：只导出人工创建的过程区间，包含 17 维状态、7 维 `action`、双相机视频、任务文本、20 Hz 时间戳、源数据索引，以及 `meta/nerovla_tcp_pairs.jsonl` 中的相邻 FK TCP 位姿和重建误差。所有可解析的已对齐帧均保留，不按相机时间误差、反馈插值跨度或动作幅度阈值排除；仅在真实时间缺口或无效 TCP 配对处拆段。动作前 6 维是相邻实测 TCP 的基座系增量，第 7 维是当前帧记录的绝对夹爪目标；每个导出片段末帧用零 TCP 增量和保持的夹爪目标。人工审核结果写入 manifest；不生成 H16 索引。页面“导出保存目录”可填写本机已存在的绝对路径；留空使用默认目录。每次成功导出的 LeRobot 目录内另存一份同版 `labels.json`，浏览器下载与标注记忆库行为保持不变。

派生的对齐视图与 LeRobot 格式中间数据保存在 `server.py` 配置的本地数据目录。当前默认路径针对开发机部署，换机器前应核对 `SOURCE_ROOT`、`DERIVED_ROOT`、`LEROBOT_ROOT` 和 `KINEMATICS_PYTHON`。

20 Hz 视图写入独立的 `20hz_tcp_actions_v1` 目录，LeRobot 中间数据写入 `episode_编号_tcp_vla_20hz`；旧的 15 Hz 视图和导出不会被覆盖。旧 `labels.json` 的帧号不能直接用于 20 Hz 视图，需按原始时间重新确认标注边界。

### Windows 环境配置

本工作台依赖安装在仓库内的 `.venv`，与 Pinocchio/Pink 运动学环境隔离。当前开发机已有运动学 Python 时，可执行：

```powershell
& 'E:\nero-agilex\.conda\nero-kinematics\python.exe' -m venv .venv
& '.\.venv\Scripts\python.exe' -m pip install --upgrade pip
& '.\.venv\Scripts\python.exe' -m pip install -r requirements.txt
& '.\.venv\Scripts\python.exe' server.py
```

`server.py` 会把 FK 请求交给 `KINEMATICS_PYTHON` 指定的 Pinocchio 环境；不需要在 `.venv` 中重复安装 Pinocchio/Pink。不要用全局 Python 直接启动，以免依赖版本不一致。

## 第二阶段（本工具当前不执行）

读取第一阶段动作中间数据，审查夹爪目标来源，生成合法 H16 锚点并审计时间因果关系；之后固定父组划分、只用训练组计算归一化统计，再写出训练专用 LeRobot 派生视图。第二阶段不得修改第一阶段输出或原始 Episode。执行器的硬件安全限制与数据筛选是两个独立问题，本工作台不会因实测动作幅度排除训练样本。

## 数据与隐私

原始图像、机器人状态、生成的视频和数据集应留在本地，不应提交到 Git。工作台不发送机器人控制命令。
