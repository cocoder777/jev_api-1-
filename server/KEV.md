# Kev-4B 本地后端

`decision_service.py` 当前默认使用 Kev-4B，也可显式指定 `--backend kev`；`--backend jev` 与 `--backend julia` 仍可切换。Kev固定速度由 `policies/kev_policy.py` 的 `KEV_SPEED_KNOTS` 统一设置，在每个决策周期内，按当前方向与配置转向角速度计算可达区间，生成**16个局部绝对航向候选**（含直行），一次模型选择。当前默认转向角速度为45°/秒，1秒周期的候选范围为当前方向±45°。`--max-speed-knots` 不改变 Kev 的固定速度。下文15节、5°/秒的测试数据与显式参数属于历史配置。

## 结构

- `policies/kev_policy.py`：航点状态、16候选的运动预测事实、最近四步决策、模型请求与动作解码。没有候选排序、剔除或规则代选。所有方向都交给模型。
- `policies/factory.py`：创建后端，Kev的`--kev-endpoint`与Jev地址独立。
- `control_runtime.py`：独立推理工作线程、周期控制发送、连续错误计数、限时续发和终点停车确认；Kev与Jev共用。
- 同艇每个有效新任务都会接管旧任务，包括相同ID/参数。旧任务的控制、停车与状态输出被撤销，后台请求的迟到结果不会下发；Kev的请求入口允许不同任务线程独立发起HTTP请求，不被旧请求的客户端锁阻塞。任务接管规则见 [README.md](README.md)。
- `local_models/setup_kev.py`：下载固定运行时、LoRA适配器、决策头和基座，并生成可离线加载的部署副本。
- `local_models/serve_kev.py`：用官方Kev运行时启动本机HTTP推理服务，模型常驻显存。
- `experiments/simulate.py`：复用现有任务/态势/控制报文及地图，新增Kev选择，记录有效速度、候选、模型概率与耗时。

官方来源：https://huggingface.co/jaredpalmer/kev-4b ，https://github.com/jaredpalmer/kev 。这是Qwen3.5-4B-Base基座上的LoRA加决策头，不是通过生成文本选择动作。

固定版本：

- 官方代码 `1f04cfa28e6a708485b28a9585010b9e436b776e`。
- 适配器 `6cfce5c2fa4b4bd64026336ab649c5ca78857d52`。
- 基座 `1001bb4d826a52d1f399e183466143f4da7b741b`，与发布决策头的base_revision一致。

## 两个独立环境

项目 `.venv` 保留原Julia依赖并运行UDP服务/绘图模拟器。新 `.venv-kev` 只运行Kev官方服务，因为两者需要不同版本的PyTorch和Transformers。不要把Kev依赖安装到原 `.venv`。

常规有venv/pip的机器上安装：

```bash
python3 -m venv .venv-kev
.venv-kev/bin/python -m pip install huggingface_hub
.venv-kev/bin/python server/local_models/setup_kev.py
```

本机原系统没有ensurepip，已在独立环境内初始化pip，未改系统包。

模型目录：`models/kev-runtime`（官方代码）、`models/kev-4b`（原发布适配器/头）、`models/kev-4b-base`（基座）、`models/kev-4b-local`（离线部署副本）。原始发布文件保持原值；部署副本只把head元数据中的base指向本地目录，数值权重不变，版本和校验值存于deployment.json。迁移目录后重新执行 `setup_kev.py --prepare-only` 更新绝对基座路径。

## 启动与测试

终端一，保持运行：

```bash
.venv-kev/bin/python server/local_models/serve_kev.py --device cuda
```

默认只监听 `127.0.0.1:18028`，HTTP接口为 `/v1/systemone`。启动脚本设置HF离线模式，关闭可选融合/图编译，使用官方bf16 CUDA标准路径；未安装额外可选内核时速度可能低于官方优化后的基准。

终端二，在1440×1440米地图测试：

```bash
.venv/bin/python server/experiments/simulate.py \
  --backend kev --scenario crossing_km --steps 2000 --dt 1 \
  --max-turn-rate-deg-s 5 --initial-course-deg 90 \
  --output server/experiments/results/kev-crossing-km
```

模拟器的 `--device` 和 `--model-path` 用于进程内模型；Kev的实际设备由终端一的 `--device` 决定，权重路径由离线服务决定。更换服务地址时，在模拟器或UDP服务上设置 `--kev-endpoint http://127.0.0.1:新端口/v1/systemone`；请求超时为 `--kev-timeout`，默认120秒。

当前任务列表只包含待访问航点，起点取态势位置；`currentRoute=0` 表示尚未完成任何点。报文服务在最后一个航点到达后主动发送0速并根据新鲜态势确认停车，再返回完成状态。本地仿真同步使用停车完成判据（`completion_mode=final_parking`），旧的“到点立即结束”测试仅保留为历史结果。

输出目录包含地图轨迹、速度/耗时三联图、逐步完整输入与模型响应JSONL、汇总与地图快照。实际速度读取当前 `KEV_SPEED_KNOTS` 常量；历史15节测试不代表后来修改的速度配置。加速仿真推理时暂停世界，不代表实艇实时性。

真实localhost UDP测试，仍需先启动Kev服务：

```bash
.venv/bin/python server/experiments/simulate.py \
  --backend kev --transport udp --scenario crossing_km \
  --udp-seconds 15 --output server/experiments/results/kev-udp
```

实际运行报文服务：

```bash
.venv/bin/python -m server.decision_service --backend kev \
  --kev-endpoint http://127.0.0.1:18028/v1/systemone \
  --host 127.0.0.1 --port 18024 --situation-port 18025 \
  --action-host 127.0.0.1 --action-port 18124 --platform-id 1010 \
  --max-turn-rate-deg-s 5 --initial-course-deg 90
```

以上本地测试地址与旧文档一致。新增加的是Kev本机推理口18028；原现场任务 `0.0.0.0:18014`、态势 `239.255.0.10:18010` 均未修改；原动力目标源码 `192.168.3.201:18123` 与环境示例/Compose `192.168.3.126:18123` 的既有差异仍保留。

## 提示词与历史版本的差异

Kev使用独立提示词，明确0/90/180/270度方位、当前目标坐标和方位、每个方向的预测距离/进度及转向量，附最近四步的距离与选择。移除了未定义的tie line要求；任务仍是按序进入航点圈，不要求贴线。相比历史Julia同时改变了模型、候选速度/航向及提示组织，结果不能作为仅换模型的受控对照。

## 本轮实测

真实RTX5090测试按序到达10/10航点，1825次决策，1824秒仿真时间，平均完整决策217毫秒、P95约219毫秒。固定15节与16选项逐步核对通过。路径存在绕行；最终按进入终点圈完成，没有停车。真实UDP显式停止流程也通过。25项回归测试及两个环境的依赖检查通过。完整结果见 [KEV_REPORT.md](experiments/results/KEV_REPORT.md)。

服务已在本次会话启动并监听本机18028口；若端口已有服务，无需重复启动。停止服务后可用上文命令重新加载。提示词在 `policies/kev_policy.py` 的 `_request()`，可直接查看 `experiments/examples/kev_request.json`；每步完整输入与响应保存在本轮JSONL。

## 精简提示词与输入精度

当前Kev输入已精简：删去重复的速度说明、候选中重复的固定速度、可由距离直接表达的进展量，以及重复的任务解释。保留目标/坐标约定、预测距离、转向量、限速事实和最近决策。模型问题缩短为“到达当前圈、遵守限速、偏好较小预测距离、避免无谓转向与反复反向”。

`policies/kev_policy.py` 的 `round_model_values()` 在请求出口统一递归四舍五入所有浮点数至两位小数，覆盖状态、16个候选、转向/时间参数及历史。保留数值类型，不为凑显示位数转成字符串；例如22.5表示22.50的同一数值。整数与空值保持类型。内部经纬度转换、运动积分和计时保持原精度；这里限制的是模型可见数据，不把经纬度直接截成0.01度。

精简前初始请求JSON字符数7469，精简后4342，减少约41.9%；这不是token数统计。更新后的输入示例仍在 `experiments/examples/kev_request.json`。历史10/10完整航线结果属于精简前版本，不自动视为精简版本的验证结果；新短测单独保存在 `experiments/results/kev-compact-smoke/`。


## 当前转向约束及注释字段修正

- `--max-turn-rate-deg-s` 当前默认45度/秒，是候选和预测使用的配置值，未实艇标定；可通过参数或 `MAX_TURN_RATE_DEG_S` 覆盖。历史5度/秒测试保留原配置和结果。
- 单周期最大转角 = 角速度上限 × 决策时长。默认1秒为当前方向±45度；0.2秒为±9度。16候选从区间内等距取样，将最接近直行的中间样本替换为直行项（偶数个对称样本不能同时精确包含中心），仍为16项。命令量化到0.01度时向区间内部取整，避免四舍五入后越界。范围不足以表示16个不同两位小数航向时显式报错。
- 提示词已移除 `speed_transition_s` 和 `course_transition_s`。速度响应时间仍用于内部启动/变速预测；Kev转向不再使用“任意角度0.5秒完成”，而是角速度受限的近似运动。预测与仿真使用同一约束；原Jev/Julia默认运动模式不变。
- 你注释的 `total_waypoints` 和数值限速说明没有运行依赖，保持删减。`arrival_radius_m` 和 `command_duration_s` 恢复为短数值字段：前者定义何时进入目标圈，后者定义预测距离对应多久，具有决策含义。历史记录改由内部观测计算距离，不再读取可被注释掉的提示词字段，避免KeyError。
- 报文服务每轮读取最新态势的位置、速度和 `targetCourse`；Kev 在速度为0时也保留态势中的有效航向，0度合法。`--initial-course-deg` 仅在静止且态势未提供航向时作为备用，不覆盖有效态势。模拟器示例90度是其明确设置的向东初始方向；不能根据目标点推测本艇方向。
- 假定无侧滑时以地速航向近似艏向，未建模舵机迟滞、横流和角加速度；这里只是角速度上限模型，不是已辨识的真实船体动力学。

资料依据：

1. [USV Yaw Modeling and Identification](https://arxiv.org/abs/2303.13064)：转向模型参数由试验数据辨识，不能从目标方位推断。
2. [IMO MSC.137(76)](https://wwwcdn.imo.org/localresources/en/KnowledgeCentre/IndexofIMOResolutions/MSCResolutions/MSC.137%2876%29.pdf)：操纵性通过试验/模型验证，适用范围有船长/船型限制，且排除其定义的高速船；不能将战术回转直径5船长要求直接套给本艇或作为最大角速度。

当前有界转向版本应重新验证；此前10/10结果属于旧的全局候选/快速转向仿真，不代表新动力学下也已通过整条航线。


受限转向还补充了 `target_relative_bearing_deg`、候选的有符号转向与 `target_heading_error_deg`，避免目标在侧后方时被“必须立即减小距离”误导。问题优先引导模型减小目标航向误差，对齐后直行；不对候选做排序或规则代选。这些量均在请求出口保留两位小数。

最初5Hz受限候选试验见 `experiments/results/kev-bounded-turn-5hz/`：600步全部16候选均在±1度，实际最大角速度约5度/秒，但初始提示词换向不佳；该试验不是最后提示词的导航成绩。随后侧后方提示探针和1Hz中间试验另存，最终短测目录为 `experiments/results/kev-bounded-turn-final/`。各版本保留，不复用旧10/10结果来描述当前约束版本。

受限转向的参数依据、逐轮实测范围与代码检查记录见 [KEV_TURNING.md](experiments/results/KEV_TURNING.md)。31项测试与真实UDP停止短测通过；当前受限模型的导航仍有不足，旧10/10只作为历史结果保留。
