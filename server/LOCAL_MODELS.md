# 本地模型接入与离线验证

本项目按民用航路点跟踪使用。模型只读取本艇经纬度、实际航速、绝对航向和航路点；协议中的敌我属性、威胁等级等历史字段不进入模型。

## 目录与接口

- `decision_service.py`：原 UDP 编解码、会话、状态和控制循环。
- `policies/waypoint.py`：各后端共用的观测校验、单位转换、航路点进度。
- `jev_policy.py`：原远程 Jev 后端。
- `policies/julia_policy.py`：常驻本地 Julia 推理，加载一次，跨会话串行使用引擎，各会话进度仍在线程本地保存。
- `policies/factory.py`：后端创建入口。后续模型添加独立文件及注册分支，不在报文层叠加模型逻辑。
- `local_models/setup_julia.py`：下载固定模型版本并安装官方运行时。
- `experiments/simulate.py`：本地二进制报文闭环、真实 UDP 测试和静态绘图。

后端须实现 `__call__(packet) -> (speed, course)`，提供 `speed_unit`；`current_route()` 返回已完成航点数量。任务列表不包含虚拟起点，`currentRoute=0` 表示从第一个点开始。终点停车、连续3次可恢复错误才失败、限时续发均由公共服务流程处理，不随机代选动作。

## 模型与环境

官方：https://huggingface.co/SupersonicLabs/Julia-1

固定 revision：`a85b127321d580d65176c89ced8273f305745d85`。144.3M 参数，FP32 权重约 550.5 MiB，Python >= 3.11。这是有限候选决策模型，不是聊天生成模型，也不能用 Transformers 的 AutoModel 接口直接替换官方 Julia 运行时。

在项目根目录执行（有 pip/venv 的常规环境）：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r server/local_models/requirements.txt
.venv/bin/python server/local_models/setup_julia.py
```

本机系统没有 pip/ensurepip，已在项目 `.venv` 内通过官方 get-pip 初始化，无系统包改动。本机虚拟环境和模型已落在项目目录；转移到实机时应重新创建虚拟环境，并复制整个 `models/Julia-1`（含官方运行时代码、分词器、encoder 配置和权重）。实机运行时不需要联网或 Jev 密钥；安装依赖需预先准备匹配平台的 wheel。

## 候选与编码

当前 Julia 使用固定两档 **10、20 节** × 8 个绝对航向（0、45、90、135、180、225、270、315 度），共 **16 个选项**，单次调用模型直接选择，不再分组复选。候选定义在 `policies/julia_policy.py` 的 `JULIA_SPEEDS_KNOTS` 和 `JULIA_ACTION_CHOICES`；与 Jev 的 64 项候选表独立，修改 Julia 不会改变 Jev。

Julia的16个模型候选仍不包含停车。当前公共观测逻辑在最终航点到达后直接返回0速，报文服务确认实际停稳后才结束任务；这一步不依赖模型选择停车候选。下文历史测试仍保留其原始判据和结果。

`--max-speed-knots` 对 Julia 固定速度不生效，仅影响 Jev；启动时会打印实际 Julia 速度，汇总保存 `effective_speeds_knots`，逐步日志保存 16 个候选、单次问题、概率和选择。模型使用近似运动事实短描述，每项不超过原生 48-token 限制，启用严格编码；组合上下文上限 8192、题目及选项预算 2048。

旧实验报告中的 64 项分组结果属于历史版本，不能与当前 16 项单次选择混为一轮实验。本轮结果见 `experiments/results/julia-fixed10-20/`。地图航点限速未随这次速度调整：参考小图仍为 1.2 节、港区图为 6 节，两档候选均高于这些限速，需区别速度候选实验与满足原限速的导航测试。

## 原现场地址与独立测试地址

未修改原现场 IP/端口默认值。已有配置本身存在差异，接现场前请确认哪份是实际部署依据。

| 项目 | 原源码默认 | 原环境示例 / Compose |
| --- | --- | --- |
| 任务接收 | `0.0.0.0:18014` | 相同 |
| 态势组播 | `239.255.0.10:18010` | 相同 |
| 动力系统目标 | `192.168.3.201:18123` | `192.168.3.126:18123` |
| 状态上报 | 回任务发送方源 IP/端口 | 对接文档行为树默认接收端口 `7612` |
| Jev 地址 | `https://api.typesafe.ai/v1/systemone` | `https://api.jevcn.cn/v1/systemone` |

原 CLI 帮助还曾写 `192.168.3.56`，与代码默认不一致；仅修正帮助文字，未改变地址。

本地 UDP 仿真仅绑定 `127.0.0.1`，自动选择空闲任务、状态、控制端口；不向现场 IP 或组播地址发送数据。态势直接单播到服务任务接收口（原服务已支持 `0x0060`），同时服务组播订阅使用临时端口。本地端口记录在输出 `summary.json`。无需改 `.env` 或 Compose。

## 启动与测试

仅本机调试服务（状态回任务源端口，控制送本机 18124；需另外运行模拟发送端）：

```bash
HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 .venv/bin/python -m server.decision_service \
  --backend julia --device cpu --local-model-path models/Julia-1 \
  --host 127.0.0.1 --port 18024 --situation-port 18025 \
  --action-host 127.0.0.1 --action-port 18124 --platform-id 1010
```

GPU 可改 `--device cuda`；没有 CUDA 或 BF16 支持时显式失败，不会悄悄切回 CPU。报文服务目前默认 `--backend kev`，使用 `--backend jev` 可切回原 Jev；环境变量为 `POLICY_BACKEND`、`LOCAL_MODEL_PATH`、`LOCAL_MODEL_DEVICE`。Dockerfile 包含公共策略目录；Compose 默认通过本机独立HTTP服务使用 Kev，也可显式选择 Jev；本地 Julia 用上述虚拟环境启动，Compose 不会自动安装 Julia。

离线加速闭环（每次真实推理后推进仿真时间，推理时世界暂停）：

```bash
.venv/bin/python server/experiments/simulate.py --device cpu --steps 100 \
  --output server/experiments/results/julia-cpu
```

真实时间 localhost UDP（会自动启动和关闭服务子进程，发送任务、持续态势、接收控制和状态，最后发送停止并验证实际降速后的状态）：

```bash
.venv/bin/python server/experiments/simulate.py --transport udp --device cpu \
  --udp-seconds 25 --output server/experiments/results/julia-udp
```

每次测试请使用不同 `--output` 目录。输出包含轨迹/航速/推理耗时 PNG、逐帧 JSONL、汇总 JSON；UDP 模式额外有状态报文和服务日志。`completed` 只在最终圈内实际低速且模型下达零速时为真；步数上限不算完成。模型不保证成功，报告保留失败、停滞和超时。

模拟运动使用现有近似地速预测器，未建模真实船体、风流、障碍和执行器状态。入圈超速检查为离散采样，不能作为连续安全验证。加速闭环不代表实机实时性，需结合 UDP 测试的实际耗时判断。

回归测试：

```bash
.venv/bin/python -m unittest discover -s server/tests -v
```

旧 `jev_api/tests/test_navigation.py` 依赖当前仓库未附带的 `Assets/StreamingAssets/fixed_route.json` 等 Unity 资源，不能完整运行；本地模型测试不依赖这些文件。

## 本轮实测

完整结果见 [实验报告](experiments/results/REPORT.md)。Julia 已成功本地部署，但当前适配三条简单航线均未完成，不能直接替代 Jev。CPU 完整决策均值约 1.29 秒；RTX 5090 约 43–45 毫秒。真实 localhost UDP 收发、停止请求及实际降速后回复状态通过。最终状态 `stopped` 仅表示停止任务成功。

本机 GPU 使用 PyTorch 2.14 的标准 CUDA 路径。后端默认在导入 PyTorch 前设置 `TORCH_DISABLE_NATIVE_JIT=1`，关闭依赖 Python 开发头文件的可选原生 JIT；没有修改第三方模型源码或安装系统开发包。部署需在导入 torch 前应用该配置。已验证依赖完整版本见 `local_models/requirements-tested.txt`；该文件为本机 Linux/CUDA 环境快照，其他平台尤其 CPU 实机需选择对应 PyTorch wheel。

## 复杂交叉地图

地图数据独立保存在 `experiments/scenarios.json`，无需修改模型后端即可复用于不同模型。新增：

- `crossing_reference`：按用户参考图估算，起点 S、P1–P9、终点 F，共 10 个有序目标；包含交叉、折返和连续急转。坐标约覆盖东向 -4.8～9.6 米、北向 -4.8～9.6 米，到达圈 1 米，航点限速 1.2 节。并非原图原始数据，也不复刻图中的 Jev 轨迹。
- `crossing_harbor`：同一几何形状放大十倍，到达圈 8 米、航点限速 6 节。

地图中的黑色虚线仅表示依次访问顺序；模型任务仍是按序进入航点圈，不要求紧贴连线。原有三个基础场景的坐标、到达圈及限速保持原值。`--scenario all` 现在包含所有五张地图；复杂小地图建议单独指定控制步长和候选最大速度。

小尺度地图复现：

```bash
.venv/bin/python server/experiments/simulate.py --device cuda \
  --scenario crossing_reference --steps 600 --dt 0.2 \
  --output server/experiments/results/julia-crossing-reference
```

放大地图复现：

```bash
.venv/bin/python server/experiments/simulate.py --device cuda \
  --scenario crossing_harbor --steps 500 --dt 1 \
  --output server/experiments/results/julia-crossing-harbor
```

除原有三联图外，每次生成单独的 `场景名-map.png`，标记 S、P1…、F、到达圈、实际轨迹、最后位置及已到达航点数。`summary.json` 保存控制步长、候选速度上限、当前目标与已到达航点数，输出目录内 `scenarios.json` 保存本轮地图参数。未改变现场 IP/端口或模型策略。

## 公里级地图（固定 10/20 节）

使用新场景 `crossing_km`，不要继续使用十几米尺度的 `crossing_reference` 测试当前高速候选。

- 保留 S → P1…P9 → F 的十目标交叉折返顺序，参考图坐标放大 100 倍。
- 东西跨度 1440 米，南北跨度 1440 米；两轴范围均为 -480～960 米。这是实际航点跨度，不仅是绘图坐标轴放大。
- 到达圈半径 50 米；所有航点通过速度上限 20 节。Julia 两档仍为 10、20 节，16 个候选单次选择。
- 参考折线总长约 11,965 米；以固定速度连续航行约需 1,163～2,326 秒，不含转向和模型偏航。采用 1 秒步长、3000 步预算，相当于最多约 50 分钟仿真时间，推理时暂停世界。
- 原小图与港区图及其历史结果不变，原现场 IP/端口不变。模型仍无停车候选，原停车完成判据未修改，请同时查看已按序到达数量。

```bash
.venv/bin/python server/experiments/simulate.py \
  --device cuda \
  --scenario crossing_km \
  --steps 3000 \
  --dt 1 \
  --output server/experiments/results/julia-crossing-km
```

输出 `crossing_km-map.png`（参考与实际轨迹）、`crossing_km.png`（三联图）、`crossing_km.jsonl`（逐步决策）、`summary.json` 与本轮地图参数副本 `scenarios.json`。如需保留多轮结果，请更换输出目录。
