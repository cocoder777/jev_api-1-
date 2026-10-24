# 外部报文决策服务

先启动 Kev-4B 推理服务（单独终端保持运行）：

```bash
.venv-kev/bin/python server/local_models/serve_kev.py --device cuda
```

再启动报文服务，默认使用 Kev-4B：

```bash
python3 server/decision_service.py --multicast-interface 192.168.3.202
```

Kev 通过独立接口 `http://127.0.0.1:18028/v1/systemone` 推理，速度由 `policies/kev_policy.py` 的 `KEV_SPEED_KNOTS` 常量统一设置，并提供16个局部可达航向候选。服务接收 `0x0660` 的 `trackingMode` 任务、`0x0666` 停止请求和 `0x0060` 态势，按 `params.platCode == platformId` 选本艇，输出 `0x0662` 控制报文。可恢复错误连续达到3次才返回 `0x0664` 失败状态；任务/停止请求支持最多10片的协议分包重组。

Jev 保留原有64个速度/航向候选及本机密钥配置，可显式切换：

```bash
python3 server/decision_service.py --backend jev --multicast-interface 192.168.3.202
```

也可通过 `POLICY_BACKEND=kev/jev/julia` 选择后端；命令行参数优先。策略由 `policies/factory.py` 创建，报文接收、态势缓存、控制与状态上报共用原有流程。启动日志显示实际后端。Compose 同样默认 Kev，须单独运行其推理服务；使用 Jev 时在 `.env` 中设置 `POLICY_BACKEND=jev` 并填写 `JEV_API_KEY`。

启动时自动读取脚本同目录的 `decision_service.local.json`，其中可保存
`JEV_API_KEY` 和 `JEV_ENDPOINT`，无需每次执行 `export`。Kev 的实际初始方向也可保存为
`INITIAL_COURSE_DEG`（0到360度，不含360），仅作为静止且态势未提供 `targetCourse` 时的备用方向；有效态势航向始终优先，0度也有效。该文件已加入 Git 忽略规则，
包含密钥时应仅在本机保存，不要复制进镜像。环境变量优先于本机配置；
本机配置还支持 `MAX_CONSECUTIVE_ERRORS` 和 `COMMAND_HOLD_TIMEOUT`。模型、地址也可用 `--model`、`--endpoint` 参数覆盖，命令行参数优先。
Jev 默认接口地址为 `https://api.jevcn.cn/v1/systemone`；Kev 使用独立的
`--kev-endpoint` / `KEV_ENDPOINT`，无需 Jev 密钥。

适配约定：

- 默认态势速度、航路点 `routeSpeed` 和控制输出均按节处理；若外部三者均为 m/s，使用 `--speed-unit mps`。协议文档未明确三者物理单位，接实艇前需核对。Jev 内部航路点上限统一为 m/s。
- 航向为北起顺时针绝对角度。Jev 最大候选速度默认 30 节（`--max-speed-knots`）；Kev 使用固定配置速度，转向角速度参数为 `--max-turn-rate-deg-s`，默认45度/秒。1秒周期对应当前方向±45度的候选范围，0.2秒周期对应±9度。该参数用于候选生成和运动预测，须按实际船舶配置。Kev 每轮读取最新态势中的位置、速度与 `targetCourse`，速度为0时也保留有效航向；只有静止且未提供航向时，才需要 `--initial-course-deg` 作为备用。
- `planWaypointList` 中每一项都是必须访问的目标，支持只有1个航点；当前位置取自态势，不把列表首项当成起点。`currentRoute` 是已完成航点数量：0从列表第1点开始，1从第2点开始。可选 `next_waypoint_index` 为从0开始的明确目标索引。`routeRadius` 允许为0或正数，负数及非有限值无效；用于到达判断时，0或未提供该字段均使用默认20米容差，正数沿用该值；列表最后一项就是终点。
- `--control-interval` 默认1秒，也是候选动作预测时长。每个任务使用独立模型工作线程，控制发送不被推理阻塞；同一任务最多一个请求在途。航点改变、到达终点或收到停止请求后，不采用之前航点的滞后结果。
- 接收任务后，控制台以 INFO 打印重组后的完整 JSON（含全部航点和原有字段，中文直接显示），并保留任务摘要（ID、平台、航点数量、起始进度）、停止请求、任务完成和错误信息。完整任务在平台过滤和字段校验前打印，便于排查被忽略或校验失败的报文。每次成功发送动力控制 `0x0662` 后，以 INFO 打印目标地址、任务ID、平台编号、编码后的航速/航向、模式（`new`新决策、`hold`续发、`stop`停车）及完整十六进制报文；这些发送日志逐条打印。态势保持逐包实时接收与缓存，接收日志在所有平台和接收入口之间共享5秒限频；其他每周期控制与推理明细保持 DEBUG。
- `--status-interval` 默认 1 秒。执行中状态由独立定时线程按单调时钟上报，不受模型请求耗时影响。
- 启动时自动加入态势组播 `239.255.0.10:18010`；加入或接收出现 socket 错误后，每隔 3 秒自动重试。多网卡环境可通过 `--multicast-interface` 或 `MULTICAST_INTERFACE` 指定本机网卡 IPv4。
- 执行中 `0x0664.params` 返回 `nextWaypointIndex`、`nextWaypoint`、`remainingWaypointNum`、`remainingDistanceM`、`estimatedRemainingTimeS` 和 `completionRate`；其中目标索引从0开始，`currentRoute` 返回已完成数量。首段距离包含首次有效态势的位置到第一个待访问点。距离单位为米，时间单位为秒，`completionRate` 为0到100的数值百分比；必要速度为0时预计时间为 `null`。
- 每个容器通过 `PLATFORM_ID` 只接收一个平台的任务。同一 `platCode` 每收到一个校验通过的新任务，就立即覆盖该艇尚未结束的旧任务；`simId/id` 和参数完全相同也会覆盖，不按重复报文忽略。新任务从自己报文中的 `currentRoute` 开始，使用当前态势，不继承旧任务的航点进度或模型历史。不同艇独立处理；校验失败的新任务不会撤销正在执行的任务。
- 覆盖时先撤销旧任务的输出权，旧任务立即退出控制循环，其滞后的模型结果、周期状态、错误处理和停车指令都不能影响新任务。不同 `simId+id` 的旧任务返回 `status=3`，`params.phase=superseded`、`errorCode=TASK_SUPERSEDED`，原因说明已被新任务覆盖，并附 `replacedByTaskId/replacedBySimId`；同 `simId+id` 不发送旧任务终态，避免覆盖行为树缓存中的新状态。新任务正常上报 `status=1`。同 ID 的后续停止报文作用于当前新实例。
- 旧在途模型请求不保证能强制中断，但返回结果会被丢弃；新任务不等待旧控制线程或 HTTP 请求结束。Kev 的 HTTP opener 和导航状态按线程隔离，客户端不再用旧请求的互斥锁阻塞新请求；模型服务繁忙时仍可能排队。此处没有恢复任务链或等待前序完成的队列。
- 收到 `0x0666` 后停止常规航迹动作，持续向动力系统下发目标航速0；组播本艇实际航速不高于 `STOP_SPEED_THRESHOLD` 后，回复 `status=2` 的停车完成状态。
- 全部航点到达后自动进入 `finishing` 停车阶段，持续下发0速；停车确认后才报告 `status=2`。到达终点后即使因惯性移出到达圈，也保持停车阶段。`completionRate=100` 表示航点已访问完，停车尚未确认时仍是执行中 `status=1`。
- `SITUATION_MAX_AGE` 限制可用于决策的本艇态势新鲜度，默认2.5秒；命令行服务 `JEV_TIMEOUT` 默认30秒，`KEV_TIMEOUT` 默认120秒。
- `--max-consecutive-errors` / `MAX_CONSECUTIVE_ERRORS` 默认3。模型请求超时、断连、HTTP 408/429/5xx、控制发送失败以及连续控制周期缺少新鲜态势均会记录错误；恢复成功后清零。连续3次才发送零速并报告 `status=3`。非法任务字段、无效模型选项、鉴权错误等不可恢复错误直接失败。
- `--command-hold-timeout` / `COMMAND_HOLD_TIMEOUT` 默认5秒，以最近有效决策的时间计算。重试或等待期间按控制周期续发上一有效指令；超过时限暂发0速并继续等待/重试，尚无有效指令时不会生成替代动作。耗时超过此时限的模型结果会被丢弃并记录超时，不能把旧态势生成的动作当作新决策下发。HTTP请求仍由后端自己的超时参数结束；5秒保持时限不代表取消在途HTTP请求。
- 失败 `description` 包含阶段、平台、航点、连续次数和原始原因，并区分“已发送停车指令”与“已确认停车”。`params` 同时提供 `phase`、`errorCode`、`errorStage`、`errorType`、`lastError`、`consecutiveErrors`、`maxConsecutiveErrors`、`stopCommandSent`、`stopConfirmed`；鉴权密钥不写入原因文本。最终状态遇到socket发送错误时会重试，状态通道错误不混记为模型错误。
- `--max-cycles=0` 持续运行到路线完成或停止请求；正数是调试用的有效控制次数上限，达到上限也会先停车确认再结束。航点、终点停车、重试和状态由公共控制流程处理，Kev/Jev/Julia只负责动作选择。

离线验证（模拟 API，无外网调用）：

```powershell
python -m unittest discover -s server/tests -v
```

## 本地模型

支持 `--backend julia` 切换到本地 Julia-1。安装、结构、离线模拟、测试结果和原现场地址记录见 [LOCAL_MODELS.md](LOCAL_MODELS.md)。

Kev-4B 独立后端使用固定配置速度、16航向，通过本机官方推理服务接入。安装、启动和测试见 [KEV.md](KEV.md)。
