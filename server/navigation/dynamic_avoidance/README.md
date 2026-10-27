# 独立动态避障

**暂不接入 Kev 或其他在线后端。** 本模块保留独立调用及离线验证能力，不读实时态势、不发控制指令，不依赖模型服务。调用方为每个任务创建一个 `DynamicAvoidancePlanner`，串行调用，切换任务时创建新实例或执行 `reset()`。本目录的 `kev.py` 保留为未启用的适配工具，当前 Kev 策略和控制循环均不调用它。

## 使用

从仓库根目录设置 `PYTHONPATH=jev_api:.`。内部速度全部使用米／秒，位置使用固定原点的东／北米坐标；航向从正北顺时针，0°为北、90°为东。

```python
from server.navigation.dynamic_avoidance import (
    Action, DynamicAvoidancePlanner, Target, Vessel,
)

planner = DynamicAvoidancePlanner()
result = planner.plan(
    Vessel(position_m=(0, 0), speed_mps=10, course_deg=0),
    [Target(target_id=20, position_m=(0, 600), speed_mps=10, course_deg=180)],
    Action(speed_mps=10, course_deg=0),
)
print(result.status, result.action, result.assessment)
```

也可以调用 `scene_from_situation(records, origin_lon_lat=(120, 30), speed_unit='knots', own_id=1010)` 转换协议态势，返回 `(Vessel, tuple[Target, ...])`。`speed_unit` 必须显式指定为 `knots` 或 `mps`。输入不会被修改；本艇必须具有有效位置、速度、航向和不超过5秒的接收年龄。相同编号采用最后一条记录，非法编号报错。使用本地 `_received_age_s`，不使用源协议时间戳；未提供年龄按当前快照处理，调用方必须确保这一假设成立。原点必须在同一任务中保持不变。

目标的 `position_m`、`speed_mps`、`course_deg` 可为 `None`，表示未知，不能自行用0填充。静止目标允许省略航向。直接构造对象时非法数值报错；协议转换时目标非法字段转为未知，本艇非法字段报错。模块不维护目标历史，调用方每次须提供完整已知目标快照，不能把暂时未探测到的目标直接从输入删除。

## 结果含义

| 状态 | 含义 |
| --- | --- |
| `clear` | 期望动作通过预测检查，原样返回 |
| `avoiding` | 期望动作存在风险，返回检查通过的替代动作 |
| `blocked` | 没有安全候选，`action=None` |
| `insufficient_data` | 缺失／过期目标的保守可达范围影响规划，无法确认安全，`action=None` |

`assessment.risks` 给出各有效目标的预测最小净空、最近会遇距离和时间（包括制动尾段）；`threat_ids` 为期望动作涉及的危险或不确定目标。避让成功时 `assessment` 对应返回动作；失败时对应期望动作。`uncertain_target_ids` 和 `diagnostics` 说明数据不足原因。没有有效目标时最小净空为 `None`。

失败结果中的 `emergency_action` 是经过评估的应急建议，**不是安全指令**；其完整检查结果在 `emergency_assessment` 中。它按不确定目标数量较少、最小净空较大的顺序选择。模块不会把零速直接当作安全，也不自动决定是否执行不安全建议。

## 默认预测与策略

- `AvoidanceConfig` 设置预测60秒、积分步长0.2秒、加速1.46m/s²、减速1.75m/s²、最大转向45°/s。输入应使用实际艇的能力配置。
- 本艇复用现有运动预测工具，保持候选指令至预测末尾，再追加具有真实时间长度的制动尾段。主预测长度至少覆盖速度转换和转向时间。
- 目标按恒速、恒航向推进，首先补偿接收年龄；静止目标也参与检查。同一时间区间内检查双方相对运动线段，不以空间轨迹相交替代同时会遇。
- 基础安全中心距离为 `80 + hypot(11.6, 3.4)/2 ≈ 86.04m`，已包含本艇尺寸，不再次叠加艇体红圈。另加 `safety_margin_m`，以及 `uncertainty_mps × (接收年龄 + 预测时间)`，默认不确定性增长速度0.5m/s。净空超过0.5米才通过检查。
- 目标数据超过5秒或运动字段缺失时，用最后位置和默认30m/s的未知目标速度上界形成保守可达圆；若已报告速度更大，采用更大的值。没有位置时始终视为相关。只有可达范围完全不影响候选轨迹，才允许继续规划并保留诊断。此上界是环境假设，可按实际目标速度范围调整。
- 危险时以当前航向为中心，默认每15°生成候选，单周期航向命令偏差上限为 `max_turn_rate_deg_s × planning_interval_s`（默认45°）；实际运动依然连续转向。速度候选含期望速度的100%、75%、50%、25%、本艇当前速度与期望速度的较小值及零速，不主动加速到期望速度以上。
- 筛除危险候选后优先右转，其次直行减速，再次左转；同类优先靠近期望航向，再减少速度变化。上一次避让方向在偏差相差不超过15°时优先保持。期望动作重新安全时恢复原动作并清除方向记忆。

这些结果只适用于输入态势、配置的运动模型和预测范围。第一版不判断完整海上会遇规则，不预测目标意图、加速度或转弯，不包含真实侧滑、风流和感知误差模型。独立模块不会绕过现有静态规划或控制守卫。

## 当前接入状态

Kev 已撤下动态候选筛选、低速扩展、临时等待和动态发送前复核，继续使用原有航点跟踪及静态避障。`KevPolicy` 和策略工厂不再接收动态配置参数。保留的适配工具和此前测试数据不表示在线启用；相关回归测试明确验证运动目标不会改变 Kev 候选及静态控制守卫。

## 离线验证

```bash
PYTHONPATH=jev_api:. python3 -m unittest server.tests.test_dynamic_avoidance -v
PYTHONPATH=jev_api:. python3 -m server.experiments.dynamic_avoidance --output /tmp/dynamic-avoidance.json
PYTHONPATH=jev_api:. python3 -m unittest discover -s server/tests -q
```

示例验证对遇、交叉、追越、右侧受静态目标阻挡的场景，以及75次重复规划的闭环过程。JSON 输出预测双方轨迹、规划状态、风险信息和闭环采样；使用独立的0.02秒积分器重新计算输出动作及制动尾段的最小净空，断言安全。该密集采样复核用于回归验证，不是连续运动的形式化证明，也不代表实艇验证。

此前真实 Kev 的动态闭环结果仅为历史接入版本验证，不代表当前版本具备在线动态避障。已运行的报文服务不会自动加载代码变化，需重启后使用当前版本。
