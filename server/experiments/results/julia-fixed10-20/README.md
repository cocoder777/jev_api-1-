# 固定 10/20 节、16 候选实测

Julia 后端使用两档速度 × 八个航向，一次模型选择；无零速候选。用户现有 tie line 提问保留。该目录为新实验，历史分组结果未覆盖。

600 步、每步 0.2 秒；按序到达 0/10，最后距首个目标约 44.63 米，平均决策约 7.1 ms。所有 600 步输出均核对为 10 或 20 节，每步模型实际收到 16 个候选。

地图原始限速仍为 1.2 节，两档候选均高于它；本轮未改变地图限速，也未移除仿真原有的停车完成判据。未到点结果不应归因于停车判据。

运行命令：

```bash
.venv/bin/python server/experiments/simulate.py --device cuda --scenario crossing_reference --steps 600 --dt 0.2 --output server/experiments/results/julia-fixed10-20
```
