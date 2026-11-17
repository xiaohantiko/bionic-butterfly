# 公开仿真验证 · 2026-11-17

从提交 `bc7800446749` 加上本轮串口批量失联修正的源码运行，Python 3.9.6。被测 `simulate.py` 的 SHA-256 为 `fe8a3f1c9d48a542562836af46bef2a759463cf693a29ab3356b5bec3d59a42f`，其余源码摘要列于[验证摘要](verification.json)。

| 检查 | 结果 |
| --- | --- |
| 混控单元测试 | 22/22 通过 |
| CRSF 综合故障回放 | 33/33 检查点通过 |
| 独立封包检查 | 9/9 通过，含直行、左右差动及同批失联与恢复 |
| 标准情景 | 5 个直行、1 个差动回放，各 65 秒模型时间 |
| 差动轨迹 | 1,969 行，虚拟脉宽 1,113–1,913 μs，最后一行撤销输出 |
| 确定性复跑 | 两次综合结果 JSON 和 CSV 逐字节一致 |

修正前，链路质量为零、链路恢复和有效控制帧若出现在同一次串口读取，最后一个链路状态会掩盖失联并保留虚拟输出。修正后先撤销输出，恢复后仍要重新完成解锁的低到高边沿。[故障前后记录](verification.json)

合成翼的直行情景结果如下。角度为模型半摆幅，力矩限幅为模型中触及力矩上限的采样占比。

| 情景 | 目标半摆幅 | 模型半摆幅 | 力矩限幅 |
| --- | ---: | ---: | ---: |
| `cap40_nominal` | 32.0° | 31.6° | 0.0% |
| `neutral40_nominal` | 40.0° | 39.5° | 0.0% |
| `neutral40_sag_derated` | 40.0° | 39.0° | 33.9% |
| `neutral40_favorable` | 40.0° | 39.7° | 0.0% |
| `neutral45_at3Hz` | 45.0° | 44.1° | 4.2% |

公开仿真把每侧前后翼等效成理想三角翼。虚拟脉宽和摆幅是模拟值，实体舵机负载、飞行和硬件保护仍待实测。

- [综合仿真结果](simulation-result.json)及[差动轨迹 CSV](simulation-trace.csv)
- [单元测试输出](unit-tests.txt)及[综合仿真日志](simulation-run.txt)
- [默认演示的状态变化](demo-transitions.jsonl)及[示例回放](example-replay.jsonl)

在仓库根目录复跑。

```sh
python3 -B -m unittest discover -v
python3 -B run_sim.py --summary
python3 -B run_sim.py --input example-commands.jsonl
python3 -B simulate.py --output output/replay
```
