# 遥控仿生蝴蝶仿真

这个项目演示双侧独立扑翼的遥控原理和离线验证方法。左右两根翼轴各有一个驱动，薄膜覆盖轻质骨架。油门改变两侧扑翼幅度，转向输入改变幅度差，俯仰输入调整翼根平均角。实际产生的推进及操纵力矩还取决于翼膜变形、来流和机身姿态。

```mermaid
flowchart LR
  RC[模拟遥控指令] --> Decode[CRSF 解码]
  Decode --> State[解锁与失联状态]
  State --> Mix[左右翼混控]
  Mix --> PWM[虚拟 PWM]
  PWM --> Actuator[带载舵机模型]
  Actuator --> Screen[气动力需求筛选]
```

上电后，状态机要求先收到有效的低解锁信号，再接收低油门下的解锁上升沿。命令超时、零链路质量或模拟检测到 USB 供电时，控制链撤销虚拟输出。链路恢复后需要新的解锁操作。真机设计中的舵机供电关断和 PWM 禁能需要由硬件实现并上机验证。

## 运行

使用 Python 3.9 或更新版本，无需第三方依赖。

```sh
python3 -B -m unittest discover -v
python3 -B run_sim.py --summary
python3 -B run_sim.py --input example-commands.jsonl
python3 -B simulate.py
```

`mixer_sim.py` 实现遥控状态机和双翼角度混控，`run_sim.py` 用于演示或回放 JSONL，`simulate.py` 合成 CRSF 字节流并检查状态转换、角度跟踪和驱动力需求。综合仿真生成 `output/simulation-trace.csv` 和 `output/simulation-result.json`，可用 `--output` 更改保存位置。

## 模型边界

综合仿真使用理想三角翼和示例质量，结果对应这组示例输入。舵机模型将数据表的空载速度和扭矩端点连成简化包络。气动力筛选根据模型翼根角速度和给定前飞速度，反算达到承重目标所需的等效竖向力系数。它没有求解翼膜扭转、实际净推进或机身姿态。实际飞行可行性需用带翼角度、电流、力和力矩测量，以及整机试飞确认。

参考资料：[CRSF 协议](https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md) · [舵机数据表](https://cdn.shopify.com/s/files/1/0712/1472/7353/files/A06_Technical_Specifcation_2026_1.pdf?v=1776068873) · [AirPulse 研究](https://arxiv.org/html/2602.06811v1)
