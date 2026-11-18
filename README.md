# 遥控仿生蝴蝶仿真

这个项目演示四片分体翼的遥控原理，并提供可在普通电脑上运行的 Python 仿真。当前验证进度为离线仿真，实物飞行能力待验证。

## 更新记录

### 2026-11-18

- 新增[四片翼动态演示、慢放和原始数据](verification/2026-11-18-dynamics/README.md)。同一侧共用驱动，每片翼有独立的角度和惯量。五组情景演示对称扑翼、左右差动、右后翼碰到弹性挡块，以及撤销驱动后的衰减。使用合成参数，结果不能判断样机是否能飞或会不会卡。

### 2026-11-17

- 同步复测证据，覆盖同一批串口数据内先失联、再恢复的情况。修复后会撤销虚拟 PWM，重新解锁才恢复。公开数据包括 33 个控制检查点、22 项单元测试和 9 个独立封包检查。[验证与轨迹](verification/2026-11-17/README.md)
- 补充四片分体翼与同侧联动的结构说明。流程图展示遥控指令怎样转换成虚拟 PWM。
- 补充结果读取方法。综合仿真输出 5 组预设直飞情景和 1 组转向回放的模型结果。
- 补充公开模型的假设。前后翼按侧等效，使用合成几何。四翼相互影响、硬件断电保护及飞行表现需通过实测确认。

## 工作原理

左前翼、左后翼、右前翼、右后翼分别有翼膜、骨架和夹座。每侧一个驱动通过共轴摇臂带动同侧前后翼。油门改变两侧扑翼幅度，转向输入改变幅度差，俯仰输入调整翼根平均角。实际产生的推进及操纵力矩还取决于翼膜变形、来流和机身姿态。

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
python3 -B four_panel_demo.py
```

- [`mixer_sim.py`](mixer_sim.py) 负责遥控状态机和左右扑翼混控。自动化测试覆盖解锁条件、失联和超时后的撤销输出、USB 互锁，以及角度和速度限幅。
- [`run_sim.py`](run_sim.py) 演示状态切换，也可回放 [JSONL 示例](example-commands.jsonl)。`--summary` 只显示状态或原因变化的事件。
- [`simulate.py`](simulate.py) 合成 CRSF 字节流，检查分段接收、无效数据包及故障恢复，并记录虚拟 PWM、模型中的舵机角度和负载情况。综合仿真生成 `output/simulation-trace.csv` 和 `output/simulation-result.json`，可用 `--output` 更改保存位置。JSON 中 `control_simulation` 记录控制检查结果，`flight_feasibility` 保持 `UNVERIFIED`。
- [`four_panel_demo.py`](four_panel_demo.py) 使用单独的合成指令和六个转动自由度演示面板变形与遇阻反应，输出到 `output/four-panel-demo/`。图像、脚本与 CSV 均可离线查看。

## 模型边界

`simulate.py` 的遥控仿真把同一侧的前后两片等效成一个理想三角翼，使用示例质量。它适合演示遥控程序与单侧合力计算。舵机模型将数据表的空载速度和扭矩端点连成简化包络。气动力筛选根据模型翼根角速度和给定前飞速度，反算达到承重目标所需的等效竖向力系数。`four_panel_demo.py` 单独建模四片翼的等效角度，未与串口、供电或升力模型联算。两者均未求解真实翼膜形状、翼间尾流、净推进或机身姿态。实际飞行可行性需用带翼角度、电流、力和力矩测量，以及整机试飞确认。

参考资料：[CRSF 协议](https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md) · [舵机数据表](https://cdn.shopify.com/s/files/1/0712/1472/7353/files/A06_Technical_Specifcation_2026_1.pdf?v=1776068873) · [AirPulse 研究](https://arxiv.org/html/2602.06811v1)
