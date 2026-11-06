# TRO_impedance_reply

将 *Nonlinear H∞ Inverse-Optimal Impedance Control for Robotic Systems* 的控制器和 A–F 比较实验移植到 Franka Research 3 / MuJoCo。

实现了 H∞ 阻抗、普通计算力矩阻抗、NRIC 导纳、双向能量流导纳和 super-twisting 滑模阻抗。FR3 的第七个自由度使用名义惯量加权的动态一致逆与零空间姿态控制。真实系统和控制器名义模型分离；摩擦、阻尼、工具质量、惯性误差及反馈测量噪声均进入闭环。

**这是跨机器人仿真复现。** 原论文采用 Indy7 真机，原始轨迹和人手施力记录未公开；本项目明确给出 FR3 轨迹、接触几何、虚拟操作者和等效磨削振动。不能将仿真运行成功等同于论文性能结论已得到复现，也不能据此宣称真实 FR3 的稳定性或无源性。

## 安装与运行

在项目根目录使用 Python 3.11 或更新版本：

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e .
export OPENBLAS_NUM_THREADS=1
python -m unittest discover -s tests -v
python -m fr3imp --experiment B --controller hinf
```

MuJoCo 固定为 3.16.0。控制周期 0.25 ms（4 kHz），F/T 采样 1 ms（1 kHz）；默认无力滤波。工厂函数将原位置执行器转换成有真实力矩限制的直接电机，不修改模型资源原文件。

本次记录的实验环境是 Python 3.14.6，直接依赖版本见 [requirements-lock.txt](requirements-lock.txt)。在匹配的 Python 环境中可先执行 `python -m pip install -r requirements-lock.txt`，再安装项目。

完整矩阵及分析：

```bash
python -m fr3imp.suite --experiments ABCDEF --d-trials 30 --workers 4 --output outputs/suite
python -m fr3imp.analyze outputs/suite --output docs/results
```

接触组的物理步长检查与结果核对：

```bash
python -m fr3imp.suite --experiments BCEF --physics-substeps 2 --workers 4 --output outputs/sensitivity-substeps2
python -m fr3imp.compare_steps outputs/suite outputs/sensitivity-substeps2 --output docs/results
```

用 `--resume` 续跑已完成且代码哈希、参数一致的运行。用 `--physics-substeps 2` 将物理积分步长减半，控制与传感器频率保持不变。`--ideal` 关闭本项目加入的模型误差、工具质量、摩擦、阻尼和传感器噪声，用于单独检查理想条件。

|组|场景|比较|默认时长/数量|
|---|---|---|---|
|A|自由空间 ∞ 轨迹|普通阻抗、导纳、H∞|30 s × 3|
|B|固定刚性墙面|上述三种 + 双向能量流、滑模|30 s × 5|
|C|白板、凸起障碍、10+3 sin(t) N 法向力|导纳、双向能量流、H∞|30 s × 3|
|D|滚转约 90° 前后的转动柔顺交互|普通阻抗、导纳、滑模、H∞|35 s × 4 × 30|
|E|下压接触与磨削等效振动|双向能量流、滑模、H∞|30 s × 3|
|F|B 场景的 REF 增益比较|H∞ α=0.25/0.5/1|30 s × 3|

总计 21 种设置、137 次运行。D 的 30 次是 10 组虚拟操作者参数各重复 3 次，不能替代论文的真人实验。E 不模拟材料去除。

## 结果与回放

每次运行在 `outputs/` 下保存 `config.json`、`summary.json` 和 `trajectory.npz`；批处理另存 `manifest.json`。原始数据包含真实/测量状态、接触力、F/T 测量、命令/施加力矩、参考误差、饱和、beta 和能量罐。

`fr3imp.analyze` 检查矩阵完整性、运行时长、采样间隔和有限值，独立复算位置 RMS 与峰值力，并生成 A–F 图和 D 组统计图。失败或提前终止的运行保留在结果中。

```bash
# 从已保存的关节轨迹生成真实 MuJoCo 画面；不是重新运行控制器。
MUJOCO_GL=egl python -m fr3imp.replay outputs/suite/B_hinf_trial00_seed100 \
  --time 15 --image outputs/B.png --video outputs/B.mp4 --speed 2
```

MP4 需要系统安装 `ffmpeg`；Linux 无显示器时可使用 EGL。`--speed 2` 表示两倍速回放。图表和逐项指标见运行后生成的 [仿真结果](docs/results/RESULTS.md)。

## 论文与实现依据

详细公式、七轴扩展、坐标约定、噪声参数和场景适配见 [实现说明](docs/IMPLEMENTATION.md)。

- 主论文：[DOI 10.1109/TRO.2026.3741518](https://doi.org/10.1109/TRO.2026.3741518)，用户提供的 PDF 保存在项目根目录。
- 双向能量流调制：[DOI 10.1109/TRO.2024.3443690](https://doi.org/10.1109/TRO.2024.3443690)，使用用户提供全文的式 (19)–(22)、(28)、(31)–(32)。
- 模型说明与原许可证：[franka_fr3/README.md](franka_fr3/README.md)、[Apache-2.0](franka_fr3/LICENSE)。参考论文版权归原权利人所有。

原始仿真输出、虚拟环境及缓存默认不进入 Git。源代码、测试、结果摘要和图可纳入版本管理。
