# TRO_impedance_reply

将 *Nonlinear H∞ Inverse-Optimal Impedance Control for Robotic Systems* 的控制器与比较实验移植到 Franka Research 3 / MuJoCo。当前依用户指示只复现 **A、B、C、D、F**；**E 砂轮机暂缓**，范围约定见 [AGENTS.md](AGENTS.md)。

实现了 H∞ 阻抗、普通计算力矩阻抗、NRIC 导纳、双向能量流导纳和 super-twisting 滑模阻抗。FR3 的第七个自由度使用名义惯量加权的动态一致逆与零空间姿态控制。真实系统和控制器名义模型分离；摩擦、阻尼、工具质量、惯性误差及反馈测量噪声均进入闭环。

**默认运行 A/B/C/D/F，采用明确适配的 `fr3` 配置及 1 kHz 时序。** [134 次主实验 + 51 次敏感性验证](docs/results/reproduction-fr3/REPORT.md)包含对比图、D 的 30 次重复及五组回放。A 的低刚度跟踪、B/C 的接触抖振差异、D 的两姿态旋转响应、F 的增益/误差趋势已分别检查。保留 `paper` 原任务参数/增益配置及其[对照结果](docs/results/reproduction-active/REPORT.md)，不把 FR3 适配值标成论文原值。

新增 [A/B/C/D/F 论文原图逐图对照与差距说明](docs/results/paper-comparison/REPORT.md)：A 的定性趋势较接近；B/C 仍有接触行为和幅值差距；D 的 pitch 统计优势、F 的明显静态误差改善尚未复现。该对照用测量 F/T 计算平衡参考，与旧报告的物理真值指标分别保留，并正确区分 C 的关节速度和 REF。

**新增并列配置 `fr3v2`。** 它恢复论文的 Λ/D/K、NRIC G 和 D 虚拟操作者，改为假设关节摩擦 ×3、F/T 延迟 3 ms 和欠阻尼接触；`fr3` 保留作对照，默认配置未变。[134 次主实验 + 148 次敏感性验证](docs/results/reproduction-fr3v2/REPORT.md)和[论文原图对照](docs/results/paper-comparison-fr3v2/REPORT.md)显示：A、F 的 REF 和平衡偏差均值、D 的 yaw 更接近论文；导纳差于 H∞ 的现象依赖论文 G 与 F/T 延迟同时存在；C 的 H∞ 力跟踪和 B 的 H∞/SMC REF 差异比 `fr3` 差；D 的 pitch 优势仍未复现。这些平台假设未经标定，其中延迟和接触参数是在观察控制器差异时选的，存在选择偏差。

**D 另有一个并列的操作者模型 `fig11_replay`。** 它把论文 Fig. 11 数字化的实测力矩（×.6）开环加到四种控制器上，默认操作者和 `fr3v2` 结果都没有改。[171 次运行的报告](docs/results/d-operator/REPORT.md)显示：H∞ 的响应与论文几乎重合，yaw 上普通阻抗为 H∞ 的 3.45 倍（论文 3.47），pitch 上 SMC 从 .98 倍升到 1.90 倍（论文 7.08），普通阻抗从 3.32 倍升到 5.56 倍（论文 16.96）；但导纳仍接近理想响应，pitch 为 .87 倍（论文 9.04），紧握和延迟都没有改变这一点。所以 D 的剩余差距主要不在操作者。SMC 与 H∞ 的差距还依赖摩擦 ×3 这个假设。

**导纳差距的诊断。** [36 次运行的报告](docs/results/d-admittance/REPORT.md)从论文曲线辨识出：H∞ 符合写明的阻抗方程，导纳却相当于多了一项 `ΛMx⁻¹m`，这在 C 中表现为平均力只有 5.9 N（期望 10 N）。软内环、延迟和恒定增益都解释不了这个形式。在 FR3 上只去掉 `-JᵀF` 这一项，pitch 倍数就接近论文（普通阻抗 15.35 / 导纳 14.39 / SMC 4.34，论文 16.96 / 9.04 / 7.08），C 的力偏差也随之出现，H∞ 则基本不变。与公开的 Indy7 辨识参数核对后，这个形式在名义模型不含转子惯量时量级相符；计入转子时 D 和 C 需要的系数互相矛盾。结果还依赖所用参数组。这只说明这个形式足以解释证据，不能证明作者的代码如此；它也不是论文的控制器，因此没有加入任何配置。

这是跨机器人仿真，不是真实 FR3 或原论文 Indy7 的实验记录。用户确认把工具质量、接触材料和命令滤波作为明确的仿真假设保留。原始控制日志、人手施力和驱动标定未公开，运行完成不等于稳定性或无源性证明。

新配置恢复 A/B/F 的 260×260 mm ∞ 轨迹、45° 斜墙和 C 的 100×300 mm 轨迹；为大轨迹配置可达的 FR3 起始姿态。同一实验的控制器使用相同任务阻抗、接触环境、虚拟操作者和 100 Hz 命令接口。B/F 调整任务惯量与阻尼；C 仅调整 x/y；D 调整旋转阻抗和交互力矩，并明确加入第二阶段被动滚转支撑。完整参数见 [实现说明](docs/IMPLEMENTATION.md)；选择过程及 9 次提前终止见 [133 次适配审计](docs/results/fr3-adaptation/REPORT.md)。

`fr3` 中 H∞ 基准有效 α=.25；F 的 .25/.5/1 是相对倍率，因此有效 α=.0625/.125/.25。D 不是原论文的真人交互，B 也未伪称发生原机急停。绝对误差、每个控制器的统计排名与原机仍有差异。

已有 E 的工具、内部转子和圆盘代码保留，当前不继续开发或验证；默认测试跳过 E 的专项测试。

## 安装与运行

在项目根目录使用 Python 3.11 或更新版本：

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e .
export OPENBLAS_NUM_THREADS=1
python -m unittest discover -s tests -v
python -m fr3imp --experiment C --controller hinf
```

MuJoCo 固定为 3.16.0。默认控制与 F/T 采样均为 1 ms（1000 Hz），无力滤波。每周期 4 个物理子步（250 μs），子步内保持七轴力矩，不重算机械臂控制器或采样反馈；E 的转子电机及物理摩擦在子步内演化。100 Hz 低通只作用于七轴力矩命令，作为共同接口假设，不是完整 FCI 驱动模型。工厂函数将原位置执行器转换成有力矩限制的直接电机，不修改模型资源原文件。

完整重跑当前范围（134 次主矩阵 + 51 次固定敏感性检查，D 每控制器 30 次，跳过 E）：

```bash
OPENBLAS_NUM_THREADS=1 python scripts/validate_reproduction.py --workers 8 --resume
OPENBLAS_NUM_THREADS=1 python scripts/report_reproduction.py
```

输出写入 `outputs/reproduction-fr3`，报告写入 `docs/results/reproduction-fr3`，保留此前各版本。重跑 `fr3v2`（134 次主矩阵 + 148 次敏感性检查），其报告还会读取 `outputs/reproduction-fr3/main` 作三方对照：

```bash
OPENBLAS_NUM_THREADS=1 python scripts/validate_reproduction.py --profile fr3v2 --output outputs/reproduction-fr3v2 --workers 8 --resume
OPENBLAS_NUM_THREADS=1 python scripts/report_reproduction.py --root outputs/reproduction-fr3v2 --output docs/results/reproduction-fr3v2
OPENBLAS_NUM_THREADS=1 python scripts/compare_paper_figures.py --runs outputs/reproduction-fr3v2/main --output docs/results/paper-comparison-fr3v2
```

重新生成已保留的适配审计：

```bash
OPENBLAS_NUM_THREADS=1 python scripts/report_fr3_adaptation.py
```

`--inertia-scale` 在所选 profile 的基础上额外缩放 Λ，并按平方根缩放 D，K 不变。D 的 `--operator-stiffness-scale` 显式覆盖该 profile 的 yaw/pitch 握持尺度。它们是诊断选项；默认适配已在 `fr3imp/profiles.py` 定义，无需再次传入缩放值。

快速检查 A/B/C/D/F 的 18 种设置，D 只跑一次：

```bash
python -m fr3imp.suite --profile fr3 --d-trials 1 --workers 4 --output outputs/quick
python -m fr3imp.analyze outputs/quick --output outputs/quick-report
```

单独检查命令滤波假设：`python -m fr3imp --experiment C --controller hinf --command-cutoff 0 --output outputs/filter-off`。关闭滤波的论文原增益在 1 kHz 下可能持续限幅，应检查 `saturation_fraction` 和实际力矩。

`fr3` 的环境记录见 [environment.json](docs/results/reproduction-fr3/environment.json)。`fr3v2` 的环境和测试记录见 [environment.json](docs/results/reproduction-fr3v2/environment.json) 和 [VALIDATION.txt](docs/results/reproduction-fr3v2/VALIDATION.txt)。当前代码的测试记录见 [VALIDATION.txt](docs/results/d-admittance/VALIDATION.txt)：共 46 项，43 项通过、3 项 E 专项跳过。加入 D 操作者后运行时代码哈希已变，所以回放早先的 `fr3`、`fr3v2` 记录要用各自输出目录里的 `source/` 快照，`--resume` 也会重跑它们。此前 C 审计使用 Python 3.14.6，旧直接依赖版本保存在 [requirements-lock.txt](requirements-lock.txt)。

C 组的 1 kHz 调整与验证见 [诊断报告](docs/results/c-1khz/REPORT.md)：

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. python scripts/validate_c.py
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. python scripts/report_c.py
```

四种配置分别保留：`fr3` 为当前默认适配；`fr3v2` 恢复论文任务参数并改用上述平台假设；`paper` 在 C 使用原 Λ/D/K、α=1、NRIC G 和恢复的论文轨迹；`legacy` 为历史紧凑场景，其中 `--c-profile fr3` 使用 α=.25、G×.1，`--c-profile legacy` 使用旧硬接触和原增益。所有笔尖/接触参数均未经实物标定。原参数完整对照可用 `python scripts/validate_reproduction.py --profile paper --output outputs/paper-comparison`，再向报告脚本传入相同 `--root`。

历史 C 机制审计使用单独的固定矩阵，不覆盖旧目录。先安装系统 `poppler-utils` 以读取论文矢量图，再运行：

```bash
OPENBLAS_NUM_THREADS=1 python scripts/digitize_paper_c.py
OPENBLAS_NUM_THREADS=1 python scripts/digitize_paper_ab.py
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. python scripts/validate_c_mechanism.py
OPENBLAS_NUM_THREADS=1 python scripts/report_c_mechanism.py
```

同一代码版本再次运行验证脚本会核对配置并续用已完成记录。FR3 组实际控制、关节反馈和 F/T 均为 1 kHz；虚拟模型内部 RK4 子步和物理积分子步不增加实际反馈频率。4 kHz 诊断组单独标记，不能作为真实 FR3 设置。

历史 4 kHz 完整矩阵重跑与分析（独立输出目录）：

```bash
python -m fr3imp.suite --profile legacy --experiments ABCDEF --d-trials 30 --workers 4 --control-period .00025 --physics-substeps 1 --c-profile legacy --output outputs/legacy-suite
python -m fr3imp.analyze outputs/legacy-suite --output outputs/legacy-report
```

接触组的物理步长检查与结果核对：

```bash
python -m fr3imp.suite --profile legacy --experiments BCEF --physics-substeps 2 --control-period .00025 --c-profile legacy --workers 4 --output outputs/legacy-substeps2
python -m fr3imp.compare_steps outputs/legacy-suite outputs/legacy-substeps2 --output outputs/legacy-report
```

用 `--resume` 续跑已完成且代码哈希、参数一致的运行。在当前默认 1 kHz / 4 子步设置下，用 `--physics-substeps 8` 将物理积分步长减半，控制与传感器频率保持不变。`--ideal` 关闭本项目加入的模型误差、工具质量、摩擦、阻尼和传感器噪声，用于单独检查理想条件。

|组|场景|比较|默认时长/数量|
|---|---|---|---|
|A|自由空间 ∞ 轨迹|普通阻抗、导纳、H∞|30 s × 3|
|B|固定斜墙、有限工具接触柔顺性|上述三种 + 双向能量流、滑模|30 s × 5|
|C|白板、凸起障碍、10+3 sin(t) N 法向力|导纳、双向能量流、H∞|30 s × 3|
|D|滚转约 90° 前后的转动柔顺交互|普通阻抗、导纳、滑模、H∞|35 s × 4 × 30|
|E|砂轮机 / 磨削，用户要求暂缓|已有代码保留|不纳入当前任务|
|F|B 场景的 REF 增益比较|H∞ 相对基准 .25/.5/1；有效 α=.0625/.125/.25|30 s × 3|

当前总计 18 种设置、134 次主实验。D 的 30 次是 10 组虚拟操作者参数各重复 3 次，不能替代论文的真人实验。

## 结果与回放

每次运行在 `outputs/` 下保存 `config.json`、`summary.json` 和 `trajectory.npz`；批处理另存 `manifest.json`。原始数据包含真实/测量状态、接触力、F/T 测量、命令/施加力矩、参考误差、饱和、beta 和能量罐。

E 还保存八轴 `plant_q/plant_qvel`、转子电机力矩/功率与滑动摩擦力/功率；七轴 `q/qvel` 的原有接口保持一致。验证输出包含可运行源码快照 `source/`，用于后续代码修改后的同版本回放。

`fr3imp.analyze` 检查矩阵完整性、运行时长、采样间隔和有限值，独立复算位置 RMS 与峰值力，并生成所选实验图和 D 组统计图。失败或提前终止的运行保留在结果中。新报告按每次实际 Λ/D/K 独立积分完整阻抗方程，核对实际限幅和对比组任务一致性，提供 F 的均值/标准差与硬接触、滤波开关、种子和积分细化对照。

```bash
# 从已保存的关节轨迹生成真实 MuJoCo 画面；不是重新运行控制器。
MUJOCO_GL=egl python -m fr3imp.replay outputs/reproduction-fr3/main/C_hinf_trial00_seed100 \
  --time 15 --image outputs/C.png --video outputs/C.mp4 --speed 2
```

MP4 需要系统安装 `ffmpeg`；Linux 无显示器时可使用 EGL。`--speed 2` 表示两倍速回放。新图表见 [当前复现报告](docs/results/reproduction-fr3/REPORT.md)；原 137 次运行见 [历史 4 kHz 结果](docs/results/RESULTS.md)。历史日志回放需使用其对应源码快照或检出对应版本。

## 论文与实现依据

详细公式、七轴扩展、坐标约定、噪声参数和场景适配见 [实现说明](docs/IMPLEMENTATION.md)。

- 主论文：[DOI 10.1109/TRO.2026.3741518](https://doi.org/10.1109/TRO.2026.3741518)，用户提供的 PDF 保存在项目根目录。
- 双向能量流调制：[DOI 10.1109/TRO.2024.3443690](https://doi.org/10.1109/TRO.2024.3443690)，使用用户提供全文的式 (19)–(22)、(28)、(31)–(32)。
- 模型说明与原许可证：[franka_fr3/README.md](franka_fr3/README.md)、[Apache-2.0](franka_fr3/LICENSE)。参考论文版权归原权利人所有。

原始仿真输出、虚拟环境及缓存默认不进入 Git。源代码、测试、结果摘要和图可纳入版本管理。
