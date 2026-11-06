# TRO_impedance_reply

用于复现论文 **Nonlinear H∞ Inverse-Optimal Impedance Control for Robotic Systems** 的研究项目。

## 当前状态

已整理参考论文和 Franka Research 3（FR3）机械臂的 MuJoCo 模型。
论文控制器、仿真实验和结果对比尚未实现。

## 项目内容

```text
.
├── Nonlinear_mathcal_H_infty__Inverse-Optimal_Impedance_Control_for_Robotic_Systems.pdf
├── franka_fr3/
│   ├── assets/       # 机械臂网格资源
│   ├── fr3.xml       # FR3 机器人模型
│   ├── scene.xml     # 包含机器人与地面的仿真场景
│   ├── README.md     # 模型说明与来源
│   ├── CHANGELOG.md  # 模型变更记录
│   ├── LICENSE       # 模型原有许可证
│   └── fr3.png       # 模型预览
├── .gitattributes
└── .gitignore
```

## 模型说明

根据模型自带的说明，需使用 **MuJoCo 3.1.3 或更新版本**。
场景入口为 `franka_fr3/scene.xml`，机器人模型为 `franka_fr3/fr3.xml`。
当前模型使用位置控制执行器；后续实现论文控制器时需要确认所需的执行器接口。

模型来源、转换过程和使用说明见 [franka_fr3/README.md](franka_fr3/README.md)。
原模型采用 [Apache-2.0 许可证](franka_fr3/LICENSE)，保留其原有许可文件。
参考论文的版权归原权利人所有。

## Git 使用

```bash
git status
git add <需要提交的文件>
git commit -m "描述本次修改"
git push
```

Python 虚拟环境、缓存、构建产物、本地环境变量文件和实验输出目录默认不纳入版本管理。
机器人模型使用的 `.obj` 和 `.stl` 网格资源纳入版本管理。
