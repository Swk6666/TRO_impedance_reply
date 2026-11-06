# FR3 参数适配审计（不含 E）

五个诊断批次共 133 次，状态 {'completed': 124, 'terminated': 9}。全部结果均保留，包含速度越界和任务参数不适用的情况。本文汇总的是选定 `fr3` 配置前的诊断，不替代 [最终 134+51 次验证](../reproduction-fr3/REPORT.md)。

## 从原参数到明确适配

1. `paper` 的 A/F 已有主要趋势，但 B 普通导纳未失稳，C 抖振主要局限在挡块，D 的若干差异不明显。机械臂惯量、减速传动、摩擦、力矩限制与采样不同，原参数不能直接保证相同闭环表现。
2. 全轴降低虚拟惯量使 B 出现接触抖振差异，却使 C 的 H∞ 限幅、D 的强握持过载。没有将这种统一缩放作为默认规则。
3. C 仅缩放 x/y 运动惯量和阻尼，就能在平板阶段出现导纳抖振，保留 z 力控制阻抗和角轴参数；这是按轴隔离机制的证据。
4. D 只降低增益、只降低惯量或额外延迟一次 F/T 均未解决强握持问题。将旋转 Λ/D/K、yaw/pitch 交互力矩与相关反馈尺度一起调整后，H∞ 不再限幅。全量滚转引导恢复约 90° 翻转；因普通阻抗释放后仍漂移，所有控制器统一加入第二阶段被动滚转支撑。这一附加条件与论文的自由滚转释放不同，已明确披露。
5. 统一降低 NRIC G 的候选使 B/C 普通导纳变得稳定。因此最终 A/B/C 保留其原 G，仅 D 与旋转尺度一起调整。不能从最终有限配置得出导纳算法必然失稳或某算法总是最优。
6. F 在较高绝对增益时，REF 仍近似反比下降，但静态偏差接近仿真噪声/接触误差平台，不再严格单调。以 FR3 基准 α=.25 扫描相对倍率 .25/.5/1，才同时观察到 REF 和静态偏差下降；实际有效 α=.0625/.125/.25 在图表和日志中明示。

这些是公开保留选择过程的仿真适配，未根据论文图像改写力或关节日志。候选参数经过主矩阵和新的种子/步长检查；没有把所有探索性配置算作成功复现。

## 批次与源码

|批次|数量|状态|运行时代码 SHA256|
|---|---:|---|---|
|[portability](../../../outputs/portability/manifest.json)|48|{'completed': 42, 'terminated': 6}|`b71d9601c33cf08de5f9de070f1706812bda542e77fb033570d89134b2b54e57`|
|[axis-adaptation](../../../outputs/axis-adaptation/manifest.json)|47|{'completed': 44, 'terminated': 3}|`b71d9601c33cf08de5f9de070f1706812bda542e77fb033570d89134b2b54e57`|
|[torque-scaling](../../../outputs/torque-scaling/manifest.json)|16|{'completed': 16}|`eb2d9d8891244cd531ec290387d3c923d6d0e37104263cc5492be2344a75a390`|
|[torque-scaling-roll](../../../outputs/torque-scaling-roll/manifest.json)|4|{'completed': 4}|`065ff28daf83676e90e946a3118650c4942b73ad060e32477833fc4fc9cdedcf`|
|[fr3-candidate](../../../outputs/fr3-candidate/manifest.json)|18|{'completed': 18}|`3bd0dd5e723c7c384c4df529fb49e40beb28cf7d0aa7bcd3a16098a87d9ef551`|

每批 `outputs/<批次>/source/` 保留对应源码；其配置不可与后续默认参数混用。`metrics.json` 保存下列各次完整摘要。原始 NPZ 仍位于对应 outputs 目录。

## 完整运行清单

列出的误差来自各自实际运行时长；提前终止与完整 30/35 s 运行不可直接等同。D 改变了旋转任务，因此跨不同旋转刚度批次的静态误差也不可直接排名。

|批次 / 案例|状态 / 时长 s|C 力 RMS / N|动态位置偏差 / mm|限幅比例|终止原因|
|---|---|---:|---:|---:|---|
|axis-adaptation/alpha0.25_C_hinf|completed / 30.000|0.477|1.223|0.000%|—|
|axis-adaptation/alpha0.25_D_hinf|completed / 35.000|—|0.140|0.031%|—|
|axis-adaptation/alpha0.5_C_hinf|completed / 30.000|0.489|0.623|0.000%|—|
|axis-adaptation/alpha0.5_D_hinf|completed / 35.000|—|0.109|0.060%|—|
|axis-adaptation/grasp4_alpha0.25_D_hinf|terminated / 9.515|—|18.298|14.912%|Joint speed exceeds numerical experiment limit (15 rad/s)|
|axis-adaptation/grasp4_alpha0.5_D_hinf|terminated / 9.301|—|5.029|13.105%|Joint speed exceeds numerical experiment limit (15 rad/s)|
|axis-adaptation/grasp4_delay1_D_admittance|completed / 35.000|—|2.884|9.983%|—|
|axis-adaptation/grasp4_delay1_D_hinf|terminated / 9.514|—|18.195|14.903%|Joint speed exceeds numerical experiment limit (15 rad/s)|
|axis-adaptation/grasp4_delay1_D_impedance|completed / 35.000|—|2.894|15.106%|—|
|axis-adaptation/grasp4_delay1_D_smc|completed / 35.000|—|3.393|9.283%|—|
|axis-adaptation/low_inertia_F_hinf_0.25|completed / 30.000|—|0.175|0.000%|—|
|axis-adaptation/low_inertia_F_hinf_0.5|completed / 30.000|—|0.127|0.000%|—|
|axis-adaptation/low_inertia_F_hinf_1|completed / 30.000|—|0.100|0.000%|—|
|axis-adaptation/low_inertia_physics8_B_admittance|completed / 30.000|—|0.132|7.277%|—|
|axis-adaptation/low_inertia_physics8_B_bidirectional|completed / 30.000|—|6.488|0.000%|—|
|axis-adaptation/low_inertia_physics8_B_hinf|completed / 30.000|—|0.100|0.000%|—|
|axis-adaptation/low_inertia_physics8_B_impedance|completed / 30.000|—|6.804|0.000%|—|
|axis-adaptation/low_inertia_physics8_B_smc|completed / 30.000|—|0.447|0.000%|—|
|axis-adaptation/low_inertia_physics8_F_hinf_0.25|completed / 30.000|—|0.175|0.000%|—|
|axis-adaptation/low_inertia_physics8_F_hinf_0.5|completed / 30.000|—|0.128|0.000%|—|
|axis-adaptation/low_inertia_physics8_F_hinf_1|completed / 30.000|—|0.100|0.000%|—|
|axis-adaptation/low_inertia_seed101_B_admittance|completed / 30.000|—|0.145|7.173%|—|
|axis-adaptation/low_inertia_seed101_B_bidirectional|completed / 30.000|—|6.434|0.000%|—|
|axis-adaptation/low_inertia_seed101_B_hinf|completed / 30.000|—|0.098|0.000%|—|
|axis-adaptation/low_inertia_seed101_B_impedance|completed / 30.000|—|6.812|0.000%|—|
|axis-adaptation/low_inertia_seed101_B_smc|completed / 30.000|—|0.446|0.000%|—|
|axis-adaptation/low_inertia_seed102_B_admittance|completed / 30.000|—|0.148|7.403%|—|
|axis-adaptation/low_inertia_seed102_B_bidirectional|completed / 30.000|—|6.505|0.000%|—|
|axis-adaptation/low_inertia_seed102_B_hinf|completed / 30.000|—|0.099|0.000%|—|
|axis-adaptation/low_inertia_seed102_B_impedance|completed / 30.000|—|6.810|0.000%|—|
|axis-adaptation/low_inertia_seed102_B_smc|completed / 30.000|—|0.446|0.000%|—|
|axis-adaptation/nric_gain_point1_C_admittance|completed / 30.000|0.412|0.501|0.000%|—|
|axis-adaptation/nric_gain_point1_C_bidirectional|completed / 30.000|1.766|60.980|0.000%|—|
|axis-adaptation/rotation_eighth_D_admittance|completed / 35.000|—|0.680|1.980%|—|
|axis-adaptation/rotation_eighth_D_hinf|completed / 35.000|—|0.724|1.723%|—|
|axis-adaptation/rotation_eighth_D_impedance|completed / 35.000|—|2.929|1.203%|—|
|axis-adaptation/rotation_eighth_D_smc|completed / 35.000|—|0.409|1.157%|—|
|axis-adaptation/rotation_quarter_D_admittance|completed / 35.000|—|0.058|0.534%|—|
|axis-adaptation/rotation_quarter_D_hinf|completed / 35.000|—|0.078|0.477%|—|
|axis-adaptation/rotation_quarter_D_impedance|completed / 35.000|—|2.844|0.214%|—|
|axis-adaptation/rotation_quarter_D_smc|completed / 35.000|—|0.127|0.297%|—|
|axis-adaptation/xy_eighth_C_admittance|completed / 30.000|18.323|4.551|88.217%|—|
|axis-adaptation/xy_eighth_C_bidirectional|completed / 30.000|1.748|63.206|0.000%|—|
|axis-adaptation/xy_eighth_C_hinf|completed / 30.000|1.003|0.289|0.000%|—|
|axis-adaptation/xy_quarter_C_admittance|completed / 30.000|8.894|1.880|28.743%|—|
|axis-adaptation/xy_quarter_C_bidirectional|completed / 30.000|1.743|62.607|0.000%|—|
|axis-adaptation/xy_quarter_C_hinf|completed / 30.000|0.898|0.289|0.000%|—|
|fr3-candidate/A_admittance|completed / 30.000|—|0.301|0.000%|—|
|fr3-candidate/A_hinf|completed / 30.000|—|0.421|0.000%|—|
|fr3-candidate/A_impedance|completed / 30.000|—|21.657|0.000%|—|
|fr3-candidate/B_admittance|completed / 30.000|—|0.249|0.000%|—|
|fr3-candidate/B_bidirectional|completed / 30.000|—|5.304|0.000%|—|
|fr3-candidate/B_hinf|completed / 30.000|—|0.175|0.000%|—|
|fr3-candidate/B_impedance|completed / 30.000|—|6.809|0.000%|—|
|fr3-candidate/B_smc|completed / 30.000|—|0.447|0.000%|—|
|fr3-candidate/C_admittance|completed / 30.000|0.491|0.212|0.000%|—|
|fr3-candidate/C_bidirectional|completed / 30.000|1.887|71.905|0.000%|—|
|fr3-candidate/C_hinf|completed / 30.000|0.625|1.154|0.000%|—|
|fr3-candidate/D_admittance|completed / 35.000|—|2.002|1.094%|—|
|fr3-candidate/D_hinf|completed / 35.000|—|0.145|0.000%|—|
|fr3-candidate/D_impedance|completed / 35.000|—|3.492|0.000%|—|
|fr3-candidate/D_smc|completed / 35.000|—|0.142|0.000%|—|
|fr3-candidate/F_hinf_0.25|completed / 30.000|—|0.396|0.000%|—|
|fr3-candidate/F_hinf_0.5|completed / 30.000|—|0.259|0.000%|—|
|fr3-candidate/F_hinf_1|completed / 30.000|—|0.175|0.000%|—|
|portability/grasp4_D_admittance|completed / 35.000|—|2.872|10.306%|—|
|portability/grasp4_D_hinf|terminated / 9.299|—|4.895|13.097%|Joint speed exceeds numerical experiment limit (15 rad/s)|
|portability/grasp4_D_impedance|completed / 35.000|—|2.888|15.066%|—|
|portability/grasp4_D_smc|completed / 35.000|—|3.364|8.606%|—|
|portability/grasp8_D_admittance|terminated / 22.508|—|3.368|5.771%|Task Jacobian near singularity: 0.014718|
|portability/grasp8_D_hinf|terminated / 9.281|—|5.991|13.004%|Joint speed exceeds numerical experiment limit (15 rad/s)|
|portability/grasp8_D_impedance|completed / 35.000|—|3.167|15.163%|—|
|portability/grasp8_D_smc|completed / 35.000|—|4.292|9.611%|—|
|portability/inertia_eighth_B_admittance|completed / 30.000|—|0.133|7.213%|—|
|portability/inertia_eighth_B_bidirectional|completed / 30.000|—|6.456|0.000%|—|
|portability/inertia_eighth_B_hinf|completed / 30.000|—|0.100|0.000%|—|
|portability/inertia_eighth_B_impedance|completed / 30.000|—|6.809|0.000%|—|
|portability/inertia_eighth_B_smc|completed / 30.000|—|0.447|0.000%|—|
|portability/inertia_eighth_C_admittance|completed / 30.000|14.493|17.413|88.300%|—|
|portability/inertia_eighth_C_bidirectional|completed / 30.000|1.758|182.724|0.000%|—|
|portability/inertia_eighth_C_hinf|completed / 30.000|14.281|14.567|68.530%|—|
|portability/inertia_eighth_D_admittance|completed / 35.000|—|0.749|18.111%|—|
|portability/inertia_eighth_D_hinf|completed / 35.000|—|0.508|1.866%|—|
|portability/inertia_eighth_D_impedance|completed / 35.000|—|0.520|1.234%|—|
|portability/inertia_eighth_D_smc|completed / 35.000|—|0.420|1.280%|—|
|portability/inertia_eighth_grasp4_D_admittance|completed / 35.000|—|4.332|29.966%|—|
|portability/inertia_eighth_grasp4_D_hinf|terminated / 8.684|—|25.947|9.234%|Joint speed exceeds numerical experiment limit (15 rad/s)|
|portability/inertia_eighth_grasp4_D_impedance|terminated / 22.240|—|2.794|7.621%|Task Jacobian near singularity: 0.014694|
|portability/inertia_eighth_grasp4_D_smc|terminated / 22.294|—|5.196|9.195%|Joint speed exceeds numerical experiment limit (15 rad/s)|
|portability/inertia_quarter_B_admittance|completed / 30.000|—|0.092|0.000%|—|
|portability/inertia_quarter_B_bidirectional|completed / 30.000|—|7.752|0.000%|—|
|portability/inertia_quarter_B_hinf|completed / 30.000|—|0.112|0.000%|—|
|portability/inertia_quarter_B_impedance|completed / 30.000|—|13.345|0.000%|—|
|portability/inertia_quarter_B_smc|completed / 30.000|—|0.479|0.000%|—|
|portability/inertia_quarter_C_admittance|completed / 30.000|10.514|7.646|53.013%|—|
|portability/inertia_quarter_C_bidirectional|completed / 30.000|1.755|127.891|0.000%|—|
|portability/inertia_quarter_C_hinf|completed / 30.000|10.294|2.807|16.837%|—|
|portability/inertia_quarter_D_admittance|completed / 35.000|—|0.070|0.526%|—|
|portability/inertia_quarter_D_hinf|completed / 35.000|—|0.067|0.469%|—|
|portability/inertia_quarter_D_impedance|completed / 35.000|—|0.711|0.217%|—|
|portability/inertia_quarter_D_smc|completed / 35.000|—|0.129|0.300%|—|
|portability/literal_B_admittance|completed / 30.000|—|0.081|0.017%|—|
|portability/literal_B_bidirectional|completed / 30.000|—|12.680|0.000%|—|
|portability/literal_B_hinf|completed / 30.000|—|0.246|0.023%|—|
|portability/literal_B_impedance|completed / 30.000|—|39.884|0.000%|—|
|portability/literal_B_smc|completed / 30.000|—|0.440|0.000%|—|
|portability/literal_C_admittance|completed / 30.000|2.305|0.117|6.250%|—|
|portability/literal_C_bidirectional|completed / 30.000|1.730|61.248|0.000%|—|
|portability/literal_C_hinf|completed / 30.000|0.765|0.310|0.000%|—|
|portability/literal_D_admittance|completed / 35.000|—|0.035|0.091%|—|
|portability/literal_D_hinf|completed / 35.000|—|0.075|0.086%|—|
|portability/literal_D_impedance|completed / 35.000|—|2.189|11.994%|—|
|portability/literal_D_smc|completed / 35.000|—|0.203|0.000%|—|
|torque-scaling-roll/admittance|completed / 35.000|—|1.978|1.046%|—|
|torque-scaling-roll/hinf|completed / 35.000|—|0.145|0.000%|—|
|torque-scaling-roll/impedance|completed / 35.000|—|2.814|0.000%|—|
|torque-scaling-roll/smc|completed / 35.000|—|0.142|0.000%|—|
|torque-scaling/torque0.125_grasp1.0_admittance|completed / 35.000|—|0.231|0.000%|—|
|torque-scaling/torque0.125_grasp1.0_hinf|completed / 35.000|—|0.174|0.000%|—|
|torque-scaling/torque0.125_grasp1.0_impedance|completed / 35.000|—|2.204|0.000%|—|
|torque-scaling/torque0.125_grasp1.0_smc|completed / 35.000|—|0.095|0.000%|—|
|torque-scaling/torque0.125_grasp4.0_admittance|completed / 35.000|—|2.026|1.026%|—|
|torque-scaling/torque0.125_grasp4.0_hinf|completed / 35.000|—|0.186|0.000%|—|
|torque-scaling/torque0.125_grasp4.0_impedance|completed / 35.000|—|2.212|0.000%|—|
|torque-scaling/torque0.125_grasp4.0_smc|completed / 35.000|—|0.102|0.000%|—|
|torque-scaling/torque0.25_grasp1.0_admittance|completed / 35.000|—|0.115|0.000%|—|
|torque-scaling/torque0.25_grasp1.0_hinf|completed / 35.000|—|0.142|0.000%|—|
|torque-scaling/torque0.25_grasp1.0_impedance|completed / 35.000|—|2.204|0.000%|—|
|torque-scaling/torque0.25_grasp1.0_smc|completed / 35.000|—|0.130|0.000%|—|
|torque-scaling/torque0.25_grasp4.0_admittance|completed / 35.000|—|1.978|1.046%|—|
|torque-scaling/torque0.25_grasp4.0_hinf|completed / 35.000|—|0.145|0.000%|—|
|torque-scaling/torque0.25_grasp4.0_impedance|completed / 35.000|—|2.213|0.000%|—|
|torque-scaling/torque0.25_grasp4.0_smc|completed / 35.000|—|0.141|0.000%|—|