# 任务 6.1 闭环仿真代码

本目录使用以前作业采集并拟合的 `Uq → 电机机械角速度` 二阶模型，完成两组 PI 参数的闭环阶跃仿真、动态指标计算、曲线保存和 Simulink 模型生成。

## 文件

| 文件 | 作用 |
|---|---|
| `task61_config.m` | 设置辨识数据、参考转速、仿真时间、电压限幅和 PI 参数 |
| `load_task61_identified_model.m` | 从以前作业的拟合汇总 CSV 中选择有效模型并取中位数 |
| `run_task61_closed_loop.m` | 运行两组 PI、计算指标、保存曲线和 CSV |
| `build_task61_simulink_model.m` | 自动生成可截图和继续修改的 Simulink 闭环模型 |
| `output/` | 运行后生成的结果目录 |

## 建模口径

现有辨识程序的输入是 SimpleFOC 的 q 轴命令电压 `Uq`，输出是机械角速度 `omega`。因此：

- PI 输出直接作为 `Uq`，单位为 V。
- 默认输出限幅为 `±3 V`，与已有闭环速度测试程序一致。
- 不再重复乘 12 V 电源和 H 桥增益，否则会重复计算驱动增益。
- 默认参考速度为 `6.28 rad/s`，与现有安全测试命令 `T6.28` 一致。

## 快速运行

在 MATLAB 中执行：

```matlab
cd("C:\Users\yanya\Desktop\控制原理\ESP32-BLDC-Motor-Control-with-SimpleFOC\analysis\task61_closed_loop")
results = run_task61_closed_loop;
modelPath = build_task61_simulink_model;
open_system(modelPath)
```

修改参数时编辑 `task61_config.m`，然后重新运行。正式报告至少保留两组 PI 的速度曲线、控制电压曲线和 `task61_metrics.csv`。
