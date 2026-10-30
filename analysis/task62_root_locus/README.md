# 任务 6.2 根轨迹分析代码

本目录沿用任务 6.1 的辨识对象，完成 PI 零点配置、标准根轨迹、每类 3 组根轨迹增益、参数换算、闭环极点、稳定裕度、阶跃响应和动态指标对比。

## 参数关系

辨识对象：

`G(s)=b0/(s^2+a1*s+a0)`

PI 控制器：

`C(s)=Kp+Ki/s=Kp(s-z0)/s`

根轨迹标准形式：

`L0(s)=(s-z0)/(s(s^2+a1*s+a0))`

参数换算：

- `z0=-Ki/Kp`
- `KR=b0*Kp`
- `Kp=KR/b0`
- `Ki=-z0*Kp`

## 默认零点配置

|配置|零点|Kp 组数|用途|
|---|---:|---:|---|
|Z5_right|-5|3|机械主导极点右侧方案|
|Z80_left|-80|3|机械主导极点左侧方案|
|Z1200_electrical|-1200|3|电气极点附近补充方案|

可在 `task62_config.m` 中修改零点、Kp 或将某组的 `Enabled` 设置为 `false`。

## 运行

```matlab
cd("C:\Users\yanya\Desktop\控制原理\ESP32-BLDC-Motor-Control-with-SimpleFOC\analysis\task62_root_locus")
results = run_task62_root_locus;
```

运行后检查 `output` 目录：

- 每类零点一张根轨迹图。
- 每类零点一张三组参数阶跃响应图。
- 每类零点一张控制电压图。
- `task62_parameter_and_metrics.csv` 参数、闭环极点、裕度和动态指标。
- `task62_summary.md` 可直接用于整理报告。
- `task62_simulation_data.csv` 完整时序数据。
