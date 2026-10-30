# 任务 6.2 根轨迹与参数对比结果

生成时间：2026-10-29 21:16:08

## 对象模型

辨识模型为 `G(s)=355073.588/(s^2+227.372596 s+18592.7609)`。

PI 控制器为 `C(s)=Kp+Ki/s=Kp(s-z0)/s`，参数关系为 `Ki=-z0*Kp`、`KR=355073.588*Kp`。

参考速度为 6.28 rad/s，Uq 限幅为 ±3 V。

## Z5_right

零点位于主导极点实部右侧，`z0=-5`。

- 根轨迹图：`C:\Users\yanya\Desktop\控制原理\ESP32-BLDC-Motor-Control-with-SimpleFOC\analysis\task62_root_locus\output\task62_root_locus_Z5_right.png`
- 阶跃曲线：`C:\Users\yanya\Desktop\控制原理\ESP32-BLDC-Motor-Control-with-SimpleFOC\analysis\task62_root_locus\output\task62_step_Z5_right.png`
- 控制电压：`C:\Users\yanya\Desktop\控制原理\ESP32-BLDC-Motor-Control-with-SimpleFOC\analysis\task62_root_locus\output\task62_voltage_Z5_right.png`

|组别|KR|Kp|Ki|主导极点|上升时间/s|调节时间/s|超调/%|相位裕度/°|
|---|---:|---:|---:|---|---:|---:|---:|---:|
|Z5_right-1|10652.2|0.03|0.15|-1.84753|0.999|1.876|0|122.497|
|Z5_right-2|21304.4|0.06|0.3|-2.71131|0.563|1.161|0|123.941|
|Z5_right-3|42608.8|0.12|0.6|-3.52651|0.307|0.767|0|70.3738|

## Z80_left

零点位于主导极点实部左侧，`z0=-80`。

- 根轨迹图：`C:\Users\yanya\Desktop\控制原理\ESP32-BLDC-Motor-Control-with-SimpleFOC\analysis\task62_root_locus\output\task62_root_locus_Z80_left.png`
- 阶跃曲线：`C:\Users\yanya\Desktop\控制原理\ESP32-BLDC-Motor-Control-with-SimpleFOC\analysis\task62_root_locus\output\task62_step_Z80_left.png`
- 控制电压：`C:\Users\yanya\Desktop\控制原理\ESP32-BLDC-Motor-Control-with-SimpleFOC\analysis\task62_root_locus\output\task62_voltage_Z80_left.png`

|组别|KR|Kp|Ki|主导极点|上升时间/s|调节时间/s|超调/%|相位裕度/°|
|---|---:|---:|---:|---|---:|---:|---:|---:|
|Z80_left-1|7101.47|0.02|1.6|-28.3267|0.069|0.133|0|89.2872|
|Z80_left-2|14202.9|0.04|3.2|-46.6284|0.024|0.071|0|81.2239|
|Z80_left-3|28405.9|0.08|6.4|-61.8104|0.01|0.047|7.77376|57.5279|

## Z1200_electrical

电气极点附近的补充讨论方案，`z0=-1200`。

- 根轨迹图：`C:\Users\yanya\Desktop\控制原理\ESP32-BLDC-Motor-Control-with-SimpleFOC\analysis\task62_root_locus\output\task62_root_locus_Z1200_electrical.png`
- 阶跃曲线：`C:\Users\yanya\Desktop\控制原理\ESP32-BLDC-Motor-Control-with-SimpleFOC\analysis\task62_root_locus\output\task62_step_Z1200_electrical.png`
- 控制电压：`C:\Users\yanya\Desktop\控制原理\ESP32-BLDC-Motor-Control-with-SimpleFOC\analysis\task62_root_locus\output\task62_voltage_Z1200_electrical.png`

|组别|KR|Kp|Ki|主导极点|上升时间/s|调节时间/s|超调/%|相位裕度/°|
|---|---:|---:|---:|---|---:|---:|---:|---:|
|Z1200_electrical-1|355.074|0.001|1.2|-34.7877|0.069|0.129|0|75.1692|
|Z1200_electrical-2|710.147|0.002|2.4|-49.228+j64.7062|0.028|0.081|6.83474|61.1781|
|Z1200_electrical-3|1420.29|0.004|4.8|-29.372+j96.1479|0.015|0.117|31.8024|38.5874|

## 使用说明

正式报告应从每类至少选择 3 组参数，保留根轨迹、全部闭环极点、阶跃响应和超调/调节时间。若老师只要求两类，可提交 Z5 与 Z80，并把 Z1200 作为电气极点附近的补充讨论。
