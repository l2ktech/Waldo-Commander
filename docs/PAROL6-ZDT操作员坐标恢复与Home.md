# PAROL6 ZDT 操作员坐标恢复与 Home

这份说明适用于 5800X 上的 Waldo Commander 8011 页面。它与 01 硬件 owner 的
[`docs/20260807-0x31多圈恢复与Home交接规范.md`](https://github.com/l2ktech/01-Parol6/blob/main/docs/20260807-0x31%E5%A4%9A%E5%9C%88%E6%81%A2%E5%A4%8D%E4%B8%8EHome%E4%BA%A4%E6%8E%A5%E8%A7%84%E8%8C%83.md)
和 03 的完整跨系统规范
[`docs/14-断电后编码器归零与Home操作规范.md`](https://github.com/l2ktech/03-Parol6-Ros2/blob/main/docs/14-%E6%96%AD%E7%94%B5%E5%90%8E%E7%BC%96%E7%A0%81%E5%99%A8%E5%BD%92%E9%9B%B6%E4%B8%8EHome%E6%93%8D%E4%BD%9C%E8%A7%84%E8%8C%83.md)
保持一致。

## 断电后的默认操作

断电、驱动/worker 重启或失能后人工挪动时：

1. 现场有人值守，把实体机械臂手动放到校准 Home 附近：`[0,-95,106,0,-42,90]°`。
2. 在 8011 点击“**不运动坐标重建**”。此按钮只请求 01 读取当前 `0x31/0x36` 并重建
   多圈坐标，不发送机械运动。
3. 等页面短暂重连完成，再确认六轴数字、零速和无 fault。
4. 只有需要实体回到 Home 时，才点击 Home 并确认主动六轴 MoveJ。

不要用碰撞式 HOME 猜圈数，也不要在页面、ROS 或 Waldo 中手工改 `0x31` datum、`0x36`
零点、`/joint_states` 或模型 URDF。

## 编码器规则

- `0x31`：16 位单圈编码器，提供当前圈内精确角度；原有 datum 不变。
- `0x36`：电机多圈位置，只用于选择最接近 Home hint 且落在软限位内的整数圈数。
- 只重建电机跨度大于一圈的 J1/J2/J3/J5/J6；J4 是单圈轴，整条轴卡不修改。
- 当前已签收圈数：`J1=0 / J2=-11 / J3=-7 / J5=0 / J6=+4`。
- 典型恢复角度：`[-1.2909, -90.6146, 108.1343, J4保持, -38.8884, 93.3846]°`。

圈数是当前 Home 附近姿态经过 `0x31` 精确读数和 `0x36` 多圈展开得到的结果，不是操作员
以后手工固定填写的 offset。遇到半圈歧义或超出软限位，系统必须拒绝恢复并交给 01 owner。

## Home 按钮目标

Home 是 canonical 坐标下的主动 MoveJ，精确目标为：

```text
[0.0102997, -95.3194387, 106.0178375, -0.0878906, -41.8739319, 90.0102016]°
```

Home 不是无运动恢复，也不是驱动器碰撞回零。点击后等待页面明确完成，不重复点击其它
动作；完成后由 01 terminal telemetry 确认六轴零速、无 fault、无 grant/lease。

## 3D 模型偏置

数字和运动命令使用 canonical 坐标；模型渲染链单独使用：

```text
[0,-20,0,0,0,0]°  # J2 only, display-only
```

J2 的硬件 canonical 总坐标迁移是 `+50°`，其中最后 `+20°` 没有改变 URDF 机械零位，
所以 3D 模型需要减回 `20°`。这个偏置不会进入 Home 目标、MoveJ、ROS 状态或 01 axis card。
Waldo 通过 `WALDO_URDF_ANGLE_OFFSETS_PATH` 读取由 03 同步器生成的
`/var/lib/parol6-zdt/waldo/urdf-angle-offsets.json`；不要手改另一份 JSON。

## 验收 proof

无运动恢复的 proof 在 5800X：

```text
/var/lib/parol6-zdt/proofs/20260807T235044Z-home-0x31-multiturn-restore/
```

本次结果：无机械运动、六轴 fresh/零速/无 fault、grant/lease 清空、CAN `ERROR-ACTIVE`
且 `tx=0/rx=0`。J4 未修改，J6 仅有约 2 count 静态采样抖动。
