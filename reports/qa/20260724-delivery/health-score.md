# Waldo Commander 交付健康度

当前得分：**93/100**

| 维度 | 得分 | 结果 |
|---|---:|---|
| 急停与 Reset | 20/20 | 真机零运动闭环 `estop=1 → SAFE_TERMINAL/STOPPED → reset=1 → NO_MOTION/STOPPED` |
| Worker 与 CAN 运行态 | 19/20 | 六轴 fresh、enabled、零速、无故障；未执行本轮运动矩阵 |
| UI 状态与输入 | 18/20 | 合法关节值、按钮 fail-closed、后端显示已修复 |
| 浏览器与响应式 | 12/15 | 快捷键、文件名、1024×768 拖动缩放通过；其余四档已写测试未执行 |
| 日志与诊断 | 14/15 | hull 重复日志和取消仿真误报已修复 |
| 定向回归与证据 | 10/10 | 集成定向测试 37/37，三条真实浏览器流通过并留图 |
| 交付同步 | 0/0 | 本机部署纳入最终步骤；上游推送权限单独记录，不扣运行健康分 |

## 已关闭问题

- Base、Shoulder、Elbow、Wrist 1 合法值 `invalid=true`
- 安全态与关节/笛卡尔按钮状态不同步
- workspace hull 日志重复
- 旧 `select_tool` UID 拒绝与无意义启动写入
- Settings 后端名称与实际进程不一致
- 打开 `precision.py` 自动仿真及 `None result` 误报
- 急停或 Reset 未确认却被 UI 当作成功

## 已复核为自动化操作问题

- 页面主体聚焦时 `[`/`]` 的真实 Chrome 事件链可正常调速。
- 文件名追加来自 DevTools MCP 未先清空；人工式全选、清空、输入可以精确替换。

## 证据

- [快捷键截图](screenshots/keyboard-speed-body-focus.png)
- [文件名替换截图](screenshots/filename-manual-replace.png)
- [1024×768 面板截图](screenshots/panel-responsive-1024x768.png)
- 机器可读基线：[baseline.json](baseline.json)
