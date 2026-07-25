# Waldo Commander 交付健康度

当前得分：**98/100**

| 维度 | 得分 | 结果 |
|---|---:|---|
| 急停与 Reset | 20/20 | 真机零运动闭环 `estop=1 → SAFE_TERMINAL/STOPPED → reset=1 → NO_MOTION/STOPPED` |
| Worker 与 CAN 运行态 | 19/20 | 六轴 fresh、enabled、零速、无故障；未执行本轮运动矩阵 |
| UI 状态与输入 | 20/20 | 合法关节值、按钮 fail-closed、后端显示和探针锁页均已修复 |
| 浏览器与响应式 | 15/15 | 快捷键、文件名及 1600×900 至 1024×768 五档拖动缩放全部通过 |
| 日志与诊断 | 14/15 | hull 缓存可写、状态流瞬时超时自动重连；保留第三方启动 warning |
| 定向回归与证据 | 10/10 | 44 项定向检查通过，真实 Chrome 流通过并留图 |
| 交付同步 | 0/0 | 本机部署纳入最终步骤；上游推送权限单独记录，不扣运行健康分 |

## 已关闭问题

- Base、Shoulder、Elbow、Wrist 1 合法值 `invalid=true`
- 安全态与关节/笛卡尔按钮状态不同步
- workspace hull 日志重复
- 旧 `select_tool` UID 拒绝与无意义启动写入
- Settings 后端名称与实际进程不一致
- 打开 `precision.py` 自动仿真及 `None result` 误报
- 急停或 Reset 未确认却被 UI 当作成功
- `curl /` 健康探针被误认为主浏览器并锁住后续 Chrome
- 状态流首次 IPC 超时后永久退出
- Simulator 在 systemd 只读 Home 下无法保存 workspace hull

## 已复核为自动化操作问题

- 页面主体聚焦时 `[`/`]` 的真实 Chrome 事件链可正常调速。
- 文件名追加来自 DevTools MCP 未先清空；人工式全选、清空、输入可以精确替换。

## 新增问题复现与修复

- 主浏览器锁定：重启服务后先执行 `curl http://127.0.0.1:8012/`，再打开 Chrome；旧版显示 `Locked to the primary 5800X Chrome window`。修复后 curl 不占控制槽，Chrome 可直接进入。
- hull 缓存失败：打开 Simulator 后旧版日志出现 `Read-only file system: ~/.waldo-commander/workspace_hull.stl`。现通过 `WALDO_CACHE_DIR` 写入服务允许的 `.cache/waldo-commander/`。
- 状态流超时：旧版首次 `IPC request timed out after 2000 ms` 后消费者退出。现 0.25 秒后自动重订阅，最终重启日志无 ERROR/Traceback。

`health.healthy=false` 是当前 `NO_MOTION/STOPPED` 安全终态下 `motion_ready=false` 的组合结果，不代表 CAN 断线或关节故障；六轴均 fresh、enabled、零速、无故障，grant/lease 为空。

## 证据

- [快捷键截图](screenshots/keyboard-speed-body-focus.png)
- [文件名替换截图](screenshots/filename-manual-replace.png)
- [1024×768 面板截图](screenshots/panel-responsive-1024x768.png)
- [最终 Simulator Settings 截图](screenshots/final-simulator-settings.png)
- 机器可读基线：[baseline.json](baseline.json)
