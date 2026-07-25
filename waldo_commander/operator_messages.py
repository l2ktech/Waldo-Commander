"""Chinese operator-facing error messages with an actionable next step."""

from __future__ import annotations


def operator_error(action: str, error: BaseException | str) -> str:
    """Return a stable Chinese message without leaking backend internals."""
    normalized = str(error).lower()

    if "capability_not_authorized" in normalized:
        reason = "目标超出本次授权的运动范围"
        solution = "请减小步长或使用分段移动；若按钮仍可点击，请刷新页面后重试"
    elif "overlapping official motion" in normalized:
        reason = "上一动作尚未完成"
        solution = "请等待按钮恢复后再操作，不要连续重复点击"
    elif "fakecan-only" in normalized:
        reason = "当前 SocketCAN 真机后端不支持页面仿真模式"
        solution = "请保持真机模式；如需仿真，请启动独立的 FakeCAN 仿真实例"
    elif "terminal target error" in normalized:
        reason = "机械臂已安全停止，但实际位置与目标存在偏差"
        solution = "系统不会自动纠偏或锁定控制；可继续操作，或手动再次移动到目标"
    elif "stable terminal encoder sampling deadline expired" in normalized:
        reason = "机械臂已停止，但终点编码器稳定采样未在规定时间内完成"
        solution = "请点击页面上的“恢复控制”，再降低速度或步长后重试"
    elif "selected-axes speed must be at least" in normalized:
        reason = "当前速度低于驱动器可执行的最小速度"
        solution = "请将页面速度提高一档后重试"
    elif "stop_already_pending" in normalized or "stop is already pending" in normalized:
        reason = "上一条停止命令仍在收尾"
        solution = "请点击“恢复控制”，页面会等待停止完成并自动复位"
    elif "current grant and lease" in normalized:
        reason = "内部控制授权未完整建立"
        solution = "请点击“恢复控制”，页面会自动停止、回收旧授权并重新开放操作"
    elif "lease" in normalized:
        reason = "页面控制授权已失效"
        solution = "请点击“恢复控制”自动回收旧授权；无需重启或修改代码"
    elif "soft limit" in normalized or "signed maximum delta" in normalized:
        reason = "目标接近软限位或单次位移过大"
        solution = "请反向移动、减小步长或使用分段移动"
    elif "command_timeout" in normalized or "timed out" in normalized:
        reason = "动作等待超时，系统已执行停止"
        solution = "请降低速度或步长，并确认 can0 与六轴状态正常后重试"
    elif "python-can send failed" in normalized or "can bus" in normalized:
        reason = "CAN 通信暂时不可用"
        solution = "请检查 can0、USB 适配器和机械臂供电，等待状态恢复后重试"
    elif "hardware connection" in normalized or "not connected" in normalized:
        reason = "机械臂硬件尚未连接"
        solution = "请确认 worker、can0 和适配器在线，或切换到仿真模式"
    elif "fault reset" in normalized or "operator stop is not confirmed" in normalized:
        reason = "停止状态尚未满足复位条件"
        solution = "请再次点击“恢复控制”；页面会先执行停止确认，再自动复位"
    elif "home assistant software estop" in normalized:
        reason = "Home Assistant 软件停止仍处于开启状态"
        solution = "请先在 Home Assistant 关闭软件停止，再执行复位"
    elif "no gripper" in normalized or (
        "tool" in normalized and "not" in normalized
    ):
        reason = "当前工具不可用"
        solution = "请在设置中选择已连接的工具，并确认工具状态在线"
    else:
        reason = "操作未完成，系统已保留诊断信息"
        solution = "请先确认页面状态正常后重试；若再次失败，请查看服务日志或联系维护人员"

    return f"{action}失败：{reason}。处理方法：{solution}。"
