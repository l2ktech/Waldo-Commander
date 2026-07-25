"""Chinese operator-facing error messages with an actionable next step."""

from __future__ import annotations


def operator_error(action: str, error: BaseException | str) -> str:
    """Return a stable Chinese message without leaking backend internals."""
    normalized = str(error).lower()

    if "capability_not_authorized" in normalized:
        reason = "目标超出本次授权的运动范围"
        solution = "请减小步长或使用分段移动；若按钮仍可点击，请刷新页面后重试"
    elif "terminal target error" in normalized:
        reason = "机械臂已停止，但终点误差超过验收范围"
        solution = "请降低速度或加速度后重试；若持续出现，请停止操作并检查机械负载"
    elif "current grant and lease" in normalized or "lease" in normalized:
        reason = "页面控制授权已失效"
        solution = "请等待上一动作结束；仍未恢复时刷新页面并重新接管"
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
        solution = "请先确认机械臂已停止、六轴零速且没有活动任务，然后再次复位"
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
