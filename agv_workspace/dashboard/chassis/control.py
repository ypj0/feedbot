import math
import secrets


class PulseControl:
    DURATION = 0.5
    DURATIONS = (0.5, 5.0, 10.0)
    ACTIONS = {"forward": (0.05, 0.0), "backward": (-0.05, 0.0),
               "left": (0.0, 0.10), "right": (0.0, -0.10), "stop": (0.0, 0.0)}

    def __init__(self):
        self.epoch = secrets.token_hex(16)
        self.sequence = 0
        self.action = "stop"
        self.deadline = 0.0
        self.duration = self.DURATION
        self.error = ""

    def accept(self, payload, now, ready):
        if payload.get("epoch") != self.epoch:
            raise ValueError("控制会话已变更，请刷新页面")
        action = payload.get("action")
        sequence = payload.get("sequence")
        if action not in self.ACTIONS or type(sequence) is not int or sequence <= self.sequence:
            raise ValueError("无效或过期的控制请求")
        if action != "stop":
            duration = payload.get("duration_s", self.DURATION)
            if type(duration) not in (int, float) or duration not in self.DURATIONS:
                raise ValueError("测试时长仅支持 0.5、5、10 秒")
            issued = payload.get("issued_at")
            if not isinstance(issued, (int, float)) or not math.isfinite(issued) or not 0 <= now - issued <= 0.75:
                raise ValueError("页面状态已过期，未执行运动")
            if not ready:
                raise ValueError("驱动未就绪或底盘状态禁止运动")
            if self.action != "stop" and now < self.deadline:
                raise ValueError("上一段测试尚未结束；不会排队或延长运动")
            self.duration = duration
        self.sequence = sequence
        self.action = action
        self.deadline = now + self.duration if action != "stop" else now
        self.error = ""

    def velocity(self, now, ready):
        if self.action != "stop" and not ready:
            self.error = "反馈过期或底盘状态异常，已请求零速；须重新点击才可运行"
            self.action = "stop"
        if now >= self.deadline:
            self.action = "stop"
        return self.ACTIONS[self.action]

    def snapshot(self, now):
        return {"epoch": self.epoch, "sequence": self.sequence, "action": self.action,
                "remaining_s": max(0.0, self.deadline - now) if self.action != "stop" else 0.0,
                "error": self.error, "pulse_s": self.duration}
