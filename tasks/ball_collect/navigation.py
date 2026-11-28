"""按墙标坐标管理巡线、对准、推送和退回。速度使用车体坐标。"""
from math import atan2, cos, dist, hypot, pi, sin
from time import monotonic

from tasks.ball_detect.ground_position import BALL_RADIUS_M


STOP = (0.0, 0.0, 0.0)
MATCH_DISTANCE_M = 0.10
POSITION_TOLERANCE_M = 0.035
TURN_TOLERANCE_RAD = 0.05
APPROACH_MISSES_BEFORE_RETRY = 3


def angle_error(target, current):
    return (target - current + pi) % (2 * pi) - pi


def turn_rate(error):
    return max(-0.40, min(0.40, error))


def drive_to(pose, point, heading, speed):
    x, y, yaw = pose
    dx, dy = point[0] - x, point[1] - y
    turn = turn_rate(angle_error(heading, yaw))
    scale = min(1.0, speed / max(hypot(dx, dy), 1e-6))
    return (scale * (cos(yaw) * dx + sin(yaw) * dy),
            scale * (-sin(yaw) * dx + cos(yaw) * dy), turn)


def field_point(pose, ball):
    x, y, yaw = pose
    forward, left = ball
    return (x + cos(yaw) * forward - sin(yaw) * left,
            y + sin(yaw) * forward + cos(yaw) * left)


def inside_zone(point, zone, margin=0.0):
    x, y = point
    xmin, ymin, xmax, ymax = zone
    return xmin + margin <= x <= xmax - margin and ymin + margin <= y <= ymax - margin


class BallFollower:
    """在车体坐标中跟随同一颗球，到叉前预留距离即停车。"""

    def __init__(self, config, minimum_candidate_forward_m):
        self.stop_forward = config["fork_forward_m"] + config["visual_standoff_m"]
        self.minimum_forward = max(config["fork_forward_m"], minimum_candidate_forward_m)
        self.speed = config["visual_approach_speed_m_s"]
        self.target = None

    def step(self, balls):
        if self.target is None:
            visible = (ball for ball in balls if ball[0] >= self.minimum_forward)
            self.target = min(visible, key=lambda ball: hypot(*ball), default=None)
            return STOP, "confirming target" if self.target is not None else "no visible ball"

        nearest = min(balls, key=lambda ball: dist(ball, self.target), default=None)
        if nearest is None or dist(nearest, self.target) > MATCH_DISTANCE_M:
            self.target = None
            return STOP, "visual target lost"

        self.target = nearest
        forward, left = nearest
        gap = forward - self.stop_forward
        if gap <= 0.01:
            return STOP, "ball at standoff"
        return drive_to(STOP, (gap, left), 0, self.speed), "following ball"


class Navigator:
    def __init__(self, config, minimum_candidate_forward_m=0.0):
        self.config = config
        self.phase = "lane_start"
        self.lane = 0
        self.deliveries = 0
        self.resume_point = None
        self.target = None
        self.visual_target = None  # 上一次观测到的球心，车体坐标（前、左）。
        self.approach_misses = 0
        xmin, ymin, xmax, ymax = config["drop_zone"]
        self.drop = ((xmin + xmax) / 2, (ymin + ymax) / 2)
        self.fork = config["fork_forward_m"]
        self.minimum_forward = max(self.fork, minimum_candidate_forward_m)
        self.half_mouth = config["fork_width_m"] / 2 - BALL_RADIUS_M
        self.heading = pi
        self.release = None
        self.staging = False
        self.turn_heading = None
        self.turn_deadline = None

    def lane_endpoints(self):
        far_x, y = self.config["lane_starts"][self.lane]
        near = (self.config["lane_end_x_m"], y)
        far = (far_x, y)
        return (far, near) if self.lane % 2 == 0 else (near, far)

    def approach_by_camera(self, pose, balls):
        if self.phase in ("visual_approach", "wait_for_tag") and (pose is None or not balls):
            self.target = None  # 墙标或球丢失，重新确认场地坐标。
        if self.visual_target is None:
            balls = [ball for ball in balls if ball[0] >= self.minimum_forward]
        if not balls:
            return STOP, "no visible ball"

        reference = self.visual_target or (0.0, 0.0)
        ball = min(balls, key=lambda point: dist(point, reference))
        if self.visual_target is not None and dist(ball, reference) > MATCH_DISTANCE_M:
            # 新目标从下一帧重新过近距筛选。无墙标时保留叉前已锁定的球。
            if pose is not None or self.phase not in ("wait_for_tag", "approach"):
                self.visual_target = self.target = None
            return STOP, "visual target lost"
        self.visual_target = ball

        if pose is not None:
            if ball[0] < self.fork:
                return STOP, "ball inside fork"
            point = field_point(pose, ball)
            if inside_zone(point, self.config["drop_zone"]):
                self.target = self.visual_target = None
                self.phase = "lane_start"
                return STOP, "ball already in drop zone"
            # 连续两帧用墙标确认同一球的位置，才交给原有的推送对准流程。
            confirmed = self.target is not None and dist(point, self.target) <= MATCH_DISTANCE_M
            self.target = point
            if not confirmed:
                return STOP, "confirming target with wall tag"
            self.staging = True
            self.phase = "approach"
            return STOP, "target confirmed"

        self.target = None
        gap = ball[0] - self.fork - self.config["visual_standoff_m"]
        if self.phase == "wait_for_tag" or gap <= 0.01:
            self.phase = "wait_for_tag"
            return STOP, "waiting for wall tag"
        self.phase = "visual_approach"
        velocity = drive_to((0, 0, 0), (gap, ball[1]), 0, self.config["visual_approach_speed_m_s"])
        return velocity, "approach by camera"

    def search(self, pose, balls):
        start, end = self.lane_endpoints()
        low, high = sorted((start[0], end[0]))
        self.resume_point = (max(low, min(high, pose[0])), start[1])
        candidates = self.field_candidates(pose, balls, self.minimum_forward)
        match = self.match_target(candidates)
        if match is not None:
            self.target, self.visual_target = match, candidates[match]
            self.phase, self.staging = "approach", True
            return STOP, "target confirmed"
        if candidates:
            self.target = min(candidates, key=lambda point: dist(point, pose[:2]))
            self.visual_target = candidates[self.target]
            return STOP, "confirming target"
        self.target = self.visual_target = None
        if dist(pose[:2], end) > POSITION_TOLERANCE_M:
            return drive_to(pose, end, pi, self.config["search_speed_m_s"]), "search lane"
        self.lane += 1
        self.resume_point = None
        self.phase = "finished" if self.lane == len(self.config["lane_starts"]) else "lane_start"
        return STOP, "lane searched"

    def field_candidates(self, pose, balls, minimum_forward):
        candidates = {}
        for ball in balls:
            point = field_point(pose, ball)
            if ball[0] >= minimum_forward and not inside_zone(point, self.config["drop_zone"]):
                candidates[point] = ball
        return candidates

    def match_target(self, candidates):
        if self.target is None or not candidates:
            return None
        nearest = min(candidates, key=lambda point: dist(point, self.target))
        return nearest if dist(nearest, self.target) <= MATCH_DISTANCE_M else None

    def approach(self, pose, balls):
        candidates = self.field_candidates(pose, balls, self.fork)
        match = self.match_target(candidates)
        if match is None:
            return self.on_approach_miss(pose)
        self.approach_misses = 0
        self.target, self.visual_target = match, candidates[match]
        heading = atan2(self.drop[1] - match[1], self.drop[0] - match[0])
        forward, lateral = self.visual_target
        entry_gap = self.config["push_entry_gap_m"]
        if (forward <= self.fork + entry_gap and abs(lateral) <= self.half_mouth / 2
                and abs(angle_error(heading, pose[2])) <= 0.10):
            self.heading, self.phase = heading, "push"
            return STOP, "fork aligned"
        dx, dy = pose[0] - match[0], pose[1] - match[1]
        ux, uy = cos(heading), sin(heading)
        ahead, side = dx * ux + dy * uy, -dx * uy + dy * ux
        # 在进入推送的距离略内侧瞄准，避免逼到镜头下方才要求对准。
        behind, offset = self.fork + 0.9 * entry_gap, 0.0
        # 车身与球并排或越过球时，先侧移让出导杆，再到球后方。
        if self.staging and ahead > -behind / 2:
            clearance = self.config["track_m"] / 2 + self.half_mouth + 0.06
            offset = clearance if side >= 0 else -clearance
            behind = behind + 0.12 if abs(side) >= clearance - POSITION_TOLERANCE_M else -ahead
        else:
            self.staging = False
        point = (match[0] - behind * ux - offset * uy, match[1] - behind * uy + offset * ux)
        return drive_to(pose, point, heading, self.config["approach_speed_m_s"]), "approach ball"

    def on_approach_miss(self, pose):
        self.approach_misses += 1
        # 结合最后一次球的场地坐标和当前车位，判断球是否可能已进入车头盲区。
        clearance = self.fork + self.config["visual_standoff_m"] + BALL_RADIUS_M
        too_close = self.target is None or dist(pose[:2], self.target) <= clearance
        if too_close or self.approach_misses < APPROACH_MISSES_BEFORE_RETRY:
            return STOP, "target lost during approach"
        self.target = self.visual_target = None
        self.approach_misses = 0
        self.phase = "lane_start"
        return STOP, "target lost, resuming lane"

    def push(self, pose):
        ux, uy = cos(self.heading), sin(self.heading)
        offset = self.fork + 2 * BALL_RADIUS_M
        stop = (self.drop[0] - offset * ux, self.drop[1] - offset * uy)
        ball = field_point(pose, (offset, 0.0))
        if dist(pose[:2], stop) <= 0.025 and inside_zone(ball, self.config["drop_zone"], BALL_RADIUS_M):
            self.release = (pose[0] - self.config["reverse_m"] * ux, pose[1] - self.config["reverse_m"] * uy)
            self.phase = "reverse"
            return STOP, "drop-off reached"
        return drive_to(pose, stop, self.heading, self.config["push_speed_m_s"]), "push ball"

    def turn(self, yaw):
        if monotonic() >= self.turn_deadline:
            raise TimeoutError("未在限定时间内完成 90° 转向，停车")
        if yaw is None:
            return STOP, "turn feedback unavailable"
        error = angle_error(self.turn_heading, yaw)
        if abs(error) > TURN_TOLERANCE_RAD:
            return (0.0, 0.0, turn_rate(error)), "quarter turn"
        self.deliveries += 1
        self.target = self.visual_target = None
        self.phase = "finished" if self.deliveries >= self.config["max_deliveries"] else "return_to_lane"
        self.turn_deadline = monotonic() + self.config["turn_timeout_s"]
        return STOP, "quarter turn complete"

    def step(self, pose, balls, *, wheel_yaw=None):
        if self.phase == "finished":
            return STOP, "finished"
        if self.phase == "turn":
            return self.turn(pose[2] if pose is not None else wheel_yaw)
        if self.phase == "return_to_lane" and pose is None:
            # 侧向转完可能看不到墙标。恢复墙标前，只允许原地转回。
            if wheel_yaw is None or monotonic() >= self.turn_deadline:
                return STOP, "waiting for wall tag"
            error = angle_error(pi, wheel_yaw)
            if abs(error) <= TURN_TOLERANCE_RAD:
                return STOP, "waiting for wall tag"
            return (0.0, 0.0, turn_rate(error)), "face wall tag"
        if self.phase in ("visual_approach", "wait_for_tag"):
            return self.approach_by_camera(pose, balls)
        if pose is None:
            self.approach_misses = 0
            if self.phase not in ("lane_start", "search", "approach"):
                return STOP, "wall tag unavailable"
            if self.phase == "search":
                self.target = None
            return self.approach_by_camera(None, balls)
        if self.phase in ("lane_start", "return_to_lane"):
            point = self.resume_point or self.lane_endpoints()[0]
            if dist(pose[:2], point) <= POSITION_TOLERANCE_M and abs(angle_error(pi, pose[2])) <= 0.12:
                self.phase = "search"
                return STOP, "lane start reached"
            return drive_to(pose, point, pi, self.config["search_speed_m_s"]), "return to lane"
        if self.phase == "search":
            return self.search(pose, balls)
        if self.phase == "approach":
            return self.approach(pose, balls)
        if self.phase == "push":
            return self.push(pose)
        if dist(pose[:2], self.release) > POSITION_TOLERANCE_M:
            return drive_to(pose, self.release, self.heading, self.config["push_speed_m_s"]), "reverse clear"
        self.turn_heading = pose[2] + self.config["turn_direction"] * pi / 2
        self.turn_deadline = monotonic() + self.config["turn_timeout_s"]
        self.phase = "turn"
        return STOP, "fork cleared"
