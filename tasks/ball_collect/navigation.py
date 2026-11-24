"""按墙标坐标管理巡线、对准、推送和退回。速度使用车体坐标。"""
from math import atan2, cos, dist, hypot, pi, sin

from tasks.ball_detect.ground_position import BALL_RADIUS_M


STOP = (0.0, 0.0, 0.0)
MATCH_DISTANCE_M = 0.10


def angle_error(target, current):
    return (target - current + pi) % (2 * pi) - pi


def drive_to(pose, point, heading, speed):
    x, y, yaw = pose
    dx, dy = point[0] - x, point[1] - y
    turn = max(-0.40, min(0.40, angle_error(heading, yaw)))
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


class Navigator:
    def __init__(self, config):
        self.config = config
        self.phase = "lane_start"
        self.lane = 0
        self.deliveries = 0
        self.empty_lanes = 0
        self.target = None
        self.visual_target = None  # 上一次观测到的球心，车体坐标（前、左）。
        xmin, ymin, xmax, ymax = config["drop_zone"]
        self.drop = ((xmin + xmax) / 2, (ymin + ymax) / 2)
        self.fork = config["fork_forward_m"]
        self.half_mouth = config["fork_width_m"] / 2 - BALL_RADIUS_M
        self.heading = pi
        self.release = None
        self.staging = False

    def next_lane(self):
        self.lane = (self.lane + 1) % len(self.config["lane_starts"])
        done = (self.deliveries >= self.config["max_deliveries"]
                or self.empty_lanes >= len(self.config["lane_starts"]))
        self.phase = "finished" if done else "lane_start"
        self.target = None
        self.visual_target = None

    def approach_by_camera(self, pose, balls):
        if self.phase in ("visual_approach", "wait_for_tag") and (pose is None or not balls):
            self.target = None  # 墙标或球丢失，重新确认场地坐标。
        if not balls:
            return STOP, "no visible ball"

        reference = self.visual_target or (0.0, 0.0)
        ball = min(balls, key=lambda point: dist(point, reference))
        if self.visual_target is not None and dist(ball, reference) > MATCH_DISTANCE_M:
            # 先停车。已到前叉附近时，即使旁边又出现一个球也要等墙标。
            if pose is not None:
                self.visual_target = ball
                self.target = None
            elif self.phase not in ("wait_for_tag", "approach"):
                self.visual_target = None
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

    def step(self, pose, balls):
        if self.phase == "finished":
            return STOP, "finished"
        if self.phase in ("visual_approach", "wait_for_tag"):
            return self.approach_by_camera(pose, balls)
        if pose is None:
            if self.phase in ("lane_start", "search", "approach"):
                if self.phase == "search":
                    self.target = None
                return self.approach_by_camera(None, balls)
            return STOP, "wall tag unavailable"

        config = self.config
        position = pose[:2]
        lane_start = config["lane_starts"][self.lane]
        valid = {field_point(pose, ball): ball for ball in balls if ball[0] >= self.fork}
        outside = [point for point in valid if not inside_zone(point, config["drop_zone"])]
        matches = [point for point in outside if self.target and dist(point, self.target) <= MATCH_DISTANCE_M]

        if self.phase == "lane_start":
            if dist(position, lane_start) <= 0.035 and abs(angle_error(pi, pose[2])) <= 0.12:
                self.phase = "search"
                return STOP, "lane start reached"
            return drive_to(pose, lane_start, pi, config["search_speed_m_s"]), "return to lane"

        if self.phase == "search":
            if matches:
                self.target = min(matches, key=lambda point: dist(point, self.target))
                self.visual_target = valid[self.target]
                self.phase = "approach"
                self.staging = True
                return STOP, "target confirmed"
            if outside:
                self.target = min(outside, key=lambda point: dist(point, position))
                self.visual_target = valid[self.target]
                return STOP, "confirming target"
            self.target = None
            self.visual_target = None
            end = (config["lane_end_x_m"], lane_start[1])
            if dist(position, end) <= 0.035:
                self.empty_lanes += 1
                self.next_lane()
                return STOP, "lane searched"
            return drive_to(pose, end, pi, config["search_speed_m_s"]), "search lane"

        if self.phase == "approach":
            if not matches:
                return STOP, "target lost during approach"
            self.target = min(matches, key=lambda point: dist(point, self.target))
            self.visual_target = valid[self.target]
            dx, dy = self.drop[0] - self.target[0], self.drop[1] - self.target[1]
            heading = atan2(dy, dx)
            along = cos(heading), sin(heading)
            ball_dx, ball_dy = self.target[0] - position[0], self.target[1] - position[1]
            forward, lateral = self.visual_target
            if (self.fork <= forward <= self.fork + 0.05 and abs(lateral) <= self.half_mouth / 2
                    and abs(angle_error(heading, pose[2])) <= 0.10):
                self.heading = heading
                self.phase = "push"
                return STOP, "fork aligned"
            behind = self.fork + 0.02
            point = self.target[0] - behind * along[0], self.target[1] - behind * along[1]
            # If the chassis is beside or ahead of the ball, clear it sideways before lining up.
            ahead = -ball_dx * along[0] - ball_dy * along[1]
            side = ball_dx * along[1] - ball_dy * along[0]
            if self.staging and ahead > -behind / 2:
                normal = -along[1], along[0]
                clearance = config["track_m"] / 2 + self.half_mouth + 0.06
                offset = clearance if side >= 0 else -clearance
                distance_back = behind + 0.12 if abs(side) >= clearance - 0.035 else -ahead
                point = (self.target[0] - distance_back * along[0] + offset * normal[0],
                         self.target[1] - distance_back * along[1] + offset * normal[1])
            else:
                self.staging = False
            return drive_to(pose, point, heading, config["approach_speed_m_s"]), "approach ball"

        if self.phase == "push":
            along = cos(self.heading), sin(self.heading)
            # The fork carries the ball slightly ahead of its front edge.
            offset = self.fork + 2 * BALL_RADIUS_M
            stop = self.drop[0] - offset * along[0], self.drop[1] - offset * along[1]
            front_ball = field_point(pose, (offset, 0.0))
            if dist(position, stop) <= 0.025 and inside_zone(
                front_ball, config["drop_zone"], BALL_RADIUS_M,
            ):
                self.release = (position[0] - config["reverse_m"] * along[0],
                                position[1] - config["reverse_m"] * along[1])
                self.phase = "reverse"
                return STOP, "drop-off reached"
            return drive_to(pose, stop, self.heading, config["push_speed_m_s"]), "push ball"

        if dist(position, self.release) <= 0.035:
            self.deliveries += 1
            self.empty_lanes = 0
            self.next_lane()
            return STOP, "fork cleared"
        return drive_to(pose, self.release, self.heading, config["push_speed_m_s"]), "reverse clear"
