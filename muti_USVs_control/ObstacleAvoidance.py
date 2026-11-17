import math
import numpy as np


class Obstacle:
    """障碍物基类"""

    def __init__(self, obs_id, position=(0.0, 0.0)):
        self.obs_id = obs_id
        self.position = position  # 世界坐标系下的位置 (x, y)

    def set_position(self, position):
        self.position = position

    def distance_to_point(self, point):
        """计算点到障碍物表面的最短距离，>0 表示在障碍物外"""
        raise NotImplementedError

    def repulsive_vector(self, point):
        """计算对某点的排斥力方向（单位向量，指向远离障碍物）"""
        raise NotImplementedError


class CircleObstacle(Obstacle):
    """圆形障碍物"""

    def __init__(self, obs_id, radius=0.1, position=(0.0, 0.0)):
        super().__init__(obs_id, position)
        self.radius = radius

    def distance_to_point(self, point):
        x, y = point
        cx, cy = self.position
        return math.hypot(x - cx, y - cy) - self.radius

    def repulsive_vector(self, point):
        x, y = point
        cx, cy = self.position
        direction = np.array([x - cx, y - cy])
        norm = np.linalg.norm(direction)
        if norm < 1e-6:
            return np.array([1.0, 0.0])
        return direction / norm


class RectangleObstacle(Obstacle):
    """矩形障碍物（支持旋转）"""

    def __init__(self, obs_id, width=0.2, height=0.2, angle=0.0, position=(0.0, 0.0)):
        super().__init__(obs_id, position)
        self.width = width
        self.height = height
        self.angle = angle  # 旋转角 (rad)

    def _to_local(self, point):
        x, y = point
        cx, cy = self.position
        dx = x - cx
        dy = y - cy
        cos_a = math.cos(-self.angle)
        sin_a = math.sin(-self.angle)
        lx = dx * cos_a - dy * sin_a
        ly = dx * sin_a + dy * cos_a
        return lx, ly

    def _closest_point_on_rect(self, point):
        lx, ly = self._to_local(point)
        hw = self.width / 2.0
        hh = self.height / 2.0
        return np.clip(lx, -hw, hw), np.clip(ly, -hh, hh)

    def distance_to_point(self, point):
        lx, ly = self._to_local(point)
        hw = self.width / 2.0
        hh = self.height / 2.0
        dx = max(0.0, abs(lx) - hw)
        dy = max(0.0, abs(ly) - hh)
        return math.hypot(dx, dy)

    def repulsive_vector(self, point):
        cx_local, cy_local = self._closest_point_on_rect(point)
        cos_a = math.cos(self.angle)
        sin_a = math.sin(self.angle)
        wx = self.position[0] + cx_local * cos_a - cy_local * sin_a
        wy = self.position[1] + cx_local * sin_a + cy_local * cos_a
        direction = np.array([point[0] - wx, point[1] - wy])
        norm = np.linalg.norm(direction)
        if norm < 1e-6:
            direction = np.array([point[0] - self.position[0],
                                  point[1] - self.position[1]])
            norm = np.linalg.norm(direction)
            if norm < 1e-6:
                return np.array([1.0, 0.0])
        return direction / norm


class ObstacleAvoidance:
    """基于人工势场法(APF)的避障模块"""

    def __init__(self,
                 influence_distance=1.0,
                 safety_margin=0.2,
                 repulsive_gain=3.0,
                 max_avoidance_angle=math.pi / 2,
                 min_speed_ratio=0.2):
        self.influence_distance = influence_distance
        self.safety_margin = safety_margin
        self.repulsive_gain = repulsive_gain
        self.max_avoidance_angle = max_avoidance_angle
        self.min_speed_ratio = min_speed_ratio
        self.last_min_distance = float('inf')
        # 平滑滤波状态（低通滤波，避免轨迹锯齿）
        self.smoothed_heading_diff = 0.0
        self.smoothed_offset_mag = 0.0
        self.smoothing_factor = 0.1  # 0~1，越小越平滑
        self.max_heading_rate = 0.08  # 单帧最大航向变化(rad)，限制转向速率
        self.path_detour_sides = {}

    def compute_avoidance(self, current_position, current_heading,
                          desired_heading, desired_speed, obstacles,
                          reference_point=None):
        """
        计算避障后期望航向、速度和偏移目标点

        参数:
            current_position: USV当前位置 (x, y)
            current_heading: USV当前航向 (rad)
            desired_heading: 制导给出的期望航向 (rad)
            desired_speed: 制导给出的期望速度 (m/s)
            obstacles: 障碍物列表 [Obstacle, ...]
            reference_point: 制导给出的参考目标点 (x, y)，用于偏移

        返回:
            (modified_heading, modified_speed, modified_target)
        """
        if not obstacles:
            self.last_min_distance = float('inf')
            self.smoothed_heading_diff *= (1.0 - self.smoothing_factor)
            self.smoothed_offset_mag *= (1.0 - self.smoothing_factor)
            return desired_heading, desired_speed, reference_point

        repulsive = np.array([0.0, 0.0])
        min_surface_dist = float('inf')

        for obs in obstacles:
            surface_dist = obs.distance_to_point(current_position) - self.safety_margin
            if surface_dist >= self.influence_distance:
                continue

            # 经典APF排斥力：F = k * (1/d - 1/d0) / d^2
            # 距离越近，力呈非线性急剧增大
            d = max(surface_dist, 0.01)
            d0 = self.influence_distance
            force_mag = self.repulsive_gain * (1.0 / d - 1.0 / d0) / (d * d)

            rep_dir = obs.repulsive_vector(current_position)
            repulsive += rep_dir * force_mag

            if surface_dist < min_surface_dist:
                min_surface_dist = surface_dist

        self.last_min_distance = min_surface_dist

        rep_mag = np.linalg.norm(repulsive)
        if rep_mag < 1e-6:
            # 无障碍物在影响范围内：平滑衰减到0
            self.smoothed_heading_diff *= (1.0 - self.smoothing_factor)
            self.smoothed_offset_mag *= (1.0 - self.smoothing_factor)
            return desired_heading, desired_speed, reference_point

        rep_dir = repulsive / rep_mag

        goal_vec = np.array([math.cos(desired_heading), math.sin(desired_heading)])

        # 切向绕行方向
        tangent = np.array([-rep_dir[1], rep_dir[0]])
        if np.dot(tangent, goal_vec) < 0:
            tangent = -tangent

        # 避障方向 = 径向排斥 + 切向绕行
        avoidance_dir = rep_dir * 0.3 + tangent * 0.95
        avoidance_norm = np.linalg.norm(avoidance_dir)
        if avoidance_norm > 1e-6:
            avoidance_dir = avoidance_dir / avoidance_norm

        avoidance_strength = min(rep_mag, 3.0)  # 限制上限避免力突变
        combined = goal_vec + avoidance_dir * avoidance_strength
        combined_norm = np.linalg.norm(combined)
        if combined_norm < 1e-6:
            combined = avoidance_dir

        modified_heading = math.atan2(combined[1], combined[0])

        # 限制单次转向幅度
        heading_diff = modified_heading - desired_heading
        heading_diff = math.atan2(math.sin(heading_diff), math.cos(heading_diff))
        heading_diff = float(np.clip(heading_diff,
                                     -self.max_avoidance_angle,
                                     self.max_avoidance_angle))
        # 低通滤波平滑航向修正
        self.smoothed_heading_diff = (self.smoothing_factor * heading_diff +
                                      (1.0 - self.smoothing_factor) * self.smoothed_heading_diff)
        # 限制单帧航向变化率，避免急转弯
        prev = getattr(self, '_prev_smoothed_heading_diff', 0.0)
        delta = self.smoothed_heading_diff - prev
        delta = float(np.clip(delta, -self.max_heading_rate, self.max_heading_rate))
        self.smoothed_heading_diff = prev + delta
        self._prev_smoothed_heading_diff = self.smoothed_heading_diff
        modified_heading = desired_heading + self.smoothed_heading_diff
        modified_heading = math.atan2(math.sin(modified_heading),
                                      math.cos(modified_heading))

        # 速度处理：避障时保持最低速度以维持机动性
        modified_speed = max(desired_speed, 0.1)

        # 偏移目标点：让NMPC绕行（小幅度，仅靠近时）
        modified_target = reference_point
        if reference_point is not None and min_surface_dist < self.influence_distance:
            max_offset = 0.4
            t = 1.0 - min_surface_dist / self.influence_distance
            offset_mag = max_offset * (t ** 2)
            # 低通滤波平滑偏移量
            self.smoothed_offset_mag = (self.smoothing_factor * offset_mag +
                                        (1.0 - self.smoothing_factor) * self.smoothed_offset_mag)
            offset = avoidance_dir * self.smoothed_offset_mag
            modified_target = (reference_point[0] + offset[0],
                               reference_point[1] + offset[1])

        return modified_heading, modified_speed, modified_target

    def modify_reference_path(self, reference_point, next_reference_point,
                              obstacles, safety_margin=0.18,
                              vessel_radius=0.12, extra_clearance=0.06):
        """Smoothly bend a short section of the reference route around obstacles.

        Unlike a reactive force, each obstacle keeps one selected passing side
        and the offset fades in and out along the route. This avoids the
        left-right switching that can make a USV oscillate near an obstacle.
        """
        if not obstacles or reference_point is None or next_reference_point is None:
            return reference_point, next_reference_point

        dx = next_reference_point[0] - reference_point[0]
        dy = next_reference_point[1] - reference_point[1]
        segment_length = math.hypot(dx, dy)
        if segment_length < 1e-8:
            return reference_point, next_reference_point

        tangent = (dx / segment_length, dy / segment_length)
        normal = (-tangent[1], tangent[0])
        shifts = [0.0, 0.0]

        for obstacle in obstacles:
            if hasattr(obstacle, 'radius'):
                obstacle_radius = float(obstacle.radius)
            elif hasattr(obstacle, 'width') and hasattr(obstacle, 'height'):
                obstacle_radius = 0.5 * math.hypot(obstacle.width, obstacle.height)
            else:
                continue

            safe_radius = obstacle_radius + safety_margin + vessel_radius
            influence_length = max(0.8, 2.5 * safe_radius)
            ox = obstacle.position[0] - reference_point[0]
            oy = obstacle.position[1] - reference_point[1]
            along = ox * tangent[0] + oy * tangent[1]
            lateral = ox * normal[0] + oy * normal[1]

            # The route already passes outside the required clearance.
            if abs(along) >= influence_length or abs(lateral) >= safe_radius + 0.35:
                continue

            obstacle_id = str(getattr(obstacle, 'obs_id', id(obstacle)))
            if obstacle_id not in self.path_detour_sides:
                if lateral > 0.03:
                    side = -1.0
                elif lateral < -0.03:
                    side = 1.0
                else:
                    # Deterministic choice when the obstacle lies on the path.
                    side = 1.0 if sum(map(ord, obstacle_id)) % 2 == 0 else -1.0
                self.path_detour_sides[obstacle_id] = side
            side = self.path_detour_sides[obstacle_id]

            required_offset = max(
                0.0, safe_radius + extra_clearance - abs(lateral)
            )
            blend = 0.5 * (1.0 + math.cos(math.pi * along / influence_length))
            shifts[0] += side * required_offset * blend

            next_ox = obstacle.position[0] - next_reference_point[0]
            next_oy = obstacle.position[1] - next_reference_point[1]
            next_along = next_ox * tangent[0] + next_oy * tangent[1]
            next_lateral = next_ox * normal[0] + next_oy * normal[1]
            if abs(next_along) < influence_length and abs(next_lateral) < safe_radius + 0.35:
                next_required_offset = max(
                    0.0, safe_radius + extra_clearance - abs(next_lateral)
                )
                next_blend = 0.5 * (
                    1.0 + math.cos(math.pi * next_along / influence_length)
                )
                shifts[1] += side * next_required_offset * next_blend

        # Bound overlapping detours from several nearby obstacles.
        shifts = [max(-1.2, min(1.2, shift)) for shift in shifts]
        return (
            (reference_point[0] + normal[0] * shifts[0],
             reference_point[1] + normal[1] * shifts[0]),
            (next_reference_point[0] + normal[0] * shifts[1],
             next_reference_point[1] + normal[1] * shifts[1])
        )
