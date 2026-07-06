import rclpy, math
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from geometry_msgs.msg import Pose, Twist

FACE_DIST       = 0.32
OBSTACLE_RADIUS = 0.20
SAFE_MARGIN     = 0.6
HOLD_RADIUS     = FACE_DIST + SAFE_MARGIN + 0.6
DOCK_TOL        = 0.03
ORBIT_TOL       = 0.05
FINAL_YAW_TOL   = 0.05
STEP_SPEED      = 0.02
YAW_STEP        = 0.04
ORBIT_LOOKAHEAD = 0.6

FACE_ANGLE_OFFSET = math.radians(30)
HEX_FACE_ANGLES = [
    math.atan2(math.sin(math.radians(a) + FACE_ANGLE_OFFSET),
               math.cos(math.radians(a) + FACE_ANGLE_OFFSET))
    for a in (0, 60, 120, 180, 240, 300)
]

FORMATION = [
    {'name': 'robot2', 'anchor': 'robot1', 'face': 0},
    {'name': 'robot3', 'anchor': 'robot1', 'face': 1},
    {'name': 'robot4', 'anchor': 'robot1', 'face': 3},
    {'name': 'robot5', 'anchor': 'robot1', 'face': 4},
]

ROOT = 'robot1'
ALL_ROBOTS = [ROOT] + [f['name'] for f in FORMATION]


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def point_seg_dist(px, py, qx, qy, ox, oy):
    dx, dy = qx - px, qy - py
    length2 = dx * dx + dy * dy
    if length2 < 1e-9:
        return math.hypot(ox - px, oy - py)
    t = max(0.0, min(1.0, ((ox - px) * dx + (oy - py) * dy) / length2))
    projx, projy = px + t * dx, py + t * dy
    return math.hypot(ox - projx, oy - projy)


class DockingNode(Node):
    def __init__(self):
        super().__init__('docking_node')
        self.poses = {}
        self.state = {f['name']: 'waiting' for f in FORMATION}
        self.assigned_face = {}
        self.taken_faces = {r: set() for r in ALL_ROBOTS}
        self.active_index = 0
        self.docked_target = {}
        self.wheel_pubs = {}

        self.create_subscription(TFMessage, '/world/dock_world/pose/info',
                                  self.on_pose_info, 10)

        self.set_pose_client = self.create_client(SetEntityPose, '/world/dock_world/set_pose')
        self.get_logger().info('Waiting for set_pose service...')
        self.set_pose_client.wait_for_service()
        self.get_logger().info('set_pose service ready.')

        self.create_timer(0.03, self.run)

    def on_pose_info(self, msg: TFMessage):
        for t in msg.transforms:
            if t.child_frame_id in ALL_ROBOTS:
                p = t.transform.translation
                q = t.transform.rotation
                self.poses[t.child_frame_id] = (p.x, p.y, yaw_from_quat(q))

    def teleport(self, name, x, y, yaw):
        if not self.set_pose_client.service_is_ready():
            return
        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = name
        req.entity.type = Entity.MODEL
        req.pose = Pose()
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = 0.1
        qx, qy, qz, qw = yaw_to_quat(yaw)
        req.pose.orientation.x = qx
        req.pose.orientation.y = qy
        req.pose.orientation.z = qz
        req.pose.orientation.w = qw
        future = self.set_pose_client.call_async(req)
        future.add_done_callback(lambda f: None)

    def spin_wheels(self, name, distance_moved):
        if name not in self.wheel_pubs:
            self.wheel_pubs[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
        tw = Twist()
        tw.linear.x = 0.3 if distance_moved > 0.005 else 0.0
        self.wheel_pubs[name].publish(tw)

    def assign_face(self, name, anchor, forced_face):
        if forced_face is not None:
            face = forced_face
        else:
            ax, ay, _ = self.poses[anchor]
            fx, fy, _ = self.poses[name]
            bearing = math.atan2(fy - ay, fx - ax)
            best_face, best_diff = None, None
            for idx, ang in enumerate(HEX_FACE_ANGLES):
                if idx in self.taken_faces[anchor]:
                    continue
                diff = abs(norm_angle(ang - bearing))
                if best_diff is None or diff < best_diff:
                    best_diff, best_face = diff, idx
            face = best_face
        self.assigned_face[name] = face
        self.taken_faces[anchor].add(face)
        self.get_logger().info(f'{name} assigned face {face} on {anchor}')

    def path_is_clear(self, name, anchor, start_x, start_y, target_x, target_y):
        obstacles = []
        if ROOT in self.poses and ROOT != anchor:
            obstacles.append((self.poses[ROOT][0], self.poses[ROOT][1]))
        for other_name, st in self.state.items():
            if other_name == name or other_name == anchor:
                continue
            if st == 'docked' and other_name in self.poses:
                obstacles.append((self.poses[other_name][0], self.poses[other_name][1]))
        for ox, oy in obstacles:
            if point_seg_dist(start_x, start_y, target_x, target_y, ox, oy) < OBSTACLE_RADIUS:
                return False
        return True

    def step_toward(self, name, fx, fy, fyaw, target_x, target_y, keep_yaw=True, target_yaw=None):
        dx, dy = target_x - fx, target_y - fy
        d = math.hypot(dx, dy)
        if d < 1e-6:
            new_x, new_y = fx, fy
        else:
            step = min(STEP_SPEED, d)
            new_x = fx + dx / d * step
            new_y = fy + dy / d * step

        new_yaw = fyaw
        if not keep_yaw and target_yaw is not None:
            yaw_err = norm_angle(target_yaw - fyaw)
            yaw_step = max(-YAW_STEP, min(YAW_STEP, yaw_err))
            new_yaw = norm_angle(fyaw + yaw_step)

        self.teleport(name, new_x, new_y, new_yaw)
        self.spin_wheels(name, d)
        return d

    def run(self):
        if self.active_index < len(FORMATION):
            entry = FORMATION[self.active_index]
            name, anchor = entry['name'], entry['anchor']
            anchor_ready = anchor == ROOT or self.state.get(anchor) == 'docked'

            if self.state[name] == 'waiting' and anchor_ready and name in self.poses and anchor in self.poses:
                if name not in self.assigned_face:
                    self.assign_face(name, anchor, entry.get('face'))

                ax, ay, _ = self.poses[anchor]
                face_idx = self.assigned_face[name]
                slot_angle = HEX_FACE_ANGLES[face_idx]
                target_x = ax + FACE_DIST * math.cos(slot_angle)
                target_y = ay + FACE_DIST * math.sin(slot_angle)
                fx, fy, _ = self.poses[name]

                if self.path_is_clear(name, anchor, fx, fy, target_x, target_y):
                    self.state[name] = 'approach'
                    self.get_logger().info(f'{name}: direct path clear, sliding straight in')
                else:
                    self.state[name] = 'transit_out'
                    self.get_logger().info(f'{name}: path blocked, routing around {anchor}')

            if self.state[name] == 'docked':
                self.active_index += 1

        for entry in FORMATION:
            name, anchor = entry['name'], entry['anchor']
            if self.state[name] not in ('transit_out', 'orbit', 'approach', 'align'):
                continue
            if name not in self.poses or anchor not in self.poses:
                continue

            ax, ay, _ = self.poses[anchor]
            face_idx = self.assigned_face[name]
            slot_angle = HEX_FACE_ANGLES[face_idx]
            fx, fy, fyaw = self.poses[name]

            if self.state[name] == 'transit_out':
                theta = math.atan2(fy - ay, fx - ax)
                target_x = ax + HOLD_RADIUS * math.cos(theta)
                target_y = ay + HOLD_RADIUS * math.sin(theta)
                d = self.step_toward(name, fx, fy, fyaw, target_x, target_y)
                if d <= ORBIT_TOL:
                    self.state[name] = 'orbit'
                continue

            if self.state[name] == 'orbit':
                current_bearing = math.atan2(fy - ay, fx - ax)
                current_radius = math.hypot(fx - ax, fy - ay)
                err = norm_angle(slot_angle - current_bearing)

                if abs(err) <= ORBIT_TOL and current_radius >= HOLD_RADIUS - 0.15:
                    self.state[name] = 'approach'
                    continue

                lookahead = min(ORBIT_LOOKAHEAD, abs(err))
                lookahead = math.copysign(lookahead, err)
                next_theta = norm_angle(current_bearing + lookahead)
                target_x = ax + HOLD_RADIUS * math.cos(next_theta)
                target_y = ay + HOLD_RADIUS * math.sin(next_theta)
                self.step_toward(name, fx, fy, fyaw, target_x, target_y)

                self.get_logger().info(
                    f'{name} orbiting {anchor} | angle_err={err:.2f} r={current_radius:.2f}',
                    throttle_duration_sec=1.0)
                continue

            if self.state[name] == 'approach':
                target_x = ax + FACE_DIST * math.cos(slot_angle)
                target_y = ay + FACE_DIST * math.sin(slot_angle)
                d = self.step_toward(name, fx, fy, fyaw, target_x, target_y)
                if d <= DOCK_TOL:
                    self.state[name] = 'align'
                self.get_logger().info(f'{name} -> {anchor} approach | dist={d:.2f}',
                                        throttle_duration_sec=1.0)
                continue

            if self.state[name] == 'align':
                target_yaw = norm_angle(slot_angle + math.pi)
                yaw_err = norm_angle(target_yaw - fyaw)
                if abs(yaw_err) <= FINAL_YAW_TOL:
                    self.state[name] = 'docked'
                    self.docked_target[name] = (
                        ax + FACE_DIST * math.cos(slot_angle),
                        ay + FACE_DIST * math.sin(slot_angle),
                        target_yaw,
                    )
                    self.get_logger().info(f'DOCKED! {name} flush on face {face_idx} of {anchor}')
                    self.spin_wheels(name, 0.0)
                    continue
                self.step_toward(name, fx, fy, fyaw, fx, fy, keep_yaw=False, target_yaw=target_yaw)

        for entry in FORMATION:
            name = entry['name']
            if self.state[name] == 'docked' and name in self.docked_target and name in self.poses:
                tx, ty, tyaw = self.docked_target[name]
                cx, cy, _ = self.poses[name]
                if math.hypot(cx - tx, cy - ty) > 0.02:
                    self.teleport(name, tx, ty, tyaw)


def main():
    rclpy.init()
    node = DockingNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
