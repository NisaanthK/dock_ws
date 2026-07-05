import rclpy, math
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist

FACE_DIST       = 0.33
OBSTACLE_RADIUS = 0.22
SAFE_MARGIN     = 0.6
HOLD_RADIUS     = FACE_DIST + SAFE_MARGIN + 0.6
DOCK_TOL        = 0.02
ORBIT_TOL       = 0.02
ORBIT_LOOKAHEAD = 0.35     
HEADING_GATE    = 0.15
FINAL_YAW_TOL   = 0.05
LIN_SPEED       = 0.15
ORBIT_LIN_SPEED = 0.35     # constant speed while orbiting -- not distance-throttled
KP_ANG          = 1.2
KP_LIN          = 0.6

HEX_FACE_ANGLES = [math.radians(a) for a in (0, 60, 120, 180, 240, 300)]

FORMATION = [
    {'name': 'robot2', 'anchor': 'robot1', 'face': 1},
    {'name': 'robot3', 'anchor': 'robot1', 'face': 2},
    {'name': 'robot4', 'anchor': 'robot1', 'face': 4},
    {'name': 'robot5', 'anchor': 'robot1', 'face': 5},
    {'name': 'robot6', 'anchor': 'robot1', 'face': 0},  # 0 deg, right
    {'name': 'robot7', 'anchor': 'robot1', 'face': 3},  # 180 deg, left
]

ROOT = 'robot1'
ALL_ROBOTS = [ROOT] + [f['name'] for f in FORMATION]


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


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

        self.create_subscription(TFMessage, '/world/dock_world/pose/info',
                                  self.on_pose_info, 10)
        self.pubs = {f['name']: self.create_publisher(Twist, f"/{f['name']}/cmd_vel", 10)
                     for f in FORMATION}
        self.create_timer(0.1, self.run)

    def on_pose_info(self, msg: TFMessage):
        for t in msg.transforms:
            if t.child_frame_id in ALL_ROBOTS:
                p = t.transform.translation
                q = t.transform.rotation
                self.poses[t.child_frame_id] = (p.x, p.y, yaw_from_quat(q))

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

    def path_is_clear(self, name, start_x, start_y, target_x, target_y):
        obstacles = []
        if ROOT in self.poses:
            obstacles.append((self.poses[ROOT][0], self.poses[ROOT][1]))
        for other_name, st in self.state.items():
            if other_name == name:
                continue
            if st == 'docked' and other_name in self.poses:
                obstacles.append((self.poses[other_name][0], self.poses[other_name][1]))
        for ox, oy in obstacles:
            if point_seg_dist(start_x, start_y, target_x, target_y, ox, oy) < OBSTACLE_RADIUS:
                return False
        return True

    def drive_toward(self, pub, fx, fy, fyaw, target_x, target_y):
        dx, dy = target_x - fx, target_y - fy
        d = math.hypot(dx, dy)
        heading = norm_angle(math.atan2(dy, dx) - fyaw)
        tw = Twist()
        if abs(heading) > HEADING_GATE:
            tw.angular.z = KP_ANG * heading
        else:
            tw.linear.x = min(LIN_SPEED, KP_LIN * d)
            tw.angular.z = KP_ANG * heading * 0.5
        pub.publish(tw)
        return d

    def drive_orbit(self, pub, fx, fy, fyaw, target_x, target_y):
        """Like drive_toward but at constant speed -- not throttled by the
        (deliberately small) distance to the lookahead point, which was
        causing the near-zero-speed stall."""
        dx, dy = target_x - fx, target_y - fy
        heading = norm_angle(math.atan2(dy, dx) - fyaw)
        tw = Twist()
        if abs(heading) > HEADING_GATE:
            tw.angular.z = KP_ANG * heading
        else:
            tw.linear.x = ORBIT_LIN_SPEED
            tw.angular.z = KP_ANG * heading * 0.5
        pub.publish(tw)

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

                if self.path_is_clear(name, fx, fy, target_x, target_y):
                    self.state[name] = 'approach'
                    self.get_logger().info(f'{name}: direct path clear, skipping orbit')
                else:
                    self.state[name] = 'transit_out'
                    self.get_logger().info(f'{name}: path blocked, orbiting around {anchor}')

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
            pub = self.pubs[name]

            if self.state[name] == 'transit_out':
                theta = math.atan2(fy - ay, fx - ax)
                target_x = ax + HOLD_RADIUS * math.cos(theta)
                target_y = ay + HOLD_RADIUS * math.sin(theta)
                d = self.drive_toward(pub, fx, fy, fyaw, target_x, target_y)
                if d <= ORBIT_TOL:
                    self.state[name] = 'orbit'
                    pub.publish(Twist())
                continue

            if self.state[name] == 'orbit':
                current_bearing = math.atan2(fy - ay, fx - ax)
                current_radius = math.hypot(fx - ax, fy - ay)
                err = norm_angle(slot_angle - current_bearing)

                if abs(err) <= ORBIT_TOL and current_radius >= HOLD_RADIUS - 0.15:
                    self.state[name] = 'approach'
                    pub.publish(Twist())
                    continue

                lookahead = min(ORBIT_LOOKAHEAD, abs(err))
                lookahead = math.copysign(lookahead, err)
                next_theta = norm_angle(current_bearing + lookahead)
                target_x = ax + HOLD_RADIUS * math.cos(next_theta)
                target_y = ay + HOLD_RADIUS * math.sin(next_theta)
                self.drive_orbit(pub, fx, fy, fyaw, target_x, target_y)

                self.get_logger().info(
                    f'{name} orbiting {anchor} | angle_err={err:.2f} r={current_radius:.2f}',
                    throttle_duration_sec=1.0)
                continue

            if self.state[name] == 'approach':
                target_x = ax + FACE_DIST * math.cos(slot_angle)
                target_y = ay + FACE_DIST * math.sin(slot_angle)
                d = self.drive_toward(pub, fx, fy, fyaw, target_x, target_y)
                if d <= DOCK_TOL:
                    self.state[name] = 'align'
                    pub.publish(Twist())
                self.get_logger().info(f'{name} -> {anchor} approach | dist={d:.2f}',
                                        throttle_duration_sec=1.0)
                continue

            if self.state[name] == 'align':
                target_yaw = norm_angle(slot_angle + math.pi)
                yaw_err = norm_angle(target_yaw - fyaw)
                if abs(yaw_err) <= FINAL_YAW_TOL:
                    self.state[name] = 'docked'
                    pub.publish(Twist())
                    self.get_logger().info(f'DOCKED! {name} flush on face {face_idx} of {anchor}')
                    continue
                tw = Twist()
                tw.angular.z = KP_ANG * yaw_err
                pub.publish(tw)


def main():
    rclpy.init()
    node = DockingNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()